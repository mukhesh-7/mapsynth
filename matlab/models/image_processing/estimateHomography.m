function [tform, isValid] = estimateHomography(matchedPoints1, matchedPoints2, previousTform)
% ESTIMATEHOMOGRAPHY Robust homography estimation with projective/affine and sanity checks
%   Computes the transform between two sets of matched points, validates the 
%   result for geometric plausibility, and chains it with the previous cumulative transform.
%
% Inputs:
%    matchedPoints1, matchedPoints2 - matched keypoint locations
%    previousTform - cumulative projective2d from prior image pair
% Outputs:
%    tform - validated cumulative projective2d transform
%    isValid - boolean indicating if the transform estimation was successful

    isValid = true;
    
    if size(matchedPoints1, 1) < 4
        fprintf('  WARNING: Not enough points (%d) to compute homography.\n', size(matchedPoints1, 1));
        tform = previousTform;
        isValid = false;
        return;
    end

    % Try 'similarity' (translation, rotation, uniform scale)
    % This is critical to prevent runaway perspective distortion (keystoning/trapezoid effect)
    % when chaining images in an aerial flight path!
    try
        [tformStep, inlierIdx] = estimateGeometricTransform2D(...
            matchedPoints2, matchedPoints1, ...
            'similarity', ...
            'Confidence', 99.9, ...
            'MaxNumTrials', 10000, ...
            'MaxDistance', 8);

        inlierRatio = sum(inlierIdx) / numel(inlierIdx);
        fprintf('  Similarity: %d/%d inliers (%.0f%%)\n', ...
            sum(inlierIdx), numel(inlierIdx), inlierRatio * 100);

    catch ex
        fprintf('  Similarity transform failed: %s. Trying affine...\n', ex.message);
        try
            [tformStep, inlierIdx] = estimateGeometricTransform2D(...
                matchedPoints2, matchedPoints1, ...
                'affine', ...
                'Confidence', 99.9, ...
                'MaxNumTrials', 5000, ...
                'MaxDistance', 8);
            
            inlierRatio = sum(inlierIdx) / numel(inlierIdx);
            fprintf('  Affine: %d/%d inliers (%.0f%%)\n', ...
                sum(inlierIdx), numel(inlierIdx), inlierRatio * 100);
        catch ex2
            warning('estimateHomography:failed', ...
                'Transform estimation failed completely: %s. Using previous transform.', ex2.message);
            tform = previousTform;
            isValid = false;
            return;
        end
    end

    % Check 1: Minimum inlier ratio and count for reliable estimation
    if inlierRatio < 0.05 || sum(inlierIdx) < 8
        fprintf('  WARNING: Low inlier count/ratio (%d, %.0f%%) — unreliable estimate.\n', sum(inlierIdx), inlierRatio * 100);
        tform = previousTform;
        isValid = false;
        return;
    end

    % Convert the resulting transform to projective2d for consistent pipeline types
    if isa(tformStep, 'affine2d')
        H = [tformStep.T(1,1:2) 0; tformStep.T(2,1:2) 0; tformStep.T(3,1:2) 1];
    else
        H = tformStep.T;
    end
    
    detH = det(H(1:2, 1:2));

    absDetH = abs(detH);
    
    % Check 2: Allow image flips/mirrors (negative determinant).
    % Scale changes of up to ×10 / ×0.1 are allowed.
    if absDetH < 0.1 || absDetH > 10
        fprintf('  WARNING: Invalid scale (det = %.4f)\n', detH);
        tform = previousTform;
        isValid = false;
        return;
    end

    % Chain pairwise step with the cumulative transform from all prior pairs
    % In MATLAB, [u v 1] = [x y 1] * T. So mapping from current to global is H * previousTform.T
    tform = projective2d(H * previousTform.T);
end
