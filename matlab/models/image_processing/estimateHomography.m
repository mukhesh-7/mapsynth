function tform = estimateHomography(matchedPoints1, matchedPoints2, previousTform)
% ESTIMATEHOMOGRAPHY Robust homography estimation with MSAC and sanity checks
%   Computes the transform between two sets of matched points, validates the 
%   result for geometric plausibility, and chains it with the previous cumulative transform.
%
%   Uses 'similarity' (translation, rotation, uniform scale) instead of
%   'projective' to prevent severe perspective keystoning ("trapezoid effect")
%   when chaining dozens of images in an aerial flight path.
%
% Inputs:
%    matchedPoints1, matchedPoints2 - matched keypoint locations
%    previousTform - cumulative projective2d from prior image pair
% Outputs:
%    tform - validated cumulative projective2d transform

    % MSAC estimation using 'similarity' to prevent runaway perspective distortion
    try
        [tformStep, inlierIdx] = estimateGeometricTransform2D(...
            matchedPoints2, matchedPoints1, ...
            'similarity', ...
            'Confidence', 99, ...
            'MaxNumTrials', 10000, ...
            'MaxDistance', 8);

        inlierRatio = sum(inlierIdx) / numel(inlierIdx);
        fprintf('  Homography: %d/%d inliers (%.0f%%)\n', ...
            sum(inlierIdx), numel(inlierIdx), inlierRatio * 100);

    catch ex
        warning('estimateHomography:failed', ...
            'Transform estimation failed: %s. Using previous transform.', ex.message);
        tform = previousTform;
        return;
    end

    % Convert the resulting affine2d to projective2d for consistent pipeline types
    H = tformStep.T;
    detH = det(H(1:2, 1:2)); % Determinant of the rotation/scale part

    isDegenerate = false;

    % Check 1: Determinant must be positive (no image flip).
    if detH <= 0
        fprintf('  WARNING: Negative determinant (%.4f) — degenerate transform\n', detH);
        isDegenerate = true;
    end

    % Check 2: Extreme scale changes.
    % Scale changes of up to ×10 / ×0.1 are allowed.
    if detH > 0 && (detH < 0.01 || detH > 100)
        fprintf('  WARNING: Extreme scale (det=%.4f)\n', detH);
        isDegenerate = true;
    end

    % Check 3: Minimum inlier ratio for reliable estimation.
    if inlierRatio < 0.05
        fprintf('  WARNING: Low inlier ratio (%.0f%%) — unreliable estimate\n', inlierRatio * 100);
        isDegenerate = true;
    end

    if isDegenerate
        fprintf('  -> Falling back to previous transform for this pair\n');
        tform = previousTform;
    else
        % Chain pairwise step with the cumulative transform from all prior pairs
        tform = projective2d(previousTform.T * H);
    end
end
