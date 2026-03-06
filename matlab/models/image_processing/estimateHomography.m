function tform = estimateHomography(matchedPoints1, matchedPoints2, previousTform)
% ESTIMATEHOMOGRAPHY Robust homography estimation with MSAC and sanity checks
%   Computes the projective transform between two sets of matched points,
%   validates the result for geometric plausibility, and chains it with
%   the previous cumulative transform.
%
% Inputs:
%    matchedPoints1, matchedPoints2 - matched keypoint locations
%    previousTform - cumulative projective2d from prior image pair
% Outputs:
%    tform - validated cumulative projective2d transform

    % MSAC estimation with generous trial budget to handle high outlier ratios
    % MaxDistance = 3 pixels rejects correspondences with large reprojection error
    try
        [tformStep, inlierIdx] = estimateGeometricTransform2D(...
            matchedPoints2, matchedPoints1, ...
            'projective', ...
            'Confidence', 99, ...
            'MaxNumTrials', 10000, ...
            'MaxDistance', 3);

        inlierRatio = sum(inlierIdx) / numel(inlierIdx);
        fprintf('  Homography: %d/%d inliers (%.0f%%)\n', ...
            sum(inlierIdx), numel(inlierIdx), inlierRatio * 100);

    catch ex
        % If estimation fails completely, carry forward previous transform
        warning('estimateHomography:failed', ...
            'Transform estimation failed: %s. Using previous transform.', ex.message);
        tform = previousTform;
        return;
    end

    % --- Sanity check the pairwise homography ---
    % A valid pairwise homography for consecutive drone images should:
    %   - Have a determinant near 1 (no extreme scale change)
    %   - Not flip the image (det > 0)
    %   - Have limited off-diagonal projective terms (no extreme perspective)
    H = tformStep.T;
    detH = det(H);

    isDegenerate = false;

    if detH <= 0
        fprintf('  WARNING: Negative determinant (%.4f) — degenerate transform\n', detH);
        isDegenerate = true;
    elseif detH < 0.1 || detH > 10
        fprintf('  WARNING: Extreme scale (det=%.4f) — suspicious transform\n', detH);
        isDegenerate = true;
    end

    % Check projective terms (H(3,1) and H(3,2)) — should be near zero
    % for aerial images where camera is roughly above the scene
    if abs(H(3,1)) > 0.005 || abs(H(3,2)) > 0.005
        fprintf('  WARNING: Large projective distortion (h31=%.6f, h32=%.6f)\n', H(3,1), H(3,2));
        isDegenerate = true;
    end

    % If inlier ratio is too low, the homography is unreliable
    if inlierRatio < 0.15
        fprintf('  WARNING: Low inlier ratio (%.0f%%) — unreliable estimate\n', inlierRatio * 100);
        isDegenerate = true;
    end

    if isDegenerate
        fprintf('  -> Falling back to previous transform for this pair\n');
        tform = previousTform;
    else
        % Chain with cumulative transform
        tform = projective2d(previousTform.T * tformStep.T);
    end
end
