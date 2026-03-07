function [indexPairs, matchedPoints1, matchedPoints2] = matchFeaturesNN(features1, features2, points1, points2)
% MATCHFEATURESNN Nearest-neighbor feature matching with automatic metric selection
%   Handles both binary descriptors (AKAZE/FREAK/ORB/BRISK → Hamming distance)
%   and float descriptors (SURF/KAZE → SSD distance) transparently.
%
%   Thresholds are tuned for aerial/drone survey imagery:
%   - High MatchThreshold: accept more candidates; MSAC filters bad ones
%   - High MaxRatio: relaxed Lowe ratio test for partial-overlap shots
%
% Inputs:
%    features1, features2 - Descriptor blocks from extractDescriptors()
%    points1, points2     - Associated keypoint locations
% Outputs:
%    indexPairs      - N×2 index correspondence matrix
%    matchedPoints1  - Matched keypoints from image 1
%    matchedPoints2  - Matched keypoints from image 2

    if isa(features1, 'binaryFeatures')
        % Binary descriptors (FREAK / ORB / BRISK) — Hamming distance 0-100%.
        % MatchThreshold = 80: accept pairs with up to 80% bit mismatch.
        % MaxRatio = 0.9: relaxed Lowe ratio test
        indexPairs = matchFeatures(features1, features2, ...
            'Unique', true, ...
            'MatchThreshold', 80, ...
            'MaxRatio', 0.9, ...
            'Method', 'Approximate');
    else
        % Float descriptors (SURF, KAZE) — SSD/SAD distance.
        indexPairs = matchFeatures(features1, features2, ...
            'Unique', true, ...
            'MatchThreshold', 10.0, ...
            'MaxRatio', 0.85, ...
            'Method', 'Approximate');
    end

    % Extract matched keypoint coordinates
    matchedPoints1 = points1(indexPairs(:, 1), :);
    matchedPoints2 = points2(indexPairs(:, 2), :);
end
