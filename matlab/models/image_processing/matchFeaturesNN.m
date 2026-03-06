function [indexPairs, matchedPoints1, matchedPoints2] = matchFeaturesNN(features1, features2, points1, points2)
% MATCHFEATURESNN Nearest-neighbor feature matching with automatic metric selection
%   Handles both binary descriptors (AKAZE/ORB/BRISK → Hamming distance)
%   and float descriptors (SURF/KAZE → SSD distance) transparently.
%
% Inputs:
%    features1, features2 - Descriptor blocks from extractDescriptors()
%    points1, points2     - Associated keypoint locations
% Outputs:
%    indexPairs      - N×2 index correspondence matrix
%    matchedPoints1  - Matched keypoints from image 1
%    matchedPoints2  - Matched keypoints from image 2

    % Detect descriptor type and set appropriate matching parameters
    if isa(features1, 'binaryFeatures')
        % Binary descriptors (AKAZE, ORB, BRISK, FREAK) use Hamming distance
        % Threshold 30 balances recall vs outlier rejection for aerial images
        indexPairs = matchFeatures(features1, features2, ...
            'Unique', true, ...
            'MatchThreshold', 30, ...
            'MaxRatio', 0.6, ...
            'Method', 'Approximate');
    else
        % Float descriptors (SURF, KAZE) use SSD/SAD distance
        indexPairs = matchFeatures(features1, features2, ...
            'Unique', true, ...
            'MatchThreshold', 1.5, ...
            'MaxRatio', 0.6, ...
            'Method', 'Approximate');
    end

    % Extract matched keypoint coordinates
    matchedPoints1 = points1(indexPairs(:, 1), :);
    matchedPoints2 = points2(indexPairs(:, 2), :);
end
