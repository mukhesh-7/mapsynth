function points = detectFeatures(img, method)
% DETECTFEATURES Standalone multi-algorithm keypoint detector
%   Supports AKAZE, SURF, FAST, Harris, MinEigen, ORB, BRISK, and KAZE.
%   Provides a unified interface for swapping detection algorithms.
%
% Inputs:
%    img    - grayscale or RGB image matrix
%    method - (optional) detection algorithm string:
%             'AKAZE' (default), 'SURF', 'FAST', 'Harris', 'MinEigen',
%             'ORB', 'BRISK', 'KAZE'
% Outputs:
%    points - detected feature point object

    if nargin < 2
        method = 'AKAZE';
    end

    % Convert to grayscale if needed
    if size(img, 3) == 3
        gray = rgb2gray(img);
    else
        gray = img;
    end

    method = upper(method);
    fprintf('Detecting features using %s...\n', method);

    switch method
        case 'AKAZE'
            % AKAZE detection = KAZE detection (same nonlinear scale space)
            points = detectKAZEFeatures(gray, ...
                'Threshold', 0.0005, ...
                'NumOctaves', 4, ...
                'NumScaleLevels', 4);

        case 'SURF'
            points = detectSURFFeatures(gray, ...
                'MetricThreshold', 500, ...
                'NumOctaves', 4, ...
                'NumScaleLevels', 6);

        case 'FAST'
            points = detectFASTFeatures(gray, ...
                'MinQuality', 0.1, ...
                'MinContrast', 0.2);

        case 'HARRIS'
            points = detectHarrisFeatures(gray, ...
                'MinQuality', 0.01, ...
                'FilterSize', 5);

        case 'MINEIGEN'
            points = detectMinEigenFeatures(gray, ...
                'MinQuality', 0.01, ...
                'FilterSize', 5);

        case 'ORB'
            points = detectORBFeatures(gray, ...
                'ScaleFactor', 1.2, ...
                'NumLevels', 8);

        case 'BRISK'
            points = detectBRISKFeatures(gray, ...
                'MinQuality', 0.1, ...
                'MinContrast', 0.2);

        case 'KAZE'
            points = detectKAZEFeatures(gray, ...
                'Threshold', 0.001, ...
                'NumOctaves', 4);

        otherwise
            error('detectFeatures:unknownMethod', ...
                'Unknown detection method: %s. Use AKAZE, SURF, FAST, Harris, MinEigen, ORB, BRISK, or KAZE.', ...
                method);
    end

    fprintf('Detected %d keypoints using %s.\n', points.Count, method);
end
