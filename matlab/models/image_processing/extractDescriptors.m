function [points, features] = extractDescriptors(img, method)
% EXTRACTDESCRIPTORS Detects features and produces descriptors using the specified algorithm
% Inputs:
%    img    - standardized drone image matrix (RGB or grayscale)
%    method - (optional) detector algorithm: 'AKAZE' (default), 'SURF', 'KAZE',
%             'FAST', 'HARRIS', 'ORB', 'BRISK', 'MINEIGEN'
% Outputs:
%    points   - detected feature point locations
%    features - descriptor vectors/matrix for matching

    if nargin < 2
        method = 'AKAZE';
    end

    % Standardize image color space to grayscale
    if size(img, 3) == 3
        grayImg = rgb2gray(img);
    else
        grayImg = img;
    end

    method = upper(method);

    switch method
        case 'AKAZE'
            % AKAZE: KAZE detection (nonlinear diffusion) + binary descriptors
            [points, features] = akazeDescriptors(img);
            return;  % akazeDescriptors handles everything internally

        case 'KAZE'
            pts = detectKAZEFeatures(grayImg, ...
                'Threshold', 0.0005, 'NumOctaves', 4);

        case 'SURF'
            pts = detectSURFFeatures(grayImg, 'MetricThreshold', 500);

        case 'FAST'
            pts = detectFASTFeatures(grayImg, ...
                'MinQuality', 0.1, 'MinContrast', 0.2);

        case 'HARRIS'
            pts = detectHarrisFeatures(grayImg, ...
                'MinQuality', 0.01, 'FilterSize', 5);

        case 'ORB'
            pts = detectORBFeatures(grayImg, ...
                'ScaleFactor', 1.2, 'NumLevels', 8);

        case 'BRISK'
            pts = detectBRISKFeatures(grayImg, ...
                'MinQuality', 0.1, 'MinContrast', 0.2);

        case 'MINEIGEN'
            pts = detectMinEigenFeatures(grayImg, ...
                'MinQuality', 0.01, 'FilterSize', 5);

        otherwise
            warning('extractDescriptors:unknownMethod', ...
                'Unknown method "%s". Falling back to AKAZE.', method);
            [points, features] = akazeDescriptors(img);
            return;
    end

    % Extract descriptors for non-AKAZE methods
    [features, points] = extractFeatures(grayImg, pts);
end
