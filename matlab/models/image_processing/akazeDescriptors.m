function [points, features] = akazeDescriptors(img)
% AKAZEDESCRIPTORS AKAZE-equivalent feature detection and description
%   Implements AKAZE (Accelerated KAZE) behavior by combining:
%     - KAZE keypoint detection (nonlinear diffusion scale space)
%     - Binary descriptor extraction (Modified-LDB style via FREAK)
%
%   Automatically downscales large images before detection to prevent
%   OpenCV out-of-memory errors, then maps keypoints back to original scale.
%
% Inputs:
%    img - drone image (RGB or grayscale)
% Outputs:
%    points   - detected feature points (in original image coordinates)
%    features - binary feature descriptors (binaryFeatures object)

    % Convert to grayscale if needed
    if size(img, 3) == 3
        grayImg = rgb2gray(img);
    else
        grayImg = img;
    end

    % Ensure uint8
    if ~isa(grayImg, 'uint8')
        grayImg = im2uint8(grayImg);
    end

    % --- Downscale large images for KAZE detection ---
    % KAZE builds a nonlinear scale space (4 octaves x 4 levels = 16 scales)
    % which needs ~16x the image memory. High-res drone images (4000x3000+)
    % cause OpenCV's internal allocator to run out of memory.
    [origH, origW] = size(grayImg);
    MAX_DETECT_DIM = 1200;  % max pixels on longest edge for detection
    scaleFactor = 1.0;

    if max(origH, origW) > MAX_DETECT_DIM
        scaleFactor = MAX_DETECT_DIM / max(origH, origW);
        detectImg = imresize(grayImg, scaleFactor);
        fprintf('AKAZE: Downscaled %dx%d -> %dx%d for detection (scale=%.3f)\n', ...
            origW, origH, size(detectImg, 2), size(detectImg, 1), scaleFactor);
    else
        detectImg = grayImg;
    end

    % --- Step 1: KAZE Detection (nonlinear diffusion scale space) ---
    % Use a low threshold (0.0003) to detect abundant features even in
    % low-contrast regions (roads, rooftops). The 5000-point cap below
    % takes only the strongest, so low threshold = better coverage.
    kazePoints = detectKAZEFeatures(detectImg, ...
        'Threshold', 0.0003, ...
        'NumOctaves', 4, ...
        'NumScaleLevels', 4);

    % Adaptive threshold fallback for very low-texture images (e.g. foggy frames)
    if kazePoints.Count < 100
        kazePoints = detectKAZEFeatures(detectImg, ...
            'Threshold', 0.00005, ...
            'NumOctaves', 4, ...
            'NumScaleLevels', 4);
        fprintf('AKAZE: Relaxed threshold -> %d keypoints\n', kazePoints.Count);
    end

    % Cap keypoints to strongest N — increased from 3000 to 5000 so that
    % adjacent images sharing 60-80% of the scene have more overlap candidates.
    maxPoints = 5000;
    if kazePoints.Count > maxPoints
        kazePoints = kazePoints.selectStrongest(maxPoints);
    end

    % --- Step 2: Binary Descriptor Extraction ---
    [features, validPoints] = extractFeatures(detectImg, kazePoints, ...
        'Method', 'FREAK', ...
        'Upright', false);

    % --- Step 3: Scale keypoints back to original resolution ---
    % KAZEPoints has: Location, Scale, Metric, Orientation
    if scaleFactor < 1.0
        scaledLocation = validPoints.Location / scaleFactor;
        scaledScale    = validPoints.Scale / scaleFactor;
        points = KAZEPoints(scaledLocation, ...
            'Scale', scaledScale, ...
            'Metric', validPoints.Metric, ...
            'Orientation', validPoints.Orientation);
    else
        points = validPoints;
    end

    fprintf('AKAZE: %d keypoints detected (KAZE + FREAK descriptors)\n', points.Count);
end
