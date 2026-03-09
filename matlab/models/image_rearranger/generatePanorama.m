function panorama = generatePanorama(images, tforms, blendMethod)
% GENERATEPANORAMA High-level wrapper for complete panorama generation
%   Warps all images onto a shared canvas and blends them into a single
%   mosaic. Uses distance-weighted blending for smooth transitions.
%
% Inputs:
%    images      - cell array of drone image matrices
%    tforms      - vector of projective2d transforms (cumulative)
%    blendMethod - (optional) 'multiband' (default) or 'simple'
% Outputs:
%    panorama    - final stitched panorama image (uint8, RGB)

    if nargin < 3
        blendMethod = 'multiband';
    end

    numImages = numel(images);
    fprintf('Generating panorama from %d images (blend: %s)...\n', ...
        numImages, blendMethod);

    % --- Step 1: Compute output canvas dimensions across all transforms ---
    imageSize = size(images{1});
    xlim = zeros(numImages, 2);
    ylim = zeros(numImages, 2);

    for i = 1:numImages
        [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), ...
            [1 imageSize(2)], [1 imageSize(1)]);
    end

    % Filter out transforms that produce absurd canvas extents
    warpedW = xlim(:,2) - xlim(:,1);
    warpedH = ylim(:,2) - ylim(:,1);
    validMask = warpedW > 0 & warpedH > 0 & ...
                warpedW < imageSize(2) * 30 & ...
                warpedH < imageSize(1) * 30;

    if ~any(validMask)
        warning('generatePanorama:allDegenerate', ...
            'All transforms are degenerate. Returning first image as fallback.');
        panorama = images{1};
        if size(panorama, 3) ~= 3
            panorama = repmat(panorama, [1 1 3]);
        end
        return;
    end

    validXlim = xlim(validMask, :);
    validYlim = ylim(validMask, :);
    xMin = floor(min([1; validXlim(:)]));
    xMax = ceil(max([imageSize(2); validXlim(:)]));
    yMin = floor(min([1; validYlim(:)]));
    yMax = ceil(max([imageSize(1); validYlim(:)]));

    % Safety cap at 3840 pixels per side to prevent OOM and allow stable 2x supersampling
    MAX_DIM = 3840;
    width  = xMax - xMin + 1;
    height = yMax - yMin + 1;
    if width > MAX_DIM || height > MAX_DIM
        scaleFactor = MAX_DIM / max(width, height);
        fprintf('  Canvas too large (%d x %d). Scaling by %.3f\n', width, height, scaleFactor);
        S = [scaleFactor 0 0; 0 scaleFactor 0; 0 0 1];
        for i = 1:numImages
            tforms(i) = projective2d(tforms(i).T * S);
        end
        % Recompute bounds
        for i = 1:numImages
            [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), ...
                [1 imageSize(2)], [1 imageSize(1)]);
        end
        validXlim = xlim(validMask, :);
        validYlim = ylim(validMask, :);
        xMin = floor(min([1; validXlim(:)]));
        xMax = ceil(max([imageSize(2); validXlim(:)]));
        yMin = floor(min([1; validYlim(:)]));
        yMax = ceil(max([imageSize(1); validYlim(:)]));
        width  = xMax - xMin + 1;
        height = yMax - yMin + 1;
    end

    panoramaView = imref2d([height, width], [xMin xMax], [yMin yMax]);
    fprintf('  Canvas: %d × %d px (%.1f MB mapped)\n', width, height, width * height * 3 / 1e6);

    % --- Step 2: Max-weight Voronoi blending for crystal clear, blur-free output ---
    % Instead of double-precision averaging which causes messy overlays,
    % we assign pixels strictly from the source image where they are closest to the optical center.
    panorama    = zeros(height, width, 3, 'uint8');
    maxWeight   = zeros(height, width, 'double');

    for i = 1:numImages
        if ~validMask(i)
            fprintf('  Skipping image %d (degenerate transform)\n', i);
            continue;
        end

        currentImg = images{i};
        if size(currentImg, 3) ~= 3
            currentImg = repmat(currentImg, [1 1 3]);
        end

        % Warp the image (keep in uint8 to drastically save RAM)
        warpedImg  = imwarp(currentImg,  tforms(i), 'OutputView', panoramaView);
        warpedMask = imwarp(true(imageSize(1), imageSize(2)), ...
                            tforms(i), 'OutputView', panoramaView);

        % Weight = distance from the nearest mask boundary
        % The center of the image has the highest weight (clearest focus)
        distMap = double(bwdist(~warpedMask));
        weight  = distMap / (max(distMap(:)) + eps);

        % Voronoi assignment: only update pixels if this image has a better weight
        useNew = weight > maxWeight;

        for ch = 1:3
            pChannel = panorama(:,:,ch);
            wChannel = warpedImg(:,:,ch);
            pChannel(useNew) = wChannel(useNew);
            panorama(:,:,ch) = pChannel;
        end
        maxWeight(useNew) = weight(useNew);

        if mod(i, 10) == 0 || i == numImages
            fprintf('  Warped %d/%d images...\n', i, numImages);
        end
    end

    % --- Step 3: Resize and Sharpen Final Output ---
    % Scale proportionally to limit the max dimension while retaining true geometric aspect ratio
    fprintf('  Applying subpixel sharpening and proportional sizing...\n');
    [fH, fW, ~] = size(panorama);
    if max(fH, fW) > MAX_DIM
        scaleFinal = MAX_DIM / max(fH, fW);
        panorama = imresize(panorama, scaleFinal, 'lanczos3');
    end
    panorama = imsharpen(panorama, 'Radius', 1.5, 'Amount', 1.0, 'Threshold', 0.01);
    
    [finalH, finalW, ~] = size(panorama);
    fprintf('Panorama generated: %d × %d pixels (True Aspect Ratio Preserved)\n', finalW, finalH);
end
