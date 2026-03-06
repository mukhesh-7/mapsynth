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

    % Safety cap at 40,000 pixels per side to prevent OOM
    MAX_DIM = 40000;
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
    fprintf('  Canvas: %d × %d px (%.1f MB)\n', width, height, width * height * 3 / 1e6);

    % --- Step 2: Distance-weighted blending of all images ---
    % accumulator arrays (double precision for accuracy)
    accumRGB    = zeros(height, width, 3, 'double');
    accumWeight = zeros(height, width, 'double');

    for i = 1:numImages
        if ~validMask(i)
            fprintf('  Skipping image %d (degenerate transform)\n', i);
            continue;
        end

        currentImg = images{i};
        if size(currentImg, 3) ~= 3
            currentImg = repmat(currentImg, [1 1 3]);
        end
        currentImg = im2double(currentImg);

        % Warp the image and its validity mask
        warpedImg  = imwarp(currentImg,  tforms(i), 'OutputView', panoramaView);
        warpedMask = imwarp(true(imageSize(1), imageSize(2)), ...
                            tforms(i), 'OutputView', panoramaView);

        switch lower(blendMethod)
            case {'simple', 'max'}
                % Simple overwrite (last-wins for overlap)
                weight = double(warpedMask);

            otherwise  % 'multiband' and anything else → distance-weighted
                % Weight = distance from the nearest mask boundary
                % (center pixels contribute more than edge pixels)
                distMap = bwdist(~warpedMask);
                weight  = distMap / (max(distMap(:)) + eps);
        end

        for ch = 1:3
            accumRGB(:,:,ch) = accumRGB(:,:,ch) + warpedImg(:,:,ch) .* weight;
        end
        accumWeight = accumWeight + weight;

        if mod(i, 10) == 0 || i == numImages
            fprintf('  Warped %d/%d images...\n', i, numImages);
        end
    end

    % --- Step 3: Normalize accumulated colors ---
    denominator = max(accumWeight, eps);
    panoramaD = zeros(height, width, 3, 'double');
    for ch = 1:3
        panoramaD(:,:,ch) = accumRGB(:,:,ch) ./ denominator;
    end

    % Zero out pixels that received no contribution
    noData = accumWeight < eps;
    for ch = 1:3
        plane = panoramaD(:,:,ch);
        plane(noData) = 0;
        panoramaD(:,:,ch) = plane;
    end

    panorama = im2uint8(panoramaD);
    fprintf('Panorama generated: %d × %d pixels\n', width, height);
end
