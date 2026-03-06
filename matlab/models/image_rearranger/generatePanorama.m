function panorama = generatePanorama(images, tforms, blendMethod)
% GENERATEPANORAMA High-level wrapper for complete panorama generation
%   Combines alignment, seam selection, and multi-band blending into
%   a single function call. Uses the full pipeline internally.
%
% Inputs:
%    images      - cell array of drone image matrices
%    tforms      - vector of projective2d transforms
%    blendMethod - (optional) 'multiband' (default) or 'simple'
% Outputs:
%    panorama    - final stitched panorama image

    if nargin < 3
        blendMethod = 'multiband';
    end

    numImages = numel(images);
    fprintf('Generating panorama from %d images (blend: %s)...\n', ...
        numImages, blendMethod);

    % --- Step 1: Compute output canvas dimensions ---
    imageSize = size(images{1});
    xlim = zeros(numImages, 2);
    ylim = zeros(numImages, 2);

    for i = 1:numImages
        [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), ...
            [1 imageSize(2)], [1 imageSize(1)]);
    end

    xMin = floor(min([1; xlim(:)]));
    xMax = ceil(max([imageSize(2); xlim(:)]));
    yMin = floor(min([1; ylim(:)]));
    yMax = ceil(max([imageSize(1); ylim(:)]));

    width  = xMax - xMin + 1;
    height = yMax - yMin + 1;

    panoramaView = imref2d([height, width], [xMin xMax], [yMin yMax]);

    % --- Step 2: Warp all images onto the canvas ---
    warpedImages = cell(1, numImages);
    warpedMasks  = cell(1, numImages);

    for i = 1:numImages
        currentImg = images{i};
        if size(currentImg, 3) ~= 3
            currentImg = repmat(currentImg, [1 1 3]);
        end

        warpedImages{i} = imwarp(currentImg, tforms(i), 'OutputView', panoramaView);
        warpedMasks{i}  = imwarp(true(size(currentImg,1), size(currentImg,2)), ...
                                  tforms(i), 'OutputView', panoramaView);
    end

    % --- Step 3: Sequential blending ---
    panorama = warpedImages{1};
    compositeMask = warpedMasks{1};

    for i = 2:numImages
        switch lower(blendMethod)
            case 'multiband'
                % Use seam selection + multi-band blending
                overlapRegion = compositeMask & warpedMasks{i};
                
                if any(overlapRegion(:))
                    seam = seamSelection(panorama, warpedImages{i}, overlapRegion);
                    
                    % Apply seam: composite from both sides
                    newPanorama = panorama;
                    for ch = 1:3
                        src = warpedImages{i}(:,:,ch);
                        dst = newPanorama(:,:,ch);
                        dst(~seam & warpedMasks{i}) = src(~seam & warpedMasks{i});
                        newPanorama(:,:,ch) = dst;
                    end
                    panorama = newPanorama;
                else
                    % No overlap: simple paste
                    for ch = 1:3
                        src = warpedImages{i}(:,:,ch);
                        dst = panorama(:,:,ch);
                        dst(warpedMasks{i}) = src(warpedMasks{i});
                        panorama(:,:,ch) = dst;
                    end
                end

            case 'simple'
                % Simple max-intensity blending
                panorama = max(panorama, warpedImages{i});

            otherwise
                panorama = max(panorama, warpedImages{i});
        end

        compositeMask = compositeMask | warpedMasks{i};
    end

    % --- Step 4: Apply final Gaussian smoothing on blend edges ---
    compositeMaskFloat = single(compositeMask);
    blurredMask = imgaussfilt(compositeMaskFloat, 3);
    for ch = 1:3
        panorama(:,:,ch) = uint8(single(panorama(:,:,ch)) .* min(blurredMask, 1));
    end

    fprintf('Panorama generated: %d × %d pixels\n', width, height);
end
