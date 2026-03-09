function [panorama, mask] = alignImages(images, tforms)
% ALIGNIMAGES Warps images onto a shared canvas using homography transforms
%   Includes canvas size safety limits and bad-transform filtering to
%   prevent out-of-memory crashes from degenerate homographies.
%
% Inputs:
%    images - cell array of original drone image matrices
%    tforms - vector of projective2d cumulative transforms
% Outputs:
%    panorama - stitched panoramic image on the global canvas
%    mask     - binary mask of valid pixel regions

    numImages = numel(tforms);
    imageSize = size(images{1});

    % --- Compute output bounds for each warped image ---
    xlim = zeros(numImages, 2);
    ylim = zeros(numImages, 2);
    validTransform = true(1, numImages);

    for i = 1:numImages
        [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), ...
            [1 imageSize(2)], [1 imageSize(1)]);

        % Flag transforms that produce absurd bounds
        warpedW = xlim(i,2) - xlim(i,1);
        warpedH = ylim(i,2) - ylim(i,1);
        if warpedW > imageSize(2) * 20 || warpedH > imageSize(1) * 20 || ...
           warpedW < 0 || warpedH < 0
            fprintf('  SKIP image %d: degenerate warp bounds (%.0f x %.0f)\n', ...
                i, warpedW, warpedH);
            validTransform(i) = false;
        end
    end

    % Use only valid transforms for canvas bounds
    validXlim = xlim(validTransform, :);
    validYlim = ylim(validTransform, :);

    if isempty(validXlim)
        error('alignImages:noValidTransforms', ...
            'All transforms are degenerate. Cannot create panorama.');
    end

    % --- Compute canvas dimensions ---
    xMin = min([1; validXlim(:)]);
    xMax = max([imageSize(2); validXlim(:)]);
    yMin = min([1; validYlim(:)]);
    yMax = max([imageSize(1); validYlim(:)]);

    width  = round(xMax - xMin);
    height = round(yMax - yMin);

    % Safety cap: limit canvas to MAX_PIXELS to prevent OOM
    MAX_DIM = 30000;  % 30k pixels per side — ~2.7GB for RGB uint8
    if width > MAX_DIM || height > MAX_DIM
        scaleFactor = MAX_DIM / max(width, height);
        fprintf('  Canvas too large (%d x %d). Scaling by %.2f\n', ...
            width, height, scaleFactor);

        % Scale all transforms to fit within the cap
        S = [scaleFactor 0 0; 0 scaleFactor 0; 0 0 1];
        for i = 1:numImages
            tforms(i) = projective2d(tforms(i).T * S);
        end

        % Recompute bounds with scaled transforms
        for i = 1:numImages
            [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), ...
                [1 imageSize(2)], [1 imageSize(1)]);
        end
        validXlim = xlim(validTransform, :);
        validYlim = ylim(validTransform, :);
        xMin = min([1; validXlim(:)]);
        xMax = max([imageSize(2); validXlim(:)]);
        yMin = min([1; validYlim(:)]);
        yMax = max([imageSize(1); validYlim(:)]);
        width  = round(xMax - xMin);
        height = round(yMax - yMin);
    end

    fprintf('  Canvas size: %d x %d px (%.1f MB)\n', ...
        width, height, width * height * 3 / 1e6);

    % --- Initialize canvas ---
    panorama = zeros([height width 3], 'like', images{1});
    mask = false(height, width);
    blendWeight = zeros(height, width);
    isFirstImage = true;

    panoramaView = imref2d([height width], [xMin xMax], [yMin yMax]);

    % --- Warp and composite each valid image ---
    for i = 1:numImages
        if ~validTransform(i)
            continue;
        end

        % Ensure 3-channel
        if size(images{i}, 3) ~= 3
            currentImg = repmat(images{i}, [1, 1, 3]);
        else
            currentImg = images{i};
        end

        warpedImg = imwarp(currentImg, tforms(i), 'OutputView', panoramaView);
        warpedMask = imwarp(true(size(currentImg, 1), size(currentImg, 2)), ...
            tforms(i), 'OutputView', panoramaView);

        % Distance from mask edge serves as absolute Voronoi confidence
        warpedDist = double(bwdist(~warpedMask));

        if isFirstImage
            panorama = warpedImg;
            blendWeight = warpedDist;
            mask = warpedMask;
            isFirstImage = false;
            continue;
        end

        % Robust Exposure compensation (Mean matching in overlap region)
        overlapMask = mask & warpedMask;
        if any(overlapMask(:))
            wImgD = im2double(warpedImg);
            pImgD = im2double(panorama);
            for c = 1:3
                wChan = wImgD(:,:,c);
                pChan = pImgD(:,:,c);
                
                wMean = mean(wChan(overlapMask));
                pMean = mean(pChan(overlapMask));
                
                % Simple mean shift provides stable exposure compensation
                wChanAdj = wChan + (pMean - wMean);
                wChanAdj = max(0, min(1, wChanAdj));
                wChan(warpedMask) = wChanAdj(warpedMask);
                wImgD(:,:,c) = wChan;
            end
            if isa(images{1}, 'uint8')
                warpedImg = cast(wImgD * 255, 'like', images{1});
            else
                warpedImg = cast(wImgD, 'like', images{1});
            end
        end

        % Calculate exact Voronoi boundary
        % 1 where the new image should take over, 0 where the old image stays
        newAlpha = double(warpedDist > blendWeight);
        
        % Ensure newAlpha is 0 completely outside the new image
        newAlpha(~warpedMask) = 0;
        % Ensure newAlpha is 1 completely in empty canvas areas where only new image exists
        newAlpha(warpedMask & ~mask) = 1;
        
        % Soften the cut very slightly to eliminate harsh seam lines
        % By feathering ONLY near the Voronoi line with a small radius (sigma=1),
        % we completely avoid wide-radius "ghosting" (overlapped look).
        blendFilter = fspecial('gaussian', 7, 1.5);
        smoothAlpha = imfilter(newAlpha, blendFilter, 'replicate');
        
        % Lock the boundaries so blending doesn't leak into empty canvas
        smoothAlpha(~warpedMask) = 0;
        smoothAlpha(warpedMask & ~mask) = 1;
        smoothAlpha(~mask & ~warpedMask) = 0;
        
        % Perform exactly controlled alpha blending
        pImgD = im2double(panorama);
        wImgD = im2double(warpedImg);
        
        for c = 1:3
            pChan = pImgD(:,:,c);
            wChan = wImgD(:,:,c);
            
            pChan = wChan .* smoothAlpha + pChan .* (1 - smoothAlpha);
            pImgD(:,:,c) = pChan;
        end
        
        if isa(images{1}, 'uint8')
            panorama = cast(pImgD * 255, 'like', images{1});
        else
            panorama = cast(pImgD, 'like', images{1});
        end
        
        % Update state for the next image using exact absolute metric 
        blendWeight = max(blendWeight, warpedDist);
        mask = mask | warpedMask;
    end
end
