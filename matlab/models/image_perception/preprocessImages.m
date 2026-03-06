function processedImages = preprocessImages(images)
% PREPROCESSIMAGES Normalizes drone image batch for consistent pipeline processing
%   Applies resolution standardization, histogram equalization, and noise
%   reduction across a cell array of raw drone captures.
%
% Inputs:
%    images - cell array of raw uint8/uint16 image matrices
% Outputs:
%    processedImages - cell array of normalized uint8 image matrices

    numImages = numel(images);
    processedImages = cell(1, numImages);

    % Establish target resolution from the first valid image, capped to
    % prevent memory issues with high-resolution drone imagery.
    % For large batches (>20 images) use a lower cap to conserve RAM.
    if numImages > 20
        MAX_DIM = 1200;
        fprintf('Large batch detected (%d images): capping resolution to %dpx for memory efficiency.\n', ...
            numImages, MAX_DIM);
    else
        MAX_DIM = 1600;  % max pixels on longest edge
    end
    refImg = images{1};
    targetHeight = size(refImg, 1);
    targetWidth  = size(refImg, 2);

    % Downscale target if images are too large
    if max(targetHeight, targetWidth) > MAX_DIM
        downScale = MAX_DIM / max(targetHeight, targetWidth);
        targetHeight = round(targetHeight * downScale);
        targetWidth  = round(targetWidth  * downScale);
        fprintf('Capping resolution to %dx%d (was %dx%d)\n', ...
            targetWidth, targetHeight, size(refImg, 2), size(refImg, 1));
    end

    fprintf('Preprocessing %d images (target: %dx%d)...\n', numImages, targetWidth, targetHeight);

    for i = 1:numImages
        img = images{i};

        if isempty(img)
            warning('Image %d is empty, skipping.', i);
            processedImages{i} = [];
            continue;
        end

        % --- Step 1: Bit Depth Normalization ---
        % Convert 16-bit or double images to standard uint8
        if isa(img, 'uint16')
            img = im2uint8(img);
        elseif isfloat(img)
            img = im2uint8(img);
        end

        % --- Step 2: Resolution Standardization ---
        % Resize to match reference dimensions for consistent feature matching
        [h, w, ~] = size(img);
        if h ~= targetHeight || w ~= targetWidth
            img = imresize(img, [targetHeight, targetWidth]);
        end

        % --- Step 3: Contrast Enhancement ---
        % Apply CLAHE (Contrast-Limited Adaptive Histogram Equalization)
        % per channel to improve feature detection in shadows/highlights
        if size(img, 3) == 3
            labImg = rgb2lab(img);
            % Enhance only the L channel (luminance)
            labImg(:,:,1) = adapthisteq(labImg(:,:,1) / 100, ...
                'ClipLimit', 0.02, 'Distribution', 'rayleigh') * 100;
            img = lab2rgb(labImg, 'OutputType', 'uint8');
        else
            img = adapthisteq(img, 'ClipLimit', 0.02, 'Distribution', 'rayleigh');
        end

        % --- Step 4: Noise Reduction ---
        % Light Gaussian denoising to preserve edges while reducing sensor noise
        if size(img, 3) == 3
            img = imgaussfilt(img, 0.8);  % RGB: mild smoothing to preserve color edges
        else
            img = imgaussfilt(img, 0.6);  % Grayscale: tighter kernel for single-channel
        end

        processedImages{i} = img;
    end

    % Remove empty entries
    emptyMask = cellfun(@isempty, processedImages);
    if any(emptyMask)
        warning('%d images were empty and removed from the batch.', sum(emptyMask));
        processedImages = processedImages(~emptyMask);
    end

    fprintf('Preprocessing complete. %d images ready.\n', numel(processedImages));
end
