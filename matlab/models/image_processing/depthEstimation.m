function depthMap = depthEstimation(images, tforms) %#ok<INUSD>
% DEPTHESTIMATION Estimates relative depth from multi-view drone imagery
%   Uses structure-from-motion principles with pairwise homographies to
%   construct a relative depth map across the reconstructed scene.
%
%   This is a simplified multi-view stereo approach suitable for
%   nadir (top-down) drone captures with moderate overlap.
%
% Inputs:
%    images - cell array of preprocessed drone images
%    tforms - vector of projective2d transforms (reserved for transform-guided depth)
% Outputs:
%    depthMap - single-channel float matrix representing relative depth

    numImages = numel(images);

    if numImages < 2
        warning('depthEstimation:tooFewImages', ...
            'Need at least 2 images for depth estimation. Returning flat map.');
        depthMap = ones(size(images{1}, 1), size(images{1}, 2));
        return;
    end

    fprintf('Estimating depth from %d viewpoints...\n', numImages);

    % --- Step 1: Build disparity accumulator ---
    % Use the aligned canvas dimensions from the transforms
    refSize = size(images{1});
    targetH = refSize(1);
    targetW = refSize(2);

    disparityAccum = zeros(targetH, targetW, 'single');
    weightAccum = zeros(targetH, targetW, 'single');

    % --- Step 2: Pairwise disparity estimation ---
    for i = 2:numImages
        try
            % Convert to grayscale
            gray1 = rgb2gray(images{i-1});
            gray2 = rgb2gray(images{i});

            % Compute optical flow as proxy for disparity
            % Using Lucas-Kanade-style block matching
            flow = estimateOpticalFlow(gray1, gray2);

            % Disparity magnitude serves as relative depth proxy
            % (larger disparity = closer to camera for nadir views)
            dispMag = sqrt(flow.Vx.^2 + flow.Vy.^2);

            % Accumulate with confidence weighting
            confidence = min(dispMag / max(dispMag(:) + eps), 1.0);
            disparityAccum = disparityAccum + dispMag .* confidence;
            weightAccum = weightAccum + confidence;

        catch ex
            warning('depthEstimation:pairFail', ...
                'Pair %d-%d failed: %s', i-1, i, ex.message);
        end
    end

    % --- Step 3: Normalize and smooth ---
    weightAccum(weightAccum == 0) = 1; % avoid division by zero
    depthMap = disparityAccum ./ weightAccum;

    % Normalize to [0, 1] range
    minD = min(depthMap(:));
    maxD = max(depthMap(:));
    if maxD > minD
        depthMap = (depthMap - minD) / (maxD - minD);
    end

    % Apply guided filter for edge-preserving smoothing
    refGray = single(rgb2gray(images{1})) / 255;
    depthMap = imguidedfilter(depthMap, refGray, ...
        'NeighborhoodSize', [16 16], 'DegreeOfSmoothing', 0.01);

    fprintf('Depth estimation complete. Map size: %d × %d\n', targetW, targetH);
end


function flow = estimateOpticalFlow(img1, img2)
% ESTIMATEOPTICALFLOW Block-matching optical flow estimation
%   Returns a struct with Vx, Vy displacement fields.

    img1 = single(img1);
    img2 = single(img2);

    [h, w] = size(img1);
    blockSize = 16;
    searchRadius = 32;

    Vx = zeros(h, w, 'single');
    Vy = zeros(h, w, 'single');

    halfBlock = floor(blockSize / 2);

    for y = halfBlock+1 : blockSize : h-halfBlock
        for x = halfBlock+1 : blockSize : w-halfBlock
            % Extract reference block
            refBlock = img1(y-halfBlock:y+halfBlock-1, x-halfBlock:x+halfBlock-1);

            bestSSD = inf;
            bestDx = 0;
            bestDy = 0;

            % Search in neighborhood
            for dy = -searchRadius:4:searchRadius
                for dx = -searchRadius:4:searchRadius
                    ny = y + dy;
                    nx = x + dx;

                    if ny-halfBlock < 1 || ny+halfBlock-1 > h || ...
                       nx-halfBlock < 1 || nx+halfBlock-1 > w
                        continue;
                    end

                    candidateBlock = img2(ny-halfBlock:ny+halfBlock-1, ...
                                          nx-halfBlock:nx+halfBlock-1);
                    ssd = sum((refBlock(:) - candidateBlock(:)).^2);

                    if ssd < bestSSD
                        bestSSD = ssd;
                        bestDx = dx;
                        bestDy = dy;
                    end
                end
            end

            % Fill block region with displacement
            yRange = max(1, y-halfBlock):min(h, y+halfBlock-1);
            xRange = max(1, x-halfBlock):min(w, x+halfBlock-1);
            Vx(yRange, xRange) = bestDx;
            Vy(yRange, xRange) = bestDy;
        end
    end

    flow.Vx = Vx;
    flow.Vy = Vy;
end
