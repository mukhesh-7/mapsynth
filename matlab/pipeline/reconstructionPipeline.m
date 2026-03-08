function pipelineResult = reconstructionPipeline(imageDir, outputDir, detector)
% RECONSTRUCTIONPIPELINE Main pipeline for drone image stitching
% Processes images from imageDir and saves outputs to outputDir
%   detector - (optional) feature algorithm: 'AKAZE' (default), 'SURF', 'KAZE', etc.

    if nargin < 3
        detector = 'AKAZE';
    end

    disp('Starting Drone Image Reconstruction Pipeline...');
    
    % --- 1. Perception Module ---
    disp('Loading and preprocessing images...');
    images = loadDroneImages(imageDir);
    if isempty(images)
        error('No images found in the specified directory.');
    end
    images = preprocessImages(images);
    images = distortionCorrection(images);
    
    numImages = numel(images);
    disp(['Successfully loaded ' num2str(numImages) ' images.']);
    
    % --- 2. Processing Module ---
    fprintf('Extracting features using %s and estimating homographies...\n', detector);
    features = cell(1, numImages);
    points = cell(1, numImages);
    
    for i = 1:numImages
        [points{i}, features{i}] = extractDescriptors(images{i}, detector);
    end
    
    % Match features between all pairs to build a robust spanning tree
    disp('Matching features across all image pairs to find optimal connections...');
    tforms(numImages) = projective2d(eye(3));
    matchScores = zeros(numImages, numImages);
    matchesI = cell(numImages, numImages);
    matchesJ = cell(numImages, numImages);

    % Compute pairwise matches
    for i = 1:numImages
        for j = i+1:numImages
            [~, ptsI, ptsJ] = matchFeaturesNN(features{i}, features{j}, points{i}, points{j});
            numMatches = size(ptsI, 1);
            matchScores(i, j) = numMatches;
            matchScores(j, i) = numMatches;
            matchesI{i, j} = ptsI;
            matchesJ{i, j} = ptsJ;
            matchesI{j, i} = ptsJ;
            matchesJ{j, i} = ptsI;
        end
    end
    
    % Build Maximum Spanning Tree to connect images resiliently
    added = false(1, numImages);
    validTransformsIdx = true(1, numImages);
    
    [maxVal, linearIdx] = max(matchScores(:));
    if maxVal == 0 || isempty(maxVal)
        warning('No valid matches found across any images!');
        added(1) = true; % Fallback
    else
        [idx1, ~] = ind2sub(size(matchScores), linearIdx);
        added(idx1) = true;
    end
    
    if sum(added) == 0
        added(1) = true;
    end
    
    % Iteratively add the best matching unadded image
    for step = 2:numImages
        bestScore = -1;
        bestAdded = -1;
        bestUnadded = -1;
        
        for i = 1:numImages
            if added(i)
                for j = 1:numImages
                    if ~added(j)
                        if matchScores(i, j) > bestScore
                            bestScore = matchScores(i, j);
                            bestAdded = i;
                            bestUnadded = j;
                        end
                    end
                end
            end
        end
        
        fprintf('Connecting image %d to %d (Score: %d)\n', bestUnadded, bestAdded, bestScore);
        
        ptsAdded = matchesI{bestAdded, bestUnadded};
        ptsUnadded = matchesJ{bestAdded, bestUnadded};
        
        % Estimate homography to the newly selected image
        [tformStep, isValid] = estimateHomography(ptsAdded, ptsUnadded, tforms(bestAdded));
        
        tforms(bestUnadded) = tformStep;
        validTransformsIdx(bestUnadded) = isValid;
        
        if ~isValid
            warning('Failed to estimate reliable transform for image %d. It will be excluded from the final panorama.', bestUnadded);
        end
        
        added(bestUnadded) = true;
    end
    
    % Filter out invalid images to avoid a jumbled output entirely
    validImages = images(validTransformsIdx);
    validTforms = tforms(validTransformsIdx);
    
    if isempty(validImages)
        error('All images failed to align properly.');
    end
    
    % --- 3. Reconstruction Module ---
    disp('Aligning images and generating panorama...');
    [panorama, ~] = alignImages(validImages, validTforms);
    
    % Resize proportionally to prevent aspect ratio distortion (squashing), but cap the max dimension
    disp('Applying sharpening and proportional resizing to preserve natural geometry...');
    [h, w, ~] = size(panorama);
    maxDim = 3840; % Allow up to 4K resolution rather than forcing 1080p
    if max(h, w) > maxDim
        scaleFactor = maxDim / max(h, w);
        panorama = imresize(panorama, scaleFactor, 'lanczos3');
    end
    panorama = imsharpen(panorama, 'Radius', 2, 'Amount', 1.0, 'Threshold', 0.02);
    
    % Save result
    outputFile = fullfile(outputDir, 'stitched_panorama.png');
    imwrite(panorama, outputFile);
    
    pipelineResult = struct('status', 'success', 'outputFile', outputFile, 'numImages', numImages);
    disp(['Pipeline completed successfully. Output saved to ' outputFile]);
end
