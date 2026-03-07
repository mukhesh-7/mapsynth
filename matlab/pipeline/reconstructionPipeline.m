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
    
    % Match features between consecutive image pairs
    tforms(numImages) = projective2d(eye(3));
    
    for i = 2:numImages
        [~, matchedPoints1, matchedPoints2] = matchFeaturesNN(features{i-1}, features{i}, points{i-1}, points{i});
        tforms(i) = estimateHomography(matchedPoints1, matchedPoints2, tforms(i-1));
    end
    
    % --- 3. Reconstruction Module ---
    disp('Aligning images and generating panorama...');
    [panorama, mask] = alignImages(images, tforms);
    
    % Enforce exactly 1920x1080 dimensions and maximize clarity
    disp('Resizing and sharpening panorama to 1920x1080...');
    panorama = imresize(panorama, [1080, 1920], 'lanczos3');
    panorama = imsharpen(panorama, 'Radius', 2, 'Amount', 1.5, 'Threshold', 0.01);
    
    % Save result
    outputFile = fullfile(outputDir, 'stitched_panorama.png');
    imwrite(panorama, outputFile);
    
    pipelineResult = struct('status', 'success', 'outputFile', outputFile, 'numImages', numImages);
    disp(['Pipeline completed successfully. Output saved to ' outputFile]);
end
