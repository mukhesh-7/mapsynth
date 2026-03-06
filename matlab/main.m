function main(varargin)
% MAIN Entry point for the Drone Image Reconstruction System
%   Runs the entire pipeline from a single MATLAB command.
%   All output is displayed in a dedicated TUI log window and
%   results are shown in separate figure windows.
%
% Usage:
%   main()                          — Interactive folder picker
%   main('path/to/images')          — Direct path
%   main('path/to/images', config)  — Custom config struct
%
% Example:
%   main('C:\drone_captures\flight_001')
%   main('storage/raw_images', struct('detector','SURF','blend','multiband'))

    % =====================================================================
    %  BOOTSTRAP — Resolve paths, add all modules to path
    % =====================================================================
    matlabDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(matlabDir);           % one level up from matlab/
    addpath(genpath(matlabDir));

    % =====================================================================
    %  CONFIGURATION — Defaults + user overrides
    % =====================================================================
    config = defaultConfig(projectRoot);

    if nargin >= 2 && isstruct(varargin{2})
        userCfg = varargin{2};
        fnames = fieldnames(userCfg);
        for k = 1:numel(fnames)
            config.(fnames{k}) = userCfg.(fnames{k});
        end
    end

    % =====================================================================
    %  TUI — Launch Log Window
    % =====================================================================
    tui = TUILogger('Drone Reconstruction Engine');

    tui.header();
    tui.log('SYSTEM', 'Pipeline engine initialized');
    tui.log('SYSTEM', sprintf('MATLAB %s  |  %s', version, computer));
    maxImgStr = 'all';
    if config.maxImages > 0
        maxImgStr = num2str(config.maxImages);
    end
    tui.log('CONFIG', sprintf('Detector: %s  |  Blend: %s  |  Max Images: %s', ...
        config.detector, config.blendMethod, maxImgStr));
    tui.separator();

    % =====================================================================
    %  INPUT — Resolve image directory
    % =====================================================================
    if nargin >= 1 && ~isempty(varargin{1})
        imageDir = varargin{1};
        if ~isfolder(imageDir)
            % Try relative to project root
            imageDir = fullfile(projectRoot, varargin{1});
        end
    else
        tui.log('INPUT', 'No path provided — opening folder picker...');
        imageDir = uigetdir(fullfile(projectRoot, 'storage', 'raw_images'), ...
                            'Select Drone Image Directory');
        if imageDir == 0
            tui.log('ERROR', 'No folder selected. Aborting.');
            return;
        end
    end

    if ~isfolder(imageDir)
        tui.log('ERROR', sprintf('Directory not found: %s', imageDir));
        return;
    end
    tui.log('INPUT', sprintf('Source: %s', imageDir));

    % Ensure output directories exist
    outputDirs = {config.outputDir, config.depthDir, config.logDir};
    for d = 1:numel(outputDirs)
        if ~isfolder(outputDirs{d})
            mkdir(outputDirs{d});
        end
    end

    % =====================================================================
    %  PIPELINE EXECUTION
    % =====================================================================
    totalTimer = tic;

    try
        % ------ PHASE 1 : PERCEPTION ------
        tui.phase('1/5', 'IMAGE PERCEPTION');

        t1 = tic;
        tui.log('LOAD', 'Scanning for drone images...');
        images = loadDroneImages(imageDir);

        if isempty(images) || all(cellfun(@isempty, images))
            tui.log('ERROR', 'No valid images found in directory.');
            tui.log('HINT', 'Supported formats: .jpg .jpeg .png .tif .tiff .bmp');
            return;
        end

        numImages = numel(images);
        tui.log('LOAD', sprintf('Found %d images', numImages));

        % Apply image cap if configured
        if config.maxImages > 0 && numImages > config.maxImages
            tui.log('LOAD', sprintf('Limiting to %d images (config.maxImages)', config.maxImages));
            images = images(1:config.maxImages);
            numImages = config.maxImages;
        end

        if numImages < 2
            tui.log('ERROR', 'Need at least 2 images for reconstruction.');
            return;
        end

        tui.log('PREPROCESS', 'Normalizing resolution, contrast, and noise...');
        images = preprocessImages(images);

        tui.log('DISTORTION', 'Applying lens distortion correction...');
        images = distortionCorrection(images);

        tui.timing('Perception', toc(t1));

        % ------ PHASE 2 : FEATURE EXTRACTION ------
        tui.phase('2/5', 'FEATURE EXTRACTION');

        t2 = tic;
        features = cell(1, numImages);
        points   = cell(1, numImages);

        for i = 1:numImages
            tui.log('FEATURES', sprintf('Image %d/%d — extracting %s descriptors...', ...
                i, numImages, config.detector));
            [points{i}, features{i}] = extractDescriptors(images{i}, config.detector);
            tui.log('FEATURES', sprintf('  -> %d keypoints detected', points{i}.Count));
        end

        tui.timing('Feature Extraction', toc(t2));

        % ------ PHASE 3 : GEOMETRIC ESTIMATION ------
        tui.phase('3/5', 'HOMOGRAPHY ESTIMATION');

        t3 = tic;
        tforms(numImages) = projective2d(eye(3));
        matchCounts = zeros(1, numImages);

        for i = 2:numImages
            tui.log('MATCH', sprintf('Pair %d→%d — nearest-neighbor matching...', i-1, i));
            [indexPairs, matchedPts1, matchedPts2] = matchFeaturesNN( ...
                features{i-1}, features{i}, points{i-1}, points{i});
            matchCounts(i) = size(indexPairs, 1);
            tui.log('MATCH', sprintf('  → %d correspondences found', matchCounts(i)));

            if matchCounts(i) < 4
                tui.log('WARN', sprintf('  ⚠ Too few matches (%d < 4). Skipping pair.', matchCounts(i)));
                tforms(i) = tforms(i-1);
                continue;
            end

            tui.log('HOMOGRAPHY', sprintf('Pair %d→%d — MSAC estimation...', i-1, i));
            tforms(i) = estimateHomography(matchedPts1, matchedPts2, tforms(i-1));
        end

        tui.timing('Homography Estimation', toc(t3));

        % ------ PHASE 4 : RECONSTRUCTION ------
        tui.phase('4/5', 'IMAGE RECONSTRUCTION');

        t4 = tic;
        tui.log('ALIGN', 'Warping images onto global canvas...');
        [panorama, mask] = alignImages(images, tforms);

        tui.log('BLEND', sprintf('Blending with method: %s', config.blendMethod));
        if strcmpi(config.blendMethod, 'multiband')
            panorama = generatePanorama(images, tforms, 'multiband');
        else
            panorama = multiBandBlend(panorama, mask);
        end

        % Save panorama
        panoramaFile = fullfile(config.outputDir, ...
            sprintf('panorama_%s.png', string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))));
        imwrite(panorama, panoramaFile);
        tui.log('SAVE', sprintf('Panorama saved: %s', panoramaFile));
        tui.log('SAVE', sprintf('Resolution: %d × %d px', size(panorama, 2), size(panorama, 1)));

        tui.timing('Reconstruction', toc(t4));

        % ------ PHASE 5 : DEPTH & VISUALIZATION ------
        tui.phase('5/5', 'DEPTH ESTIMATION & VISUALIZATION');

        t5 = tic;
        if config.enableDepth
            tui.log('DEPTH', 'Computing multi-view depth map...');
            depthMap = depthEstimation(images, tforms);

            depthFile = fullfile(config.depthDir, ...
                sprintf('depth_%s.png', string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))));
            imwrite(depthMap, depthFile);
            tui.log('SAVE', sprintf('Depth map saved: %s', depthFile));
        else
            depthMap = [];
            tui.log('DEPTH', 'Depth estimation disabled (config.enableDepth = false)');
        end

        % --- Visualization Windows ---
        tui.log('VIZ', 'Opening result windows...');
        showPanorama(panorama);

        if config.enableDepth && ~isempty(depthMap)
            showDepthMap(depthMap);
            showPointCloud(depthMap, images{1});
        end

        % Show feature matches for first pair
        if numImages >= 2
            [~, mp1, mp2] = matchFeaturesNN(features{1}, features{2}, points{1}, points{2});
            showFeatureMatches(images{1}, images{2}, mp1, mp2, ...
                'Feature Matches — Image 1 ↔ 2');
        end

        tui.timing('Depth & Visualization', toc(t5));

        % =====================================================================
        %  SUMMARY
        % =====================================================================
        totalTime = toc(totalTimer);
        tui.separator();
        tui.log('DONE', '═══ PIPELINE COMPLETE ═══');
        tui.log('STATS', sprintf('Images processed:    %d', numImages));
        tui.log('STATS', sprintf('Total matches:       %d', sum(matchCounts)));
        tui.log('STATS', sprintf('Panorama resolution: %d × %d', size(panorama, 2), size(panorama, 1)));
        tui.log('STATS', sprintf('Total time:          %.2f seconds', totalTime));
        tui.log('OUTPUT', sprintf('Panorama: %s', panoramaFile));
        if config.enableDepth && ~isempty(depthMap)
            tui.log('OUTPUT', sprintf('Depth:    %s', depthFile));
        end
        tui.separator();

        % Save run log
        logFile = fullfile(config.logDir, ...
            sprintf('run_%s.log', string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))));
        tui.saveLog(logFile);
        tui.log('LOG', sprintf('Full log saved: %s', logFile));

    catch ex
        tui.log('FATAL', sprintf('Pipeline crashed: %s', ex.message));
        tui.log('FATAL', sprintf('In: %s (line %d)', ex.stack(1).name, ex.stack(1).line));
        for s = 1:min(5, numel(ex.stack))
            tui.log('TRACE', sprintf('  %s:%d', ex.stack(s).name, ex.stack(s).line));
        end
        tui.separator();
    end
end


% =========================================================================
%  DEFAULT CONFIGURATION
% =========================================================================
function config = defaultConfig(projectRoot)
    config.detector     = 'AKAZE';      % Feature detector: AKAZE | SURF | KAZE | FAST | HARRIS | ORB
    config.blendMethod  = 'multiband';  % Blend method: multiband | simple
    config.enableDepth  = true;         % Enable depth estimation
    config.maxImages    = 0;            % Max images to process (0 = all). Set to 15-30 for large datasets
    config.maxWorkers   = 4;            % Max parallel workers (reserved for future parfor)
    config.outputDir    = fullfile(projectRoot, 'storage', 'panoramas');
    config.depthDir     = fullfile(projectRoot, 'storage', 'depth_maps');
    config.logDir       = fullfile(projectRoot, 'storage', 'logs');
end
