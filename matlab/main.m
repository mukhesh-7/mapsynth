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
    config = defaultConfig(matlabDir);

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
    outputDirs = {config.outputDir, config.depthDir, config.logDir, config.processedDir};
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
        tui.phase('3/5', 'HOMOGRAPHY ESTIMATION & CLUSTERING');

        t3 = tic;

        % --- Step 3a: Build match graph (sequential + global fallback) ---
        % We store edge lists for graph construction.
        srcNodes = [];
        dstNodes = [];
        edgeCounts = [];  % number of matches per edge (for logging)

        for i = 2:numImages
            tui.log('MATCH', sprintf('Pair %d→%d — sequential matching...', i-1, i));
            [indexPairs, ~, ~] = matchFeaturesNN( ...
                features{i-1}, features{i}, points{i-1}, points{i});
            n = size(indexPairs, 1);
            tui.log('MATCH', sprintf('  → %d correspondences found', n));

            if n >= 4
                srcNodes(end+1) = i-1;  %#ok<AGROW>
                dstNodes(end+1) = i;    %#ok<AGROW>
                edgeCounts(end+1) = n;  %#ok<AGROW>
            else
                % --- Fallback: global search against all earlier images ---
                tui.log('WARN', sprintf('  ⚠ Too few matches (%d < 4). Running global fallback search...', n));
                bestN = 0;
                bestJ = -1;
                for j = 1:i-2
                    [ip2, ~, ~] = matchFeaturesNN( ...
                        features{j}, features{i}, points{j}, points{i});
                    nj = size(ip2, 1);
                    if nj > bestN
                        bestN = nj;
                        bestJ = j;
                    end
                end
                if bestN >= 4
                    tui.log('MATCH', sprintf('  ✔ Fallback: image %d connects to image %d (%d matches)', i, bestJ, bestN));
                    srcNodes(end+1) = bestJ;   %#ok<AGROW>
                    dstNodes(end+1) = i;       %#ok<AGROW>
                    edgeCounts(end+1) = bestN; %#ok<AGROW>
                else
                    tui.log('WARN', sprintf('  ✘ No connection found for image %d across all previous images.', i));
                end
            end
        end

        % --- Step 3b: Find connected components (clusters) ---
        if isempty(srcNodes)
            % No valid matches at all — treat every image as its own cluster
            tui.log('WARN', 'No valid matches found across any image pair. Each image is its own cluster.');
            clusterBins = 1:numImages;
        else
            G = graph(srcNodes, dstNodes, edgeCounts, numImages);
            clusterBins = conncomp(G);  % clusterBins(i) = cluster index for image i
        end

        numClusters = max(clusterBins);
        tui.log('CLUSTER', sprintf('Identified %d image cluster(s):', numClusters));
        for k = 1:numClusters
            clusterIdx = find(clusterBins == k);
            tui.log('CLUSTER', sprintf('  Cluster %d: images [%s]', k, num2str(clusterIdx)));
        end

        tui.timing('Homography Estimation & Clustering', toc(t3));

        % ------ PHASE 4 & 5 : RECONSTRUCTION + DEPTH (per cluster) ------
        tui.phase('4/5', 'IMAGE RECONSTRUCTION (per cluster)');
        tui.phase('5/5', 'DEPTH ESTIMATION & VISUALIZATION (per cluster)');

        t45 = tic;
        timestampStr = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
        panoramaFiles = {};  % collect output paths for summary
        depthFiles    = {};
        totalEdgeMatches = sum(edgeCounts);

        for k = 1:numClusters
            clusterIdx = find(clusterBins == k);

            if numel(clusterIdx) < 2
                tui.log('SKIP', sprintf('Cluster %d has only 1 image — skipping reconstruction.', k));
                continue;
            end

            tui.separator();
            tui.log('CLUSTER', sprintf('Processing Cluster %d/%d  (%d images: [%s])', ...
                k, numClusters, numel(clusterIdx), num2str(clusterIdx)));

            % Extract images/points/features for this cluster
            clusterImages   = images(clusterIdx);
            clusterPoints   = points(clusterIdx);
            clusterFeatures = features(clusterIdx);
            nCluster        = numel(clusterIdx);

            % --- Compute homographies relative to first image of the cluster ---
            tui.log('HOMOGRAPHY', sprintf('Computing homographies for cluster %d...', k));
            % Pre-fill the entire array with identity transforms so every
            % element is defined even if estimation falls back.
            clusterTforms = repmat(projective2d(eye(3)), 1, nCluster);
            for ci = 2:nCluster
                tui.log('MATCH', sprintf('  Cluster %d — Pair %d→%d...', k, ci-1, ci));
                [~, mp1c, mp2c] = matchFeaturesNN( ...
                    clusterFeatures{ci-1}, clusterFeatures{ci}, ...
                    clusterPoints{ci-1},   clusterPoints{ci});
                if size(mp1c.Location, 1) >= 4
                    clusterTforms(ci) = estimateHomography(mp1c, mp2c, clusterTforms(ci-1));
                else
                    tui.log('WARN', sprintf('    Too few matches for pair %d→%d inside cluster %d; carrying forward.', ci-1, ci, k));
                    clusterTforms(ci) = clusterTforms(ci-1);
                end
            end

            % --- Reconstruction (Phase 4) ---
            tui.log('ALIGN', sprintf('  Building panorama for cluster %d...', k));
            if strcmpi(config.blendMethod, 'multiband')
                panorama = generatePanorama(clusterImages, clusterTforms, 'multiband');
            else
                [panorama, mask] = alignImages(clusterImages, clusterTforms);
                panorama = multiBandBlend(panorama, mask);
            end

            panoramaFile = fullfile(config.outputDir, ...
                sprintf('panorama_cluster_%d_%s.png', k, timestampStr));
            imwrite(panorama, panoramaFile);
            panoramaFiles{end+1} = panoramaFile; %#ok<AGROW>
            tui.log('SAVE', sprintf('  Panorama saved: %s', panoramaFile));
            
            processedFile = fullfile(config.processedDir, ...
                sprintf('final_model_cluster_%d_%s.png', k, timestampStr));
            imwrite(panorama, processedFile);
            tui.log('SAVE', sprintf('  Final model saved to processed_images: %s', processedFile));

            tui.log('SAVE', sprintf('  Resolution: %d × %d px', size(panorama, 2), size(panorama, 1)));

            % --- Visualization ---
            tui.log('VIZ', sprintf('  Displaying cluster %d panorama...', k));
            showPanorama(panorama);

            % Show feature matches for first pair in this cluster
            if nCluster >= 2
                [~, mp1v, mp2v] = matchFeaturesNN( ...
                    clusterFeatures{1}, clusterFeatures{2}, ...
                    clusterPoints{1},   clusterPoints{2});
                showFeatureMatches(clusterImages{1}, clusterImages{2}, mp1v, mp2v, ...
                    sprintf('Feature Matches — Cluster %d (Image %d ↔ %d)', ...
                            k, clusterIdx(1), clusterIdx(2)));
            end

            % --- Depth Estimation (Phase 5) ---
            if config.enableDepth
                tui.log('DEPTH', sprintf('  Computing depth map for cluster %d...', k));
                depthMap = depthEstimation(panorama);

                depthFile = fullfile(config.depthDir, ...
                    sprintf('depth_cluster_%d_%s.png', k, timestampStr));
                imwrite(depthMap, depthFile);
                depthFiles{end+1} = depthFile; %#ok<AGROW>
                tui.log('SAVE', sprintf('  Depth map saved: %s', depthFile));

                showDepthMap(depthMap);
                showPointCloud(depthMap, panorama, ...
                    sprintf('3D Point Cloud — Cluster %d Reconstruction', k));

                clear depthMap;
            else
                tui.log('DEPTH', sprintf('  Depth estimation disabled (cluster %d skipped).', k));
            end

            % Free large cluster arrays to conserve RAM before the next iteration
            clear panorama mask clusterImages clusterPoints clusterFeatures clusterTforms;
        end

        tui.timing('Reconstruction & Depth (all clusters)', toc(t45));

        % =====================================================================
        %  SUMMARY
        % =====================================================================
        totalTime = toc(totalTimer);
        tui.separator();
        tui.log('DONE', '═══ PIPELINE COMPLETE ═══');
        tui.log('STATS', sprintf('Images processed:    %d', numImages));
        tui.log('STATS', sprintf('Clusters found:      %d', numClusters));
        tui.log('STATS', sprintf('Total edge matches:  %d', totalEdgeMatches));
        tui.log('STATS', sprintf('Total time:          %.2f seconds', totalTime));
        for pf = 1:numel(panoramaFiles)
            tui.log('OUTPUT', sprintf('Panorama %d: %s', pf, panoramaFiles{pf}));
        end
        for df = 1:numel(depthFiles)
            tui.log('OUTPUT', sprintf('Depth    %d: %s', df, depthFiles{df}));
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
function config = defaultConfig(matlabDir)
    config.detector     = 'AKAZE';      % Feature detector: AKAZE | SURF | KAZE | FAST | HARRIS | ORB
    config.blendMethod  = 'multiband';  % Blend method: multiband | simple
    config.enableDepth  = true;         % Enable depth estimation
    config.maxImages    = 0;            % Max images to process (0 = all)
    config.maxWorkers   = 4;            % Max parallel workers (reserved for future parfor)
    
    % Ensure files save explicitly inside matlab/storage/
    baseStorageDir      = fullfile(matlabDir, 'storage');
    config.outputDir    = fullfile(baseStorageDir, 'panoramas');
    config.depthDir     = fullfile(baseStorageDir, 'depth_maps');
    config.logDir       = fullfile(baseStorageDir, 'logs');
    config.processedDir = fullfile(baseStorageDir, 'processed_images');
end
