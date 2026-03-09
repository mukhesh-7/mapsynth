function [panoramaFiles, depthFiles, totalEdgeMatches] = runReconstruction(images, points, features, matchScores, matchesI, matchesJ, clusterBins, numClusters, edgeCounts, config, tui)
% RUNRECONSTRUCTION Executes Phase 4 (Reconstruction) and Phase 5 (Depth Estimation) per cluster
    tui.phase('4/5', 'IMAGE RECONSTRUCTION (per cluster)');
    tui.phase('5/5', 'DEPTH ESTIMATION & VISUALIZATION (per cluster)');

    t45 = tic;
    timestampStr = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    panoramaFiles = {};  
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

        clusterImages   = images(clusterIdx);
        clusterPoints   = points(clusterIdx);
        clusterFeatures = features(clusterIdx);
        nCluster        = numel(clusterIdx);

        tui.log('HOMOGRAPHY', sprintf('Computing homographies for cluster %d...', k));
        clusterTforms = repmat(projective2d(eye(3)), 1, nCluster);
        added = false(1, nCluster);
        added(1) = true; 
        validTransformsIdx = true(1, nCluster);
        
        for step = 2:nCluster
            bestScore = -1;
            bestUnadded = -1;
            
            for c_u = 1:nCluster
                if ~added(c_u)
                    score = 0;
                    global_u = clusterIdx(c_u);
                    for c_a = 1:nCluster
                        if added(c_a)
                            global_a = clusterIdx(c_a);
                            score = score + matchScores(global_a, global_u);
                        end
                    end
                    if score > bestScore
                        bestScore = score;
                        bestUnadded = c_u;
                    end
                end
            end
            
            tui.log('MATCH', sprintf('  Cluster %d — Adding Image %d to panorama (Global Link Score: %d)', ...
                k, clusterIdx(bestUnadded), bestScore));
                
            allGlobalPts = [];
            allUnaddedPts = [];
            
            global_u = clusterIdx(bestUnadded);
            for c_a = 1:nCluster
                if added(c_a)
                    global_a = clusterIdx(c_a);
                    if matchScores(global_a, global_u) >= 4
                        ptsA = matchesI{global_a, global_u};
                        ptsU = matchesJ{global_a, global_u};
                        
                        [xG, yG] = transformPointsForward(clusterTforms(c_a), ...
                            double(ptsA.Location(:,1)), double(ptsA.Location(:,2)));
                            
                        allGlobalPts = [allGlobalPts; double(xG), double(yG)]; %#ok<AGROW>
                        allUnaddedPts = [allUnaddedPts; double(ptsU.Location)]; %#ok<AGROW>
                    end
                end
            end
            
            [tformStep, isValid] = estimateHomography(allGlobalPts, allUnaddedPts, projective2d(eye(3)));
            clusterTforms(bestUnadded) = tformStep;
            validTransformsIdx(bestUnadded) = isValid;
            
            if ~isValid
                tui.log('WARN', sprintf('    Failed to reliably orient image %d. Excluding.', clusterIdx(bestUnadded)));
            end
            added(bestUnadded) = true;
        end
        
        clusterImages = clusterImages(validTransformsIdx);
        clusterTforms = clusterTforms(validTransformsIdx);
        nCluster = numel(clusterImages);
        
        if nCluster < 2
            tui.log('WARN', sprintf('Cluster %d has < 2 valid aligned images after estimation. Skipping.', k));
            continue;
        end

        % --- Reconstruction ---
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

        tui.log('VIZ', sprintf('  Displaying cluster %d panorama...', k));
        showPanorama(panorama);

        if nCluster >= 2
            [~, mp1v, mp2v] = matchFeaturesNN( ...
                clusterFeatures{1}, clusterFeatures{2}, ...
                clusterPoints{1},   clusterPoints{2});
            showFeatureMatches(clusterImages{1}, clusterImages{2}, mp1v, mp2v, ...
                sprintf('Feature Matches — Cluster %d (Image %d ↔ %d)', ...
                        k, clusterIdx(1), clusterIdx(2)));
        end

        % --- Depth Estimation ---
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

        clear panorama mask clusterImages clusterPoints clusterFeatures clusterTforms;
    end

    tui.timing('Reconstruction & Depth (all clusters)', toc(t45));
end
