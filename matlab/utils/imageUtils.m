% IMAGEUTILS Collection of shared image I/O and validation helpers
%   Static utility functions used across the drone reconstruction pipeline.
%   Call as: imageUtils.functionName(args)

classdef imageUtils

    methods (Static)

        function isValid = validateImageFile(filePath)
        % VALIDATEIMAGEFILE Checks if a file is a readable image
            isValid = false;
            if ~isfile(filePath)
                return;
            end
            try
                info = imfinfo(filePath);
                isValid = ~isempty(info);
            catch
                isValid = false;
            end
        end


        function images = loadMultiFormat(directoryPath)
        % LOADMULTIFORMAT Loads images from directory supporting jpg, png, tif, bmp
            extensions = {'*.jpg', '*.jpeg', '*.png', '*.tif', '*.tiff', '*.bmp'};
            allFiles = [];
            for e = 1:numel(extensions)
                found = dir(fullfile(directoryPath, extensions{e}));
                allFiles = [allFiles; found]; %#ok<AGROW>
            end

            numFiles = numel(allFiles);
            images = cell(1, numFiles);
            loaded = 0;

            for i = 1:numFiles
                fp = fullfile(directoryPath, allFiles(i).name);
                try
                    images{i} = imread(fp);
                    loaded = loaded + 1;
                catch ex
                    warning('imageUtils:loadFail', ...
                        'Failed to load %s: %s', fp, ex.message);
                    images{i} = [];
                end
            end

            % Purge empties
            images = images(~cellfun(@isempty, images));
            fprintf('imageUtils: loaded %d / %d image files.\n', loaded, numFiles);
        end


        function img = ensureRGB(img)
        % ENSURERGB Converts grayscale to 3-channel if needed
            if ndims(img) == 2 || size(img, 3) == 1
                img = repmat(img, [1 1 3]);
            end
        end


        function gray = toGray(img)
        % TOGRAY Converts to grayscale safely
            if size(img, 3) == 3
                gray = rgb2gray(img);
            else
                gray = img;
            end
        end


        function img = resizeToMax(img, maxDim)
        % RESIZETOMAX Resizes image so the largest dimension equals maxDim
        %   Preserves aspect ratio.
            [h, w, ~] = size(img);
            scaleFactor = maxDim / max(h, w);
            if scaleFactor < 1
                img = imresize(img, scaleFactor);
            end
        end


        function saveWithMetadata(img, outputPath, metadata)
        % SAVEWITHMETADATA Writes an image and an accompanying .json metadata file
            imwrite(img, outputPath);
            
            % Write metadata sidecar
            [folder, name, ~] = fileparts(outputPath);
            jsonPath = fullfile(folder, [name '_meta.json']);
            
            metadata.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
            metadata.resolution = [size(img, 2), size(img, 1)];
            metadata.channels = size(img, 3);
            
            fid = fopen(jsonPath, 'w');
            if fid ~= -1
                fwrite(fid, jsonencode(metadata));
                fclose(fid);
            end
        end


        function [h, w, c] = dimensions(img)
        % DIMENSIONS Returns height, width, channels of an image
            h = size(img, 1);
            w = size(img, 2);
            if ndims(img) >= 3
                c = size(img, 3);
            else
                c = 1;
            end
        end

    end
end
