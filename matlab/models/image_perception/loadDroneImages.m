function images = loadDroneImages(directoryPath)
% LOADDRONEIMAGES Recursively retrieves valid image files from a specified raw image directory
% Inputs:
%    directoryPath - string path to the storage folder housing raw drone frames
% Outputs:
%    images - structural cell array of uncompressed pixel matrices

    fprintf('Looking for drone artifacts in %s...\n', directoryPath);
    
    filePattern = fullfile(directoryPath, '*.jpg'); % Alternatively 'png', 'tif' etc
    imgFiles = dir(filePattern);
    
    numFiles = length(imgFiles);
    images = cell(1, numFiles);
    
    for i = 1:numFiles
        fileName = fullfile(directoryPath, imgFiles(i).name);
        try
            images{i} = imread(fileName);
        catch e
            warning('Failed to load %s: %s', fileName, e.message);
        end
    end
end
