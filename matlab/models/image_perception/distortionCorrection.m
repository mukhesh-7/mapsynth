function correctedImages = distortionCorrection(images)
% DISTORTIONCORRECTION Compensates for radial and tangential lens distortions
%   Applies camera calibration-based undistortion when calibration parameters
%   are available, otherwise applies an adaptive parametric correction model.
%
% Inputs:
%    images - cell array of preprocessed drone image matrices
% Outputs:
%    correctedImages - cell array of undistorted image matrices

    numImages = numel(images);
    correctedImages = cell(1, numImages);

    fprintf('Applying distortion correction to %d images...\n', numImages);

    % --- Attempt to load camera calibration parameters ---
    % Look for calibration file in the project root
    calibFile = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), ...
                         'calibration', 'cameraParams.mat');

    useCalibration = false;
    if isfile(calibFile)
        loaded = load(calibFile, 'cameraParams');
        if isfield(loaded, 'cameraParams')
            cameraParams = loaded.cameraParams;
            useCalibration = true;
            fprintf('Using stored camera calibration parameters.\n');
        end
    end

    for i = 1:numImages
        img = images{i};

        if isempty(img)
            correctedImages{i} = [];
            continue;
        end

        if useCalibration
            % --- Calibrated Undistortion ---
            % Use Computer Vision Toolbox undistortImage with known intrinsics
            correctedImages{i} = undistortImage(img, cameraParams);
        else
            % --- Adaptive Parametric Correction ---
            % When no calibration exists, apply a mild radial correction
            % using a barrel/pincushion compensation model.
            % This works for typical drone cameras with moderate distortion.
            correctedImages{i} = applyRadialCorrection(img);
        end
    end

    % Remove empty entries
    emptyMask = cellfun(@isempty, correctedImages);
    if any(emptyMask)
        correctedImages = correctedImages(~emptyMask);
    end

    fprintf('Distortion correction complete.\n');
end


function corrected = applyRadialCorrection(img)
% APPLYRADIALCORRECTION Performs mild barrel distortion correction
%   Uses a polynomial radial distortion model centered on the image.
%   Suitable for typical drone cameras (DJI Mavic, Phantom, etc.)
%   where exact calibration data is unavailable.

    [height, width, channels] = size(img);

    % Normalized coordinate grid centered at image center
    [X, Y] = meshgrid(1:width, 1:height);
    cx = width / 2;
    cy = height / 2;

    % Normalize to [-1, 1] range
    Xn = (X - cx) / cx;
    Yn = (Y - cy) / cy;

    % Radial distance from center
    R = sqrt(Xn.^2 + Yn.^2);

    % Barrel distortion correction coefficients
    % k1 < 0 corrects barrel distortion (typical for wide-angle drone cameras)
    % k2 provides fine-tuning for higher-order distortion
    k1 = -0.10;  
    k2 =  0.02;  

    % Distortion correction: r_corrected = r * (1 + k1*r^2 + k2*r^4)
    correctionFactor = 1 + k1 * R.^2 + k2 * R.^4;

    % Compute corrected pixel coordinates
    Xc = Xn .* correctionFactor * cx + cx;
    Yc = Yn .* correctionFactor * cy + cy;

    % Interpolate corrected image using bilinear interpolation
    corrected = zeros(size(img), 'like', img);
    for c = 1:channels
        corrected(:,:,c) = interp2(double(img(:,:,c)), Xc, Yc, 'linear', 0);
    end

    corrected = cast(corrected, class(img));
end
