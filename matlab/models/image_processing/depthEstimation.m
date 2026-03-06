function depthMap = depthEstimation(panorama)
% DEPTHESTIMATION Estimates relative depth from the stitched panorama
%   Uses a texture and gradient-based approach to generate a 2.5D pseudo-depth 
%   map representing the topographic variations of the full reconstructed map.
%   Structures like buildings, trees, and cars naturally yield higher depth
%   values due to their local edge contrast and variance.
%
% Inputs:
%    panorama - the final stitched RGB panorama image
% Outputs:
%    depthMap - single-channel float matrix representing relative depth

    fprintf('Estimating 2.5D topographic depth map from panorama...\n');
    
    [h, w, c] = size(panorama);
    
    % Step 1: Convert to grayscale and normalize
    if c == 3
        grayImg = single(rgb2gray(panorama)) / 255;
    else
        grayImg = single(panorama) / 255;
    end
    
    % Ignore empty (black) regions from the stitched canvas
    validMask = grayImg > 0.01;
    
    % Step 2: Compute local image gradient (edges)
    % We use gradient magnitude as a strong proxy for structural height
    % because man-made objects and trees are highly textured/edged.
    [Gx, Gy] = imgradientxy(grayImg);
    gradMag = hypot(Gx, Gy);
    
    % Step 3: Apply heavy Gaussian smoothing to create "elevation hills"
    % A large sigma turns sharp edges into smooth topographic bumps
    sigma = min(h, w) * 0.01; % Dynamic smoothing based on panorama size (e.g. 1% of width)
    sigma = max(sigma, 10);   % Minimum of 10
    
    depthMap = imgaussfilt(gradMag, sigma);
    
    % Step 4: Normalize the valid regions to a gentle [0.4, 0.6] range
    % This ensures the base ground isn't at 0, and the structures pop up mildly
    if any(validMask(:))
        minVal = min(depthMap(validMask));
        maxVal = max(depthMap(validMask));
        
        if maxVal > minVal
            depthMap(validMask) = (depthMap(validMask) - minVal) ./ (maxVal - minVal);
        else
            depthMap(validMask) = 0;
        end
    end
    
    % Mask out the black empty canvas background entirely
    depthMap(~validMask) = 0;
    
    fprintf('Topographic depth map generated. Size: %d × %d\n', w, h);
end
