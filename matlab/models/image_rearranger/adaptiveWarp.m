function warpedImg = adaptiveWarp(img, tform, outputView)
% ADAPTIVEWARP Content-aware image warping with mesh deformation
%   Applies a projective transformation with adaptive mesh subdivision
%   to reduce distortion artifacts in regions far from the projection center.
%
%   Uses a grid-based approach that subdivides the image into patches
%   and warps each independently for better local accuracy.
%
% Inputs:
%    img        - source image (RGB or grayscale)
%    tform      - projective2d transformation object
%    outputView - imref2d reference object for the output canvas
% Outputs:
%    warpedImg  - warped image on the output canvas

    if nargin < 3
        % Default: compute output view from transform limits
        [h, w, ~] = size(img);
        [xLimits, yLimits] = outputLimits(tform, [1 w], [1 h]);
        xMin = floor(min(1, xLimits(1)));
        xMax = ceil(max(w, xLimits(2)));
        yMin = floor(min(1, yLimits(1)));
        yMax = ceil(max(h, yLimits(2)));
        outputView = imref2d([yMax-yMin+1, xMax-xMin+1], ...
                             [xMin xMax], [yMin yMax]);
    end

    [srcH, srcW, numChannels] = size(img);

    % --- Mesh Subdivision ---
    % Divide source image into a grid of patches
    gridRows = 8;
    gridCols = 8;

    patchH = ceil(srcH / gridRows);
    patchW = ceil(srcW / gridCols);

    % Pre-allocate output
    warpedImg = zeros(outputView.ImageSize(1), outputView.ImageSize(2), numChannels, 'like', img);
    weightMap = zeros(outputView.ImageSize(1), outputView.ImageSize(2), 'single');

    for r = 1:gridRows
        for c = 1:gridCols
            % Compute patch boundaries
            y1 = (r-1) * patchH + 1;
            y2 = min(r * patchH, srcH);
            x1 = (c-1) * patchW + 1;
            x2 = min(c * patchW, srcW);

            % Extract patch with 2-pixel overlap for seam blending
            py1 = max(1, y1 - 2);
            py2 = min(srcH, y2 + 2);
            px1 = max(1, x1 - 2);
            px2 = min(srcW, x2 + 2);

            patch = img(py1:py2, px1:px2, :);

            % Compute local transform (adjust for patch offset)
            offsetTform = computeLocalTransform(tform, px1, py1);

            % Warp the patch
            warpedPatch = imwarp(patch, offsetTform, 'OutputView', outputView);
            patchMask = imwarp(ones(size(patch,1), size(patch,2), 'single'), ...
                               offsetTform, 'OutputView', outputView);

            % Accumulate with alpha blending
            for ch = 1:numChannels
                warpedImg(:,:,ch) = warpedImg(:,:,ch) + ...
                    cast(single(warpedPatch(:,:,ch)) .* patchMask, class(img));
            end
            weightMap = weightMap + patchMask;
        end
    end

    % Normalize by accumulated weights
    weightMap(weightMap == 0) = 1;
    for ch = 1:numChannels
        warpedImg(:,:,ch) = cast(single(warpedImg(:,:,ch)) ./ weightMap, class(img));
    end

    fprintf('Adaptive warp complete: %d×%d grid, output %d×%d\n', ...
        gridCols, gridRows, outputView.ImageSize(2), outputView.ImageSize(1));
end


function localTform = computeLocalTransform(globalTform, offsetX, offsetY)
% COMPUTELOCALTRANSFORM Adjusts global transform for a patch offset
    T_offset = [1 0 0; 0 1 0; offsetX-1, offsetY-1, 1];
    localTform = projective2d(T_offset * globalTform.T);
end
