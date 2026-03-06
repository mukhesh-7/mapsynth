function seamMask = seamSelection(img1, img2, overlapMask)
% SEAMSELECTION Finds the optimal seam between two overlapping images
%   Uses a minimum-cost path algorithm on the color difference map
%   to determine where to cut between overlapping warped images.
%   This produces seamless transitions in the final panorama.
%
% Inputs:
%    img1, img2    - two warped images on the same canvas (same dimensions)
%    overlapMask   - logical mask indicating the overlap region
% Outputs:
%    seamMask      - logical mask (true = use img1, false = use img2)

    [h, w, ~] = size(img1);
    seamMask = true(h, w);

    if nargin < 3
        % Compute overlap from non-zero pixels
        overlapMask = any(img1, 3) & any(img2, 3);
    end

    if ~any(overlapMask(:))
        % No overlap: use img1 where it has data, img2 elsewhere
        seamMask = any(img1, 3);
        return;
    end

    % --- Compute cost image ---
    % Color difference between images in the overlap region
    diff = single(img1) - single(img2);
    costMap = sqrt(sum(diff.^2, 3));

    % Zero out cost outside overlap
    costMap(~overlapMask) = 0;

    % --- Find the vertical seam using dynamic programming ---
    % Identify overlap bounding box
    [rows, cols] = find(overlapMask);
    rMin = min(rows); rMax = max(rows);
    cMin = min(cols); cMax = max(cols);

    overlapCost = costMap(rMin:rMax, cMin:cMax);
    overlapH = rMax - rMin + 1;
    overlapW = cMax - cMin + 1;

    if overlapW < 2 || overlapH < 2
        return;
    end

    % --- Dynamic programming: minimum cost vertical path ---
    % Accumulate from top to bottom
    cumCost = zeros(overlapH, overlapW, 'single');
    cumCost(1, :) = overlapCost(1, :);
    backPtr = zeros(overlapH, overlapW, 'int32');

    for r = 2:overlapH
        for c = 1:overlapW
            neighbors = c;
            if c > 1
                neighbors = [c-1, neighbors]; %#ok<AGROW>
            end
            if c < overlapW
                neighbors = [neighbors, c+1]; %#ok<AGROW>
            end

            [minVal, minIdx] = min(cumCost(r-1, neighbors));
            cumCost(r, c) = overlapCost(r, c) + minVal;
            backPtr(r, c) = neighbors(minIdx);
        end
    end

    % --- Trace back the minimum-cost path ---
    seamPath = zeros(overlapH, 1, 'int32');
    [~, seamPath(overlapH)] = min(cumCost(overlapH, :));

    for r = overlapH-1 : -1 : 1
        seamPath(r) = backPtr(r+1, seamPath(r+1));
    end

    % --- Build seam mask ---
    % Pixels left of seam: use img1, right of seam: use img2
    for r = 1:overlapH
        globalR = r + rMin - 1;
        seamCol = seamPath(r) + cMin - 1;
        seamMask(globalR, seamCol:end) = false;
    end

    fprintf('Seam selected: %d-pixel path through %d×%d overlap region\n', ...
        overlapH, overlapW, overlapH);
end
