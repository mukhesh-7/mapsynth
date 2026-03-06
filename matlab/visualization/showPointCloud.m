function showPointCloud(depthMap, referenceImage, titleText)
% SHOWPOINTCLOUD Visualizes a 3D point cloud from a depth map
%   Projects the depth map into 3D space using the reference image
%   for coloring, creating an interactive 3D visualization.
%
% Inputs:
%    depthMap       - 2D float matrix (relative depth values)
%    referenceImage - RGB image for point coloring
%    titleText      - (optional) figure title

    if nargin < 3
        titleText = '3D Point Cloud — Drone Reconstruction';
    end

    [h, w] = size(depthMap);

    % --- Step 1: Generate 3D coordinate grid ---
    [X, Y] = meshgrid(1:w, 1:h);
    Z = depthMap * 100; % Scale depth for visualization

    % Downsample for performance (keep every Nth point)
    downsampleFactor = max(1, floor(sqrt(h * w / 500000)));
    if downsampleFactor > 1
        X = X(1:downsampleFactor:end, 1:downsampleFactor:end);
        Y = Y(1:downsampleFactor:end, 1:downsampleFactor:end);
        Z = Z(1:downsampleFactor:end, 1:downsampleFactor:end);
        if nargin >= 2 && ~isempty(referenceImage)
            referenceImage = referenceImage(1:downsampleFactor:end, ...
                                            1:downsampleFactor:end, :);
        end
        fprintf('Point cloud downsampled by %dx for performance.\n', downsampleFactor);
    end

    % --- Step 2: Extract colors ---
    if nargin >= 2 && ~isempty(referenceImage)
        if size(referenceImage, 3) == 1
            referenceImage = repmat(referenceImage, [1 1 3]);
        end
        colors = reshape(double(referenceImage) / 255, [], 3);
    else
        % Use depth-based coloring
        normalizedZ = Z(:) / max(Z(:) + eps);
        colors = turboColormap(normalizedZ);
    end

    % --- Step 3: Create point cloud object ---
    points = [X(:), Y(:), Z(:)];

    % Remove invalid points (zero depth)
    validMask = Z(:) > 0.01;
    points = points(validMask, :);
    colors = colors(validMask, :);

    ptCloud = pointCloud(points, 'Color', uint8(colors * 255));

    % --- Step 4: Display ---
    figure('Name', 'Point Cloud Viewer', ...
           'NumberTitle', 'off', ...
           'Color', [0.05 0.05 0.08], ...
           'Position', [80, 80, 1200, 800]);

    pcshow(ptCloud, 'MarkerSize', 20);

    xlabel('X', 'Color', 'w');
    ylabel('Y', 'Color', 'w');
    zlabel('Depth', 'Color', 'w');

    title(titleText, ...
          'Color', 'w', ...
          'FontSize', 14, ...
          'FontWeight', 'bold');

    set(gca, 'Color', [0.05 0.05 0.08], ...
             'XColor', [0.5 0.5 0.5], ...
             'YColor', [0.5 0.5 0.5], ...
             'ZColor', [0.5 0.5 0.5]);

    view(0, -90); % Top-down initial view
    camproj('perspective');

    fprintf('Point cloud displayed: %d points\n', ptCloud.Count);
    drawnow;
end


function colors = turboColormap(values)
% TURBOCOLORMAP Approximation of the turbo colormap for coloring
    cmap = turbo(256);
    indices = max(1, min(256, round(values * 255) + 1));
    colors = cmap(indices, :);
end
