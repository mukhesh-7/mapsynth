% MATHUTILS Collection of shared mathematical utility functions
%   Provides geometric, statistical, and linear algebra helpers used
%   across the reconstruction pipeline. Call as: mathUtils.functionName(args)

classdef mathUtils

    methods (Static)

        function H = computeHomographyDLT(pts1, pts2)
        % COMPUTEHOMOGRAPHYDLT Direct Linear Transform for homography estimation
        %   Given N >= 4 point correspondences, computes the 3x3 homography
        %   matrix H such that pts2 ~ H * pts1 (in homogeneous coordinates).
        %
        %   pts1, pts2: Nx2 arrays of matched 2D coordinates

            n = size(pts1, 1);
            if n < 4
                error('mathUtils:notEnoughPoints', ...
                    'At least 4 point correspondences required (got %d).', n);
            end

            % Build the 2N x 9 coefficient matrix A
            A = zeros(2*n, 9);
            for i = 1:n
                x = pts1(i, 1); y = pts1(i, 2);
                u = pts2(i, 1); v = pts2(i, 2);

                A(2*i-1, :) = [-x, -y, -1,  0,  0,  0,  u*x, u*y, u];
                A(2*i,   :) = [ 0,  0,  0, -x, -y, -1,  v*x, v*y, v];
            end

            % Solve via SVD: the solution is the last column of V
            [~, ~, V] = svd(A, 'econ');
            h = V(:, end);
            H = reshape(h, [3, 3])';
        end


        function [inliers, bestModel] = ransac(pts1, pts2, fitFn, distFn, ...
                                                threshold, maxIter)
        % RANSAC Generic RANdom SAmple Consensus implementation
        %   pts1, pts2: Nx2 coordinate arrays
        %   fitFn:      function handle @(p1, p2) returning a model
        %   distFn:     function handle @(model, p1, p2) returning Nx1 distances
        %   threshold:  inlier distance threshold
        %   maxIter:    maximum iterations
        %
        %   Returns: logical inlier mask and best-fit model

            n = size(pts1, 1);
            bestInlierCount = 0;
            bestModel = [];
            inliers = false(n, 1);

            for iter = 1:maxIter
                % Sample 4 random correspondences
                idx = randperm(n, min(4, n));
                try
                    model = fitFn(pts1(idx, :), pts2(idx, :));
                catch
                    continue;
                end

                % Evaluate all points
                distances = distFn(model, pts1, pts2);
                currentInliers = distances < threshold;
                currentCount = sum(currentInliers);

                if currentCount > bestInlierCount
                    bestInlierCount = currentCount;
                    bestModel = model;
                    inliers = currentInliers;
                end
            end

            if isempty(bestModel)
                warning('mathUtils:ransacFail', ...
                    'RANSAC failed to find a valid model.');
            end
        end


        function d = homographyTransferError(H, pts1, pts2)
        % HOMOGRAPHYTRANSFERERROR Symmetric transfer error for a homography
        %   Returns the Euclidean distance between projected and actual points

            n = size(pts1, 1);
            
            % Forward projection: pts2_hat = H * pts1
            ones_col = ones(n, 1);
            p1h = [pts1, ones_col]';
            projected = H * p1h;
            projected = projected ./ projected(3, :);
            
            d = sqrt(sum((pts2' - projected(1:2, :)).^2, 1))';
        end


        function angle = vectorAngle(v1, v2)
        % VECTORANGLE Angle in degrees between two 2D/3D vectors
            cosTheta = dot(v1, v2) / (norm(v1) * norm(v2));
            cosTheta = max(-1, min(1, cosTheta)); % clamp for numerical safety
            angle = acosd(cosTheta);
        end


        function R = rotationMatrix2D(angleDeg)
        % ROTATIONMATRIX2D Returns 2x2 rotation matrix for given angle in degrees
            theta = deg2rad(angleDeg);
            R = [cos(theta), -sin(theta);
                 sin(theta),  cos(theta)];
        end


        function [centroid, spread] = pointCloudStats(points)
        % POINTCLOUDSTATS Computes centroid and spatial spread of a point set
        %   points: Nx2 or Nx3 matrix
            centroid = mean(points, 1);
            spread = std(points, 0, 1);
        end


        function normalized = normalizePoints(points)
        % NORMALIZEPOINTS Normalizes 2D points for numerical conditioning
        %   Centers at origin and scales so mean distance from origin is sqrt(2)
            centroid = mean(points, 1);
            shifted = points - centroid;
            avgDist = mean(sqrt(sum(shifted.^2, 2)));
            scale = sqrt(2) / max(avgDist, eps);
            normalized = shifted * scale;
        end

    end
end
