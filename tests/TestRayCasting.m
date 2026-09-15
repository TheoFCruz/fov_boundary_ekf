classdef TestRayCasting < matlab.unittest.TestCase
    %TESTRAYCASTING Tests for the initial geometry primitives.

    methods (Test)
        function samplesPartialFovIncludingBoundaries(testCase)
            observer = fov.Observer([0, 0], 0, 10, pi/2);
            angles = fov.sampleRayAngles(observer, 5);
            expected = (-pi/4:pi/8:pi/4).';

            verifySize(testCase, angles, [5, 1]);
            verifyLessThanOrEqual(testCase, max(abs(angles - expected)), 1e-12);
        end

        function samplesFullFovWithoutDuplicateEndpoint(testCase)
            observer = fov.Observer([0, 0], 0, 10, 2*pi);
            angles = fov.sampleRayAngles(observer, 4);

            verifySize(testCase, angles, [4, 1]);
            verifyLessThanOrEqual(testCase, max(abs(diff(angles) - pi/2)), 1e-12);
            verifyLessThanOrEqual(testCase, abs(angles(end) - (pi/2)), 1e-12);
        end

        function returnsAllPolygonBoundaryEdges(testCase)
            obstacle = fov.polygonObstacle('box', ...
                [0, 0; 2, 0; 2, 1; 0, 1]);
            [starts, ends] = fov.internal.polygonEdges(obstacle);

            verifyEqual(testCase, starts, obstacle.Vertices);
            verifyEqual(testCase, ends, ...
                [2, 0; 2, 1; 0, 1; 0, 0]);
        end

        function detectsVectorizedRaySegmentIntersections(testCase)
            starts = [2, -1; 2, 1; -2, -1; 1, 1; 2, 0; 2, 0];
            ends = [2, 1; 2, 2; -2, 1; 3, 1; 4, 0; 2, 1];

            [distance, points, parameter, isHit] = ...
                fov.internal.raySegmentIntersection( ...
                [0, 0], [2, 0], starts, ends);

            verifyEqual(testCase, isHit, [true; false; false; false; true; true]);
            verifyEqual(testCase, distance([1, 5, 6]), [2; 2; 2]);
            verifyEqual(testCase, points([1, 5, 6], :), ...
                [2, 0; 2, 0; 2, 0]);
            verifyLessThanOrEqual(testCase, ...
                max(abs(parameter([1, 5, 6]) - [0.5; 0; 0])), 1e-12);
            verifyTrue(testCase, isinf(distance(2)));
            verifyTrue(testCase, isinf(distance(3)));
            verifyTrue(testCase, isinf(distance(4)));
        end

        function detectsCollinearOverlap(testCase)
            [distance, point, parameter, isHit] = ...
                fov.internal.raySegmentIntersection( ...
                [0, 0], [1, 0], [2, 0], [4, 0]);

            verifyTrue(testCase, isHit);
            verifyEqual(testCase, distance, 2);
            verifyEqual(testCase, point, [2, 0]);
            verifyEqual(testCase, parameter, 0);
        end

        function rejectsZeroRayDirection(testCase)
            verifyError(testCase, @() fov.internal.raySegmentIntersection( ...
                [0, 0], [0, 0], [1, -1], [1, 1]), ...
                'fov:internal:raySegmentIntersection:ZeroDirection');
        end

        function reachesMaximumRangeInEmptyField(testCase)
            observer = fov.Observer([0, 0], 0, 10, pi/2);
            emptyObstacles = struct('Name', {}, 'Vertices', {});
            result = fov.castRays(observer, emptyObstacles, 'NumRays', 11);

            verifyEqual(testCase, result.Distances, ...
                repmat(observer.MaxRange, 11, 1));
            verifyFalse(testCase, any(result.IsOccluded));
            verifyEqual(testCase, result.HitObstacleId, zeros(11, 1));
        end

        function rejectsDegenerateDirectObstacle(testCase)
            observer = fov.Observer([0, 0], 0, 10, pi/2);
            obstacle = struct('Vertices', [2, 0; 2, 1]);

            verifyError(testCase, @() fov.castRays(observer, obstacle), ...
                'fov:castRays:InvalidObstacles');
        end

        function stopsCentralRayAtSingleWall(testCase)
            observer = fov.Observer([0, 0], 0, 10, pi/2);
            wall = fov.polygonObstacle('wall', ...
                [4, -2; 4, 2; 4.3, 2; 4.3, -2]);
            result = fov.castRays(observer, wall, 'NumRays', 181);
            centralRay = 91;

            verifyEqual(testCase, result.Distances(centralRay), 4, ...
                'AbsTol', 1e-12);
            verifyTrue(testCase, result.IsOccluded(centralRay));
            verifyEqual(testCase, result.HitObstacleId(centralRay), 1);
            verifyEqual(testCase, result.EndPoints(centralRay, :), [4, 0], ...
                'AbsTol', 1e-12);
        end

        function nearestObstacleWins(testCase)
            observer = fov.Observer([0, 0], 0, 10, pi/2);
            nearObstacle = fov.polygonObstacle('near', ...
                [3, -1; 3, 1; 3.2, 1; 3.2, -1]);
            farObstacle = fov.polygonObstacle('far', ...
                [6, -1; 6, 1; 6.2, 1; 6.2, -1]);

            result = fov.castRays(observer, [nearObstacle, farObstacle], ...
                'NumRays', 9);

            centralRay = 5;
            verifyEqual(testCase, result.Distances(centralRay), 3, ...
                'AbsTol', 1e-12);
            verifyEqual(testCase, result.HitObstacleId(centralRay), 1);
        end

        function storesClosedVisibilityBoundaries(testCase)
            observer = fov.Observer([0, 0], 0, 10, pi/2);
            wall = fov.polygonObstacle('wall', ...
                [4, -2; 4, 2; 4.3, 2; 4.3, -2]);
            result = fov.castRays(observer, wall, 'NumRays', 11);

            verifyEqual(testCase, result.VisibleBoundary(1, :), ...
                observer.Position);
            verifyEqual(testCase, result.VisibleBoundary(end, :), ...
                observer.Position);
            verifyEqual(testCase, result.NominalBoundary(1, :), ...
                observer.Position);
            verifyEqual(testCase, result.NominalBoundary(end, :), ...
                observer.Position);
        end
    end
end
