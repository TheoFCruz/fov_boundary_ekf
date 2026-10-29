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

        function emptyObstacleCastHasNoDepthJumpsAndIsUnbridged(testCase)
            observer = fov.Observer([0, 0], 0, 10, pi/2);
            emptyObstacles = struct('Name', {}, 'Vertices', {});
            result = fov.castRays(observer, emptyObstacles, 'NumRays', 11);
            expected = fov.boundaryFromRanges([observer.Position, ...
                observer.Heading], result.RayAngles - observer.Heading, ...
                result.Distances, observer.OpeningAngle);

            verifyFalse(testCase, any(result.IsDepthJump));
            verifyEqual(testCase, result.VisibleBoundary, expected, 'AbsTol', 1e-12);
        end

        function flagsDepthJumpsAtBoxAndWallTransitions(testCase)
            observer = fov.Observer([0, 0], 0, 10, pi);
            box = fov.polygonObstacle('box', [4, -0.5; 5, -0.5; 5, 0.5; 4, 0.5]);
            wall = fov.polygonObstacle('wall', [9, -5; 9.3, -5; 9.3, 5; 9, 5]);
            result = fov.castRays(observer, [box, wall], 'NumRays', 13);

            jumpPairs = find(result.IsDepthJump);
            jumpIdentityPairs = [result.HitObstacleId(jumpPairs), ...
                result.HitObstacleId(jumpPairs + 1)];
            verifyNotEmpty(testCase, jumpPairs);
            verifyTrue(testCase, any(all(sort(jumpIdentityPairs, 2) == [1, 2], 2)), ...
                'Expected a box/wall depth jump.');
            verifyTrue(testCase, any(all(sort(jumpIdentityPairs, 2) == [0, 2], 2)), ...
                'Expected a wall/no-return depth jump.');

            % The oracle boundary must remain a valid polygon for the metric.
            metrics.signedEuclideanDistance(result.VisibleBoundary, [0, 0]);

            % A point just behind the box's front-top corner, in the gap the
            % old chord bridge wrongly drew as visible, must read as outside.
            shadowBearing = deg2rad(5);
            shadowPoint = 4.5 * [cos(shadowBearing), sin(shadowBearing)];
            signedDistance = metrics.signedEuclideanDistance( ...
                result.VisibleBoundary, shadowPoint);
            verifyGreaterThan(testCase, signedDistance, 0);
        end

        function insertsStepVertexAtNearToFarJump(testCase)
            angles = deg2rad([0; 10; 20; 30]);
            unit = [cos(angles), sin(angles)];

            boundary = fov.boundaryFromRanges([0, 0, 0], angles, [3; 3; 5; 5], ...
                pi/2, [false; true; false]);

            expected = [0, 0; 3 * unit(1, :); 3 * unit(2, :); 3 * unit(3, :); ...
                5 * unit(3, :); 5 * unit(4, :); 0, 0];
            verifyEqual(testCase, boundary, expected, 'AbsTol', 1e-12);
            verifyLessThan(testCase, metrics.signedEuclideanDistance( ...
                boundary, [2, 0.5]), 0);
        end

        function insertsStepVertexAtFarToNearJump(testCase)
            angles = deg2rad([0; 10; 20; 30]);
            unit = [cos(angles), sin(angles)];

            boundary = fov.boundaryFromRanges([0, 0, 0], angles, [5; 5; 3; 3], ...
                pi/2, [false; true; false]);

            expected = [0, 0; 5 * unit(1, :); 5 * unit(2, :); 3 * unit(2, :); ...
                3 * unit(3, :); 3 * unit(4, :); 0, 0];
            verifyEqual(testCase, boundary, expected, 'AbsTol', 1e-12);
            verifyLessThan(testCase, metrics.signedEuclideanDistance( ...
                boundary, [2, 0.5]), 0);
        end

        function collapsesFarApertureEdgeRayInsteadOfSpike(testCase)
            % The far edge ray would double back over the aperture side edge.
            angles = deg2rad([0; 10; 20]);
            unit = [cos(angles), sin(angles)];

            boundary = fov.boundaryFromRanges([0, 0, 0], angles, [5; 3; 3], ...
                pi/2, [true; false]);

            expected = [0, 0; 3 * unit; 0, 0];
            verifyEqual(testCase, boundary, expected, 'AbsTol', 1e-12);
            verifyLessThan(testCase, metrics.signedEuclideanDistance( ...
                boundary, [2, 0.2]), 0);
        end

        function collapsesIsolatedFarRayInsteadOfSpike(testCase)
            % One ray seeing through a gap pulls in to its farther near neighbour.
            angles = deg2rad([0; 10; 20; 30]);
            unit = [cos(angles), sin(angles)];

            boundary = fov.boundaryFromRanges([0, 0, 0], angles, [3; 6; 4; 4], ...
                pi/2, [true; true; false]);

            expected = [0, 0; 3 * unit(1, :); 3 * unit(2, :); 4 * unit(2, :); ...
                4 * unit(3, :); 4 * unit(4, :); 0, 0];
            verifyEqual(testCase, boundary, expected, 'AbsTol', 1e-12);
            verifyLessThan(testCase, metrics.signedEuclideanDistance( ...
                boundary, [2, 0.5]), 0);
        end

        function omittedOrEmptyIsJumpReproducesOriginalConstruction(testCase)
            angles = [0; deg2rad(10)];
            ranges = [3; 5];
            openingAngle = pi/2;
            observerPose = [0, 0, 0];

            withoutArgument = fov.boundaryFromRanges(observerPose, angles, ...
                ranges, openingAngle);
            withEmptyFlags = fov.boundaryFromRanges(observerPose, angles, ...
                ranges, openingAngle, false(1, 0).');

            verifySize(testCase, withoutArgument, [4, 2]);
            verifyEqual(testCase, withEmptyFlags, withoutArgument, 'AbsTol', 1e-12);
        end

        function rejectsMismatchedIsJumpLength(testCase)
            angles = [0; deg2rad(10); deg2rad(20)];
            ranges = [3; 5; 5];
            verifyError(testCase, @() fov.boundaryFromRanges([0, 0, 0], ...
                angles, ranges, pi/2, true(3, 1)), ...
                'fov:boundaryFromRanges:InvalidIsJump');
        end

        function isPointVisibleChecksRangeApertureAndOcclusion(testCase)
            pose = [0, 0, 0];
            spec = fov.FovSpec(10, pi/2);
            wall = fov.polygonObstacle('wall', [4, -2; 4, 2; 4.3, 2; 4.3, -2]);

            verifyTrue(testCase, fov.isPointVisible(pose, spec, wall, [2, 0]));
            verifyFalse(testCase, fov.isPointVisible(pose, spec, wall, [5, 0]));
            verifyFalse(testCase, fov.isPointVisible(pose, spec, wall, ...
                10 * [cos(deg2rad(80)), sin(deg2rad(80))]));
            verifyFalse(testCase, fov.isPointVisible(pose, spec, wall, [11, 0]));

            fullCircleSpec = fov.FovSpec(10, 2*pi);
            verifyTrue(testCase, fov.isPointVisible(pose, fullCircleSpec, ...
                wall, [-3, 0]));
        end
    end
end
