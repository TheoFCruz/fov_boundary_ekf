classdef TestMetrics < matlab.unittest.TestCase
    %TESTMETRICS Tests for the sampled-polygon signed-distance metric.

    methods (Test)
        function calculatesSignedDistancesForSquare(testCase)
            boundary = squareBoundary();
            points = [0, 0; 0.5, 0; 1, 0; 2, 0; 2, 2];

            [distance, details] = metrics.signedEuclideanDistance( ...
                boundary, points);

            verifyEqual(testCase, distance, ...
                [-1; -0.5; 0; 1; sqrt(2)], 'AbsTol', 1e-12);
            verifyEqual(testCase, details.ClosestPoint(4, :), [1, 0], ...
                'AbsTol', 1e-12);
            verifyEqual(testCase, details.ClosestEdgeIndex(4), 2);
            verifyTrue(testCase, details.IsInside(1));
            verifyTrue(testCase, details.IsOnBoundary(3));
        end

        function acceptsOpenReversedAndDuplicateBoundaries(testCase)
            boundary = squareBoundary();
            openBoundary = boundary(1:end-1, :);
            reversedBoundary = flipud(openBoundary);
            duplicatedBoundary = [openBoundary(1, :); openBoundary(1, :); ...
                openBoundary(2:end, :)];
            repeatedClosingBoundary = [openBoundary; openBoundary(1, :); ...
                openBoundary(1, :)];
            points = [0, 0; 2, 0];

            expected = metrics.signedEuclideanDistance(boundary, points);
            verifyEqual(testCase, ...
                metrics.signedEuclideanDistance(openBoundary, points), expected, ...
                'AbsTol', 1e-12);
            verifyEqual(testCase, ...
                metrics.signedEuclideanDistance(reversedBoundary, points), expected, ...
                'AbsTol', 1e-12);
            verifyEqual(testCase, ...
                metrics.signedEuclideanDistance(duplicatedBoundary, points), expected, ...
                'AbsTol', 1e-12);
            verifyEqual(testCase, ...
                metrics.signedEuclideanDistance(repeatedClosingBoundary, points), ...
                expected, 'AbsTol', 1e-12);
        end

        function rejectsInvalidAndDegenerateBoundaries(testCase)
            verifyError(testCase, @() metrics.signedEuclideanDistance( ...
                [0, 0, 0], [0, 0]), ...
                'metrics:signedEuclideanDistance:InvalidBoundary');
            verifyError(testCase, @() metrics.signedEuclideanDistance( ...
                [0, 0; NaN, 1; 1, 0], [0, 0]), ...
                'metrics:signedEuclideanDistance:InvalidBoundary');
            verifyError(testCase, @() metrics.signedEuclideanDistance( ...
                [0, 0; Inf, 1; 1, 0], [0, 0]), ...
                'metrics:signedEuclideanDistance:InvalidBoundary');
            verifyError(testCase, @() metrics.signedEuclideanDistance( ...
                [0, 0; 1, 0; 2, 0], [0, 0]), ...
                'metrics:internal:normalizeBoundary:DegenerateBoundary');
            verifyError(testCase, @() metrics.signedEuclideanDistance( ...
                [0, 0; 4, 0; 0, 4; 1, 0.5; 4, 4], [0, 0]), ...
                'metrics:internal:normalizeBoundary:SelfIntersectingBoundary');
        end

        function handlesConcavePolygon(testCase)
            boundary = [0, 0; 3, 0; 3, 3; 1, 1; 0, 3];
            distance = metrics.signedEuclideanDistance( ...
                boundary, [0.5, 0.5; 1.5, 2.5; 2, 2]);

            verifyLessThan(testCase, distance(1), 0);
            verifyGreaterThan(testCase, distance(2), 0);
            verifyEqual(testCase, distance(3), 0, 'AbsTol', 1e-12);
        end

        function integratesWithVisibleFovBoundaries(testCase)
            partialObserver = fov.Observer([0, 0], 0, 10, pi/2);
            emptyObstacles = struct('Name', {}, 'Vertices', {});
            partialResult = fov.castRays(partialObserver, emptyObstacles, ...
                'NumRays', 31);
            partialDistance = metrics.signedEuclideanDistance( ...
                partialResult.VisibleBoundary, partialResult.VisibleBoundary);

            verifyEqual(testCase, partialDistance, zeros(size(partialDistance)), ...
                'AbsTol', 1e-12);
            verifyEqual(testCase, metrics.signedEuclideanDistance( ...
                partialResult.VisibleBoundary, partialObserver.Position), 0, ...
                'AbsTol', 1e-12);
            verifyGreaterThan(testCase, metrics.signedEuclideanDistance( ...
                partialResult.VisibleBoundary, [-1, 0]), 0);

            fullObserver = fov.Observer([0, 0], 0, 10, 2*pi);
            fullResult = fov.castRays(fullObserver, [], 'NumRays', 181);
            verifyLessThan(testCase, metrics.signedEuclideanDistance( ...
                fullResult.VisibleBoundary, [0, 0]), 0);

            wall = fov.polygonObstacle('wall', ...
                [4, -2; 4, 2; 4.3, 2; 4.3, -2]);
            wallResult = fov.castRays(partialObserver, wall, 'NumRays', 181);
            verifyGreaterThan(testCase, metrics.signedEuclideanDistance( ...
                wallResult.VisibleBoundary, [6, 0]), 0);
        end

        function obeysLipschitzBound(testCase)
            points = [-0.75, -0.25; 0, 0; 1.5, 0.5; 2, 2];
            distance = metrics.signedEuclideanDistance(squareBoundary(), points);

            for firstIndex = 1:size(points, 1) - 1
                for secondIndex = firstIndex + 1:size(points, 1)
                    pointDistance = norm(points(firstIndex, :) - points(secondIndex, :));
                    verifyLessThanOrEqual(testCase, ...
                        abs(distance(firstIndex) - distance(secondIndex)), ...
                        pointDistance + 1e-12);
                end
            end
        end

        function filterConsistencySmokeTestOnAShortEkfRun(testCase)
            % Structural check only (E2 measures and reports consistency
            % separately; this is not a tuning assertion): a short boundary
            % EKF run over two seeds produces a PerLabel table, a finite
            % overall NEES, and step-ratio summary fields.
            scenario = scenarios.openRoom();
            scenario.Time.Stop = 0.5;
            scenario.Observer.Reference = struct('Times', 0, 'Values', [0, 0, 0.2]);
            scenario.Sensor.RangeNoiseStd = 0.01;
            params = estimation.defaultTransportParams();
            params.MeasurementNoiseStd = scenario.Sensor.RangeNoiseStd;
            scenario.Estimator = estimation.makeBoundaryEkf(params);

            results = cell(1, 2);
            for seed = 1:2
                seededScenario = scenario;
                seededScenario.Sensor.Seed = seed;
                results{seed} = simulation.runScenario(seededScenario);
            end

            report = metrics.filterConsistency(results);
            verifyClass(testCase, report.PerLabel, 'table');
            verifyGreaterThan(testCase, height(report.PerLabel), 0);
            verifyTrue(testCase, isfinite(report.Overall.MeanNEES));
            verifyTrue(testCase, isfinite(report.Overall.RMSE));
            verifyTrue(testCase, isfinite(report.StepRatioMax));

            % A single result (not wrapped in a cell) must work the same way.
            singleReport = metrics.filterConsistency(results{1});
            verifyEqual(testCase, singleReport.Overall.Count, ...
                accumulatedCountOf(results{1}));
        end

    end
end

function count = accumulatedCountOf(result)
%ACCUMULATEDCOUNTOF Expected Overall.Count: beams over samples k >= 2.

count = 0;
for k = 2:numel(result.Beliefs)
    count = count + numel(result.Beliefs{k}.Mean);
end
end

function boundary = squareBoundary()
boundary = [-1, -1; 1, -1; 1, 1; -1, 1; -1, -1];
end
