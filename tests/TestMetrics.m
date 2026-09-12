classdef TestMetrics < matlab.unittest.TestCase
    %TESTMETRICS Tests for signed-distance metrics and metric fields.

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
            partialScenario = scenarios.emptyField();
            partialResult = fov.castRays( ...
                partialScenario.Observer, partialScenario.Obstacles, ...
                'NumRays', 31);
            partialDistance = metrics.signedEuclideanDistance( ...
                partialResult.VisibleBoundary, partialResult.VisibleBoundary);

            verifyEqual(testCase, partialDistance, zeros(size(partialDistance)), ...
                'AbsTol', 1e-12);
            verifyEqual(testCase, metrics.signedEuclideanDistance( ...
                partialResult.VisibleBoundary, partialScenario.Observer.Position), 0, ...
                'AbsTol', 1e-12);
            verifyGreaterThan(testCase, metrics.signedEuclideanDistance( ...
                partialResult.VisibleBoundary, [-1, 0]), 0);

            fullObserver = fov.Observer([0, 0], 0, 10, 2*pi);
            fullResult = fov.castRays(fullObserver, [], 'NumRays', 181);
            verifyLessThan(testCase, metrics.signedEuclideanDistance( ...
                fullResult.VisibleBoundary, [0, 0]), 0);

            wallScenario = scenarios.singleWall();
            wallResult = fov.castRays( ...
                wallScenario.Observer, wallScenario.Obstacles, ...
                'NumRays', wallScenario.NumRays);
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

        function samplesMetricFieldWithExpectedGrid(testCase)
            evaluator = @(points) points(:, 1) + 2 * points(:, 2);
            field = metrics.sampleField(evaluator, ...
                'Bounds', [0, 2, -1, 1], ...
                'GridSize', [3, 5], ...
                'Name', 'Linear field', ...
                'Units', 'u');

            verifySize(testCase, field.X, [3, 5]);
            verifyEqual(testCase, field.Bounds, [0, 2, -1, 1]);
            verifyEqual(testCase, field.X(1, :), [0, 0.5, 1, 1.5, 2]);
            verifyEqual(testCase, field.Y(:, 1), [-1; 0; 1]);
            verifyEqual(testCase, field.Values, ...
                [-2, -1.5, -1, -0.5, 0; ...
                  0, 0.5,  1,  1.5, 2; ...
                  2, 2.5,  3,  3.5, 4]);
            verifyEqual(testCase, field.SignConvention, 'negative-inside');

            rowEvaluator = @(points) (points(:, 1) + 2 * points(:, 2)).';
            rowField = metrics.sampleField(rowEvaluator, ...
                'Bounds', [0, 2, -1, 1], 'GridSize', [3, 5]);
            verifyEqual(testCase, rowField.Values, field.Values);
        end

        function rejectsInvalidFieldInputsAndEvaluatorOutput(testCase)
            evaluator = @(points) points(:, 1);
            verifyError(testCase, @() metrics.sampleField(evaluator, ...
                'Bounds', [0, 0, -1, 1]), ...
                'metrics:sampleField:InvalidBounds');
            verifyError(testCase, @() metrics.sampleField(evaluator, ...
                'Bounds', [0, 1, -1, 1], 'GridSize', [1, 4]), ...
                'metrics:sampleField:InvalidGridSize');
            verifyError(testCase, @() metrics.sampleField(@(points) 1, ...
                'Bounds', [0, 1, 0, 1]), ...
                'metrics:sampleField:InvalidEvaluatorOutput');
        end

        function plotsContourGraphicsHandles(testCase)
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@() close(figureHandle));
            axesHandle = axes('Parent', figureHandle);
            field = metrics.sampleField(@(points) points(:, 1), ...
                'Bounds', [-1, 1, -1, 1], 'GridSize', [11, 11]);

            handles = viz.plotMetricContours(field, ...
                'Parent', axesHandle, 'Levels', [-0.5, 0.5]);

            verifyTrue(testCase, isgraphics(handles.Contours));
            verifyTrue(testCase, isgraphics(handles.ZeroContour));
            verifyTrue(testCase, isgraphics(handles.Colorbar));
            verifyEqual(testCase, handles.Axes, axesHandle);
            clear cleanup
        end

        function runsSingleScenarioContourDemo(testCase)
            originalVisibility = get(groot, 'DefaultFigureVisible');
            beforeFigures = findall(groot, 'Type', 'figure');
            set(groot, 'DefaultFigureVisible', 'off');
            cleanup = onCleanup(@() restoreFigures(originalVisibility, beforeFigures));

            projectRoot = fileparts(fileparts(mfilename('fullpath')));
            run(fullfile(projectRoot, 'scripts', 'runSingleScenario.m'));

            figures = findall(groot, 'Type', 'figure');
            verifyNotEmpty(testCase, figures);
            clear cleanup
        end

        function updatesInteractiveHeadingExplorer(testCase)
            scenario = scenarios.singleWall();
            ui = viz.interactiveScenario(scenario, ...
                'GridSize', [31, 31], ...
                'ContourLevels', -2:1:2, ...
                'Visible', false);
            cleanup = onCleanup(@() close(ui.Figure));

            verifyTrue(testCase, isgraphics(ui.Figure));
            verifyTrue(testCase, isgraphics(ui.Axes));
            verifyTrue(testCase, isgraphics(ui.HeadingSlider));
            verifyTrue(testCase, isgraphics(ui.HeadingLabel));
            verifyTrue(testCase, isgraphics(ui.StatusLabel));
            verifyEqual(testCase, ui.Bounds, [-10.5, 10.5, -10.5, 10.5]);
            verifyNotEmpty(testCase, findall(ui.Figure, 'Type', 'ColorBar'));
            verifyNotEmpty(testCase, findall(ui.Axes, 'Type', 'Contour'));

            ui.HeadingSlider.ValueChangingFcn( ...
                ui.HeadingSlider, struct('Value', 30));
            verifyEmpty(testCase, findall(ui.Figure, 'Type', 'ColorBar'));
            verifyEmpty(testCase, findall(ui.Axes, 'Type', 'Contour'));
            verifyEqual(testCase, xlim(ui.Axes), ui.Bounds(1:2), 'AbsTol', 1e-12);
            verifyEqual(testCase, ylim(ui.Axes), ui.Bounds(3:4), 'AbsTol', 1e-12);

            ui.HeadingSlider.ValueChangedFcn( ...
                ui.HeadingSlider, struct('Value', 30));
            verifyEqual(testCase, numel(findall(ui.Figure, 'Type', 'ColorBar')), 1);
            verifyNotEmpty(testCase, findall(ui.Axes, 'Type', 'Contour'));
            verifyEqual(testCase, xlim(ui.Axes), ui.Bounds(1:2), 'AbsTol', 1e-12);
            verifyEqual(testCase, ylim(ui.Axes), ui.Bounds(3:4), 'AbsTol', 1e-12);
            verifyEqual(testCase, ui.StatusLabel.Text, 'Ready');

            ui.HeadingSlider.ValueChangedFcn( ...
                ui.HeadingSlider, struct('Value', -45));
            verifyEqual(testCase, numel(findall(ui.Figure, 'Type', 'ColorBar')), 1);
            legendHandle = legend(ui.Axes);
            verifyFalse(testCase, any(startsWith(string(legendHandle.String), 'data')));
            verifyEqual(testCase, legendHandle.Location, 'northeast');
            clear cleanup
        end
    end
end

function boundary = squareBoundary()
boundary = [-1, -1; 1, -1; 1, 1; -1, 1; -1, -1];
end

function restoreFigures(originalVisibility, beforeFigures)
set(groot, 'DefaultFigureVisible', originalVisibility);
figures = findall(groot, 'Type', 'figure');
delete(figures(~ismember(figures, beforeFigures)));
end
