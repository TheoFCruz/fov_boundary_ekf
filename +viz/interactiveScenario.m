function ui = interactiveScenario(scenario, varargin)
%INTERACTIVESCENARIO Launch an interactive heading explorer for a scenario.
%
%   ui = viz.interactiveScenario(scenario)
%   ui = viz.interactiveScenario(scenario, 'GridSize', [241, 241], ...
%       'ContourLevels', -5:0.5:5, 'Padding', 0.5, 'Visible', true)
%
% Moving the slider previews ray-cast visibility. Releasing it recomputes the
% signed Euclidean distance field and its contours.

validateScenario(scenario);

parser = inputParser;
parser.FunctionName = 'viz.interactiveScenario';
addParameter(parser, 'GridSize', [241, 241]);
addParameter(parser, 'ContourLevels', -5:0.5:5);
addParameter(parser, 'Padding', 0.5);
addParameter(parser, 'Visible', true);
parse(parser, varargin{:});
options = parser.Results;

validateGridSize(options.GridSize);
validateContourLevels(options.ContourLevels);
validatePadding(options.Padding);
validateLogicalOption(options.Visible, 'Visible');

observer = scenario.Observer;
numRays = double(scenario.NumRays);
gridSize = reshape(double(options.GridSize), 1, 2);
contourLevels = double(options.ContourLevels(:).');
padding = double(options.Padding);
extent = observer.MaxRange + padding;
bounds = [observer.Position(1) - extent, observer.Position(1) + extent, ...
    observer.Position(2) - extent, observer.Position(2) + extent];
initialHeadingDegrees = normalizeDegrees(rad2deg(observer.Heading));

figureVisibility = 'off';
if options.Visible
    figureVisibility = 'on';
end

figureHandle = uifigure( ...
    'Name', sprintf('%s interactive heading', char(scenario.Name)), ...
    'Color', 'w', ...
    'Visible', figureVisibility);
layout = uigridlayout(figureHandle, [3, 1]);
layout.RowHeight = {'1x', 38, 22};
layout.Padding = [8, 8, 8, 8];

axesHandle = uiaxes(layout);
axesHandle.Layout.Row = 1;
controls = uigridlayout(layout, [1, 2]);
controls.Layout.Row = 2;
controls.ColumnWidth = {160, '1x'};
headingLabel = uilabel(controls, ...
    'Text', headingText(initialHeadingDegrees), ...
    'HorizontalAlignment', 'left');
headingSlider = uislider(controls, ...
    'Limits', [-180, 180], ...
    'Value', initialHeadingDegrees);
statusLabel = uilabel(layout, ...
    'Text', 'Ready', ...
    'HorizontalAlignment', 'left');
statusLabel.Layout.Row = 3;

cachedHeading = NaN;
cachedScenario = struct();
cachedResult = struct();
isRendering = false;
pendingFullHeading = [];

headingSlider.ValueChangingFcn = @(~, event) renderPreview(event.Value);
headingSlider.ValueChangedFcn = @(~, event) renderFull(event.Value);
renderFull(initialHeadingDegrees);

ui = struct();
ui.Figure = figureHandle;
ui.Axes = axesHandle;
ui.HeadingSlider = headingSlider;
ui.HeadingLabel = headingLabel;
ui.StatusLabel = statusLabel;
ui.Bounds = bounds;
ui.Preview = @renderPreview;
ui.Update = @renderFull;

    function renderPreview(headingDegrees)
        if isRendering || ~isvalid(figureHandle)
            return;
        end

        isRendering = true;
        cleanup = onCleanup(@finishRendering);
        headingDegrees = normalizeDegrees(headingDegrees);
        headingLabel.Text = headingText(headingDegrees);
        statusLabel.Text = 'Previewing visibility';

        [updatedScenario, result] = scenarioAtHeading(headingDegrees);
        clearAxes();
        viz.plotScenario(updatedScenario, result, ...
            'Parent', axesHandle, ...
            'ShowRays', false, ...
            'ShowNominalFov', true, ...
            'ShowHitPoints', false);
        applyAxesPresentation(headingDegrees, false);
        drawnow limitrate nocallbacks
        clear cleanup
    end

    function renderFull(headingDegrees)
        if ~isvalid(figureHandle)
            return;
        end

        headingDegrees = normalizeDegrees(headingDegrees);
        if isRendering
            pendingFullHeading = headingDegrees;
            return;
        end

        isRendering = true;
        cleanup = onCleanup(@finishRendering);
        headingLabel.Text = headingText(headingDegrees);
        statusLabel.Text = 'Computing signed-distance contours';
        drawnow

        try
            [updatedScenario, result] = scenarioAtHeading(headingDegrees);
            evaluator = @(points) metrics.signedEuclideanDistance( ...
                result.VisibleBoundary, points, 'Tolerance', result.Tolerance);
            field = metrics.sampleField(evaluator, ...
                'Bounds', bounds, ...
                'GridSize', gridSize, ...
                'Name', 'Signed Euclidean distance', ...
                'Units', 'coordinate units');

            clearAxes();
            viz.plotScenario(updatedScenario, result, ...
                'Parent', axesHandle, ...
                'ShowRays', false, ...
                'ShowNominalFov', true, ...
                'ShowHitPoints', false, ...
                'MetricField', field, ...
                'MetricContourLevels', contourLevels);
            applyAxesPresentation(headingDegrees, true);
        catch exception
            warning('viz:interactiveScenario:UpdateFailed', '%s', exception.message);
        end

        clear cleanup
    end

    function [updatedScenario, result] = scenarioAtHeading(headingDegrees)
        if isequal(headingDegrees, cachedHeading)
            updatedScenario = cachedScenario;
            result = cachedResult;
            return;
        end

        updatedObserver = fov.Observer( ...
            'Position', observer.Position, ...
            'Heading', deg2rad(headingDegrees), ...
            'Fov', observer.Fov);
        updatedScenario = scenario;
        updatedScenario.Observer = updatedObserver;
        result = fov.castRays(updatedObserver, scenario.Obstacles, ...
            'NumRays', numRays);
        cachedHeading = headingDegrees;
        cachedScenario = updatedScenario;
        cachedResult = result;
    end

    function clearAxes()
        colorbars = findall(figureHandle, 'Type', 'ColorBar');
        delete(colorbars);
        legend(axesHandle, 'off');
        delete(allchild(axesHandle));
        cla(axesHandle);
    end

    function applyAxesPresentation(headingDegrees, hasContours)
        xlim(axesHandle, bounds(1:2));
        ylim(axesHandle, bounds(3:4));
        if hasContours
            title(axesHandle, sprintf( ...
                '%s: %.1f degrees, signed distance (negative inside)', ...
                char(scenario.Name), headingDegrees));
            statusLabel.Text = 'Ready';
        else
            title(axesHandle, sprintf('%s: %.1f degrees preview', ...
                char(scenario.Name), headingDegrees));
            statusLabel.Text = 'Release slider to update contours';
        end
    end

    function finishRendering()
        isRendering = false;
        if isvalid(statusLabel) && ~strcmp(statusLabel.Text, 'Ready')
            if startsWith(statusLabel.Text, 'Computing')
                statusLabel.Text = 'Ready';
            end
        end

        if ~isempty(pendingFullHeading) && isvalid(figureHandle)
            headingDegrees = pendingFullHeading;
            pendingFullHeading = [];
            renderFull(headingDegrees);
        end
    end
end

function validateScenario(scenario)
requiredFields = {'Name', 'Observer', 'Obstacles', 'NumRays'};
if ~isstruct(scenario) || ~all(isfield(scenario, requiredFields)) || ...
        ~isa(scenario.Observer, 'fov.Observer')
    error('viz:interactiveScenario:InvalidScenario', ...
        'scenario must contain Name, Observer, Obstacles, and NumRays fields.');
end

if ~(isnumeric(scenario.NumRays) && isscalar(scenario.NumRays) && ...
        isreal(scenario.NumRays) && isfinite(scenario.NumRays) && ...
        scenario.NumRays >= 2 && floor(scenario.NumRays) == scenario.NumRays)
    error('viz:interactiveScenario:InvalidScenario', ...
        'scenario.NumRays must be an integer scalar greater than or equal to 2.');
end
end

function validateGridSize(gridSize)
if ~(isnumeric(gridSize) && isreal(gridSize) && numel(gridSize) == 2 && ...
        all(isfinite(gridSize(:))) && all(gridSize(:) >= 2) && ...
        all(floor(gridSize(:)) == gridSize(:)))
    error('viz:interactiveScenario:InvalidGridSize', ...
        'GridSize must be a two-element integer vector with values at least 2.');
end
end

function validateContourLevels(levels)
if ~(isnumeric(levels) && isreal(levels) && isvector(levels) && ...
        ~isempty(levels) && all(isfinite(levels(:))) && ...
        all(diff(levels(:)) > 0))
    error('viz:interactiveScenario:InvalidContourLevels', ...
        'ContourLevels must be a nonempty strictly increasing numeric vector.');
end
end

function validatePadding(padding)
if ~(isnumeric(padding) && isscalar(padding) && isreal(padding) && ...
        isfinite(padding) && padding > 0)
    error('viz:interactiveScenario:InvalidPadding', ...
        'Padding must be a finite positive scalar.');
end
end

function validateLogicalOption(value, optionName)
if ~(islogical(value) && isscalar(value))
    error('viz:interactiveScenario:InvalidOption', ...
        '%s must be a logical scalar.', optionName);
end
end

function degrees = normalizeDegrees(degrees)
degrees = mod(degrees + 180, 360) - 180;
end

function text = headingText(headingDegrees)
text = sprintf('Heading: %.1f degrees', headingDegrees);
end
