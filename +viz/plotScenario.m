function [figureHandle, axesHandle] = plotScenario(scenario, result, varargin)
%PLOTSCENARIO Plot obstacles, nominal FOV, and ray-cast visibility.
%
%   viz.plotScenario(scenario, result)
%   [fig, ax] = viz.plotScenario(scenario, result, ...
%       'ShowRays', true, 'ShowNominalFov', true, ...
%       'ShowHitPoints', false, 'MetricField', field)

if ~isstruct(scenario) || ~isfield(scenario, 'Observer') || ...
        ~isfield(scenario, 'Obstacles')
    error('viz:plotScenario:InvalidScenario', ...
        'scenario must contain Observer and Obstacles fields.');
end

if ~isstruct(result) || ~isfield(result, 'Origin') || ...
        ~isfield(result, 'EndPoints') || ...
        ~isfield(result, 'NominalBoundary') || ...
        ~isfield(result, 'VisibleBoundary')
    error('viz:plotScenario:InvalidResult', ...
        'result must be returned by fov.castRays.');
end

parser = inputParser;
parser.FunctionName = 'viz.plotScenario';
addParameter(parser, 'ShowRays', true);
addParameter(parser, 'ShowNominalFov', true);
addParameter(parser, 'ShowHitPoints', false);
addParameter(parser, 'Parent', []);
addParameter(parser, 'MetricField', []);
addParameter(parser, 'MetricContourLevels', []);
parse(parser, varargin{:});
options = parser.Results;

validateLogicalOption(options.ShowRays, 'ShowRays');
validateLogicalOption(options.ShowNominalFov, 'ShowNominalFov');
validateLogicalOption(options.ShowHitPoints, 'ShowHitPoints');

if isempty(options.Parent)
    figureHandle = figure('Name', char(scenario.Name), 'Color', 'w');
    axesHandle = axes('Parent', figureHandle);
else
    if ~isgraphics(options.Parent, 'axes')
        error('viz:plotScenario:InvalidParent', ...
            'Parent must be an axes graphics object.');
    end
    axesHandle = options.Parent;
    figureHandle = ancestor(axesHandle, 'figure');
end

wasHolding = ishold(axesHandle);
hold(axesHandle, 'on');

if options.ShowNominalFov
    patch(axesHandle, ...
        result.NominalBoundary(:, 1), result.NominalBoundary(:, 2), ...
        [0.75, 0.75, 0.75], ...
        'FaceAlpha', 0.18, ...
        'EdgeColor', [0.45, 0.45, 0.45], ...
        'LineStyle', '--', ...
        'DisplayName', 'Nominal FOV');
end

patch(axesHandle, ...
    result.VisibleBoundary(:, 1), result.VisibleBoundary(:, 2), ...
    [0.20, 0.55, 0.90], ...
    'FaceAlpha', 0.28, ...
    'EdgeColor', [0.05, 0.25, 0.55], ...
    'DisplayName', 'Visible FOV');

if ~isempty(options.MetricField)
    contourOptions = {'Parent', axesHandle};
    if ~isempty(options.MetricContourLevels)
        contourOptions = [contourOptions, ...
            {'Levels', options.MetricContourLevels}];
    end
    viz.plotMetricContours(options.MetricField, contourOptions{:});
end

if options.ShowRays
    origin = result.Origin;
    rayX = [repmat(origin(1), size(result.EndPoints, 1), 1), ...
        result.EndPoints(:, 1), nan(size(result.EndPoints, 1), 1)].';
    rayY = [repmat(origin(2), size(result.EndPoints, 1), 1), ...
        result.EndPoints(:, 2), nan(size(result.EndPoints, 1), 1)].';
    plot(axesHandle, rayX, rayY, ...
        'Color', [0.20, 0.40, 0.70], ...
        'LineWidth', 0.5, ...
        'HandleVisibility', 'off');
end

for obstacleIndex = 1:numel(scenario.Obstacles)
    vertices = scenario.Obstacles(obstacleIndex).Vertices;
    closedVertices = [vertices; vertices(1, :)];
    obstacleName = sprintf('Obstacle %d', obstacleIndex);
    if isfield(scenario.Obstacles, 'Name')
        obstacleName = char(scenario.Obstacles(obstacleIndex).Name);
    end

    patch(axesHandle, closedVertices(:, 1), closedVertices(:, 2), ...
        [0.25, 0.25, 0.25], ...
        'FaceAlpha', 0.92, ...
        'EdgeColor', [0.05, 0.05, 0.05], ...
        'DisplayName', obstacleName);
end

if options.ShowHitPoints
    hitPoints = result.EndPoints(result.IsOccluded, :);
    if ~isempty(hitPoints)
        scatter(axesHandle, hitPoints(:, 1), hitPoints(:, 2), 24, ...
            [0.85, 0.15, 0.10], 'filled', ...
            'DisplayName', 'Obstacle intersections');
    end
end

origin = result.Origin;
plot(axesHandle, origin(1), origin(2), 'ko', ...
    'MarkerFaceColor', 'k', ...
    'DisplayName', 'Observer');
quiver(axesHandle, origin(1), origin(2), ...
    cos(scenario.Observer.Heading), sin(scenario.Observer.Heading), ...
    0.8, 'Color', 'k', 'LineWidth', 1.2, ...
    'MaxHeadSize', 1.5, 'DisplayName', 'Heading');

axis(axesHandle, 'equal');
grid(axesHandle, 'on');
xlabel(axesHandle, 'x');
ylabel(axesHandle, 'y');
title(axesHandle, char(scenario.Name));
legend(axesHandle, 'show', 'Location', 'northeast');

if ~wasHolding
    hold(axesHandle, 'off');
end
end

function validateLogicalOption(value, optionName)
if ~(islogical(value) && isscalar(value))
    error('viz:plotScenario:InvalidOption', ...
        '%s must be a logical scalar.', optionName);
end
end
