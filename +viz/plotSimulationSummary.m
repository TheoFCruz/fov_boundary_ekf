function handles = plotSimulationSummary(result, varargin)
%PLOTSIMULATIONSUMMARY Plot trajectories and checkpoint-one diagnostics.

validateResult(result);
parser = inputParser;
parser.FunctionName = 'viz.plotSimulationSummary';
addParameter(parser, 'Visible', true);
parse(parser, varargin{:});
if ~(islogical(parser.Results.Visible) && isscalar(parser.Results.Visible))
    error('viz:plotSimulationSummary:InvalidOption', ...
        'Visible must be a logical scalar.');
end

visibility = 'off';
if parser.Results.Visible
    visibility = 'on';
end
figureHandle = figure('Name', 'Simulation summary', 'Color', 'w', ...
    'Visible', visibility);
% Separate world motion from the sampled-boundary diagnostic time series.
worldAxes = subplot(1, 2, 1, 'Parent', figureHandle);
metricAxes = subplot(1, 2, 2, 'Parent', figureHandle);
plotWorldSummary(worldAxes, result);
plotMetricSummary(metricAxes, result);

handles = struct('Figure', figureHandle, 'WorldAxes', worldAxes, ...
    'MetricAxes', metricAxes);
end

function plotWorldSummary(ax, result)
%PLOTWORLDSUMMARY Draw static geometry beneath both agent trajectories.

hold(ax, 'on');
drawStaticWorld(ax, result.Config);
plot(ax, result.ObserverPose(:, 1), result.ObserverPose(:, 2), '-', ...
    'Color', [0.05, 0.25, 0.70], 'LineWidth', 1.5, ...
    'DisplayName', 'Observer trajectory');
plot(ax, result.FollowerPose(:, 1), result.FollowerPose(:, 2), '-', ...
    'Color', [0.85, 0.20, 0.10], 'LineWidth', 1.5, ...
    'DisplayName', 'Follower trajectory');
plot(ax, result.ObserverPose(end, 1), result.ObserverPose(end, 2), 'o', ...
    'Color', [0.05, 0.25, 0.70], 'MarkerFaceColor', [0.05, 0.25, 0.70], ...
    'DisplayName', 'Observer final');
plot(ax, result.FollowerPose(end, 1), result.FollowerPose(end, 2), 'o', ...
    'Color', [0.85, 0.20, 0.10], 'MarkerFaceColor', [0.85, 0.20, 0.10], ...
    'DisplayName', 'Follower final');
finishWorldAxes(ax, result.Config, 'Simulation trajectories');
end

function plotMetricSummary(ax, result)
%PLOTMETRICSUMMARY Show invalid-boundary samples explicitly at zero.

hold(ax, 'on');
plot(ax, result.Time, result.Metrics.SampledPolygonSignedDistance, ...
    'LineWidth', 1.4, 'Color', [0.10, 0.45, 0.75], ...
    'DisplayName', 'Follower sampled-polygon signed distance');
yline(ax, 0, '--', 'Color', [0.25, 0.25, 0.25], ...
    'HandleVisibility', 'off');
invalid = ~result.Metrics.IsSampledPolygonValid;
if any(invalid)
    plot(ax, result.Time(invalid), zeros(sum(invalid), 1), 'x', ...
        'Color', [0.85, 0.15, 0.10], 'DisplayName', 'Invalid sampled polygon');
end
grid(ax, 'on');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Distance (m)');
title(ax, 'Sampled-polygon visibility approximation');
legend(ax, 'show', 'Location', 'best');
end

function drawStaticWorld(ax, config)
%DRAWSTATICWORLD Render the scenario geometry shared by all replay samples.

for index = 1:numel(config.Obstacles)
    vertices = config.Obstacles(index).Vertices;
    patch(ax, vertices(:, 1), vertices(:, 2), [0.25, 0.25, 0.25], ...
        'FaceAlpha', 0.9, 'EdgeColor', [0.05, 0.05, 0.05], ...
        'DisplayName', char(config.Obstacles(index).Name));
end
plot(ax, config.Base.Position(1), config.Base.Position(2), 's', ...
    'Color', [0.10, 0.50, 0.20], 'MarkerFaceColor', [0.10, 0.50, 0.20], ...
    'DisplayName', 'Base');
end

function finishWorldAxes(ax, config, titleText)
axis(ax, 'equal');
xlim(ax, config.Playback.Bounds(1:2));
ylim(ax, config.Playback.Bounds(3:4));
grid(ax, 'on');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
title(ax, titleText);
legend(ax, 'show', 'Location', 'best');
end

function validateResult(result)
if ~isstruct(result) || ~all(isfield(result, ...
        {'Config', 'Time', 'ObserverPose', 'FollowerPose', 'Metrics'}))
    error('viz:plotSimulationSummary:InvalidResult', ...
        'result must be returned by simulation.runScenario.');
end
end
