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
% Separate world motion from the visibility time series.
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
plot(ax, result.LeaderPose(:, 1), result.LeaderPose(:, 2), '-', ...
    'Color', [0.85, 0.20, 0.10], 'LineWidth', 1.5, ...
    'DisplayName', 'Leader trajectory');
plot(ax, result.ObserverPose(end, 1), result.ObserverPose(end, 2), 'o', ...
    'Color', [0.05, 0.25, 0.70], 'MarkerFaceColor', [0.05, 0.25, 0.70], ...
    'DisplayName', 'Observer final');
plot(ax, result.LeaderPose(end, 1), result.LeaderPose(end, 2), 'o', ...
    'Color', [0.85, 0.20, 0.10], 'MarkerFaceColor', [0.85, 0.20, 0.10], ...
    'DisplayName', 'Leader final');
finishWorldAxes(ax, result.Config, 'Simulation trajectories');
end

function plotMetricSummary(ax, result)
%PLOTMETRICSUMMARY Barrier h over time, with the true-visibility losses shaded.
% h comes from the CBF policy's diagnostics (absent for reference policies);
% the shading is the oracle scoring metric Metrics.TrueVisible.

hold(ax, 'on');
hasBarrier = isfield(result, 'PolicyDiagnostics') && ~isempty(result.PolicyDiagnostics) ...
    && isfield(result.PolicyDiagnostics{1}, 'BarrierValue');
if hasBarrier
    diagnostics = result.PolicyDiagnostics;
    barrier = cellfun(@(diagnostic) diagnostic.BarrierValue, diagnostics);
    plot(ax, result.Time(1:end-1), barrier, 'LineWidth', 1.4, ...
        'Color', [0.10, 0.45, 0.75], 'DisplayName', 'Barrier h');
    yline(ax, 0, '--', 'Color', [0.25, 0.25, 0.25], 'HandleVisibility', 'off');
end

% Shade every interval in which the leader is truly hidden.
hidden = [false; result.Metrics.TrueVisible(:) == 0; false];
starts = find(diff(hidden) == 1);
stops = find(diff(hidden) == -1) - 1;
for k = 1:numel(starts)
    region = xregion(ax, result.Time(starts(k)), result.Time(stops(k)), ...
        'FaceColor', [0.85, 0.20, 0.10], 'FaceAlpha', 0.2, ...
        'DisplayName', 'Leader truly hidden');
    if k > 1
        region.HandleVisibility = 'off';
    end
end

grid(ax, 'on');
xlabel(ax, 'Time (s)');
ylabel(ax, 'h (m)');
title(ax, sprintf('Visibility: leader truly visible %.0f%% of the time', ...
    100 * mean(result.Metrics.TrueVisible)));
legend(ax, 'show', 'Location', 'southoutside', 'NumColumns', 2);
end

function drawStaticWorld(ax, config)
%DRAWSTATICWORLD Render the scenario geometry shared by all replay samples.

% One legend entry for all obstacles, however many there are.
for index = 1:numel(config.Obstacles)
    vertices = config.Obstacles(index).Vertices;
    patch(ax, vertices(:, 1), vertices(:, 2), [0.25, 0.25, 0.25], ...
        'FaceAlpha', 0.9, 'EdgeColor', [0.05, 0.05, 0.05], ...
        'DisplayName', 'Obstacles', ...
        'HandleVisibility', visibilityValue(index == 1));
end
end

function finishWorldAxes(ax, config, titleText)
axis(ax, 'equal');
xlim(ax, config.Playback.Bounds(1:2));
ylim(ax, config.Playback.Bounds(3:4));
grid(ax, 'on');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
title(ax, titleText);
legend(ax, 'show', 'Location', 'southoutside', 'NumColumns', 2);
end

function validateResult(result)
if ~isstruct(result) || ~all(isfield(result, ...
        {'Config', 'Time', 'ObserverPose', 'LeaderPose', 'Metrics'}))
    error('viz:plotSimulationSummary:InvalidResult', ...
        'result must be returned by simulation.runScenario.');
end
end

function value = visibilityValue(isVisible)
value = 'off';
if isVisible
    value = 'on';
end
end
