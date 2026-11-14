function handles = animateComparison(results, names, varargin)
%ANIMATECOMPARISON Replay several completed runs side by side, in sync.
%
%   handles = viz.animateComparison(results, names)
%   handles = viz.animateComparison(results, names, 'Speed', 1, ...
%       'FrameRate', 20, 'Visible', true, 'AutoPlay', true)
%
% results is a cell array of simulation.runScenario results on the same
% time grid (for example one scenario run with different gradient sources);
% names labels each panel. Each world panel shows the obstacles, the true
% visible polygon, both trajectories so far, the observer and leader with
% headings, and the direction of the position part of the policy gradient
% [dh/dx, dh/dy] the run applied. Titles give t, h, the policy status, and
% whether the leader is truly visible. Like viz.animateSimulation this
% replays logs only and always shows the final sample; AutoPlay=false
% returns handles with UpdateFrame(index) for smoke tests.

parser = inputParser;
parser.FunctionName = 'viz.animateComparison';
addParameter(parser, 'Visible', true);
addParameter(parser, 'FrameRate', results{1}.Config.Playback.FrameRate);
addParameter(parser, 'Speed', results{1}.Config.Playback.Speed);
addParameter(parser, 'AutoPlay', true);
parse(parser, varargin{:});
options = parser.Results;
names = string(names);
if ~(iscell(results) && ~isempty(results) && numel(names) == numel(results))
    error('viz:animateComparison:InvalidInput', ...
        'results must be a nonempty cell array with one name per result.');
end
for index = 2:numel(results)
    if ~isequal(results{index}.Time, results{1}.Time)
        error('viz:animateComparison:InvalidInput', ...
            'All results must share the same time grid.');
    end
end
if ~(islogical(options.Visible) && isscalar(options.Visible) ...
        && islogical(options.AutoPlay) && isscalar(options.AutoPlay) ...
        && isPositiveScalar(options.FrameRate) && isPositiveScalar(options.Speed))
    error('viz:animateComparison:InvalidOption', ...
        'Visible and AutoPlay must be logical scalars; FrameRate and Speed positive.');
end

visibility = 'off';
if options.Visible
    visibility = 'on';
end
figureHandle = figure('Name', 'Run comparison', 'Color', 'w', 'Visible', visibility);
panels = cell(1, numel(results));
for index = 1:numel(results)
    worldAxes = subplot(1, numel(results), index, 'Parent', figureHandle);
    panels{index} = createPanel(worldAxes, results{index});
end
handles = struct('Figure', figureHandle, 'Panels', {panels});
handles.UpdateFrame = @(frame) updateAll(panels, results, names, frame);
if ~options.AutoPlay
    return;
end

% Same pacing as viz.animateSimulation: decimate display samples only and
% always include the final logged frame.
time = results{1}.Time;
stride = max(1, round(options.Speed ...
    / (options.FrameRate * results{1}.Config.Time.Step)));
frames = (1:stride:numel(time)).';
if frames(end) ~= numel(time)
    frames(end + 1, 1) = numel(time);
end
for position = 1:numel(frames)
    if ~isgraphics(figureHandle)
        return;
    end
    renderTimer = tic;
    updateAll(panels, results, names, frames(position));
    drawnow;
    if position < numel(frames) && isgraphics(figureHandle)
        delay = (time(frames(position + 1)) - time(frames(position))) / options.Speed;
        remaining = delay - toc(renderTimer);
        if remaining > 0
            pause(remaining);
        end
    end
end
end

function panel = createPanel(worldAxes, result)
% Create persistent artists once; frames only update their data.
hold(worldAxes, 'on');
for index = 1:numel(result.Config.Obstacles)
    vertices = result.Config.Obstacles(index).Vertices;
    patch(worldAxes, vertices(:, 1), vertices(:, 2), [0.25, 0.25, 0.25], ...
        'FaceAlpha', 0.9, 'EdgeColor', [0.05, 0.05, 0.05], 'HandleVisibility', 'off');
end
panel.Visible = patch(worldAxes, nan, nan, [0.20, 0.55, 0.90], ...
    'FaceAlpha', 0.22, 'EdgeColor', [0.05, 0.25, 0.55], ...
    'DisplayName', 'True visible polygon');
panel.ObserverTrajectory = plot(worldAxes, nan, nan, '-', ...
    'Color', [0.05, 0.25, 0.70], 'DisplayName', 'Observer trajectory');
panel.LeaderTrajectory = plot(worldAxes, nan, nan, '-', ...
    'Color', [0.85, 0.20, 0.10], 'DisplayName', 'Leader trajectory');
panel.Observer = plot(worldAxes, nan, nan, 'o', 'Color', [0.05, 0.25, 0.70], ...
    'MarkerFaceColor', [0.05, 0.25, 0.70], 'DisplayName', 'Observer');
panel.Leader = plot(worldAxes, nan, nan, 'o', 'Color', [0.85, 0.20, 0.10], ...
    'MarkerFaceColor', [0.85, 0.20, 0.10], 'DisplayName', 'Leader');
panel.ObserverHeading = quiver(worldAxes, nan, nan, nan, nan, 0, ...
    'Color', [0.05, 0.25, 0.70], 'LineWidth', 1.2, 'HandleVisibility', 'off');
panel.Gradient = quiver(worldAxes, nan, nan, nan, nan, 0, ...
    'Color', [0.10, 0.60, 0.35], 'LineWidth', 1.8, 'MaxHeadSize', 0.8, ...
    'DisplayName', 'Gradient direction (dh/dx, dh/dy)');
axis(worldAxes, 'equal');
xlim(worldAxes, result.Config.Playback.Bounds(1:2));
ylim(worldAxes, result.Config.Playback.Bounds(3:4));
grid(worldAxes, 'on');
xlabel(worldAxes, 'x (m)');
ylabel(worldAxes, 'y (m)');
legend(worldAxes, 'show', 'Location', 'southoutside');
panel.Axes = worldAxes;
end

function updateAll(panels, results, names, frame)
for index = 1:numel(panels)
    updatePanel(panels{index}, results{index}, names(index), frame);
end
end

function updatePanel(panel, result, name, frame)
%UPDATEPANEL Redraw one logged sample of one run.

if ~isgraphics(panel.Axes)
    return;
end
observerPose = result.ObserverPose(frame, :);
leaderPose = result.LeaderPose(frame, :);
raw = result.RawCasts{frame};
set(panel.Visible, 'XData', raw.VisibleBoundary(:, 1), 'YData', raw.VisibleBoundary(:, 2));
set(panel.ObserverTrajectory, 'XData', result.ObserverPose(1:frame, 1), ...
    'YData', result.ObserverPose(1:frame, 2));
set(panel.LeaderTrajectory, 'XData', result.LeaderPose(1:frame, 1), ...
    'YData', result.LeaderPose(1:frame, 2));
set(panel.Observer, 'XData', observerPose(1), 'YData', observerPose(2));
set(panel.Leader, 'XData', leaderPose(1), 'YData', leaderPose(2));
headingLength = 0.15;
set(panel.ObserverHeading, 'XData', observerPose(1), 'YData', observerPose(2), ...
    'UData', headingLength * cos(observerPose(3)), ...
    'VData', headingLength * sin(observerPose(3)));

% The final sample has no interval diagnostics; reuse the last one.
diagnostics = result.PolicyDiagnostics{min(frame, numel(result.PolicyDiagnostics))};
titleText = sprintf('%s, t = %.2f s', name, result.Time(frame));
gradient = [NaN, NaN];
if isstruct(diagnostics) && all(isfield(diagnostics, ...
        {'BarrierValue', 'Status', 'ObserverPoseGradient'}))
    titleText = sprintf('%s\nh = %.3f | %s', titleText, ...
        diagnostics.BarrierValue, diagnostics.Status);
    gradient = diagnostics.ObserverPoseGradient(1:2);
end
if result.Metrics.TrueVisible(frame)
    titleText = sprintf('%s | leader visible', titleText);
else
    titleText = sprintf('%s | leader hidden', titleText);
end
title(panel.Axes, titleText, 'Interpreter', 'none');

% Unit arrow: only the direction is compared across runs.
gradientLength = 0.3;
if all(isfinite(gradient)) && norm(gradient) > 0
    direction = gradientLength * gradient / norm(gradient);
    set(panel.Gradient, 'XData', observerPose(1), 'YData', observerPose(2), ...
        'UData', direction(1), 'VData', direction(2), 'Visible', 'on');
else
    set(panel.Gradient, 'Visible', 'off');
end
end

function result = isPositiveScalar(value)
result = isnumeric(value) && isscalar(value) && isreal(value) ...
    && isfinite(value) && value > 0;
end
