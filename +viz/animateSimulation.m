function handles = animateSimulation(result, varargin)
%ANIMATESIMULATION Replay a completed checkpoint-one simulation result.
%
%   Rendering speed and frame rate select displayed samples only; they never
%   alter the numerical result. The returned handles are useful for smoke tests.
%   handles.UpdateFrame(index) redraws one logged sample without advancing physics.
%   ShowRays draws origin-to-endpoint segments; ShowHitPoints marks only
%   obstacle-intersection endpoints.

validateResult(result);
parser = inputParser;
parser.FunctionName = 'viz.animateSimulation';
addParameter(parser, 'Visible', true);
addParameter(parser, 'FrameRate', result.Config.Playback.FrameRate);
addParameter(parser, 'Speed', result.Config.Playback.Speed);
addParameter(parser, 'ShowRays', true);
addParameter(parser, 'ShowHitPoints', false);
parse(parser, varargin{:});
options = parser.Results;
if ~(islogical(options.Visible) && isscalar(options.Visible) && ...
        isPositiveScalar(options.FrameRate) && isPositiveScalar(options.Speed) && ...
        isLogicalScalar(options.ShowRays) && isLogicalScalar(options.ShowHitPoints))
    error('viz:animateSimulation:InvalidOption', ...
        ['Visible, ShowRays, and ShowHitPoints must be logical scalars; ', ...
        'FrameRate and Speed must be positive scalars.']);
end

visibility = 'off';
if options.Visible
    visibility = 'on';
end
figureHandle = figure('Name', 'Simulation replay', 'Color', 'w', ...
    'Visible', visibility);
worldAxes = subplot(1, 2, 1, 'Parent', figureHandle);
boundaryAxes = subplot(1, 2, 2, 'Parent', figureHandle);
handles = createGraphics(figureHandle, worldAxes, boundaryAxes, result, options);
handles.UpdateFrame = @(index) updateGraphics(handles, result, index);
frameIndices = selectFrames(result.Time, result.Config.Time.Step, ...
    options.FrameRate, options.Speed);

for framePosition = 1:numel(frameIndices)
    frameIndex = frameIndices(framePosition);
    if ~isgraphics(figureHandle)
        return;
    end
    updateGraphics(handles, result, frameIndex);
    drawnow;
    if framePosition < numel(frameIndices) && isgraphics(figureHandle)
        pause((result.Time(frameIndices(framePosition + 1)) - ...
            result.Time(frameIndex)) / options.Speed);
    end
end
end

function handles = createGraphics(figureHandle, worldAxes, boundaryAxes, result, options)
hold(worldAxes, 'on');
for index = 1:numel(result.Config.Obstacles)
    vertices = result.Config.Obstacles(index).Vertices;
    patch(worldAxes, vertices(:, 1), vertices(:, 2), [0.25, 0.25, 0.25], ...
        'FaceAlpha', 0.9, 'EdgeColor', [0.05, 0.05, 0.05], ...
        'HandleVisibility', 'off');
end
base = result.Config.Base.Position;
plot(worldAxes, base(1), base(2), 's', 'Color', [0.10, 0.50, 0.20], ...
    'MarkerFaceColor', [0.10, 0.50, 0.20], 'DisplayName', 'Base');
handles.BaseLink = plot(worldAxes, nan, nan, '--', 'Color', [0.35, 0.55, 0.35], ...
    'DisplayName', 'Observer-base reference');
handles.ObserverTrajectory = plot(worldAxes, nan, nan, '-', ...
    'Color', [0.05, 0.25, 0.70], 'DisplayName', 'Observer trajectory');
handles.FollowerTrajectory = plot(worldAxes, nan, nan, '-', ...
    'Color', [0.85, 0.20, 0.10], 'DisplayName', 'Follower trajectory');
handles.RawBoundary = patch(worldAxes, nan, nan, [0.20, 0.55, 0.90], ...
    'FaceAlpha', 0.22, 'EdgeColor', [0.05, 0.25, 0.55], ...
    'DisplayName', 'Raw sampled boundary');
handles.EstimatedBoundary = plot(worldAxes, nan, nan, '--', ...
    'Color', [0.95, 0.65, 0.10], 'LineWidth', 1.5, ...
    'DisplayName', 'Estimated boundary');
handles.Rays = plot(worldAxes, nan, nan, '-', 'Color', [0.20, 0.40, 0.70], ...
    'LineWidth', 0.4, 'HandleVisibility', 'off', ...
    'Visible', visibilityValue(options.ShowRays));
handles.HitPoints = plot(worldAxes, nan, nan, 'o', ...
    'Color', [0.85, 0.15, 0.10], 'MarkerFaceColor', [0.85, 0.15, 0.10], ...
    'MarkerSize', 4, 'HandleVisibility', 'off', ...
    'Visible', visibilityValue(options.ShowHitPoints));
handles.Observer = plot(worldAxes, nan, nan, 'o', 'Color', [0.05, 0.25, 0.70], ...
    'MarkerFaceColor', [0.05, 0.25, 0.70], 'DisplayName', 'Observer');
handles.Follower = plot(worldAxes, nan, nan, 'o', 'Color', [0.85, 0.20, 0.10], ...
    'MarkerFaceColor', [0.85, 0.20, 0.10], 'DisplayName', 'Follower');
handles.ObserverHeading = quiver(worldAxes, nan, nan, nan, nan, 0.8, ...
    'Color', [0.05, 0.25, 0.70], 'LineWidth', 1.2, 'HandleVisibility', 'off');
handles.FollowerHeading = quiver(worldAxes, nan, nan, nan, nan, 0.8, ...
    'Color', [0.85, 0.20, 0.10], 'LineWidth', 1.2, 'HandleVisibility', 'off');
axis(worldAxes, 'equal');
xlim(worldAxes, result.Config.Playback.Bounds(1:2));
ylim(worldAxes, result.Config.Playback.Bounds(3:4));
grid(worldAxes, 'on');
xlabel(worldAxes, 'x (m)');
ylabel(worldAxes, 'y (m)');
legend(worldAxes, 'show', 'Location', 'best');

hold(boundaryAxes, 'on');
handles.ScanLine = plot(boundaryAxes, nan, nan, '-', 'Color', [0.10, 0.35, 0.80], ...
    'DisplayName', 'Scan range');
handles.StdBand = fill(boundaryAxes, nan, nan, [0.95, 0.70, 0.20], ...
    'FaceAlpha', 0.25, 'EdgeColor', 'none', 'DisplayName', 'Posterior +/- 1 std');
handles.MeanLine = plot(boundaryAxes, nan, nan, '--', 'Color', [0.90, 0.50, 0.05], ...
    'LineWidth', 1.4, 'DisplayName', 'Posterior mean');
handles.BoundaryNote = text(boundaryAxes, 0.02, 0.96, '', ...
    'Units', 'normalized', 'VerticalAlignment', 'top');
grid(boundaryAxes, 'on');
xlabel(boundaryAxes, 'Relative angle (rad)');
ylabel(boundaryAxes, 'Range (m)');
legend(boundaryAxes, 'show', 'Location', 'best');
handles.Figure = figureHandle;
handles.WorldAxes = worldAxes;
handles.BoundaryAxes = boundaryAxes;
end

function updateGraphics(handles, result, index)
if ~isgraphics(handles.Figure)
    return;
end
validateattributes(index, {'numeric'}, {'scalar', 'real', 'finite', ...
    'integer', '>=', 1, '<=', numel(result.Time)});
raw = result.RawCasts{index};
belief = result.Beliefs{index};
scan = result.Scans{index};
estimatedBoundary = fov.boundaryFromRanges(belief.ObserverPose, ...
    belief.Angles, belief.Mean, belief.OpeningAngle);
observerPose = result.ObserverPose(index, :);
followerPose = result.FollowerPose(index, :);
base = result.Config.Base.Position;

set(handles.ObserverTrajectory, 'XData', result.ObserverPose(1:index, 1), ...
    'YData', result.ObserverPose(1:index, 2));
set(handles.FollowerTrajectory, 'XData', result.FollowerPose(1:index, 1), ...
    'YData', result.FollowerPose(1:index, 2));
set(handles.BaseLink, 'XData', [observerPose(1), base(1)], ...
    'YData', [observerPose(2), base(2)]);
set(handles.RawBoundary, 'XData', raw.VisibleBoundary(:, 1), ...
    'YData', raw.VisibleBoundary(:, 2));
set(handles.EstimatedBoundary, 'XData', estimatedBoundary(:, 1), ...
    'YData', estimatedBoundary(:, 2));
% Do not connect unsupported wedges; segmented rendering is later work.
if all(belief.IsSupported)
    set(handles.EstimatedBoundary, 'Visible', 'on');
else
    set(handles.EstimatedBoundary, 'Visible', 'off');
end
rayX = [repmat(raw.Origin(1), numel(raw.Distances), 1), raw.EndPoints(:, 1), ...
    nan(numel(raw.Distances), 1)].';
rayY = [repmat(raw.Origin(2), numel(raw.Distances), 1), raw.EndPoints(:, 2), ...
    nan(numel(raw.Distances), 1)].';
set(handles.Rays, 'XData', rayX(:), 'YData', rayY(:));
hitPoints = raw.EndPoints(raw.IsOccluded, :);
set(handles.HitPoints, 'XData', hitPoints(:, 1), 'YData', hitPoints(:, 2));
set(handles.Observer, 'XData', observerPose(1), 'YData', observerPose(2));
set(handles.Follower, 'XData', followerPose(1), 'YData', followerPose(2));
set(handles.ObserverHeading, 'XData', observerPose(1), 'YData', observerPose(2), ...
    'UData', cos(observerPose(3)), 'VData', sin(observerPose(3)));
set(handles.FollowerHeading, 'XData', followerPose(1), 'YData', followerPose(2), ...
    'UData', cos(followerPose(3)), 'VData', sin(followerPose(3)));
title(handles.WorldAxes, sprintf('World replay, t = %.2f s', result.Time(index)));

std = full(sqrt(diag(belief.Covariance)));
set(handles.ScanLine, 'XData', scan.Angles, 'YData', scan.Ranges);
set(handles.MeanLine, 'XData', belief.Angles, 'YData', belief.Mean);
if any(std > 0)
    set(handles.StdBand, 'XData', [belief.Angles; flipud(belief.Angles)], ...
        'YData', [belief.Mean - std; flipud(belief.Mean + std)], 'Visible', 'on');
    set(handles.BoundaryNote, 'String', 'Pointwise standard deviation; not a visibility guarantee.');
else
    set(handles.StdBand, 'Visible', 'off');
    set(handles.BoundaryNote, 'String', sprintf('%s: zero covariance (no confidence guarantee).', ...
        char(belief.Method)));
end
title(handles.BoundaryAxes, sprintf('Boundary estimate, t = %.2f s', result.Time(index)));
end

function indices = selectFrames(time, step, frameRate, speed)
stride = max(1, round(double(speed) / (double(frameRate) * double(step))));
indices = (1:stride:numel(time)).';
if indices(end) ~= numel(time)
    indices(end + 1, 1) = numel(time);
end
end

function validateResult(result)
if ~isstruct(result) || ~all(isfield(result, ...
        {'Config', 'Time', 'ObserverPose', 'FollowerPose', 'Scans', ...
        'Beliefs', 'RawCasts'}))
    error('viz:animateSimulation:InvalidResult', ...
        'result must be returned by simulation.runScenario.');
end
end

function result = isPositiveScalar(value)
result = isnumeric(value) && isscalar(value) && isreal(value) && ...
    isfinite(value) && value > 0;
end

function result = isLogicalScalar(value)
result = islogical(value) && isscalar(value);
end

function value = visibilityValue(isVisible)
value = 'off';
if isVisible
    value = 'on';
end
end
