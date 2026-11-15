function handles = plotGradientComparison(series, varargin)
%PLOTGRADIENTCOMPARISON Plot gradient-angle and true visibility for several runs.
%
%   handles = viz.plotGradientComparison(series)
%   handles = viz.plotGradientComparison(series, 'Visible', false)
%
% series is a struct array, one entry per run, with Time (K-by-1, from
% result.Time(1:end-1)), PositionAngleDeg (control.compareGradients' angle
% to the oracle gradient, K-by-1) and TrueVisible (result.Metrics.TrueVisible
% (1:end-1), K-by-1), plus a Name for the legend. Top panel: the gradient
% angle over time, one line per run. Bottom panel: true visibility over
% time, offset per run so overlapping stretches stay readable. Replay-only
% diagnostic: reads nothing but its input, never affects simulation state.

parser = inputParser;
parser.FunctionName = 'viz.plotGradientComparison';
addParameter(parser, 'Visible', true);
parse(parser, varargin{:});
if ~(islogical(parser.Results.Visible) && isscalar(parser.Results.Visible))
    error('viz:plotGradientComparison:InvalidOption', ...
        'Visible must be a logical scalar.');
end

visibility = 'off';
if parser.Results.Visible
    visibility = 'on';
end
figureHandle = figure('Name', 'Gradient source comparison', 'Color', 'w', ...
    'Visible', visibility);

anglesAxes = subplot(2, 1, 1, 'Parent', figureHandle);
hold(anglesAxes, 'on');
visibleAxes = subplot(2, 1, 2, 'Parent', figureHandle);
hold(visibleAxes, 'on');

colors = lines(numel(series));
for i = 1:numel(series)
    plot(anglesAxes, series(i).Time, series(i).PositionAngleDeg, '-', ...
        'Color', colors(i, :), 'DisplayName', series(i).Name);
    plot(visibleAxes, series(i).Time, double(series(i).TrueVisible) - 0.05 * (i - 1), ...
        '-', 'Color', colors(i, :), 'DisplayName', series(i).Name);
end

ylabel(anglesAxes, 'Angle to oracle gradient (deg)');
legend(anglesAxes, 'Location', 'best');
grid(anglesAxes, 'on');
hold(anglesAxes, 'off');

xlabel(visibleAxes, 'Time (s)');
ylabel(visibleAxes, 'True visible');
ylim(visibleAxes, [-0.1, 1.1]);
legend(visibleAxes, 'Location', 'best');
grid(visibleAxes, 'on');
hold(visibleAxes, 'off');

handles = struct('Figure', figureHandle, 'AnglesAxes', anglesAxes, ...
    'VisibleAxes', visibleAxes);
end
