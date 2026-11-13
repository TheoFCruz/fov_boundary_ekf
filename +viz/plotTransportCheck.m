function handles = plotTransportCheck(prediction, truth, varargin)
%PLOTTRANSPORTCHECK Compare a transported prediction with the truth scan.
%
%   handles = viz.plotTransportCheck(prediction, truth)
%   handles = viz.plotTransportCheck(prediction, truth, 'Visible', false)
%
% prediction is estimation.transportBoundary's output at some pose; truth
% is sensing.raycastScan's noiseless scan at that same pose. Plots true and
% predicted range against relative angle, with predicted beams marked by
% their Label (blend/no-return/jump-gap/new). Replay-only diagnostic, reads
% nothing but its two inputs, and never affects simulation state or logs.

parser = inputParser;
parser.FunctionName = 'viz.plotTransportCheck';
addParameter(parser, 'Visible', true);
parse(parser, varargin{:});
if ~(islogical(parser.Results.Visible) && isscalar(parser.Results.Visible))
    error('viz:plotTransportCheck:InvalidOption', ...
        'Visible must be a logical scalar.');
end

visibility = 'off';
if parser.Results.Visible
    visibility = 'on';
end
figureHandle = figure('Name', 'Transport check', 'Color', 'w', ...
    'Visible', visibility);
axesHandle = axes('Parent', figureHandle);
hold(axesHandle, 'on');

plot(axesHandle, truth.Angles, truth.Ranges, '-', ...
    'Color', [0.6, 0.6, 0.6], 'LineWidth', 1.5, 'DisplayName', 'Truth');

labelNames = ["blend", "no-return", "jump-gap", "new"];
colors = {[0.05, 0.25, 0.70], [0.70, 0.20, 0.20], [0.85, 0.55, 0.10], [0.10, 0.60, 0.30]};
markers = {'o', 'x', '^', 's'};
for i = 1:numel(labelNames)
    mask = prediction.Label == labelNames(i);
    plot(axesHandle, prediction.Angles(mask), prediction.Mean(mask), ...
        markers{i}, 'Color', colors{i}, 'MarkerFaceColor', colors{i}, ...
        'MarkerSize', 4, 'DisplayName', char(labelNames(i)));
end

xlabel(axesHandle, 'Relative beam angle (rad)');
ylabel(axesHandle, 'Range (m)');
title(axesHandle, 'Predicted mean vs. truth');
legend(axesHandle, 'Location', 'best');
grid(axesHandle, 'on');
hold(axesHandle, 'off');

handles = struct('Figure', figureHandle, 'Axes', axesHandle);
end
