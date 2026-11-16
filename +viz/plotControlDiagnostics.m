function handles = plotControlDiagnostics(result, varargin)
%PLOTCONTROLDIAGNOSTICS Plot CBF-QP behavior from a completed simulation log.
%
%   handles = viz.plotControlDiagnostics(result)
%   handles = viz.plotControlDiagnostics(result, 'Visible', false)
%
% Barrier h, applied body inputs and QP slack over time, and a printed table
% of QP status counts. handles.Figure is the figure, for export.

parser = inputParser;
parser.FunctionName = 'viz.plotControlDiagnostics';
addParameter(parser, 'Visible', true);
parse(parser, varargin{:});
if ~(islogical(parser.Results.Visible) && isscalar(parser.Results.Visible))
    error('viz:plotControlDiagnostics:InvalidOption', 'Visible must be a logical scalar.');
end
visibility = 'off';
if parser.Results.Visible
    visibility = 'on';
end

time = result.Time(1:end-1);
diagnostics = result.PolicyDiagnostics;
status = string(cellfun(@(diagnostic) char(diagnostic.Status), diagnostics, ...
    'UniformOutput', false)).';
barrier = cellfun(@(diagnostic) diagnostic.BarrierValue, diagnostics).';
slack = cellfun(@(diagnostic) diagnostic.Slack, diagnostics).';

% Barrier, held commands, and relaxation show when the policy intervenes.
figureHandle = figure('Name', 'CBF-QP diagnostics', 'Color', 'w', 'Visible', visibility);
tiledlayout(figureHandle, 3, 1);
nexttile;
plot(time, barrier, 'LineWidth', 1.2);
yline(0, '--r');
ylabel('Barrier h (m)');
grid on;
nexttile;
stairs(time, result.ObserverInput, 'LineWidth', 1.1);
ylabel('Body input');
legend('vxBody', 'vyBody', 'omega', 'Location', 'best');
grid on;
nexttile;
semilogy(time, max(slack, eps), 'LineWidth', 1.2);
ylabel('Slack');
xlabel('Time (s)');
grid on;

[statusNames, ~, group] = unique(status);
disp(table(statusNames(:), accumarray(group(:), 1), ...
    'VariableNames', {'Status', 'Count'}));
handles = struct('Figure', figureHandle);
end
