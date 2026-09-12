%PLOTCONTROLDIAGNOSTICS Plot CBF-QP diagnostics from an existing result.
%
% Run scripts/runMovingPair.m first, then run this script in the same workspace.

if ~exist('result', 'var')
    error('scripts:plotControlDiagnostics:MissingResult', ...
        'Run a scenario first so its simulation result is available as result.');
end
viz.plotControlDiagnostics(result);
