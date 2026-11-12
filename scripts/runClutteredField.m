%RUNCLUTTEREDFIELD Run, summarize, and replay the cluttered-field CBF-QP example.

run('startup.m');
% The follower weaves between small posts; the CBF uses the oracle re-cast boundary.
scenario = scenarios.clutteredField();
% scenario.Time.Stop = 8;        % default 32 s; shorten for quick checks
result = simulation.runScenario(scenario);
viz.plotSimulationSummary(result);
viz.plotControlDiagnostics(result);

viz.animateSimulation(result, ...
    'ShowRays', true, ...
    'ShowHitPoints', true, ...
    'Speed', 1, ...
    'FrameRate', 30);
