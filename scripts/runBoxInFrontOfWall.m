%RUNBOXINFRONTOFWALL Run, summarize, and replay the oracle-boundary occlusion example.

run('startup.m');
% The follower passes behind the box; the CBF uses the oracle re-cast boundary.
scenario = scenarios.boxInFrontOfWall();
result = simulation.runScenario(scenario);
viz.plotSimulationSummary(result);
viz.plotControlDiagnostics(result);

viz.animateSimulation(result, ...
    'ShowRays', true, ...
    'ShowHitPoints', true, ...
    'Speed', 1, ...
    'FrameRate', 30);
