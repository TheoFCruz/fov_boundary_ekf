%RUNMOVINGPAIR Run, summarize, and replay the checkpoint-one example.

run('startup.m');
scenario = scenarios.movingPair();
result = simulation.runScenario(scenario);
viz.plotSimulationSummary(result);

% Set ShowRays true for full origin-to-endpoint segments.
viz.animateSimulation(result, 'ShowRays', false, 'ShowHitPoints', true);
