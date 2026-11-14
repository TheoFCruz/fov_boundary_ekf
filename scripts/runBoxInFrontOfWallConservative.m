%RUNBOXINFRONTOFWALLCONSERVATIVE Run, summarize, and replay gradient source (iv).
%
% boxInFrontOfWall with the boundary EKF at the conservative free-space
% hypothesis (eta = 0) feeding the transport predictor
% (scenarios.withGradientSource, "filter-conservative").

run('startup.m');
scenario = scenarios.boxInFrontOfWall();
scenario.Sensor.RangeNoiseStd = 0.01;
scenario.Sensor.Seed = 1;
scenario = scenarios.withGradientSource(scenario, "filter-conservative");

result = simulation.runScenario(scenario);
viz.plotSimulationSummary(result);
viz.plotControlDiagnostics(result);

viz.animateSimulation(result, ...
    'ShowRays', true, ...
    'ShowHitPoints', true, ...
    'Speed', 1, ...
    'FrameRate', 30);
