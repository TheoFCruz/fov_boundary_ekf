%RUNBOXINFRONTOFWALLFILTER Run, summarize, and replay gradient source (iii).
%
% boxInFrontOfWall with the boundary EKF (methodology eta = 0.5 baseline)
% feeding the transport predictor (scenarios.withGradientSource, "filter").

run('startup.m');
scenario = scenarios.boxInFrontOfWall();
scenario.Sensor.RangeNoiseStd = 0.01;
scenario.Sensor.Seed = 1;
scenario = scenarios.withGradientSource(scenario, "filter");

result = simulation.runScenario(scenario);
viz.plotSimulationSummary(result);
viz.plotControlDiagnostics(result);

viz.animateSimulation(result, ...
    'ShowRays', true, ...
    'ShowHitPoints', true, ...
    'Speed', 1, ...
    'FrameRate', 30);
