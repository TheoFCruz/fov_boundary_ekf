%RUNMOVINGPAIR Run, summarize, and replay the checkpoint-one example.

run('startup.m');
scenario = scenarios.controlTest();
% Tune a deliberately difficult CBF visibility experiment, not a safety demo.
policyConfig = struct('DistanceMargin', 1.0, ...
    'CbfRate', 2, ...
    'PoseFiniteDifferenceStep', [1e-3; 1e-3; 1e-5]);
scenario.Observer.Policy = control.makeSignedDistanceCbfPolicy(policyConfig);
scenario.Sensor.RangeNoiseStd = 0.1;
result = simulation.runScenario(scenario);
% The export default selects five replay key frames, including first and last.
frameManifest = viz.saveSimulationFrames(result);
viz.plotSimulationSummary(result);

viz.animateSimulation(result, ...
    'ShowRays', true, ...
    'ShowHitPoints', true, ...
    'Speed', 1, ...
    'FrameRate', 30);
