%CHECKFILTERCONSISTENCY Report the boundary EKF's NEES/NIS/RMSE per label.
%
% Runs the boundary EKF (no controller, @control.referencePolicy) over
% several seeds for (a) openRoom with its scripted rotate-then-translate
% reference at RangeNoiseStd 0.01 and 0.02, and (b) boxInFrontOfWall with a
% scripted sideways-sliding observer reference (so disocclusions happen) at
% RangeNoiseStd 0.01. Prints metrics.filterConsistency's per-label table for
% each case (sec. 8.1). This only measures and reports consistency; it does
% not tune the process-noise parameters.

run('startup.m');
seeds = 1:10;

cases = struct('Name', {}, 'Scenario', {});
cases(end + 1) = struct('Name', 'openRoom, RangeNoiseStd 0.01', ...
    'Scenario', openRoomCase(0.01));
cases(end + 1) = struct('Name', 'openRoom, RangeNoiseStd 0.02', ...
    'Scenario', openRoomCase(0.02));
cases(end + 1) = struct('Name', 'boxInFrontOfWall sideways slide, RangeNoiseStd 0.01', ...
    'Scenario', boxSlideCase(0.01));

for caseIndex = 1:numel(cases)
    fprintf('\n=== %s ===\n', cases(caseIndex).Name);
    scenario = cases(caseIndex).Scenario;
    results = cell(1, numel(seeds));
    for seedIndex = 1:numel(seeds)
        seededScenario = scenario;
        seededScenario.Sensor.Seed = seeds(seedIndex);
        results{seedIndex} = simulation.runScenario(seededScenario);
    end
    report = metrics.filterConsistency(results);
    metrics.printFilterConsistency(report);
end

function scenario = openRoomCase(rangeNoiseStd)
%OPENROOMCASE The plan's S1 room with its own scripted reference, no controller.

scenario = scenarios.openRoom();
scenario.Sensor.RangeNoiseStd = rangeNoiseStd;
params = estimation.defaultTransportParams();
params.MeasurementNoiseStd = rangeNoiseStd;
scenario.Estimator = estimation.makeBoundaryEkf(params);
end

function scenario = boxSlideCase(rangeNoiseStd)
%BOXSLIDECASE boxInFrontOfWall with a scripted sideways slide, no controller.
%
% Replaces the default oracle-boundary CBF-QP policy with the no-control
% reference policy, so the observer's sideways motion is scripted and
% repeatable across seeds while still crossing the box's shadow.

scenario = scenarios.boxInFrontOfWall();
scenario.Observer.Reference = struct('Times', 0, 'Values', [0, 0.08, 0]);
scenario.Observer.Policy = @control.referencePolicy;
scenario.Sensor.RangeNoiseStd = rangeNoiseStd;
params = estimation.defaultTransportParams();
params.MeasurementNoiseStd = rangeNoiseStd;
scenario.Estimator = estimation.makeBoundaryEkf(params);
end
