%CHECKFILTERCONSISTENCY Report the boundary EKF's NEES/NIS/RMSE per label.
%
% Runs the boundary EKF (no controller, @control.referencePolicy) over
% several seeds for (a) openRoom with its scripted rotate-then-translate
% reference at RangeNoiseStd 0.01 and 0.02, and (b) boxInFrontOfWall with a
% scripted sideways-sliding observer reference (so disocclusions happen) at
% RangeNoiseStd 0.01. Prints metrics.filterConsistency's per-label table for
% each case (sec. 8.1), then a quick Q check: the three process-noise
% coefficients (q0, qv, qw) scaled by a common factor, with mean NEES per
% factor (NEES near 1 is consistent; below 1 means Q is too large).

run('startup.m');
seeds = 1:10;
qScales = [1, 0.3, 0.1, 0.03, 0.01];   % factors on q0, qv, qw for the quick Q check
qCheckSeeds = 1:5;

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

% Quick Q check: rebuild each case's filter with Q scaled, no controller.
fprintf('\n=== Quick Q check: mean NEES (overall / interpolated beams) ===\n');
fprintf('%-52s', 'case');
fprintf('  x%-12g', qScales);
fprintf('\n');
for caseIndex = 1:numel(cases)
    fprintf('%-52s', cases(caseIndex).Name);
    for qScale = qScales
        scenario = cases(caseIndex).Scenario;
        scenario.Estimator = estimation.makeBoundaryEkf( ...
            scaledParams(scenario.Sensor.RangeNoiseStd, qScale));
        results = cell(1, numel(qCheckSeeds));
        for seedIndex = 1:numel(qCheckSeeds)
            seededScenario = scenario;
            seededScenario.Sensor.Seed = qCheckSeeds(seedIndex);
            results{seedIndex} = simulation.runScenario(seededScenario);
        end
        report = metrics.filterConsistency(results);
        blend = report.PerLabel(report.PerLabel.Label == "blend", :);
        fprintf('  %5.2f / %5.2f ', report.Overall.MeanNEES, blend.MeanNEES);
    end
    fprintf('\n');
end

function params = scaledParams(rangeNoiseStd, qScale)
%SCALEDPARAMS Default filter parameters with q0, qv, qw scaled by qScale.

params = estimation.defaultTransportParams();
params.MeasurementNoiseStd = rangeNoiseStd;
params.ProcessNoiseFloor = qScale * params.ProcessNoiseFloor;
params.TranslationNoise = qScale * params.TranslationNoise;
params.RotationNoise = qScale * params.RotationNoise;
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
