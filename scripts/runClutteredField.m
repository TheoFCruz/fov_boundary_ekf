%RUNCLUTTEREDFIELD Run any gradient source on scenarios.clutteredField.
%
% Edit the settings below. source is one of the methodology's gradient
% sources (sec. 1.5): "oracle" (i), "latest" (ii, latest scan + prediction
% model), "filter" (iii, boundary EKF), "filter-conservative" (iv, EKF with
% eta = 0). Everything else is shared by all sources, so runs compare fairly.

run('startup.m');

% --- Settings: source and sensor ---
source = "filter";               % "oracle" | "latest" | "filter" | "filter-conservative"
rangeNoiseStd = 0.05;            % range noise (m); the filter assumes the same
seed = 1;                        % noise seed
stopTime = 32;                   % run length (s), a multiple of 0.025; 8 for quick checks

% --- Settings: controller (CBF-QP) ---
inputWeights = diag([1, 1, 0.25]);   % W: cost of deviating from the nominal input
rateFactor = 6;                  % R = rateFactor * W: cost of changing the input per step
cbfRate = 4;                     % gamma in hdot + gamma*h >= -slack
distanceMargin = 0.1;            % m kept inside the visible set
gradientDirections = "axes+diagonals";   % or "axes" (single-axis samples only)
gradientBeamMultiples = [1, 2];  % gradient sample steps, in beam spacings

% --- Settings: prediction model (sources latest / filter) ---
params = estimation.defaultTransportParams();
params.FreeSpaceFraction = 0.5;  % eta (ignored by "filter-conservative", which uses 0)

% --- Settings: replay ---
replaySpeed = 1;                 % 2 = twice real time

% Scenario: the leader weaves between small posts.
scenario = scenarios.clutteredField();
scenario.Time.Stop = stopTime;
scenario.Sensor.RangeNoiseStd = rangeNoiseStd;
scenario.Sensor.Seed = seed;
policyConfig = struct( ...
    'InputWeights', inputWeights, ...
    'InputRatePenalty', rateFactor * inputWeights, ...
    'CbfRate', cbfRate, ...
    'DistanceMargin', distanceMargin, ...
    'GradientDirections', gradientDirections, ...
    'GradientBeamMultiples', gradientBeamMultiples);
scenario = scenarios.withGradientSource(scenario, source, params, policyConfig);

result = simulation.runScenario(scenario);
viz.plotSimulationSummary(result);
viz.plotControlDiagnostics(result);
viz.animateSimulation(result, ...
    'ShowRays', true, ...
    'ShowHitPoints', true, ...
    'Speed', replaySpeed, ...
    'FrameRate', 30);
