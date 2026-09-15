function result = runScenario(scenario)
%RUNSCENARIO Run a deterministic, headless checkpoint-one simulation.
%
%   The runner has no graphics or wall-clock pacing. It logs K+1 synchronized
%   poses, scans, and beliefs for K zero-order-held input intervals.

scenario = simulation.validateScenario(scenario);

% Set the synchronized sample grid and preallocate one log entry per sample.
startTime = double(scenario.Time.Start);
stopTime = double(scenario.Time.Stop);
step = double(scenario.Time.Step);
intervalCount = round((stopTime - startTime) / step);
time = startTime + (0:intervalCount).' * step;
time(end) = stopTime;

observerPose = zeros(intervalCount + 1, 3);
followerPose = zeros(intervalCount + 1, 3);
observerInput = zeros(intervalCount, 3);
followerInput = zeros(intervalCount, 3);
scans = cell(intervalCount + 1, 1);
beliefs = cell(intervalCount + 1, 1);
rawCasts = cell(intervalCount + 1, 1);
policyDiagnostics = cell(intervalCount, 1);

observerPose(1, :) = reshape(double(scenario.Observer.InitialPose), 1, 3);
followerPose(1, :) = reshape(double(scenario.Follower.InitialPose), 1, 3);
estimatorConfig = struct( ...
    'NumRays', double(scenario.Sensor.NumRays), ...
    'MaxRange', scenario.Observer.Fov.MaxRange, ...
    'OpeningAngle', scenario.Observer.Fov.OpeningAngle);
estimatorState = scenario.Estimator.Initialize(estimatorConfig);
% Own scan randomness locally so simulation never mutates MATLAB's global RNG.
randomStream = RandStream('mt19937ar', 'Seed', sensorSeed(scenario.Sensor));

% The initial posterior exists before the first policy decision.
[scans{1}, rawCasts{1}] = sensing.raycastScan(observerPose(1, :), ...
    scenario.Observer.Fov, scenario.Obstacles, scenario.Sensor, time(1), randomStream);
[estimatorState, beliefs{1}] = scenario.Estimator.Correct( ...
    estimatorState, scans{1});
validateBelief(beliefs{1}, scans{1}, observerPose(1, :));

policyState = struct();
for index = 1:intervalCount
    % Policy uses the current posterior and both agents' pre-step snapshot.
    observation = struct( ...
        'Time', time(index), ...
        'Belief', beliefs{index}, ...
        'ObserverPose', observerPose(index, :), ...
        'FollowerPose', followerPose(index, :), ...
        'BasePosition', reshape(double(scenario.Base.Position), 1, 2));
    observerReference = simulation.sampleVelocityReference( ...
        scenario.Observer.Reference, time(index));
    followerReference = simulation.sampleVelocityReference( ...
        scenario.Follower.Reference, time(index));
    observation.FollowerReference = followerReference;
    [appliedObserverInput, policyState, policyDiagnostics{index}] = ...
        scenario.Observer.Policy(observation, observerReference, policyState);
    validateInput(appliedObserverInput, 'observer policy output');
    observerInput(index, :) = reshape(double(appliedObserverInput), 1, 3);
    followerInput(index, :) = followerReference;

    % Both agents advance from the same held-input snapshot.
    previousObserverPose = observerPose(index, :);
    observerPose(index + 1, :) = simulation.stepPose( ...
        previousObserverPose, observerInput(index, :), step);
    followerPose(index + 1, :) = simulation.stepPose( ...
        followerPose(index, :), followerInput(index, :), step);
    motion = struct( ...
        'PreviousObserverPose', previousObserverPose, ...
        'CurrentObserverPose', observerPose(index + 1, :), ...
        'AppliedInput', observerInput(index, :));
    % Transport completed observer motion before correcting with the next scan.
    estimatorState = scenario.Estimator.Predict(estimatorState, motion, step);
    [scans{index + 1}, rawCasts{index + 1}] = sensing.raycastScan( ...
        observerPose(index + 1, :), scenario.Observer.Fov, ...
        scenario.Obstacles, scenario.Sensor, time(index + 1), randomStream);
    [estimatorState, beliefs{index + 1}] = scenario.Estimator.Correct( ...
        estimatorState, scans{index + 1});
    validateBelief(beliefs{index + 1}, scans{index + 1}, ...
        observerPose(index + 1, :));
end

result = struct();
scenario.SchemaVersion = "checkpoint-1";
result.Config = scenario;
result.Time = time;
result.ObserverPose = observerPose;
result.FollowerPose = followerPose;
result.ObserverInput = observerInput;
result.FollowerInput = followerInput;
result.Scans = scans;
result.Beliefs = beliefs;
result.RawCasts = rawCasts;
result.PolicyDiagnostics = policyDiagnostics;
result.Metrics = calculateMetrics(result);
end

function seed = sensorSeed(sensor)
seed = 0;
if isfield(sensor, 'Seed')
    seed = double(sensor.Seed);
end
end

function validateInput(value, name)
if ~(isnumeric(value) && isreal(value) && numel(value) == 3 && ...
        all(isfinite(value(:))))
    error('simulation:runScenario:InvalidPolicyOutput', ...
        '%s must contain three finite real numeric values.', name);
end
end

function validateBelief(belief, scan, pose)
required = {'Time', 'Angles', 'Mean', 'Covariance', 'IsSupported', ...
    'HasReturn', 'ObserverPose', 'MaxRange', 'OpeningAngle', 'Method'};
if ~isstruct(belief) || ~all(isfield(belief, required))
    error('simulation:runScenario:InvalidBelief', ...
        'Estimator Correct must return the checkpoint-one belief fields.');
end
count = numel(scan.Ranges);
if ~(isnumeric(belief.Mean) && isreal(belief.Mean) && iscolumn(belief.Mean) && ...
        numel(belief.Mean) == count && all(isfinite(belief.Mean)) && ...
        all(belief.Mean >= 0) && isnumeric(belief.Angles) && ...
        isreal(belief.Angles) && iscolumn(belief.Angles) && ...
        isequal(belief.Angles, scan.Angles) && ...
        isnumeric(belief.Covariance) && isreal(belief.Covariance) && ...
        isequal(size(belief.Covariance), [count, count]) && ...
        all(isfinite(nonzeros(belief.Covariance))) && ...
        all(diag(belief.Covariance) >= 0) && ...
        isequal(belief.MaxRange, scan.MaxRange) && ...
        isequal(belief.OpeningAngle, scan.OpeningAngle) && ...
        islogical(belief.IsSupported) && iscolumn(belief.IsSupported) && ...
        numel(belief.IsSupported) == count && islogical(belief.HasReturn) && ...
        iscolumn(belief.HasReturn) && numel(belief.HasReturn) == count && ...
        isequal(belief.ObserverPose, pose) && ...
        isequal(double(belief.Time), double(scan.Time)))
    error('simulation:runScenario:InvalidBelief', ...
        'Estimator belief must be synchronized with the current scan and pose.');
end
end

function metricLog = calculateMetrics(result)
sampleCount = numel(result.Time);
signedDistance = nan(sampleCount, 1);
isVisible = false(sampleCount, 1);
isValid = false(sampleCount, 1);
rangeResidual = nan(sampleCount, 1);
basePosition = reshape(double(result.Config.Base.Position), 1, 2);

for index = 1:sampleCount
    belief = result.Beliefs{index};
    scan = result.Scans{index};
    rangeResidual(index) = sqrt(mean((belief.Mean - scan.Ranges) .^ 2));
    % Unsupported wedges require future segmented geometry, not a closed
    % polygon that silently asserts visibility between unsupported samples.
    if ~all(belief.IsSupported)
        continue;
    end
    boundary = fov.boundaryFromRanges(belief.ObserverPose, belief.Angles, ...
        belief.Mean, belief.OpeningAngle);
    try
        signedDistance(index) = metrics.signedEuclideanDistance( ...
            boundary, result.FollowerPose(index, 1:2));
        isVisible(index) = signedDistance(index) <= 0;
        isValid(index) = true;
    catch exception
        invalidGeometry = { ...
            'metrics:internal:normalizeBoundary:DegenerateBoundary', ...
            'metrics:internal:normalizeBoundary:DegenerateEdge', ...
            'metrics:internal:normalizeBoundary:SelfIntersectingBoundary'};
        if ~any(strcmp(exception.identifier, invalidGeometry))
            rethrow(exception);
        end
    end
end

metricLog = struct();
metricLog.SampledPolygonSignedDistance = signedDistance;
metricLog.SampledPolygonVisible = isVisible;
metricLog.IsSampledPolygonValid = isValid;
metricLog.ObserverFollowerDistance = sqrt(sum((result.ObserverPose(:, 1:2) - ...
    result.FollowerPose(:, 1:2)) .^ 2, 2));
metricLog.ObserverBaseDistance = sqrt(sum((result.ObserverPose(:, 1:2) - ...
    basePosition) .^ 2, 2));
metricLog.PassThroughRangeResidual = rangeResidual;
end
