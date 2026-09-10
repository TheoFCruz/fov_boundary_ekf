function scenario = validateScenario(scenario)
%VALIDATESCENARIO Validate checkpoint-one simulation configuration.

if ~isstruct(scenario)
    error('simulation:validateScenario:InvalidScenario', ...
        'scenario must be a structure.');
end
required = {'Name', 'Time', 'Observer', 'Follower', 'Base', 'Obstacles', ...
    'Sensor', 'Estimator', 'Playback'};
if ~all(isfield(scenario, required))
    error('simulation:validateScenario:MissingField', ...
        'scenario is missing one or more required checkpoint-one fields.');
end

validateTime(scenario.Time);
validateAgent(scenario.Observer, true, 'Observer');
validateAgent(scenario.Follower, false, 'Follower');
validatePosition(scenario.Base, 'Position', 'Base');
validateObstacles(scenario.Obstacles);
validateSensor(scenario.Sensor);
validateEstimator(scenario.Estimator);
validatePlayback(scenario.Playback);

startTime = double(scenario.Time.Start);
step = double(scenario.Time.Step);
stopTime = double(scenario.Time.Stop);
validateReference(scenario.Observer.Reference, startTime, stopTime, step, ...
    'Observer');
validateReference(scenario.Follower.Reference, startTime, stopTime, step, ...
    'Follower');
end

function validateTime(time)
if ~isstruct(time) || ~all(isfield(time, {'Start', 'Stop', 'Step'})) || ...
        ~all(cellfun(@(name) isFiniteScalar(time.(name)), {'Start', 'Stop', 'Step'})) || ...
        time.Stop <= time.Start || time.Step <= 0
    error('simulation:validateScenario:InvalidTime', ...
        'Time must contain finite Start, Stop, and positive Step values.');
end
intervals = (double(time.Stop) - double(time.Start)) / double(time.Step);
if abs(intervals - round(intervals)) > alignmentTolerance(intervals)
    error('simulation:validateScenario:UnalignedTime', ...
        '(Stop-Start)/Step must be integral within floating-point tolerance.');
end
end

function validateAgent(agent, requiresFov, name)
required = {'InitialPose', 'Reference'};
if requiresFov
    required = [required, {'Fov', 'Policy'}];
end
if ~isstruct(agent) || ~all(isfield(agent, required))
    error('simulation:validateScenario:InvalidAgent', ...
        '%s configuration is incomplete.', name);
end
validatePose(agent.InitialPose, [name, '.InitialPose']);
if requiresFov
    if ~isa(agent.Fov, 'fov.FovSpec')
        error('simulation:validateScenario:InvalidFov', ...
            'Observer.Fov must be an fov.FovSpec object.');
    end
    if ~isa(agent.Policy, 'function_handle')
        error('simulation:validateScenario:InvalidPolicy', ...
            'Observer.Policy must be a function handle.');
    end
end
end

function validatePosition(container, field, name)
if ~isstruct(container) || ~isfield(container, field)
    error('simulation:validateScenario:InvalidPosition', ...
        '%s.%s is required.', name, field);
end
value = container.(field);
if ~(isnumeric(value) && isreal(value) && numel(value) == 2 && ...
        all(isfinite(value(:))))
    error('simulation:validateScenario:InvalidPosition', ...
        '%s.%s must contain two finite real numeric values.', name, field);
end
end

function validateObstacles(obstacles)
if ~isempty(obstacles) && (~isstruct(obstacles) || ~isfield(obstacles, 'Vertices'))
    error('simulation:validateScenario:InvalidObstacles', ...
        'Obstacles must be an empty or polygon-obstacle struct array.');
end
end

function validateSensor(sensor)
if ~isstruct(sensor) || ~isfield(sensor, 'NumRays') || ...
        ~(isnumeric(sensor.NumRays) && isscalar(sensor.NumRays) && ...
        isreal(sensor.NumRays) && isfinite(sensor.NumRays) && ...
        sensor.NumRays >= 2 && floor(sensor.NumRays) == sensor.NumRays)
    error('simulation:validateScenario:InvalidSensor', ...
        'Sensor.NumRays must be an integer scalar greater than or equal to 2.');
end
if isfield(sensor, 'Tolerance') && ...
        ~(isnumeric(sensor.Tolerance) && isscalar(sensor.Tolerance) && ...
        isreal(sensor.Tolerance) && isfinite(sensor.Tolerance) && sensor.Tolerance > 0)
    error('simulation:validateScenario:InvalidSensor', ...
        'Sensor.Tolerance must be a finite positive scalar.');
end
if isfield(sensor, 'RangeNoiseStd') && ...
        ~(isnumeric(sensor.RangeNoiseStd) && isscalar(sensor.RangeNoiseStd) && ...
        isreal(sensor.RangeNoiseStd) && isfinite(sensor.RangeNoiseStd) && ...
        sensor.RangeNoiseStd >= 0)
    error('simulation:validateScenario:InvalidSensor', ...
        'Sensor.RangeNoiseStd must be a finite nonnegative scalar.');
end
if isfield(sensor, 'Seed') && ...
        ~(isnumeric(sensor.Seed) && isscalar(sensor.Seed) && isreal(sensor.Seed) && ...
        isfinite(sensor.Seed) && sensor.Seed >= 0 && sensor.Seed <= 2^32 - 1 && ...
        floor(sensor.Seed) == sensor.Seed)
    error('simulation:validateScenario:InvalidSensor', ...
        'Sensor.Seed must be a nonnegative integer scalar less than 2^32.');
end
end

function validateEstimator(estimator)
if ~isstruct(estimator) || ~all(isfield(estimator, ...
        {'Initialize', 'Predict', 'Correct'})) || ...
        ~isa(estimator.Initialize, 'function_handle') || ...
        ~isa(estimator.Predict, 'function_handle') || ...
        ~isa(estimator.Correct, 'function_handle')
    error('simulation:validateScenario:InvalidEstimator', ...
        'Estimator must provide Initialize, Predict, and Correct callbacks.');
end
end

function validatePlayback(playback)
if ~isstruct(playback) || ~all(isfield(playback, ...
        {'Bounds', 'FrameRate', 'Speed'})) || ...
        ~(isnumeric(playback.Bounds) && isreal(playback.Bounds) && ...
        numel(playback.Bounds) == 4 && all(isfinite(playback.Bounds(:))) && ...
        playback.Bounds(1) < playback.Bounds(2) && ...
        playback.Bounds(3) < playback.Bounds(4)) || ...
        ~isFiniteScalar(playback.FrameRate) || playback.FrameRate <= 0 || ...
        ~isFiniteScalar(playback.Speed) || playback.Speed <= 0
    error('simulation:validateScenario:InvalidPlayback', ...
        'Playback must provide valid Bounds, positive FrameRate, and positive Speed.');
end
end

function validateReference(reference, startTime, stopTime, step, name)
if ~isstruct(reference) || ~all(isfield(reference, {'Times', 'Values'}))
    error('simulation:validateScenario:InvalidReference', ...
        '%s.Reference must contain Times and Values.', name);
end
times = reference.Times;
values = reference.Values;
if ~(isnumeric(times) && isreal(times) && isvector(times) && ~isempty(times) && ...
        all(isfinite(times(:))) && isnumeric(values) && isreal(values) && ...
        size(values, 1) == numel(times) && size(values, 2) == 3 && ...
        all(isfinite(values(:))) && all(diff(times(:)) > 0))
    error('simulation:validateScenario:InvalidReference', ...
        '%s.Reference must have increasing finite Times and N-by-3 Values.', name);
end
times = double(times(:));
if abs(times(1) - startTime) > alignmentTolerance(startTime) || ...
        times(end) > stopTime + alignmentTolerance(stopTime)
    error('simulation:validateScenario:InvalidReference', ...
        '%s.Reference must start at Time.Start and remain within Time.Stop.', name);
end
indices = (times - startTime) / step;
if any(abs(indices - round(indices)) > alignmentTolerance(indices))
    error('simulation:validateScenario:UnalignedReference', ...
        '%s.Reference breakpoints must align with Time.Step.', name);
end
end

function validatePose(value, name)
if ~(isnumeric(value) && isreal(value) && numel(value) == 3 && ...
        all(isfinite(value(:))))
    error('simulation:validateScenario:InvalidPose', ...
        '%s must contain three finite real numeric values.', name);
end
end

function result = isFiniteScalar(value)
result = isnumeric(value) && isscalar(value) && isreal(value) && isfinite(value);
end

function tolerance = alignmentTolerance(value)
tolerance = 128 * eps(max(1, max(abs(double(value(:))))));
end
