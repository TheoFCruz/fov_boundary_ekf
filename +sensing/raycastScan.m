function [scan, rawCast] = raycastScan(pose, fovSpec, obstacles, sensorConfig, time, varargin)
%RAYCASTSCAN Cast an observer scan and expose only measurement information.
%
%   A positive sensorConfig.RangeNoiseStd requires a caller-provided local
%   RandStream. Noise is applied only to first returns and clipped to the
%   interval [0, MaxRange]; capped no-return samples remain unchanged.

if ~(isnumeric(pose) && isreal(pose) && numel(pose) == 3 && ...
        all(isfinite(pose(:))))
    error('sensing:raycastScan:InvalidPose', ...
        'pose must contain three finite real numeric values.');
end
if ~isa(fovSpec, 'fov.FovSpec')
    error('sensing:raycastScan:InvalidFov', ...
        'fovSpec must be an fov.FovSpec object.');
end
if ~isstruct(sensorConfig) || ~isfield(sensorConfig, 'NumRays')
    error('sensing:raycastScan:InvalidSensor', ...
        'sensorConfig must contain NumRays.');
end
if ~(isnumeric(time) && isscalar(time) && isreal(time) && isfinite(time))
    error('sensing:raycastScan:InvalidTime', ...
        'time must be a finite real scalar.');
end
if numel(varargin) > 1
    error('sensing:raycastScan:InvalidArguments', ...
        'raycastScan accepts at most one random-stream argument.');
end

% Range noise belongs to this measurement adapter, never the oracle ray cast.
rangeNoiseStd = getRangeNoiseStd(sensorConfig);
randomStream = [];
if ~isempty(varargin)
    randomStream = varargin{1};
    if ~isa(randomStream, 'RandStream')
        error('sensing:raycastScan:InvalidRandomStream', ...
            'randomStream must be a RandStream object.');
    end
elseif rangeNoiseStd > 0
    error('sensing:raycastScan:MissingRandomStream', ...
        'Positive RangeNoiseStd requires a caller-provided RandStream.');
end

% Rebuild the observer snapshot so sensing depends only on the supplied pose.
pose = reshape(double(pose), 1, 3);
observer = fov.Observer('Position', pose(1:2), 'Heading', pose(3), ...
    'Fov', fovSpec);
castOptions = {'NumRays', sensorConfig.NumRays};
if isfield(sensorConfig, 'Tolerance')
    castOptions = [castOptions, {'Tolerance', sensorConfig.Tolerance}];
end
% The raw cast remains oracle data for simulation and replay only.
rawCast = fov.castRays(observer, obstacles, castOptions{:});

% Expose the measurement contract without copying obstacle geometry.
scan = struct();
scan.Time = double(time);
scan.ObserverPose = pose;
scan.Angles = rawCast.RayAngles - pose(3);
scan.Ranges = rawCast.Distances;
scan.HasReturn = rawCast.IsOccluded;
if rangeNoiseStd > 0
    % Capped no-return rays remain censored range-cap observations.
    returnCount = sum(scan.HasReturn);
    noisyReturns = scan.Ranges(scan.HasReturn) + ...
        rangeNoiseStd * randn(randomStream, returnCount, 1);
    scan.Ranges(scan.HasReturn) = min(max(noisyReturns, 0), rawCast.MaxRange);
end
scan.IsValid = true(rawCast.NumRays, 1);
scan.MaxRange = rawCast.MaxRange;
scan.OpeningAngle = rawCast.OpeningAngle;
end

function rangeNoiseStd = getRangeNoiseStd(sensorConfig)
%GETRANGENOISESTD Read the optional measurement-noise setting.

rangeNoiseStd = 0;
if isfield(sensorConfig, 'RangeNoiseStd')
    rangeNoiseStd = sensorConfig.RangeNoiseStd;
end
if ~(isnumeric(rangeNoiseStd) && isscalar(rangeNoiseStd) && ...
        isreal(rangeNoiseStd) && isfinite(rangeNoiseStd) && rangeNoiseStd >= 0)
    error('sensing:raycastScan:InvalidSensor', ...
        'sensorConfig.RangeNoiseStd must be a finite nonnegative scalar.');
end
rangeNoiseStd = double(rangeNoiseStd);
end
