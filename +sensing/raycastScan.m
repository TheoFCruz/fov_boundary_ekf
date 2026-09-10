function [scan, rawCast] = raycastScan(pose, fovSpec, obstacles, sensorConfig, time)
%RAYCASTSCAN Cast an observer scan and expose only measurement information.

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

pose = reshape(double(pose), 1, 3);
observer = fov.Observer('Position', pose(1:2), 'Heading', pose(3), ...
    'Fov', fovSpec);
castOptions = {'NumRays', sensorConfig.NumRays};
if isfield(sensorConfig, 'Tolerance')
    castOptions = [castOptions, {'Tolerance', sensorConfig.Tolerance}];
end
rawCast = fov.castRays(observer, obstacles, castOptions{:});

scan = struct();
scan.Time = double(time);
scan.ObserverPose = pose;
scan.Angles = rawCast.RayAngles - pose(3);
scan.Ranges = rawCast.Distances;
scan.HasReturn = rawCast.IsOccluded;
scan.IsValid = true(rawCast.NumRays, 1);
scan.MaxRange = rawCast.MaxRange;
scan.OpeningAngle = rawCast.OpeningAngle;
end
