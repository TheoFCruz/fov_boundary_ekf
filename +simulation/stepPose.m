function nextPose = stepPose(pose, bodyVelocity, dt)
%STEPPOSE Advance a world pose using body-frame forward Euler kinematics.
%
%   nextPose = simulation.stepPose(pose, bodyVelocity, dt) advances the
%   finite row pose [x, y, yaw] using [vxBody, vyBody, omega] over dt.

validatePose(pose, 'pose');
validatePose(bodyVelocity, 'bodyVelocity');
if ~(isnumeric(dt) && isscalar(dt) && isreal(dt) && isfinite(dt) && dt > 0)
    error('simulation:stepPose:InvalidStep', ...
        'dt must be a finite positive scalar.');
end

pose = reshape(double(pose), 1, 3);
bodyVelocity = reshape(double(bodyVelocity), 1, 3);
dt = double(dt);

heading = pose(3);
worldVelocity = [cos(heading), -sin(heading); sin(heading), cos(heading)] * ...
    bodyVelocity(1:2).';
nextPose = [pose(1:2) + dt * worldVelocity.', ...
    heading + dt * bodyVelocity(3)];
end

function validatePose(value, name)
if ~(isnumeric(value) && isreal(value) && numel(value) == 3 && ...
        all(isfinite(value(:))))
    error('simulation:stepPose:InvalidInput', ...
        '%s must contain three finite real numeric values.', name);
end
end
