function nextPose = stepPose(pose, bodyVelocity, dt)
%STEPPOSE Advance a world pose using body-frame forward Euler kinematics.
%
%   The runner supplies validated finite row vectors and a positive step.

pose = reshape(pose, 1, 3);
bodyVelocity = reshape(bodyVelocity, 1, 3);

% Rotate planar body velocity into world coordinates; yaw deliberately remains unwrapped.
heading = pose(3);
worldVelocity = [cos(heading), -sin(heading); sin(heading), cos(heading)] * ...
    bodyVelocity(1:2).';
nextPose = [pose(1:2) + dt * worldVelocity.', ...
    heading + dt * bodyVelocity(3)];
end
