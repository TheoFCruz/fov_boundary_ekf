function [deltaP, deltaPsi] = relativeMotion(fromPose, toPose)
%RELATIVEMOTION Known observer motion expressed in the old sensor frame.
%
%   [deltaP, deltaPsi] = estimation.transport.relativeMotion(fromPose, toPose)
%
% Poses are world [x, y, yaw]. deltaP (2-by-1) is the translation from
% fromPose to toPose rotated into fromPose's sensor frame,
% deltaP = R(-psi_old) * (p_new - p_old); deltaPsi = psi_new - psi_old
% (methodology sec. 1.1).

oldHeading = fromPose(3);
deltaPsi = toPose(3) - oldHeading;
worldDisplacement = reshape(toPose(1:2) - fromPose(1:2), 2, 1);
deltaP = [cos(oldHeading), sin(oldHeading); -sin(oldHeading), cos(oldHeading)] ...
    * worldDisplacement;
end
