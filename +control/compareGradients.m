function comparison = compareGradients(result, predictor, referencePredictor, policyConfig)
%COMPAREGRADIENTS Offline comparison of two boundary predictors' CBF gradients.
%
%   comparison = control.compareGradients(result, predictor, ...
%       referencePredictor, policyConfig)
%
% For every logged interval k of a simulation.runScenario result, rebuilds
% the observation exactly as the runner does (Time, Belief = result.Beliefs
% {k}, ObserverPose, LeaderPose, BasePosition, LeaderReference =
% result.LeaderInput(k, :)) and the observer's nominal input from
% simulation.sampleVelocityReference(result.Config.Observer.Reference,
% Time), then evaluates control.makeSignedDistanceCbfPolicy(policyConfig
% with BoundaryPredictor = predictor) and the same policy built with
% referencePredictor instead, one step each, from a fresh policy state.
% policyConfig defaults to struct() (no BoundaryPredictor field); both
% policies share every other setting.
%
% This is an offline evaluation, never a replacement for the oracle inside
% a closed loop (decision D7): result is a log already produced by some
% run (oracle, source (ii), or otherwise), and both predictors here are
% queried only against that log's belief and poses, never against truth
% obstacles unless predictor or referencePredictor themselves are oracle
% closures.
%
% comparison has one row per interval (K = numel(result.LeaderInput, 1)):
%
%   PositionAngleDeg    angle (deg) between the [dh/dx, dh/dy] parts of the
%                       two ObserverPoseGradients; NaN if either is
%                       unusable or (near-)zero.
%   HeadingDifference   predictor's dh/dpsi minus referencePredictor's.
%   BarrierValue / ReferenceBarrierValue    h at the current pose, each source.
%   Status / ReferenceStatus                 each policy's diagnostics.Status.

if nargin < 4 || isempty(policyConfig)
    policyConfig = struct();
end
configA = policyConfig;
configA.BoundaryPredictor = predictor;
configB = policyConfig;
configB.BoundaryPredictor = referencePredictor;
policyA = control.makeSignedDistanceCbfPolicy(configA);
policyB = control.makeSignedDistanceCbfPolicy(configB);

intervalCount = size(result.LeaderInput, 1);
basePosition = reshape(double(result.Config.Base.Position), 1, 2);
positionAngleDeg = nan(intervalCount, 1);
headingDifference = nan(intervalCount, 1);
barrierValue = nan(intervalCount, 1);
referenceBarrierValue = nan(intervalCount, 1);
status = strings(intervalCount, 1);
referenceStatus = strings(intervalCount, 1);

for k = 1:intervalCount
    time = result.Time(k);
    observation = struct( ...
        'Time', time, ...
        'Belief', result.Beliefs{k}, ...
        'ObserverPose', result.ObserverPose(k, :), ...
        'LeaderPose', result.LeaderPose(k, :), ...
        'BasePosition', basePosition, ...
        'LeaderReference', result.LeaderInput(k, :));
    nominal = simulation.sampleVelocityReference(result.Config.Observer.Reference, time);

    [~, ~, diagnosticsA] = policyA(observation, nominal, struct());
    [~, ~, diagnosticsB] = policyB(observation, nominal, struct());

    positionAngleDeg(k) = poseGradientAngleDeg( ...
        diagnosticsA.ObserverPoseGradient, diagnosticsB.ObserverPoseGradient);
    headingDifference(k) = diagnosticsA.ObserverPoseGradient(3) ...
        - diagnosticsB.ObserverPoseGradient(3);
    barrierValue(k) = diagnosticsA.BarrierValue;
    referenceBarrierValue(k) = diagnosticsB.BarrierValue;
    status(k) = diagnosticsA.Status;
    referenceStatus(k) = diagnosticsB.Status;
end

comparison = struct();
comparison.PositionAngleDeg = positionAngleDeg;
comparison.HeadingDifference = headingDifference;
comparison.BarrierValue = barrierValue;
comparison.ReferenceBarrierValue = referenceBarrierValue;
comparison.Status = status;
comparison.ReferenceStatus = referenceStatus;
end

function angleDeg = poseGradientAngleDeg(gradientA, gradientB)
%POSEGRADIENTANGLEDEG Angle between two gradients' [dh/dx, dh/dy] parts.

positionA = gradientA(1:2);
positionB = gradientB(1:2);
normProduct = norm(positionA) * norm(positionB);
if ~all(isfinite(positionA)) || ~all(isfinite(positionB)) || normProduct < eps
    angleDeg = NaN;
    return;
end
cosineAngle = min(max(dot(positionA, positionB) / normProduct, -1), 1);
angleDeg = acosd(cosineAngle);
end
