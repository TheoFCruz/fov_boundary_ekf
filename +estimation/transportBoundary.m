function prediction = transportBoundary(belief, pose, params)
%TRANSPORTBOUNDARY Mean-only transport and resampling of the boundary belief.
%
%   prediction = estimation.transportBoundary(belief, pose, params)
%
% Moves the boundary mean belief.Mean, defined on belief.Angles at
% belief.ObserverPose, to pose and resamples it onto the same fixed beam grid
% (docs/metodologia.pdf sec. 1.2, eqs. 1-2, 7-10; docs/
% matlab-implementation-plan.md secs. 4.1-4.7, 4.9-4.10). It has the
% (belief, pose) shape of the CBF policy's BoundaryPredictor, so the
% predictor wrapper calls it with a hypothetical pose and the EKF's Predict
% with the real one.
%
% Each stage is one function in estimation.transport:
%
%   relativeMotion              Delta p, Delta psi in the old sensor frame
%   transportEndpoints          old endpoints into the new frame (eq. 1)
%   classifyPairs                same surface, open pair, or jump (eq. 7)
%   resampleOntoBeams            blend/overlap within surfaces, open space (eq. 2)
%   edgeAndShadowCandidates      edges and new occlusions/disocclusions (eqs. 8-10)
%   applyFreeSpaceHypothesis     free-space blend behind a disocclusion (eq. 11)
%   fillUncoveredBeams           newly explored beams from the nearest old point (sec. 4.7)
%
% prediction has Mean, HasReturn, IsSupported and Label per beam ("blend",
% "overlap", "edge", "disoccluded", "no-return" or "new"), plus ShadowRange
% (s_i) and FarRange (r_far,i), both NaN off disoccluded beams, Angles and
% ObserverPose = pose. IsSupported is false only for "disoccluded" and "new"
% (decision D3): hypothesis beams, not evidence, even though the predicted
% mean covers every beam regardless (plan sec. 4.10).
%
% Not yet handled (later checkpoints): covariance propagation (F, Q).

if belief.OpeningAngle == 2*pi
    error('estimation:transportBoundary:FullFovUnsupported', ...
        ['transportBoundary assumes a partial FoV; a full-circle belief ', ...
         'needs an angle-wrap bracket search not yet implemented.']);
end

angles = belief.Angles(:);
maxRange = belief.MaxRange;
oldMean = belief.Mean(:);
hasReturnOld = belief.HasReturn(:);
% The CBF policy's gradient fit (control.barrierGradient) queries this with
% a displaced-pose column vector; relativeMotion needs both poses the same
% shape as belief.ObserverPose (row), so fix the orientation once here.
pose = reshape(double(pose), 1, 3);

[deltaP, deltaPsi] = estimation.transport.relativeMotion(belief.ObserverPose, pose);
endpoints = estimation.transport.transportEndpoints( ...
    angles, oldMean, full(diag(belief.Covariance)), deltaP, deltaPsi);
pairs = estimation.transport.classifyPairs(endpoints, hasReturnOld, params);
[predictedMean, hasReturn, label] = ...
    estimation.transport.resampleOntoBeams(angles, pairs, maxRange, params);
[predictedMean, hasReturn, label, shadowRange, nearIndex, farIndex] = ...
    estimation.transport.edgeAndShadowCandidates(angles, pairs, endpoints, ...
        oldMean, hasReturnOld, deltaP, deltaPsi, ...
        predictedMean, hasReturn, label, maxRange, params);
[predictedMean, farRange] = estimation.transport.applyFreeSpaceHypothesis( ...
    angles, pairs, endpoints, hasReturnOld, nearIndex, farIndex, ...
    shadowRange, label, predictedMean, maxRange, params);
[predictedMean, hasReturn] = estimation.transport.fillUncoveredBeams( ...
    angles, endpoints, pairs, hasReturnOld, predictedMean, hasReturn, label, maxRange);

% A surface transported beyond the cap is out of range: no return (sec. 4.9).
isBeyondCap = predictedMean > maxRange;
hasReturn(isBeyondCap) = false;
predictedMean = min(max(predictedMean, 0), maxRange);

% Disoccluded and newly explored beams carry a hypothesis, not evidence
% (decision D3; plan sec. 4.10).
isSupported = label ~= "disoccluded" & label ~= "new";

prediction = struct();
prediction.Mean = predictedMean;
prediction.HasReturn = hasReturn;
prediction.IsSupported = isSupported;
prediction.Label = label;
prediction.ShadowRange = shadowRange;
prediction.FarRange = farRange;
prediction.Angles = angles;
prediction.ObserverPose = reshape(double(pose), 1, 3);
end
