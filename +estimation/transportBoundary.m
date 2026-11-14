function prediction = transportBoundary(belief, pose, params)
%TRANSPORTBOUNDARY Mean-only transport and resampling of the boundary belief.
%
%   prediction = estimation.transportBoundary(belief, pose, params)
%
% Moves the boundary mean belief.Mean, defined on belief.Angles at
% belief.ObserverPose, to pose and resamples it onto the same fixed beam grid
% (docs/metodologia.pdf sec. 1.2; docs/matlab-implementation-plan.md
% secs. 4.1-4.3, 4.7, 4.9). It has the (belief, pose) shape of the CBF
% policy's BoundaryPredictor, so the predictor wrapper calls it with a
% hypothetical pose and the EKF's Predict with the real one.
%
% Each stage is one function in estimation.transport:
%
%   relativeMotion       Delta p, Delta psi in the old sensor frame
%   transportEndpoints   old endpoints into the new frame (eq. 1)
%   classifyPairs        same surface, open pair, or jump (eq. 7)
%   resampleOntoBeams    blend within surfaces, keep open space (eq. 2)
%   fillUncoveredBeams   newly explored and jump-gap beams (sec. 4.7)
%
% prediction has Mean, HasReturn and Label per beam ("blend", "no-return",
% "new", and temporarily "jump-gap"), plus Angles and ObserverPose = pose.
%
% Not yet handled (later checkpoints): edges and disocclusion (eqs. 8-11;
% jump gaps use the newly explored fill meanwhile), covariance propagation
% (F, Q), and a full 2*pi field of view.

if belief.OpeningAngle == 2*pi
    error('estimation:transportBoundary:FullFovUnsupported', ...
        ['transportBoundary assumes a partial FoV; a full-circle belief ', ...
         'needs an angle-wrap bracket search not yet implemented.']);
end

angles = belief.Angles(:);
maxRange = belief.MaxRange;
hasReturnOld = belief.HasReturn(:);

[deltaP, deltaPsi] = estimation.transport.relativeMotion(belief.ObserverPose, pose);
endpoints = estimation.transport.transportEndpoints( ...
    angles, belief.Mean(:), full(diag(belief.Covariance)), deltaP, deltaPsi);
pairs = estimation.transport.classifyPairs(endpoints, hasReturnOld, params);
[predictedMean, hasReturn, label] = ...
    estimation.transport.resampleOntoBeams(angles, pairs, maxRange);
[predictedMean, hasReturn] = estimation.transport.fillUncoveredBeams( ...
    angles, predictedMean, hasReturn, label, maxRange, params);

% A surface transported beyond the cap is out of range: no return (sec. 4.9).
isBeyondCap = predictedMean > maxRange;
hasReturn(isBeyondCap) = false;
predictedMean = min(max(predictedMean, 0), maxRange);

prediction = struct();
prediction.Mean = predictedMean;
prediction.HasReturn = hasReturn;
prediction.Label = label;
prediction.Angles = angles;
prediction.ObserverPose = reshape(double(pose), 1, 3);
end
