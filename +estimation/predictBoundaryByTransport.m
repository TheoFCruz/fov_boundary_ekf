function boundary = predictBoundaryByTransport(belief, pose, params)
%PREDICTBOUNDARYBYTRANSPORT Methodology boundary predictor (gradient source ii).
%
%   boundary = estimation.predictBoundaryByTransport(belief, pose, params)
%
% The boundary predictor the methodology prescribes for the CBF gradient
% (docs/metodologia.pdf secs. 1.2, 1.4): transport belief.Mean from
% belief.ObserverPose to pose with estimation.transportBoundary (eqs. 1-2,
% 7-11), then build the world-frame visible polygon from the predicted mean
% on every beam, regardless of IsSupported (plan sec. 4.10: the hypothesis
% beams disocclusion and new-FoV fill are exactly what the CBF gradient
% needs, so the polygon is a predicted boundary for the gradient, not a
% visibility claim). Decision D6: a predicted pair is a jump (and gets the
% conservative radial step instead of a chord, fov.boundaryFromRanges) when
% |mu_bar_i - mu_bar_{i+1}| exceeds max(JumpThreshold, JumpRelative *
% min(mu_bar_i, mu_bar_{i+1})) or HasReturn differs between the two beams.
%
% This has the same (belief, pose) signature as the oracle predictor
% scenarios build from fov.visibleBoundaryAt; a scenario or script plugs it
% into a CBF policy with a closure over params,
%
%   predictor = @(belief, pose) estimation.predictBoundaryByTransport( ...
%       belief, pose, params);
%
% exactly as the oracle closure captures its FovSpec and obstacles
% (decision D5). This is source (ii) (methodology sec. 1.5): the latest
% reading, with the pass-through estimator and no covariance. Sources (iii)
% and (iv) (the EKF, with eta = 0.5 and eta = 0) reuse the same transport
% through Estimator.Predict once the EKF exists; they are not implemented
% here.

prediction = estimation.transportBoundary(belief, pose, params);

meanA = prediction.Mean(1:end - 1);
meanB = prediction.Mean(2:end);
jumpBound = max(params.JumpThreshold, params.JumpRelative .* min(meanA, meanB));
isJump = abs(meanA - meanB) > jumpBound ...
    | prediction.HasReturn(1:end - 1) ~= prediction.HasReturn(2:end);

boundary = fov.boundaryFromRanges( ...
    pose, prediction.Angles, prediction.Mean, belief.OpeningAngle, isJump);
end
