function boundary = predictBoundaryByTransport(belief, pose) %#ok<INUSD>
%PREDICTBOUNDARYBYTRANSPORT Methodology boundary predictor (placeholder).
%
%   boundary = estimation.predictBoundaryByTransport(belief, pose)
%
% Not implemented. This is the boundary predictor the methodology prescribes
% for the CBF gradient (docs/metodologia.pdf, sections 1.2 and 1.4), with the
% same signature as the oracle predictor the scenarios build from
% fov.visibleBoundaryAt:
%
%   1. Transport the belief mean from belief.ObserverPose to pose (known
%      motion dp, dpsi in the old sensor frame) and resample it onto the
%      fixed beam angles (eqs. 1-2). Interpolate only within one surface
%      (edge test, eq. 7); place each edge's corner at the mid-beam angle
%      (eqs. 8-10); take the nearest pair for new occlusions; fill
%      disoccluded beams with the free-space hypothesis
%      mu = s + eta * (r_far - s) (eq. 11); extend surfaces into newly
%      explored beams. This is the same transport the boundary EKF's Predict
%      will use; the covariance it carries feeds the edge test, not the
%      barrier.
%   2. Return the world-frame visible polygon built from the predicted mean
%      seen from pose. The barrier uses the expected boundary, not a
%      confidence bound such as mu - kappa * sigma (section 1.4).
%
% Until it exists, scenarios use the oracle predictor as the baseline.

error('estimation:predictBoundaryByTransport:NotImplemented', ...
    'The transport predictor is not implemented yet; use the oracle predictor.');
end
