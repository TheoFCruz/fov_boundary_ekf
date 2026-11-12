function boundary = predictBoundaryByTransport(belief, pose) %#ok<INUSD>
%PREDICTBOUNDARYBYTRANSPORT Methodology boundary predictor (placeholder).
%
%   boundary = estimation.predictBoundaryByTransport(belief, pose)
%
% Not implemented. This is the boundary predictor the methodology prescribes
% for the CBF gradient (docs/matlab-implementation-plan.md, sections 4 and
% 7.2), with the same signature as the oracle predictor the scenarios build
% from fov.visibleBoundaryAt:
%
%   1. Transport the belief mean and covariance from belief.ObserverPose to
%      pose (known motion dp, dpsi in the old sensor frame) onto the fixed
%      beam angles, within supported segments only: blend, edge, overlap,
%      disoccluded (shadow bound s_i) and new-in-FoV beams. This is the same
%      mean/covariance transport the boundary EKF's Predict will use.
%   2. Build the confidence boundary B_eps = mu - kappa_eps * sigma, clamped
%      to [0, MaxRange] and to s_i on unmeasured disoccluded beams.
%   3. Return the world-frame visible polygon V_eps seen from pose.
%
% Until it exists, scenarios use the oracle predictor as the baseline.

error('estimation:predictBoundaryByTransport:NotImplemented', ...
    'The transport predictor is not implemented yet; use the oracle predictor.');
end
