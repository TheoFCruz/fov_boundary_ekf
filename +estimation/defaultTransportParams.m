function params = defaultTransportParams()
%DEFAULTTRANSPORTPARAMS Parameters used by estimation.transportBoundary.
%
%   params = estimation.defaultTransportParams()
%
% Only the jump-test constants needed by the eq. 7 same-surface test
% (checkpoint 1, P1-P2) are included here. The free-space fraction eta, the
% newly-explored variance terms sigma0/kappa, and the process-noise Q
% coefficients (docs/matlab-implementation-plan.md secs. 4.6, 4.7, 4.8) are
% added by whichever change introduces covariance propagation and the
% disocclusion variance.

params = struct();
% Absolute jump threshold tau (m): two transported ranges farther apart than
% this never belong to the same surface (methodology eq. 7).
params.JumpThreshold = 0.35;
% Relative jump threshold (fraction of the nearer range): widens tau for
% distant, grazing surfaces so they are not split into false edges.
params.JumpRelative = 0.08;
end
