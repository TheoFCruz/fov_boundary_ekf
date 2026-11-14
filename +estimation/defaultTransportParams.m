function params = defaultTransportParams()
%DEFAULTTRANSPORTPARAMS Parameters used by estimation.transportBoundary.
%
%   params = estimation.defaultTransportParams()
%
% The jump-test constants needed by the eq. 7 same-surface test and the
% free-space fraction eta for disocclusions (eq. 11). The newly-explored
% variance terms sigma0/kappa and the process-noise Q coefficients (docs/
% matlab-implementation-plan.md secs. 4.7, 4.8) are added by whichever
% change introduces covariance propagation.

params = struct();
% Absolute jump threshold tau (m): two transported ranges farther apart than
% this never belong to the same surface (methodology eq. 7).
params.JumpThreshold = 0.35;
% Relative jump threshold (fraction of the nearer range): widens tau for
% distant, grazing surfaces so they are not split into false edges.
params.JumpRelative = 0.08;
% Free-space fraction eta (eq. 11): the baseline hypothesis that a constant
% fraction of a disoccluded beam's seen-free-to-background interval is
% already free space. eta = 0 recovers the conservative seen-free limit,
% used as the gradient source (iv) ablation; eta = 1 assumes nothing lies
% between the corner and the background.
params.FreeSpaceFraction = 0.5;
end
