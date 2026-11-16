function params = defaultTransportParams()
%DEFAULTTRANSPORTPARAMS Parameters used by estimation.transportBoundary.
%
%   params = estimation.defaultTransportParams()
%
% The jump-test constants needed by the eq. 7 same-surface test, the
% free-space fraction eta for disocclusions (eq. 11), the process-noise Q
% coefficients (eq. 6) and the newly-explored variance terms sigma0/kappa
% (sec. 4.7), used by estimation.transport.propagateCovariance.

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

% Process noise Q_ii = q0 + qv*|dp|^2 + qw*dpsi^2 (+ qe,i on edge beams,
% eq. 6). Tuned in closed loop (2026-11-15): with the CBF-QP moving the
% observer these values give surface-beam NEES of about 0.7-1.5 for range
% noise 0.05-0.2 m. Scripted, controller-free motion
% (scripts/checkFilterConsistency.m) prefers values 100 times smaller; the
% filter is then over-confident in closed loop (NEES up to 10).
% Interpolation-error floor (m^2).
params.ProcessNoiseFloor = 1e-4;
% Translation growth (m^2 per m^2 of |dp|^2).
params.TranslationNoise = 2.5e-3;
% Rotation growth (m^2 per rad^2 of dpsi^2).
params.RotationNoise = 0.04;

% Newly-explored beam variance sigma_bar^2 = sigma0^2 + (kappa*mu_bar*delta)^2
% (sec. 4.7), with delta the angular distance to the old point the fill
% used. sigma0 is a fraction of MaxRange; kappa scales the angular term.
% Starting values only, to be tuned the same way as the Q coefficients
% above.
params.NewBeamSigmaFraction = 1 / 3;
params.NewBeamSlope = 1;
end
