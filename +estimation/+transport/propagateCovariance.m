function [covariance, F, Q] = propagateCovariance(belief, prediction, params)
%PROPAGATECOVARIANCE Predicted covariance Sigma_bar = F*Sigma*F' + Q (eqs. 3, 6).
%
%   [covariance, F, Q] = estimation.transport.propagateCovariance( ...
%       belief, prediction, params)
%
% Only the future EKF's Predict step calls this; estimation.transportBoundary
% and estimation.predictBoundaryByTransport stay mean-only. It recomputes the
% same motion, transported endpoints (extended with the eq. 4 tilt alpha)
% and old-pair classification that built prediction
% (estimation.transport.relativeMotion/transportEndpoints/classifyPairs),
% then builds the Jacobian F from prediction's per-beam provenance
% (FromIndex, FromWeight; see estimation.transportBoundary) and the process
% noise Q from the labels.
%
% F, one row per beam i:
%   - a two-index row (FromIndex(i, :) = [a, b], weight w = FromWeight(i)):
%     a blend/overlap beam, eq. 5,
%       F_ia = w * (cos(alpha_a) - m_i * sin(alpha_a) / rho_a),
%       F_ib = (1 - w) * (cos(alpha_b) - m_i * sin(alpha_b) / rho_b),
%     with the local slope m_i = (rho_b - rho_a) / (phi_b - phi_a) and the
%     eq. 4 tilt alpha_j = theta_j - dpsi - phi_j (endpoints.Tilt);
%   - a single-index row (FromIndex(i, 2) = NaN): an "edge" beam (from the
%     corner construction or the exact-bearing match), eq. 4,
%       F_ia = cos(alpha_a);
%   - zero otherwise: "disoccluded" and "new" beams (no cross-covariance
%     with the old estimate, eq. 11), and any beam whose final HasReturn is
%     false (label "no-return", or a blend/overlap/edge candidate clamped
%     beyond MaxRange in estimation.transportBoundary) -- a confident "open"
%     the next measurement confirms or contradicts, not an interpolated
%     surface.
%
% Q (diagonal), Qii:
%   - a surface beam with a return (blend/overlap/edge): q0 + qv*|dp|^2 +
%     qw*dpsi^2 (eq. 6's motion terms; the edge term qe,i is added by the
%     covariance-stage commit that completes eq. 6).
%   - otherwise (no return, "disoccluded" or "new"): the same motion floor,
%     pending each hypothesis's own variance (added by the same commit).
%
% Sigma_bar = F*Sigma*F' + Q, symmetrised ((Sigma_bar + Sigma_bar')/2); kept
% full (interpolation correlates neighbouring beams).

angles = prediction.Angles(:);
numBeams = numel(angles);
oldMean = belief.Mean(:);

[deltaP, deltaPsi] = estimation.transport.relativeMotion( ...
    belief.ObserverPose, prediction.ObserverPose);
endpoints = estimation.transport.transportEndpoints( ...
    angles, oldMean, full(diag(belief.Covariance)), deltaP, deltaPsi);

qBase = params.ProcessNoiseFloor + params.TranslationNoise * (deltaP.' * deltaP) ...
    + params.RotationNoise * deltaPsi ^ 2;
Qdiag = qBase * ones(numBeams, 1);

% F has at most two nonzero entries per row; preallocate for the worst case
% (every beam a blend/overlap row) and trim the unused tail.
rows = zeros(2 * numBeams, 1);
cols = zeros(2 * numBeams, 1);
vals = zeros(2 * numBeams, 1);
numEntries = 0;
for i = 1:numBeams
    if ~prediction.HasReturn(i) || prediction.Label(i) == "disoccluded" ...
            || prediction.Label(i) == "new"
        continue;
    end
    a = prediction.FromIndex(i, 1);
    b = prediction.FromIndex(i, 2);
    if isnan(b)
        numEntries = numEntries + 1;
        rows(numEntries) = i;
        cols(numEntries) = a;
        vals(numEntries) = cos(endpoints.Tilt(a));
    else
        w = prediction.FromWeight(i);
        slope = (endpoints.Rho(b) - endpoints.Rho(a)) / (endpoints.Phi(b) - endpoints.Phi(a));
        fia = w * (cos(endpoints.Tilt(a)) - slope * sin(endpoints.Tilt(a)) / endpoints.Rho(a));
        fib = (1 - w) ...
            * (cos(endpoints.Tilt(b)) - slope * sin(endpoints.Tilt(b)) / endpoints.Rho(b));
        rows(numEntries + (1:2)) = i;
        cols(numEntries + (1:2)) = [a, b];
        vals(numEntries + (1:2)) = [fia, fib];
        numEntries = numEntries + 2;
    end
end

F = sparse(rows(1:numEntries), cols(1:numEntries), vals(1:numEntries), numBeams, numBeams);
Q = diag(Qdiag);
covariance = F * belief.Covariance * F.' + Q;
covariance = (covariance + covariance.') / 2;
end
