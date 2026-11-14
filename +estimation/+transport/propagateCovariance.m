function [covariance, F, Q] = propagateCovariance(belief, prediction, params)
%PROPAGATECOVARIANCE Predicted covariance Sigma_bar = F*Sigma*F' + Q (eqs. 3, 6, 11).
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
% noise Q from the labels and the eq. 8 jump-pair corners.
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
%     qw*dpsi^2 + qe,i (eq. 6). qe,i = (rho_f - rho_n)^2 / 4 (eq. 8's error
%     order) is added on beams within half a beam spacing of a jump pair's
%     transported corner c (strictly, |theta_i - angle(c)| < dtheta/2); if
%     several jump-pair corners qualify for the same beam, the largest
%     qe,i wins. rho_f is the far point's transported range, or MaxRange
%     when the far point has no return.
%   - a beam with no final return (label "no-return", or clamped beyond
%     MaxRange): the same q0 + qv*|dp|^2 + qw*dpsi^2, without qe,i -- a
%     confident "open", not a measured edge.
%   - "disoccluded": (FarRange - ShadowRange)^2 / 12 + q0 (eq. 11's
%     uniform-interval variance).
%   - "new": sigma0^2 + (kappa * mu_bar_i * delta_i)^2 (sec. 4.7), sigma0 =
%     NewBeamSigmaFraction * MaxRange, delta_i = |theta_i - phi_k| the
%     angular distance to the old point k = FromIndex(i, 1) the fill used.
%
% Sigma_bar = F*Sigma*F' + Q, symmetrised ((Sigma_bar + Sigma_bar')/2); kept
% full (interpolation correlates neighbouring beams).

angles = prediction.Angles(:);
numBeams = numel(angles);
oldMean = belief.Mean(:);
hasReturnOld = belief.HasReturn(:);
maxRange = belief.MaxRange;

[deltaP, deltaPsi] = estimation.transport.relativeMotion( ...
    belief.ObserverPose, prediction.ObserverPose);
endpoints = estimation.transport.transportEndpoints( ...
    angles, oldMean, full(diag(belief.Covariance)), deltaP, deltaPsi);
pairs = estimation.transport.classifyPairs(endpoints, hasReturnOld, params);

qBase = params.ProcessNoiseFloor + params.TranslationNoise * (deltaP.' * deltaP) ...
    + params.RotationNoise * deltaPsi ^ 2;

% Eq. 8's edge term qe,i: for each jump pair, the same mid-beam corner
% estimate estimation.transport.edgeAndShadowCandidates uses, widened here
% only to its bearing, to flag beams within half a beam spacing of it.
beamSpacing = mean(diff(angles));
qEdge = zeros(numBeams, 1);
jumpPairIndices = find(pairs.IsJumpPair);
for p = 1:numel(jumpPairIndices)
    j = jumpPairIndices(p);
    if pairs.ReturnA(j) && pairs.ReturnB(j)
        isANear = oldMean(j) <= oldMean(j + 1);
    else
        isANear = pairs.ReturnA(j);
    end
    if isANear
        thetaNear = angles(j); muNear = oldMean(j); nIndex = j; fIndex = j + 1;
    else
        thetaNear = angles(j + 1); muNear = oldMean(j + 1); nIndex = j + 1; fIndex = j;
    end
    thetaFar = angles(j) + angles(j + 1) - thetaNear;
    thetaStar = (thetaNear + thetaFar) / 2;
    rotation = [cos(-deltaPsi), -sin(-deltaPsi); sin(-deltaPsi), cos(-deltaPsi)];
    corner = (muNear * [cos(thetaStar), sin(thetaStar)] - deltaP.') * rotation.';
    cornerBearing = atan2(corner(2), corner(1));

    rhoFar = endpoints.Rho(fIndex);
    if ~hasReturnOld(fIndex)
        rhoFar = maxRange;
    end
    candidateQe = (rhoFar - endpoints.Rho(nIndex)) ^ 2 / 4;

    % Strictly inside half a beam: beams n and f sit exactly at half a beam
    % from the mid-beam corner when nothing moves, and rounding must not put
    % them inside (that would inflate edge beams of a stationary observer).
    angularOffset = mod(angles - cornerBearing + pi, 2 * pi) - pi;
    nearCorner = abs(angularOffset) < beamSpacing / 2 * (1 - 1e-9);
    qEdge(nearCorner) = max(qEdge(nearCorner), candidateQe);
end

sigma0 = params.NewBeamSigmaFraction * maxRange;

% F has at most two nonzero entries per row; preallocate for the worst case
% (every beam a blend/overlap row) and trim the unused tail.
rows = zeros(2 * numBeams, 1);
cols = zeros(2 * numBeams, 1);
vals = zeros(2 * numBeams, 1);
numEntries = 0;
Qdiag = zeros(numBeams, 1);
for i = 1:numBeams
    if prediction.Label(i) == "disoccluded"
        Qdiag(i) = (prediction.FarRange(i) - prediction.ShadowRange(i)) ^ 2 / 12 ...
            + params.ProcessNoiseFloor;
        continue;
    end
    if prediction.Label(i) == "new"
        k = prediction.FromIndex(i, 1);
        delta = abs(mod(angles(i) - endpoints.Phi(k) + pi, 2 * pi) - pi);
        Qdiag(i) = sigma0 ^ 2 + (params.NewBeamSlope * prediction.Mean(i) * delta) ^ 2;
        continue;
    end
    if ~prediction.HasReturn(i)
        Qdiag(i) = qBase;
        continue;
    end
    % A surface beam (blend/overlap/edge) with a return: F row from
    % FromIndex/FromWeight, Q with the edge term.
    Qdiag(i) = qBase + qEdge(i);
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
