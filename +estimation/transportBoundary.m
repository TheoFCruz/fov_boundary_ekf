function prediction = transportBoundary(belief, pose, params)
%TRANSPORTBOUNDARY Mean-only transport and resampling of the boundary belief.
%
%   prediction = estimation.transportBoundary(belief, pose, params)
%
% Moves the boundary mean belief.Mean, defined on belief.Angles at
% belief.ObserverPose, to the hypothetical pose, and resamples it onto the
% same fixed beam grid (docs/metodologia.pdf sec. 1.2, eqs. 1-2, 7; docs/
% matlab-implementation-plan.md secs. 4.1-4.3, 4.7, 4.9). Delta p and Delta
% psi (both in the old sensor frame) are computed internally from
% belief.ObserverPose to pose. This is the same (belief, pose) signature as
% the CBF policy's BoundaryPredictor, so a thin polygon wrapper and the
% EKF's Predict step can both call this core function directly with the
% real or a hypothetical pose.
%
% A no-return old beam is not a surface: it contributes a transported
% bearing only (its cap endpoint at MaxRange, already belief.Mean's value).
% A pair with one return and one no-return is always a jump; a pair of two
% no-return beams brackets a "no-return" fill (MaxRange, HasReturn false);
% a return/return pair that fails the eq. 7 jump test, or a mixed pair,
% brackets a "jump-gap" fill. Jump gaps are filled by the same
% newly-explored rule as beams with no bracketing pair at all ("new"); only
% the label differs, so later work can exclude or score them separately.
% Priority when several kinds of candidate bracket one beam: the smallest
% blend candidate wins; else "no-return" if a no-return pair brackets it;
% else "jump-gap" if some pair brackets it, otherwise "new".
%
% NOT YET HANDLED (later checkpoints): actually splitting a jump gap into
% near/far shadow geometry (eqs. 8-10), the free-space disocclusion fraction
% eta (eq. 11) -- jump gaps use the newly-explored fill instead --,
% covariance propagation (F, Q) beyond reading the diagonal for the jump
% test, and the polygon/predictor wrapper used by the EKF and the CBF
% gradient.

if belief.OpeningAngle == 2*pi
    error('estimation:transportBoundary:FullFovUnsupported', ...
        ['transportBoundary assumes a partial FoV; a full-circle belief ', ...
         'needs an angle-wrap bracket search not yet implemented.']);
end

angles = belief.Angles(:);
meanOld = belief.Mean(:);
hasReturnOld = belief.HasReturn(:);
diagCovOld = full(diag(belief.Covariance));
numBeams = numel(angles);
maxRange = belief.MaxRange;

% --- Known motion, in the old sensor frame ---
% Delta p = R(-psi_old) * (p_new - p_old); Delta psi = psi_new - psi_old.
oldHeading = belief.ObserverPose(3);
deltaPsi = pose(3) - oldHeading;
worldDisplacement = reshape(pose(1:2) - belief.ObserverPose(1:2), 2, 1);
deltaP = rot2(-oldHeading) * worldDisplacement;

% --- Transport every old endpoint into the new frame (eq. 1) ---
% A no-return beam's endpoint is its cap at MaxRange (already belief.Mean);
% transporting it gives a bearing for the bracket search, not a surface.
oldPoints = meanOld .* [cos(angles), sin(angles)];
transportedPoints = (oldPoints - deltaP.') * rot2(-deltaPsi).';
rho = hypot(transportedPoints(:, 1), transportedPoints(:, 2));
phi = atan2(transportedPoints(:, 2), transportedPoints(:, 1));
% Range-derivative tilt angle (eq. 4), used only by the eq. 7 jump test.
alpha = wrapAngle(angles - deltaPsi - phi);
sigma2 = (cos(alpha) .^ 2) .* diagCovOld;

% --- Classify each old neighbouring pair (eq. 7) ---
phiA = phi(1:end-1);
phiB = phi(2:end);
rhoA = rho(1:end-1);
rhoB = rho(2:end);
returnA = hasReturnOld(1:end-1);
returnB = hasReturnOld(2:end);
distinctBearings = phiB ~= phiA;

bothReturn = returnA & returnB;
bothNoReturn = ~returnA & ~returnB;
% Same-surface test: the uncertainty term keeps noise from splitting a
% surface into false edges; a mixed return/no-return pair is always a jump.
jumpBound = max(max(params.JumpThreshold, ...
    params.JumpRelative .* min(rhoA, rhoB)), ...
    3 * sqrt(sigma2(1:end-1) + sigma2(2:end)));
isSameSurface = bothReturn & distinctBearings & abs(rhoA - rhoB) <= jumpBound;
isNoReturnPair = bothNoReturn & distinctBearings;
isAnyPair = distinctBearings;

lowerBound = min(phiA, phiB);
upperBound = max(phiA, phiB);

% --- Bracket and blend within each same-surface pair (eq. 2) ---
meanNew = nan(numBeams, 1);
hasReturnNew = false(numBeams, 1);
label = strings(numBeams, 1);
for i = 1:numBeams
    thetaI = angles(i);
    inSurface = isSameSurface & lowerBound <= thetaI & thetaI <= upperBound;
    if any(inSurface)
        % Collision rule: several pairs may bracket the same new beam; the
        % nearer surface (smallest candidate range) wins (sec. 4.5).
        weight = (phiB(inSurface) - thetaI) ./ (phiB(inSurface) - phiA(inSurface));
        candidates = weight .* rhoA(inSurface) + (1 - weight) .* rhoB(inSurface);
        meanNew(i) = min(candidates);
        hasReturnNew(i) = true;
        label(i) = "blend";
        continue;
    end
    inNoReturn = isNoReturnPair & lowerBound <= thetaI & thetaI <= upperBound;
    if any(inNoReturn)
        meanNew(i) = maxRange;
        hasReturnNew(i) = false;
        label(i) = "no-return";
        continue;
    end
    inAnyPair = isAnyPair & lowerBound <= thetaI & thetaI <= upperBound;
    if any(inAnyPair)
        label(i) = "jump-gap";
    else
        label(i) = "new";
    end
end

% --- Fill jump-gap and newly-explored beams from the nearest covered beam:
% extend its surface, or stay open if it is open (sec. 4.7; jump gaps get
% the same fill, pending eqs. 8-11) ---
isBlend = label == "blend";
% Covered beams got a value from an old pair: a surface (blend) or open
% space (no-return). The nearest one decides whether a surface continues.
coveredIndices = find(isBlend | label == "no-return");
fillIndices = find(label == "jump-gap" | label == "new");
for n = 1:numel(fillIndices)
    i = fillIndices(n);
    if isempty(coveredIndices)
        meanNew(i) = maxRange;
        hasReturnNew(i) = false;
        continue;
    end
    thetaI = angles(i);
    [~, nearestPosition] = min(abs(angles(coveredIndices) - thetaI));
    k = coveredIndices(nearestPosition);

    % Next to open space the beam stays open: never extend a distant surface.
    if ~isBlend(k)
        meanNew(i) = maxRange;
        hasReturnNew(i) = false;
        continue;
    end

    % The neighbour on the covered side of k (away from the uncovered beam)
    % continues the same surface if it is also a blended return and passes
    % the eq. 7 range test (mean only here, so without the variance term).
    if angles(k) < thetaI
        neighborIndex = k - 1;
    else
        neighborIndex = k + 1;
    end
    hasNeighbor = neighborIndex >= 1 && neighborIndex <= numBeams ...
        && isBlend(neighborIndex) ...
        && abs(meanNew(k) - meanNew(neighborIndex)) <= max(params.JumpThreshold, ...
            params.JumpRelative * min(meanNew(k), meanNew(neighborIndex)));

    candidateRange = meanNew(k);
    if hasNeighbor
        pointK = meanNew(k) * [cos(angles(k)), sin(angles(k))];
        pointNeighbor = meanNew(neighborIndex) * ...
            [cos(angles(neighborIndex)), sin(angles(neighborIndex))];
        direction = pointNeighbor - pointK;
        rayDirection = [cos(thetaI), sin(thetaI)];
        denominator = cross2(rayDirection, direction);
        % Skip an (almost) parallel line: no useful crossing.
        if abs(denominator) > 1e-9
            crossingRange = cross2(pointK, direction) / denominator;
            if crossingRange > 0
                candidateRange = crossingRange;
            end
        end
    end

    if candidateRange >= maxRange
        meanNew(i) = maxRange;
        hasReturnNew(i) = false;
    else
        meanNew(i) = candidateRange;
        hasReturnNew(i) = hasReturnNew(k);
    end
end

% --- Housekeeping: clamp means to [0, R] (sec. 4.9) ---
overflow = meanNew > maxRange;
hasReturnNew(overflow) = false;
meanNew = min(max(meanNew, 0), maxRange);

prediction = struct();
prediction.Mean = meanNew;
prediction.HasReturn = hasReturnNew;
prediction.Label = label;
prediction.Angles = angles;
prediction.ObserverPose = reshape(double(pose), 1, 3);
end

function r = rot2(angle)
%ROT2 Standard 2-D rotation matrix, R(angle) * v rotates v by angle.

r = [cos(angle), -sin(angle); sin(angle), cos(angle)];
end

function z = cross2(a, b)
%CROSS2 Scalar z-component of the 2-D cross product a x b.

z = a(1) * b(2) - a(2) * b(1);
end

function wrapped = wrapAngle(angle)
%WRAPANGLE Wrap an angle (rad) to [-pi, pi).

wrapped = mod(angle + pi, 2 * pi) - pi;
end
