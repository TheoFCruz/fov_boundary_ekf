function prediction = transportBoundary(belief, pose, params) %#ok<INUSD>
%TRANSPORTBOUNDARY Mean-only transport and resampling of the boundary belief.
%
%   prediction = estimation.transportBoundary(belief, pose, params)
%
% Moves the boundary mean belief.Mean, defined on belief.Angles at
% belief.ObserverPose, to the hypothetical pose, and resamples it onto the
% same fixed beam grid (docs/metodologia.pdf sec. 1.2, eqs. 1-2; docs/
% matlab-implementation-plan.md secs. 4.1, 4.3, 4.7). Delta p and Delta psi
% (both in the old sensor frame) are computed internally from
% belief.ObserverPose to pose. This is the same (belief, pose) signature as
% the CBF policy's BoundaryPredictor, so a thin polygon wrapper and the
% EKF's Predict step can both call this core function directly with the
% real or a hypothetical pose.
%
% NOT YET HANDLED (later checkpoints): the jump test between old neighbours
% (eq. 7) -- every consecutive pair with two returns is currently treated as
% one surface --, no-return old beams (dropped from the bracket search
% rather than transported as cap endpoints), covariance propagation (F, Q),
% the mid-beam corner position and shadow-edge intersection (eqs. 8-10), the
% free-space disocclusion fraction eta (eq. 11), and the polygon/predictor
% wrapper used by the EKF and the CBF gradient.

if belief.OpeningAngle == 2*pi
    error('estimation:transportBoundary:FullFovUnsupported', ...
        ['transportBoundary assumes a partial FoV; a full-circle belief ', ...
         'needs an angle-wrap bracket search not yet implemented.']);
end

angles = belief.Angles(:);
meanOld = belief.Mean(:);
hasReturnOld = belief.HasReturn(:);
numBeams = numel(angles);
maxRange = belief.MaxRange;

% --- Known motion, in the old sensor frame ---
% Delta p = R(-psi_old) * (p_new - p_old); Delta psi = psi_new - psi_old.
oldHeading = belief.ObserverPose(3);
deltaPsi = pose(3) - oldHeading;
worldDisplacement = reshape(pose(1:2) - belief.ObserverPose(1:2), 2, 1);
deltaP = rot2(-oldHeading) * worldDisplacement;

% --- Transport every old endpoint into the new frame (eq. 1) ---
oldPoints = meanOld .* [cos(angles), sin(angles)];
transportedPoints = (oldPoints - deltaP.') * rot2(-deltaPsi).';
rho = hypot(transportedPoints(:, 1), transportedPoints(:, 2));
phi = atan2(transportedPoints(:, 2), transportedPoints(:, 1));

% --- Bracket and blend: interpolate within each old same-surface pair ---
% (eq. 2). In P1 every pair of neighbouring returns counts as one surface;
% the jump test (eq. 7) arrives with no-return handling in a later change.
phiA = phi(1:end-1);
phiB = phi(2:end);
rhoA = rho(1:end-1);
rhoB = rho(2:end);
pairIsSurface = hasReturnOld(1:end-1) & hasReturnOld(2:end) & phiB ~= phiA;
lowerBound = min(phiA, phiB);
upperBound = max(phiA, phiB);

meanNew = nan(numBeams, 1);
hasReturnNew = false(numBeams, 1);
label = strings(numBeams, 1);
for i = 1:numBeams
    thetaI = angles(i);
    inBracket = pairIsSurface & lowerBound <= thetaI & thetaI <= upperBound;
    if any(inBracket)
        % Collision rule: several pairs may bracket the same new beam; the
        % nearer surface (smallest candidate range) wins (sec. 4.5).
        weight = (phiB(inBracket) - thetaI) ./ (phiB(inBracket) - phiA(inBracket));
        candidates = weight .* rhoA(inBracket) + (1 - weight) .* rhoB(inBracket);
        meanNew(i) = min(candidates);
        hasReturnNew(i) = true;
        label(i) = "blend";
    end
end

% --- Newly explored beams: extend the nearest covered surface (sec. 4.7) ---
% No old pair brackets these beams (they entered through a sector side).
isBlend = label == "blend";
filledIndices = find(isBlend);
newBeamIndices = find(~isBlend);
for n = 1:numel(newBeamIndices)
    i = newBeamIndices(n);
    if isempty(filledIndices)
        meanNew(i) = maxRange;
        hasReturnNew(i) = false;
        label(i) = "new";
        continue;
    end
    thetaI = angles(i);
    [~, nearestPosition] = min(abs(angles(filledIndices) - thetaI));
    k = filledIndices(nearestPosition);

    % The neighbour on the covered side of k (away from the uncovered beam)
    % continues the same surface, if it is also a blended return.
    if angles(k) < thetaI
        neighborIndex = k - 1;
    else
        neighborIndex = k + 1;
    end
    hasNeighbor = neighborIndex >= 1 && neighborIndex <= numBeams && isBlend(neighborIndex);

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
    label(i) = "new";
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
