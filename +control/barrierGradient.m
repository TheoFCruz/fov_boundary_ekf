function [observerGradient, targetGradient, sampleCount, quality] = barrierGradient( ...
        barrierAtPose, ...
        barrierAtTarget, ...
        observerPose, ...
        targetPosition, ...
        barrier, ...
        dtheta, ...
        multiples, ...
        directions)
%BARRIERGRADIENT Least-squares dh/dpose and dh/dtarget over beam-spacing samples.
%
%   [observerGradient, targetGradient, sampleCount, quality] = ...
%       control.barrierGradient(barrierAtPose, barrierAtTarget, ...
%       observerPose, targetPosition, barrier, dtheta, multiples, directions)
%
% h has no closed form (there is no map), so it is differentiated
% numerically. barrierAtPose(pose) returns h with the boundary re-predicted
% from the displaced observer pose (3-by-1); barrierAtTarget(target) returns
% h for a displaced target (2-by-1) against the current boundary. Both
% return NaN when the sampled polygon is invalid geometry; such samples are
% dropped. barrier is h at the current pose and target.
%
% Steps follow the methodology (eq. 12) and are scaled to the beam spacing
% dtheta, so the gradient sees the geometry rather than the sampled
% polygon's sub-beam artifacts:
%
%     delta_psi = dtheta,    delta_x = delta_y = |target - position| * dtheta,
%
% about one beam spacing at the target. Each axis is displaced by
% +-multiples of its step, one axis at a time, and the gradient g solves
% D * g' = h(samples) - h by least squares (D holds the displacements),
% which equals eq. 12's weighted central differences.
%
% directions = "axes" (default) displaces the observer along x, y and psi
% only. "axes+diagonals" also displaces it along every pair of axes at once
% (x with y, x with psi, y with psi, all sign combinations, same step
% sizes). That adds 12 observer samples per multiple, and the fit is still
% one gradient (the best plane over the neighbourhood), but it now
% registers moves that only help in combination, e.g. translating and
% rotating together when either alone barely changes h. The target gradient
% always uses the axes: moving the leader does not change the polygon.
%
% observerGradient is 1-by-3 over [x, y, psi], targetGradient 1-by-2 over
% [x, y]. sampleCount counts the finite observer samples. quality is
% "usable" only when both fits are full rank and finite.

if nargin < 8 || isempty(directions)
    directions = "axes";
end
rangeToTarget = norm(targetPosition(:) - observerPose(1:2));
axisSteps = [rangeToTarget * dtheta; rangeToTarget * dtheta; dtheta];

observerOffsets = sampleOffsets(axisSteps, multiples, directions == "axes+diagonals");
[observerGradient, sampleCount, observerUsable] = fitGradient( ...
    barrierAtPose, observerPose, observerOffsets, barrier);
targetOffsets = sampleOffsets(axisSteps(1:2), multiples, false);
[targetGradient, ~, targetUsable] = fitGradient( ...
    barrierAtTarget, targetPosition(:), targetOffsets, barrier);

quality = "unusable";
if observerUsable && targetUsable
    quality = "usable";
end
end

function offsets = sampleOffsets(steps, multiples, includeDiagonals)
%SAMPLEOFFSETS Displacements to sample: +-multiples along each axis, and
% optionally along each pair of axes at once (all four sign combinations).

numAxes = numel(steps);
unitDirections = eye(numAxes);
if includeDiagonals
    pairs = nchoosek(1:numAxes, 2);
    for p = 1:size(pairs, 1)
        for signs = [1, 1; 1, -1].'
            direction = zeros(1, numAxes);
            direction(pairs(p, :)) = signs.';
            unitDirections(end + 1, :) = direction; %#ok<AGROW>
        end
    end
end
% Same order as the original axis loop (direction, multiple, sign), so the
% default "axes" fit is unchanged.
offsets = zeros(0, numAxes);
for d = 1:size(unitDirections, 1)
    for multiple = multiples
        for signValue = [1, -1]
            offset = signValue * multiple * unitDirections(d, :) .* steps(:).';
            offsets(end + 1, :) = offset; %#ok<AGROW>
        end
    end
end
end

function [gradient, sampleCount, isUsable] = fitGradient( ...
        barrierAt, center, offsets, barrier)
%FITGRADIENT Evaluate h at each offset from center and fit the gradient by
% least squares, D * g' = h(samples) - h, over the finite samples.

values = nan(size(offsets, 1), 1);
for sampleIndex = 1:size(offsets, 1)
    values(sampleIndex) = barrierAt(center + offsets(sampleIndex, :).');
end

% Least squares over the finite samples; NaN when it is not well posed.
isFinite = isfinite(values);
sampleCount = sum(isFinite);
numAxes = numel(center);
gradient = nan(1, numAxes);
isUsable = isfinite(barrier) && rank(offsets(isFinite, :)) == numAxes;
if isUsable
    fit = offsets(isFinite, :) \ (values(isFinite) - barrier);
    gradient = fit.';
    isUsable = all(isfinite(gradient));
end
end
