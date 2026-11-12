function [observerGradient, targetGradient, sampleCount, quality] = barrierGradient( ...
        barrierAtPose, ...
        barrierAtTarget, ...
        observerPose, ...
        targetPosition, ...
        barrier, ...
        dtheta, ...
        multiples)
%BARRIERGRADIENT Least-squares dh/dpose and dh/dtarget over beam-spacing samples.
%
%   [observerGradient, targetGradient, sampleCount, quality] = ...
%       control.barrierGradient(barrierAtPose, barrierAtTarget, ...
%       observerPose, targetPosition, barrier, dtheta, multiples)
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
% observerGradient is 1-by-3 over [x, y, psi], targetGradient 1-by-2 over
% [x, y]. sampleCount counts the finite observer samples. quality is
% "usable" only when both fits are full rank and finite.

rangeToTarget = norm(targetPosition(:) - observerPose(1:2));
axisSteps = [rangeToTarget * dtheta; rangeToTarget * dtheta; dtheta];

[observerGradient, sampleCount, observerUsable] = fitGradient( ...
    barrierAtPose, observerPose, axisSteps, multiples, barrier);
[targetGradient, ~, targetUsable] = fitGradient( ...
    barrierAtTarget, targetPosition(:), axisSteps(1:2), multiples, barrier);

quality = "unusable";
if observerUsable && targetUsable
    quality = "usable";
end
end

function [gradient, sampleCount, isUsable] = fitGradient( ...
        barrierAt, center, steps, multiples, barrier)
%FITGRADIENT Displace one axis at a time, both signs, and fit h by least squares.

numAxes = numel(center);
numSamples = numAxes * numel(multiples) * 2;
displacements = zeros(numSamples, numAxes);
values = nan(numSamples, 1);
sampleIndex = 0;
for axis = 1:numAxes
    for multiple = multiples
        step = multiple * steps(axis);
        for signValue = [1, -1]
            sampleIndex = sampleIndex + 1;
            displaced = center;
            displaced(axis) = displaced(axis) + signValue * step;
            displacements(sampleIndex, axis) = signValue * step;
            values(sampleIndex) = barrierAt(displaced);
        end
    end
end

% Least squares over the finite samples; NaN when it is not well posed.
isFinite = isfinite(values);
sampleCount = sum(isFinite);
gradient = nan(1, numAxes);
isUsable = isfinite(barrier) && rank(displacements(isFinite, :)) == numAxes;
if isUsable
    fit = displacements(isFinite, :) \ (values(isFinite) - barrier);
    gradient = fit.';
    isUsable = all(isfinite(gradient));
end
end
