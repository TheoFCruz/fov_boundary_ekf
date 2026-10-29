function policy = makeSignedDistanceCbfPolicy(varargin)
%MAKESIGNEDDISTANCECBFPOLICY Construct a world-frame continuous CBF-QP policy.
%
%   policy = control.makeSignedDistanceCbfPolicy() returns a callback for
%   simulation.runScenario. By default the barrier is built from the current
%   belief's frozen polygon (today's behavior, expressed directly in world
%   frame rather than the observer-local frame). Passing config.BoundarySource
%   = source, with boundary = source(observation, observerPose) returning a
%   world-frame closed polygon, lets a caller supply oracle re-cast geometry
%   (see control.makeOracleBoundarySource) or, later, EKF mean-transport
%   geometry through the same seam.

% Freeze the experiment's CBF choices when the policy callback is constructed.
config = validateConfig(parseConfig(varargin{:}));
if isempty(which('quadprog'))
    error('control:makeSignedDistanceCbfPolicy:MissingOptimizationToolbox', ...
        ['Milestone B requires MATLAB Optimization Toolbox and quadprog; ', ...
        'no alternate QP backend is provided.']);
end
options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
policy = @applyPolicy;

    function [input, state, diagnostics] = applyPolicy(observation, reference, state)
        % The runner supplies the current posterior and pre-step agent snapshot.
        reference = reshape(double(reference), 3, 1);
        diagnostics = initialDiagnostics(reference);
        observerPose = reshape(double(observation.ObserverPose), 3, 1);
        followerPose = reshape(double(observation.FollowerPose), 3, 1);
        followerReference = reshape(double(observation.FollowerReference), 3, 1);
        belief = observation.Belief;
        % Every fallback below returns the bounded nominal, not a stop command.
        clippedReference = min(max(reference, config.InputLower), config.InputUpper);

        % Select the world-frame boundary source for h(pose, target).
        if isempty(config.BoundarySource)
            % Unsupported directions must not become an invented closed region.
            if ~all(belief.IsSupported)
                [input, diagnostics] = fallbackDiagnostics(diagnostics, ...
                    clippedReference, "invalid-belief-fallback");
                return;
            end
            boundaryAt = @(pose) fov.boundaryFromRanges(pose.', belief.Angles, ...
                belief.Mean, belief.OpeningAngle);
            diagnostics.BoundarySource = "belief-frozen";
        else
            boundaryAt = @(pose) config.BoundarySource(observation, pose);
            diagnostics.BoundarySource = "supplied";
        end

        % h(pose, target) = -signedEuclideanDistance(source(pose), target) - margin.
        try
            nominalBoundary = boundaryAt(observerPose);
            [barrier, signedDistance] = evaluateBarrier(nominalBoundary, ...
                followerPose(1:2), config.DistanceMargin);
        catch exception
            if isInvalidGeometryException(exception)
                [input, diagnostics] = fallbackDiagnostics(diagnostics, ...
                    clippedReference, "invalid-belief-fallback");
                return;
            end
            rethrow(exception);
        end
        diagnostics.SignedDistance = signedDistance;
        diagnostics.BarrierValue = barrier;

        % Differentiate h by least squares over beam-spacing-scaled samples.
        [observerGradient, targetGradient, sampleCount, quality] = ...
            gradientsByLeastSquares(boundaryAt, nominalBoundary, observerPose, ...
            followerPose(1:2), barrier, belief.Angles, config);
        diagnostics.ObserverPoseGradient = observerGradient;
        diagnostics.TargetPositionGradient = targetGradient;
        diagnostics.GradientSampleCount = sampleCount;
        diagnostics.GradientQuality = quality;

        % An ambiguous sampled-polygon derivative is not a valid QP constraint.
        if quality ~= "usable"
            [input, diagnostics] = fallbackDiagnostics(diagnostics, ...
                clippedReference, "unusable-gradient-fallback");
            return;
        end

        % Map body-frame observer input and follower reference into h-dot terms.
        heading = observerPose(3);
        observerInputMap = [cos(heading), -sin(heading), 0; ...
            sin(heading), cos(heading), 0; 0, 0, 1];
        coefficient = observerGradient * observerInputMap;
        followerWorldVelocity = bodyToWorldVelocity(followerPose(3), followerReference);
        targetDrift = targetGradient * followerWorldVelocity;
        diagnostics.BodyInputCoefficient = coefficient;
        diagnostics.TargetDrift = targetDrift;

        % Skip the QP when bounded observer motion cannot affect the barrier.
        minimumContribution = sum(min(coefficient.' .* config.InputLower, ...
            coefficient.' .* config.InputUpper));
        maximumContribution = sum(max(coefficient.' .* config.InputLower, ...
            coefficient.' .* config.InputUpper));
        if norm(coefficient) <= sqrt(eps) || ...
                maximumContribution - minimumContribution <= sqrt(eps)
            [input, diagnostics] = fallbackDiagnostics(diagnostics, ...
                clippedReference, "zero-control-authority");
            return;
        end

        % The QP minimally modifies the bounded nominal command.
        diagnostics.NominalInput = clippedReference;
        constraintRightHandSide = -config.CbfRate * barrier - targetDrift;
        % Give quadprog a bound-feasible point with only required CBF slack.
        initialSlack = max(0, constraintRightHandSide - coefficient * clippedReference);
        initialPoint = [clippedReference; initialSlack];
        H = blkdiag(config.InputWeights, config.SlackPenalty);
        f = [-config.InputWeights * clippedReference; 0];
        A = [-coefficient, -1];
        inequalityBound = -constraintRightHandSide;
        lowerBound = [config.InputLower; 0];
        upperBound = [config.InputUpper; Inf];

        % Solve a bounded slack QP for a*u + slack >= -alpha*h - targetDrift.
        try
            [solution, ~, exitFlag, output] = quadprog(H, f, A, inequalityBound, ...
                [], [], lowerBound, upperBound, initialPoint, options);
            diagnostics.QpExitFlag = exitFlag;
            diagnostics.QpIterations = output.iterations;
            if ~isscalar(exitFlag) || ~isfinite(exitFlag) || exitFlag <= 0 || ...
                    numel(solution) ~= 4 || any(~isfinite(solution)) || ...
                    ~isfinite(diagnostics.QpIterations)
                [input, diagnostics] = fallbackDiagnostics(diagnostics, ...
                    clippedReference, "qp-failure-fallback");
                return;
            end
        catch
            [input, diagnostics] = fallbackDiagnostics(diagnostics, ...
                clippedReference, "qp-failure-fallback");
            return;
        end

        % Classify the outcome exactly from the constraint geometry, not slack
        % magnitude: slack is reported numerically but never used for status.
        controlInput = solution(1:3);
        input = controlInput.';
        slack = solution(4);
        diagnostics.AppliedInput = input;
        diagnostics.Slack = slack;
        diagnostics.ConstraintResidual = coefficient * controlInput + slack - ...
            constraintRightHandSide;
        diagnostics.ActiveBounds = abs(controlInput - config.InputLower) <= sqrt(eps) | ...
            abs(controlInput - config.InputUpper) <= sqrt(eps);
        if coefficient * clippedReference >= constraintRightHandSide - sqrt(eps)
            diagnostics.Status = "nominal-feasible";
        elseif maximumContribution < constraintRightHandSide - sqrt(eps)
            diagnostics.Status = "relaxed-with-slack";
        else
            diagnostics.Status = "cbf-corrected";
        end
    end
end

function config = parseConfig(varargin)
%PARSECONFIG Merge an optional experiment struct with visible baseline values.

% These defaults are teaching configuration, not universal controller tuning.
defaults = struct( ...
    'DistanceMargin', 0.1, ...
    'CbfRate', 4, ...
    'InputLower', [-0.5; -0.5; -1], ...
    'InputUpper', [0.5; 0.5; 1], ...
    'InputWeights', diag([1, 1, 0.25]), ...
    'SlackPenalty', 1e4, ...
    'GradientBeamMultiples', [1, 2], ...
    'BoundarySource', []);
if isempty(varargin)
    % A scalar struct changes only the fields relevant to this baseline.
    config = defaults;
    return;
end
if numel(varargin) ~= 1 || ~isstruct(varargin{1}) || ~isscalar(varargin{1})
    error('control:makeSignedDistanceCbfPolicy:InvalidConfig', ...
        'config must be an optional scalar structure.');
end
config = defaults;
provided = varargin{1};
providedNames = fieldnames(provided);
allowedNames = fieldnames(defaults);
if ~all(ismember(providedNames, allowedNames))
    error('control:makeSignedDistanceCbfPolicy:InvalidConfig', ...
        'config contains an unsupported field.');
end
for index = 1:numel(providedNames)
    config.(providedNames{index}) = provided.(providedNames{index});
end
end

function config = validateConfig(config)
%VALIDATECONFIG Keep only the QP assumptions needed by the constructor.

if ~isscalar(config.DistanceMargin) || ~isfinite(config.DistanceMargin) || ...
        config.DistanceMargin < 0 || ~isscalar(config.CbfRate) || ...
        ~isfinite(config.CbfRate) || config.CbfRate <= 0 || ...
        ~isscalar(config.SlackPenalty) || ~isfinite(config.SlackPenalty) || ...
        config.SlackPenalty <= 0
    error('control:makeSignedDistanceCbfPolicy:InvalidConfig', ...
        'DistanceMargin must be nonnegative; CbfRate and SlackPenalty must be positive.');
end
config.InputLower = double(config.InputLower(:));
config.InputUpper = double(config.InputUpper(:));
if any(~isfinite(config.InputLower)) || any(~isfinite(config.InputUpper)) || ...
        numel(config.InputLower) ~= 3 || numel(config.InputUpper) ~= 3 || ...
        any(config.InputLower > config.InputUpper) || ...
        any(config.InputLower > 0) || any(config.InputUpper < 0)
    error('control:makeSignedDistanceCbfPolicy:InvalidConfig', ...
        'Input bounds must be ordered and contain zero (a feasible hold input).');
end
config.GradientBeamMultiples = double(config.GradientBeamMultiples(:).');
if isempty(config.GradientBeamMultiples) || ...
        any(~isfinite(config.GradientBeamMultiples)) || ...
        any(config.GradientBeamMultiples <= 0)
    error('control:makeSignedDistanceCbfPolicy:InvalidConfig', ...
        'GradientBeamMultiples must be a positive finite vector.');
end
if ~isempty(config.BoundarySource) && ~isa(config.BoundarySource, 'function_handle')
    error('control:makeSignedDistanceCbfPolicy:InvalidConfig', ...
        'BoundarySource must be empty or a function handle.');
end
if ~isequal(size(config.InputWeights), [3, 3]) || ...
        any(~isfinite(config.InputWeights(:)))
    error('control:makeSignedDistanceCbfPolicy:InvalidConfig', ...
        'InputWeights must be a finite 3-by-3 numeric matrix.');
end
config.InputWeights = double(config.InputWeights);
weightScale = max(1, norm(config.InputWeights, 'fro'));
[~, factorizationFlag] = chol((config.InputWeights + config.InputWeights.') / 2);
if norm(config.InputWeights - config.InputWeights.', 'fro') > 32 * eps(weightScale) || ...
        factorizationFlag ~= 0
    error('control:makeSignedDistanceCbfPolicy:InvalidConfig', ...
        'InputWeights must be symmetric positive definite.');
end
config.DistanceMargin = double(config.DistanceMargin);
config.CbfRate = double(config.CbfRate);
config.SlackPenalty = double(config.SlackPenalty);
end

function [barrier, signedDistance] = evaluateBarrier(boundary, targetPosition, margin)
%EVALUATEBARRIER h = -signedEuclideanDistance(boundary, target) - margin, world frame.

signedDistance = metrics.signedEuclideanDistance(boundary, reshape(targetPosition, 1, 2));
barrier = -signedDistance - margin;
end

function [observerGradient, targetGradient, observerSampleCount, quality] = ...
        gradientsByLeastSquares(boundaryAt, nominalBoundary, observerPose, ...
        targetPosition, currentBarrier, beliefAngles, config)
%GRADIENTSBYLEASTSQUARES Fit dh/dpose and dh/dtarget over beam-spacing samples.
%
% Perturbations are scaled to the sensor's own angular grid (dtheta) so the
% gradient sees the geometry, not the sampled-polygon's sub-beam artifacts.

dtheta = mean(diff(beliefAngles));
rangeToTarget = max(norm(targetPosition(:) - observerPose(1:2)), 1);
axisSteps = [rangeToTarget * dtheta; rangeToTarget * dtheta; dtheta];
multiples = config.GradientBeamMultiples;

% Observer pose samples: perturb one of x, y, psi at a time, both signs.
numObserverSamples = 3 * numel(multiples) * 2;
designObserver = zeros(numObserverSamples, 3);
valuesObserver = nan(numObserverSamples, 1);
sampleIndex = 0;
for axis = 1:3
    for multiple = multiples
        step = multiple * axisSteps(axis);
        for signValue = [1, -1]
            sampleIndex = sampleIndex + 1;
            displacedPose = observerPose;
            displacedPose(axis) = displacedPose(axis) + signValue * step;
            designObserver(sampleIndex, axis) = signValue * step;
            try
                sampleBoundary = boundaryAt(displacedPose);
                valuesObserver(sampleIndex) = evaluateBarrier(sampleBoundary, ...
                    targetPosition, config.DistanceMargin);
            catch exception
                if isInvalidGeometryException(exception)
                    valuesObserver(sampleIndex) = NaN;
                else
                    rethrow(exception);
                end
            end
        end
    end
end
finiteObserver = isfinite(valuesObserver);
observerSampleCount = sum(finiteObserver);
observerGradient = nan(1, 3);
observerUsable = isfinite(currentBarrier) && ...
    rank(designObserver(finiteObserver, :)) == 3;
if observerUsable
    fit = designObserver(finiteObserver, :) \ (valuesObserver(finiteObserver) - currentBarrier);
    observerGradient = fit.';
    observerUsable = all(isfinite(observerGradient));
end

% Target position samples: perturb x, y against the fixed nominal boundary.
numTargetSamples = 2 * numel(multiples) * 2;
designTarget = zeros(numTargetSamples, 2);
valuesTarget = nan(numTargetSamples, 1);
sampleIndex = 0;
for axis = 1:2
    for multiple = multiples
        step = multiple * axisSteps(axis);
        for signValue = [1, -1]
            sampleIndex = sampleIndex + 1;
            displacedTarget = targetPosition(:);
            displacedTarget(axis) = displacedTarget(axis) + signValue * step;
            designTarget(sampleIndex, axis) = signValue * step;
            try
                valuesTarget(sampleIndex) = evaluateBarrier(nominalBoundary, ...
                    displacedTarget, config.DistanceMargin);
            catch exception
                if isInvalidGeometryException(exception)
                    valuesTarget(sampleIndex) = NaN;
                else
                    rethrow(exception);
                end
            end
        end
    end
end
finiteTarget = isfinite(valuesTarget);
targetGradient = nan(1, 2);
targetUsable = isfinite(currentBarrier) && ...
    rank(designTarget(finiteTarget, :)) == 2;
if targetUsable
    fit = designTarget(finiteTarget, :) \ (valuesTarget(finiteTarget) - currentBarrier);
    targetGradient = fit.';
    targetUsable = all(isfinite(targetGradient));
end

quality = "unusable";
if observerUsable && targetUsable
    quality = "usable";
end
end

function velocity = bodyToWorldVelocity(heading, bodyReference)
%BODYTOWORLDVELOCITY Convert the follower's translational reference only.

velocity = [cos(heading), -sin(heading); sin(heading), cos(heading)] * ...
    bodyReference(1:2);
end

function [input, diagnostics] = fallbackDiagnostics(diagnostics, clippedReference, status)
%FALLBACKDIAGNOSTICS Record the explicit bounded clipped-nominal fallback.
%
% Every fallback path returns the reference clipped to the input bounds
% rather than a stop command, so a nonzero nominal does not cause stop-go
% motion whenever geometry or the gradient is unusable.

input = clippedReference.';
diagnostics.AppliedInput = input;
diagnostics.NominalInput = clippedReference;
diagnostics.Status = status;
end

function diagnostics = initialDiagnostics(reference)
%INITIALDIAGNOSTICS Preallocate one complete record for every policy interval.

diagnostics = struct( ...
    'Method', "signed-distance-cbf-qp", ...
    'BoundarySource', "not-computed", ...
    'SignedDistance', NaN, ...
    'BarrierValue', NaN, ...
    'ObserverPoseGradient', nan(1, 3), ...
    'TargetPositionGradient', nan(1, 2), ...
    'BodyInputCoefficient', nan(1, 3), ...
    'TargetDrift', NaN, ...
    'NominalInput', reference, ...
    'AppliedInput', nan(1, 3), ...
    'Slack', NaN, ...
    'ConstraintResidual', NaN, ...
    'ActiveBounds', false(3, 1), ...
    'QpExitFlag', NaN, ...
    'QpIterations', NaN, ...
    'GradientSampleCount', 0, ...
    'GradientQuality', "not-computed", ...
    'Status', "not-computed");
end

function result = isInvalidGeometryException(exception)
%ISINVALIDGEOMETRYEXCEPTION Keep known geometry failures separate from bugs.

identifiers = { ...
    'metrics:internal:normalizeBoundary:DegenerateBoundary', ...
    'metrics:internal:normalizeBoundary:DegenerateEdge', ...
    'metrics:internal:normalizeBoundary:SelfIntersectingBoundary', ...
    'metrics:internal:normalizeBoundary:InvalidBoundary', ...
    'fov:boundaryFromRanges:InvalidAngles', ...
    'fov:boundaryFromRanges:InvalidRanges', ...
    'fov:boundaryFromRanges:InvalidOpeningAngle', ...
    'fov:boundaryFromRanges:InvalidPose', ...
    'fov:boundaryFromRanges:InvalidIsJump'};
result = any(strcmp(exception.identifier, identifiers));
end
