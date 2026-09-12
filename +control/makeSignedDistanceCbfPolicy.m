function policy = makeSignedDistanceCbfPolicy(varargin)
%MAKESIGNEDDISTANCECBFPOLICY Construct a belief-derived continuous CBF-QP policy.
%
%   policy = control.makeSignedDistanceCbfPolicy() returns a callback for
%   simulation.runScenario. The policy builds a frozen observer-local polygon
%   from the current belief, never from raw casts or obstacle geometry.

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

        % Evaluate h = -d(p, F) - margin in the frozen local belief frame.
        try
            % Build only belief-derived geometry so oracle data cannot enter the policy.
            localBoundary = localBoundaryFromBelief(belief);
            [barrier, signedDistance] = evaluateBarrier(localBoundary, observerPose, ...
                followerPose(1:2), config.DistanceMargin);
            [observerGradient, targetGradient, differenceMode, differenceQuality] = ...
                finiteDifferenceGradients(localBoundary, observerPose, ...
                followerPose(1:2), barrier, config);
        catch exception
            if isInvalidBeliefGeometry(exception)
                [input, diagnostics] = fallbackDiagnostics(diagnostics, ...
                    "invalid-belief-fallback");
                return;
            end
            rethrow(exception);
        end

        % Keep intermediate geometry quantities visible for offline diagnosis.
        diagnostics.SignedDistance = signedDistance;
        diagnostics.BarrierValue = barrier;
        diagnostics.ObserverPoseGradient = observerGradient;
        diagnostics.TargetPositionGradient = targetGradient;
        diagnostics.FiniteDifferenceMode = differenceMode;
        diagnostics.FiniteDifferenceQuality = differenceQuality;

        % An ambiguous sampled-polygon derivative is not a valid QP constraint.
        if differenceQuality ~= "usable" || ...
                ~(all(isfinite(observerGradient)) && all(isfinite(targetGradient)))
            [input, diagnostics] = fallbackDiagnostics(diagnostics, ...
                "unusable-gradient-fallback");
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
                "zero-control-authority");
            return;
        end

        % The QP minimally modifies the bounded nominal command.
        reference = min(max(reference, config.InputLower), config.InputUpper);
        diagnostics.NominalInput = reference;
        constraintRightHandSide = -config.CbfRate * barrier - targetDrift;
        % Give quadprog a bound-feasible point with only required CBF slack.
        initialSlack = max(0, constraintRightHandSide - coefficient * reference);
        initialPoint = [reference; initialSlack];
        H = blkdiag(config.InputWeights, config.SlackPenalty);
        f = [-config.InputWeights * reference; 0];
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
                    "qp-failure-fallback");
                return;
            end
        catch
            [input, diagnostics] = fallbackDiagnostics(diagnostics, ...
                "qp-failure-fallback");
            return;
        end

        % Classify the accepted command without claiming a true-FoV guarantee.
        controlInput = solution(1:3);
        input = controlInput.';
        slack = solution(4);
        diagnostics.AppliedInput = input;
        diagnostics.Slack = slack;
        diagnostics.ConstraintResidual = coefficient * controlInput + slack - ...
            constraintRightHandSide;
        diagnostics.ActiveBounds = abs(controlInput - config.InputLower) <= sqrt(eps) | ...
            abs(controlInput - config.InputUpper) <= sqrt(eps);
        if slack > sqrt(eps)
            diagnostics.Status = "relaxed-with-slack";
        elseif norm(controlInput - reference) <= sqrt(eps)
            diagnostics.Status = "nominal-feasible";
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
    'PoseFiniteDifferenceStep', [1e-3; 1e-3; 1e-5]);
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
        'Input bounds must be ordered and contain the zero-command fallback.');
end
config.PoseFiniteDifferenceStep = double(config.PoseFiniteDifferenceStep(:));
if numel(config.PoseFiniteDifferenceStep) ~= 3 || ...
        any(~isfinite(config.PoseFiniteDifferenceStep)) || ...
        any(config.PoseFiniteDifferenceStep <= 0)
    error('control:makeSignedDistanceCbfPolicy:InvalidConfig', ...
        'PoseFiniteDifferenceStep entries must be positive.');
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

function boundary = localBoundaryFromBelief(belief)
%LOCALBOUNDARYFROMBELIEF Construct the current supported sampled polygon.

% Unsupported directions must not become an invented closed visibility region.
if ~all(belief.IsSupported)
    error('control:makeSignedDistanceCbfPolicy:InvalidBeliefGeometry', ...
        'The current belief must provide fully supported boundary geometry.');
end
% Boundary and distance validation preserve invalid-geometry fallback behavior.
try
    boundary = fov.boundaryFromRanges([0, 0, 0], belief.Angles, belief.Mean, ...
        belief.OpeningAngle);
    metrics.signedEuclideanDistance(boundary, [0, 0]);
catch exception
    if isInvalidGeometryException(exception)
        error('control:makeSignedDistanceCbfPolicy:InvalidBeliefGeometry', ...
            'The current belief does not construct a valid sampled polygon.');
    end
    rethrow(exception);
end
end

function [barrier, signedDistance] = evaluateBarrier(boundary, observerPose, targetPosition, margin)
%EVALUATEBARRIER Express the fixed world target in observer-local coordinates.

rotation = [cos(observerPose(3)), sin(observerPose(3)); ...
    -sin(observerPose(3)), cos(observerPose(3))];
targetLocal = rotation * (targetPosition(:) - observerPose(1:2));
signedDistance = metrics.signedEuclideanDistance(boundary, targetLocal.');
barrier = -signedDistance - margin;
end

function [observerGradient, targetGradient, mode, quality] = finiteDifferenceGradients( ...
        boundary, observerPose, targetPosition, currentBarrier, config)
%FINITEDIFFERENCEGRADIENTS Differentiate h without rebuilding belief geometry.

% Perturb observer world pose while holding the local boundary fixed.
observerGradient = zeros(1, 3);
observerModes = strings(1, 3);
observerUsable = false(1, 3);
for index = 1:3
    step = config.PoseFiniteDifferenceStep(index);
    plus = observerPose;
    minus = observerPose;
    plus(index) = plus(index) + step;
    minus(index) = minus(index) - step;
    plusBarrier = evaluateBarrier(boundary, plus, targetPosition, config.DistanceMargin);
    minusBarrier = evaluateBarrier(boundary, minus, targetPosition, config.DistanceMargin);
    [observerGradient(index), observerModes(index), observerUsable(index)] = ...
        finiteDifference(currentBarrier, plusBarrier, minusBarrier, step);
end
% Perturb the known target position independently of observer motion.
targetGradient = zeros(1, 2);
targetModes = strings(1, 2);
targetUsable = false(1, 2);
for index = 1:2
    step = config.PoseFiniteDifferenceStep(index);
    plus = targetPosition;
    minus = targetPosition;
    plus(index) = plus(index) + step;
    minus(index) = minus(index) - step;
    plusBarrier = evaluateBarrier(boundary, observerPose, plus, config.DistanceMargin);
    minusBarrier = evaluateBarrier(boundary, observerPose, minus, config.DistanceMargin);
    [targetGradient(index), targetModes(index), targetUsable(index)] = ...
        finiteDifference(currentBarrier, plusBarrier, minusBarrier, step);
end
% Report whether every directional derivative can support the CBF constraint.
allModes = [observerModes, targetModes];
if all(allModes == "central")
    mode = "central";
elseif all(observerUsable) && all(targetUsable)
    mode = "one-sided";
else
    mode = "inconsistent";
end
quality = "usable";
if ~all(observerUsable) || ~all(targetUsable)
    quality = "unusable";
end
end

function [gradient, mode, isUsable] = finiteDifference(current, plus, minus, step)
%FINITEDIFFERENCE Prefer central differences and reject inconsistent secants.

if isfinite(plus) && isfinite(minus)
    forward = (plus - current) / step;
    backward = (current - minus) / step;
    gradient = (plus - minus) / (2 * step);
    scale = max([1, abs(forward), abs(backward)]);
    isUsable = isfinite(gradient) && abs(forward - backward) <= 0.02 * scale + 1e-8;
    mode = "central";
    if ~isUsable
        gradient = NaN;
        mode = "inconsistent";
    end
elseif isfinite(plus)
    gradient = (plus - current) / step;
    mode = "forward";
    isUsable = isfinite(gradient);
elseif isfinite(minus)
    gradient = (current - minus) / step;
    mode = "backward";
    isUsable = isfinite(gradient);
else
    gradient = NaN;
    mode = "inconsistent";
    isUsable = false;
end
end

function velocity = bodyToWorldVelocity(heading, bodyReference)
%BODYTOWORLDVELOCITY Convert the follower's translational reference only.

velocity = [cos(heading), -sin(heading); sin(heading), cos(heading)] * ...
    bodyReference(1:2);
end

function [input, diagnostics] = fallbackDiagnostics(diagnostics, status)
%FALLBACKDIAGNOSTICS Record the explicit bounded zero-command experiment fallback.

input = zeros(1, 3);
diagnostics.AppliedInput = input;
diagnostics.Status = status;
end

function diagnostics = initialDiagnostics(reference)
%INITIALDIAGNOSTICS Preallocate one complete record for every policy interval.

diagnostics = struct( ...
    'Method', "signed-distance-cbf-qp", ...
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
    'FiniteDifferenceMode', "not-computed", ...
    'FiniteDifferenceQuality', "not-computed", ...
    'Status', "not-computed");
end

function result = isInvalidBeliefGeometry(exception)
%ISINVALIDBELIEFGEOMETRY Identify expected constructed-boundary failures.

result = strcmp(exception.identifier, ...
    'control:makeSignedDistanceCbfPolicy:InvalidBeliefGeometry') || ...
    isInvalidGeometryException(exception);
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
    'fov:boundaryFromRanges:InvalidOpeningAngle'};
result = any(strcmp(exception.identifier, identifiers));
end
