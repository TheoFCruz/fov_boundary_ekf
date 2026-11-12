function policy = makeSignedDistanceCbfPolicy(varargin)
%MAKESIGNEDDISTANCECBFPOLICY Construct a world-frame continuous CBF-QP policy.
%
%   policy = control.makeSignedDistanceCbfPolicy(config) returns a callback
%   for simulation.runScenario. config.BoundaryPredictor is required:
%   boundary = predictor(belief, pose) returns the closed world-frame visible
%   polygon the observer would see from pose given the current belief. The
%   barrier and its least-squares gradient are evaluated on that polygon at
%   the current and displaced poses. The baseline is the oracle predictor
%   built from fov.visibleBoundaryAt; the methodology's transport predictor
%   (estimation.predictBoundaryByTransport) will plug into the same field.

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
        leaderPose = reshape(double(observation.LeaderPose), 3, 1);
        leaderReference = reshape(double(observation.LeaderReference), 3, 1);
        leaderPosition = leaderPose(1:2);
        belief = observation.Belief;
        % Geometry fallbacks return the bounded nominal, not a stop command.
        clippedReference = min(max(reference, config.InputLower), config.InputUpper);

        % Predict: world-frame visible polygon from the current belief at a pose.
        boundaryAt = @(pose) config.BoundaryPredictor(belief, pose);

        % Barrier: h = -signedEuclideanDistance(boundary, leader) - margin.
        [barrier, signedDistance, nominalBoundary] = barrierOrNaN( ...
            boundaryAt, observerPose, leaderPosition, config);
        if ~isfinite(barrier)
            [input, diagnostics] = fallbackDiagnostics( ...
                diagnostics, clippedReference, "invalid-geometry-fallback");
            return;
        end
        diagnostics.SignedDistance = signedDistance;
        diagnostics.BarrierValue = barrier;

        % Gradient: re-predict the boundary at displaced poses; displace the
        % leader against the current boundary.
        barrierAtPose = @(pose) barrierOrNaN(boundaryAt, pose, leaderPosition, config);
        barrierAtTarget = @(target) barrierOrNaN( ...
            @(~) nominalBoundary, observerPose, target, config);
        [observerGradient, targetGradient, sampleCount, quality] = ...
            control.barrierGradient( ...
                barrierAtPose, ...
                barrierAtTarget, ...
                observerPose, ...
                leaderPosition, ...
                barrier, ...
                mean(diff(belief.Angles)), ...
                config.GradientBeamMultiples);
        diagnostics.ObserverPoseGradient = observerGradient;
        diagnostics.TargetPositionGradient = targetGradient;
        diagnostics.GradientSampleCount = sampleCount;
        diagnostics.GradientQuality = quality;
        % An ambiguous sampled-polygon derivative is not a valid QP constraint.
        if quality ~= "usable"
            [input, diagnostics] = fallbackDiagnostics( ...
                diagnostics, clippedReference, "unusable-gradient-fallback");
            return;
        end

        % CBF terms: hdot = (dh/dq * G(psi)) * u + dh/dy * leaderVelocity, with
        % G mapping body-frame observer input to world-frame pose rates.
        heading = observerPose(3);
        observerInputMap = [
            cos(heading), -sin(heading), 0
            sin(heading), cos(heading), 0
            0, 0, 1];
        coefficient = observerGradient * observerInputMap;
        leaderWorldVelocity = bodyToWorldVelocity(leaderPose(3), leaderReference);
        targetDrift = targetGradient * leaderWorldVelocity;
        diagnostics.BodyInputCoefficient = coefficient;
        diagnostics.TargetDrift = targetDrift;

        % Solve: coefficient * u + slack >= -gamma * h - targetDrift.
        rightHandSide = -config.CbfRate * barrier - targetDrift;
        [input, qp] = control.solveCbfQp( ...
            coefficient, rightHandSide, reference, config, options);
        diagnostics.NominalInput = qp.NominalInput;
        diagnostics.AppliedInput = input;
        diagnostics.Slack = qp.Slack;
        diagnostics.ConstraintResidual = qp.ConstraintResidual;
        diagnostics.ActiveBounds = qp.ActiveBounds;
        diagnostics.QpExitFlag = qp.ExitFlag;
        diagnostics.QpIterations = qp.Iterations;
        diagnostics.Status = qp.Status;
    end
end

function config = parseConfig(varargin)
%PARSECONFIG Merge the experiment struct with visible baseline values.

% These defaults are teaching configuration, not universal controller tuning.
defaults = struct( ...
    'DistanceMargin', 0.1, ...
    'CbfRate', 4, ...
    'InputLower', [-0.5; -0.5; -1], ...
    'InputUpper', [0.5; 0.5; 1], ...
    'InputWeights', diag([1, 1, 0.25]), ...
    'SlackPenalty', 1e4, ...
    'GradientBeamMultiples', [1, 2], ...
    'BoundaryPredictor', []);
if numel(varargin) ~= 1 || ~isstruct(varargin{1}) || ~isscalar(varargin{1})
    error('control:makeSignedDistanceCbfPolicy:InvalidConfig', ...
        'config must be a scalar structure with a BoundaryPredictor.');
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
if ~isa(config.BoundaryPredictor, 'function_handle')
    error('control:makeSignedDistanceCbfPolicy:InvalidConfig', ...
        'BoundaryPredictor must be a function handle @(belief, pose).');
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

function [barrier, signedDistance, boundary] = barrierOrNaN( ...
        boundaryAt, pose, targetPosition, config)
%BARRIERORNAN h = -signedEuclideanDistance(boundaryAt(pose), target) - margin.
% Known invalid-geometry failures of the predicted polygon give NaN (and an
% empty boundary); any other error is a bug and propagates.

try
    boundary = boundaryAt(pose);
    signedDistance = metrics.signedEuclideanDistance( ...
        boundary, reshape(targetPosition, 1, 2));
catch exception
    if ~isInvalidGeometryException(exception)
        rethrow(exception);
    end
    boundary = [];
    signedDistance = NaN;
end
barrier = -signedDistance - config.DistanceMargin;
end

function velocity = bodyToWorldVelocity(heading, bodyReference)
%BODYTOWORLDVELOCITY Convert the leader's translational reference only.

velocity = [cos(heading), -sin(heading); sin(heading), cos(heading)] * bodyReference(1:2);
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
