function estimator = makePassThroughEstimator()
%MAKEPASSTHROUGHESTIMATOR Construct the deterministic checkpoint-one estimator.
%
% The posterior copies each scan range and uses an exactly zero sparse
% covariance. It is plumbing for later estimators, not an EKF or confidence
% guarantee.

estimator = struct();
estimator.Method = "pass-through";
estimator.Initialize = @initialize;
estimator.Predict = @predict;
estimator.Correct = @correct;
end

function state = initialize(~)
state = struct('CurrentBelief', []);
end

function state = predict(state, ~, ~)
% Prediction is intentionally a no-op: no boundary is motion transported.
end

function [state, belief] = correct(state, scan)
required = {'Time', 'ObserverPose', 'Angles', 'Ranges', 'HasReturn', ...
    'IsValid', 'MaxRange', 'OpeningAngle'};
if ~isstruct(scan) || ~all(isfield(scan, required))
    error('estimation:makePassThroughEstimator:InvalidScan', ...
        'scan does not meet the checkpoint-one measurement contract.');
end
count = numel(scan.Ranges);
if ~(iscolumn(scan.Angles) && iscolumn(scan.Ranges) && ...
        iscolumn(scan.HasReturn) && iscolumn(scan.IsValid) && ...
        numel(scan.Angles) == count && numel(scan.HasReturn) == count && ...
        numel(scan.IsValid) == count && all(isfinite(scan.Ranges)) && ...
        all(isfinite(scan.Angles)))
    error('estimation:makePassThroughEstimator:InvalidScan', ...
        'scan vectors must be finite, ordered column vectors of equal length.');
end

belief = struct();
belief.Time = scan.Time;
belief.Angles = scan.Angles;
belief.Mean = scan.Ranges;
belief.Covariance = sparse(count, count);
belief.IsSupported = scan.IsValid;
belief.HasReturn = scan.HasReturn;
belief.ObserverPose = scan.ObserverPose;
belief.MaxRange = scan.MaxRange;
belief.OpeningAngle = scan.OpeningAngle;
belief.Method = "pass-through";
state.CurrentBelief = belief;
end
