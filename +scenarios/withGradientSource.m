function scenario = withGradientSource(scenario, source, params)
%WITHGRADIENTSOURCE Set the estimator and CBF predictor for one gradient source.
%
%   scenario = scenarios.withGradientSource(scenario, source, params)
%
% Sets scenario.Estimator and rebuilds scenario.Observer.Policy (a fresh
% control.makeSignedDistanceCbfPolicy with only BoundaryPredictor set, so
% every other policy setting, including the rate penalty, stays at its
% default) consistently for one of the methodology's four gradient sources
% (sec. 1.5):
%
%   "oracle"               (i)  pass-through estimator; the oracle predictor
%                                re-casts scenario.Observer.Fov/Obstacles at
%                                the queried pose (fov.visibleBoundaryAt).
%   "latest"               (ii) pass-through estimator; predictor is
%                                estimation.predictBoundaryByTransport(., ., params),
%                                the latest reading, no fusion.
%   "filter"               (iii) estimation.makeBoundaryEkf, with
%                                MeasurementNoiseStd taken from
%                                scenario.Sensor.RangeNoiseStd; predictor is
%                                the same transport, with the same params.
%   "filter-conservative"  (iv) same as "filter", with params.FreeSpaceFraction
%                                forced to 0 in both the filter and the
%                                predictor (the conservative seen-free limit).
%
% params defaults to estimation.defaultTransportParams().

if nargin < 3 || isempty(params)
    params = estimation.defaultTransportParams();
end
validSources = ["oracle", "latest", "filter", "filter-conservative"];
source = string(source);
if ~isscalar(source) || ~ismember(source, validSources)
    error('scenarios:withGradientSource:InvalidSource', ...
        'source must be one of "oracle", "latest", "filter", "filter-conservative".');
end

switch source
    case "oracle"
        predictor = @(belief, pose) fov.visibleBoundaryAt( ...
            pose, scenario.Observer.Fov, scenario.Obstacles, scenario.Sensor.NumRays);
        scenario.Estimator = estimation.makePassThroughEstimator();
    case "latest"
        predictor = @(belief, pose) estimation.predictBoundaryByTransport(belief, pose, params);
        scenario.Estimator = estimation.makePassThroughEstimator();
    otherwise % "filter" or "filter-conservative"
        if source == "filter-conservative"
            params.FreeSpaceFraction = 0;
        end
        predictor = @(belief, pose) estimation.predictBoundaryByTransport(belief, pose, params);
        filterParams = params;
        filterParams.MeasurementNoiseStd = scenario.Sensor.RangeNoiseStd;
        scenario.Estimator = estimation.makeBoundaryEkf(filterParams);
end

scenario.Observer.Policy = control.makeSignedDistanceCbfPolicy( ...
    struct('BoundaryPredictor', predictor));
end
