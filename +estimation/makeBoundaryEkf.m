function estimator = makeBoundaryEkf(params)
%MAKEBOUNDARYEKF Construct the boundary EKF estimator (gradient sources iii, iv).
%
%   estimator = estimation.makeBoundaryEkf(params)
%
% params must hold every estimation.defaultTransportParams() field (reused by
% Predict, below), plus:
%   MeasurementNoiseStd   sigma_r (m): the sensor's RangeNoiseStd. Floored at
%                         1 mm inside the filter only (sigma = max(.., 1e-3)),
%                         so a noiseless scenario still gets a well-posed
%                         update.
%   GateConfidence        gating confidence (methodology sec. 1.3), default
%                         0.99 if absent. The chi-squared(1) quantile is
%                         computed from it without the Statistics Toolbox:
%                         chi2inv(p, 1) = (sqrt(2)*erfinv(p))^2.
%
% Returns Initialize/Predict/Correct handles like
% estimation.makePassThroughEstimator. Predict transports the current belief
% to the real motion with estimation.transportBoundary and propagates its
% covariance with estimation.transport.propagateCovariance (both shared with
% gradient source (ii)); the result is held as the pending prediction
% (mu_bar, Sigma_bar) until the next Correct. Correct runs the methodology's
% linear correction (sec. 1.3): per measured beam, gate the innovation
% (z_i - mu_bar_i)^2 / (Sigma_bar_ii + sigma_r^2) against the chi-squared(1)
% threshold; a beam that fails is re-initialised (mean z_i, variance
% sigma_r^2, zero cross-covariance) instead of being fused, and the
% remaining beams get one joint Kalman update in Joseph form.

params = validateParams(params);

estimator = struct();
estimator.Initialize = @initialize;
estimator.Predict = @predict;
estimator.Correct = @correct;

    function state = initialize(~)
        %INITIALIZE No belief and no pending prediction before the first scan.
        state = struct('Belief', [], 'Pending', []);
    end

    function state = predict(state, motion, ~)
        %PREDICT Transport the current belief to the real observer motion.

        belief = state.Belief;
        prediction = estimation.transportBoundary( ...
            belief, motion.CurrentObserverPose, params);
        sigmaBar = estimation.transport.propagateCovariance(belief, prediction, params);

        % Step-size ratio of plan sec. 4.8 (no substepping, dt is already
        % small; logged instead so a scenario violating it is visible).
        [deltaP, deltaPsi] = estimation.transport.relativeMotion( ...
            belief.ObserverPose, motion.CurrentObserverPose);
        if any(belief.HasReturn)
            nearestRange = min(belief.Mean(belief.HasReturn));
        else
            nearestRange = belief.MaxRange;
        end
        beamSpacing = mean(diff(belief.Angles));
        stepRatio = (norm(deltaP) / nearestRange + abs(deltaPsi)) / beamSpacing;

        state.Pending = struct( ...
            'Mean', prediction.Mean, ...
            'Covariance', full(sigmaBar), ...
            'HasReturn', prediction.HasReturn, ...
            'IsSupported', prediction.IsSupported, ...
            'Labels', prediction.Label, ...
            'StepRatio', stepRatio);
    end

    function [state, belief] = correct(state, scan)
        %CORRECT Gate, re-initialise and fuse the scan into the posterior.

        sigma = max(params.MeasurementNoiseStd, 1e-3);
        sigma2 = sigma ^ 2;
        numBeams = numel(scan.Ranges);

        if isempty(state.Belief)
            % First scan: no prior to transport, so the posterior is the
            % measurement itself with an independent sigma_r^2 per beam.
            mu = scan.Ranges;
            covariance = sigma2 * eye(numBeams);
            hasReturn = scan.HasReturn;
            isSupported = scan.IsValid;
            labels = repmat("measured", numBeams, 1);
            innovation = nan(numBeams, 1);
            nis = nan(numBeams, 1);
            gated = false(numBeams, 1);
            firstMeasurementNis = nan(numBeams, 1);
            stepRatio = NaN;
        else
            pending = state.Pending;
            muBar = pending.Mean;
            covariance = pending.Covariance;
            labels = pending.Labels;
            stepRatio = pending.StepRatio;
            measured = scan.IsValid;
            z = scan.Ranges;

            innovation = nan(numBeams, 1);
            nis = nan(numBeams, 1);
            gated = false(numBeams, 1);
            measuredIndex = find(measured);
            priorVariance = diag(covariance);
            innovation(measuredIndex) = z(measuredIndex) - muBar(measuredIndex);
            nis(measuredIndex) = innovation(measuredIndex) .^ 2 ...
                ./ (priorVariance(measuredIndex) + sigma2);
            gated(measuredIndex) = nis(measuredIndex) > params.GateThreshold;

            mu = muBar;
            gatedIndex = measuredIndex(gated(measuredIndex));
            % Re-initialise gated beams first, before the joint update below,
            % so that update runs on a covariance whose gated rows/columns
            % are already zeroed (a gated beam keeps no cross-covariance and
            % is excluded from the fused set).
            for i = gatedIndex.'
                mu(i) = z(i);
                covariance(i, :) = 0;
                covariance(:, i) = 0;
                covariance(i, i) = sigma2;
            end

            keepIndex = measuredIndex(~gated(measuredIndex));
            if ~isempty(keepIndex)
                keepCount = numel(keepIndex);
                H = zeros(keepCount, numBeams);
                H(sub2ind([keepCount, numBeams], (1:keepCount).', keepIndex)) = 1;
                R = sigma2 * eye(keepCount);
                innovationCovariance = H * covariance * H.' + R;
                % Joint Kalman update, Joseph form; \ and / only, never inv.
                gain = (covariance * H.') / innovationCovariance;
                mu = mu + gain * (z(keepIndex) - H * mu);
                identityMinusGainH = eye(numBeams) - gain * H;
                covariance = identityMinusGainH * covariance * identityMinusGainH.' ...
                    + gain * R * gain.';
                covariance = (covariance + covariance.') / 2;
            end

            mu = min(max(mu, 0), scan.MaxRange);
            hasReturn = scan.HasReturn;
            isSupported = pending.IsSupported;
            isSupported(measured) = true;

            % Eq. 1.3's eta-calibration check: the first innovation on a
            % disoccluded or newly explored beam.
            firstMeasurementNis = nan(numBeams, 1);
            isFirstMeasurement = labels == "disoccluded" | labels == "new";
            firstMeasurementNis(isFirstMeasurement) = nis(isFirstMeasurement);
        end

        belief = struct();
        belief.Time = scan.Time;
        belief.Angles = scan.Angles;
        belief.Mean = mu;
        belief.Covariance = covariance;
        belief.IsSupported = isSupported;
        belief.HasReturn = hasReturn;
        belief.ObserverPose = scan.ObserverPose;
        belief.MaxRange = scan.MaxRange;
        belief.OpeningAngle = scan.OpeningAngle;
        belief.Method = "boundary-ekf";
        belief.Labels = labels;
        belief.Innovation = innovation;
        belief.NIS = nis;
        belief.Gated = gated;
        belief.FirstMeasurementNIS = firstMeasurementNis;
        belief.StepRatio = stepRatio;

        state.Belief = belief;
        state.Pending = [];
    end
end

function params = validateParams(params)
%VALIDATEPARAMS Check params once, at this estimator's construction boundary.

requiredTransportFields = fieldnames(estimation.defaultTransportParams());
if ~isstruct(params) || ~isscalar(params) || ~all(isfield(params, requiredTransportFields))
    error('estimation:makeBoundaryEkf:InvalidParams', ...
        'params must contain every estimation.defaultTransportParams field.');
end
if ~isfield(params, 'MeasurementNoiseStd') || ~isscalar(params.MeasurementNoiseStd) ...
        || ~isreal(params.MeasurementNoiseStd) || ~isfinite(params.MeasurementNoiseStd) ...
        || params.MeasurementNoiseStd < 0
    error('estimation:makeBoundaryEkf:InvalidParams', ...
        'params.MeasurementNoiseStd must be a finite nonnegative scalar.');
end
if ~isfield(params, 'GateConfidence') || isempty(params.GateConfidence)
    params.GateConfidence = 0.99;
end
if ~isscalar(params.GateConfidence) || ~isreal(params.GateConfidence) ...
        || ~isfinite(params.GateConfidence) || params.GateConfidence <= 0 ...
        || params.GateConfidence >= 1
    error('estimation:makeBoundaryEkf:InvalidParams', ...
        'params.GateConfidence must be a real scalar in (0, 1).');
end
params.MeasurementNoiseStd = double(params.MeasurementNoiseStd);
params.GateConfidence = double(params.GateConfidence);
params.GateThreshold = (sqrt(2) * erfinv(params.GateConfidence)) ^ 2;
end
