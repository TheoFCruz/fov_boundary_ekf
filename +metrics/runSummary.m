function summary = runSummary(result)
%RUNSUMMARY One row of results metrics for a completed closed-loop run.
%
%   summary = metrics.runSummary(result)
%
% Visibility is scored against the true visible set (Metrics.TrueVisible,
% oracle line of sight), never the estimate. The fields are:
%
%   VisiblePercent     share of samples with the leader truly visible (%)
%   HiddenTime         total time with the leader truly hidden (s)
%   InfeasibleSteps    QP steps with no bounded input meeting the CBF
%                      constraint (status "relaxed-with-slack")
%   ActivePercent      share of steps where the CBF changed the nominal
%                      input ("cbf-corrected" or "relaxed-with-slack") (%)
%   MinBarrier         smallest barrier value h the policy saw (m)
%   InputChangeRate    total change of the applied input per second, a
%                      smoothness measure (sum of |u_k+1 - u_k| / duration)
%   BoundaryRmse       RMS error of the belief mean against the noiseless
%                      ray cast over all beams and samples after the first (m)
%   SurfaceNees        mean NEES on interpolated surface beams for filter
%                      beliefs (metrics.filterConsistency), NaN otherwise

dt = result.Config.Time.Step;
duration = result.Time(end) - result.Time(1);
status = cellfun(@(diagnostic) string(diagnostic.Status), result.PolicyDiagnostics);
barrier = cellfun(@(diagnostic) diagnostic.BarrierValue, result.PolicyDiagnostics);
inputChange = vecnorm(diff(result.ObserverInput), 2, 2);

% Belief error against the noiseless cast, pooled over samples 2..K+1.
squaredError = 0;
count = 0;
for k = 2:numel(result.Beliefs)
    beamError = result.Beliefs{k}.Mean - result.RawCasts{k}.Distances;
    squaredError = squaredError + sum(beamError .^ 2);
    count = count + numel(beamError);
end

surfaceNees = NaN;
if isfield(result.Beliefs{end}, 'Labels')
    report = metrics.filterConsistency(result);
    isBlend = report.PerLabel.Label == "blend";
    if any(isBlend)
        surfaceNees = report.PerLabel.MeanNEES(isBlend);
    end
end

summary = struct();
summary.VisiblePercent = 100 * mean(result.Metrics.TrueVisible);
summary.HiddenTime = dt * sum(~result.Metrics.TrueVisible);
summary.InfeasibleSteps = sum(status == "relaxed-with-slack");
isActive = status == "cbf-corrected" | status == "relaxed-with-slack";
summary.ActivePercent = 100 * mean(isActive);
summary.MinBarrier = min(barrier);
summary.InputChangeRate = sum(inputChange) / duration;
summary.BoundaryRmse = sqrt(squaredError / count);
summary.SurfaceNees = surfaceNees;
end
