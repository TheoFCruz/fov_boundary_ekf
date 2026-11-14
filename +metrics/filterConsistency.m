function report = filterConsistency(results)
%FILTERCONSISTENCY Per-beam NEES/NIS/RMSE/gate-rate consistency report.
%
%   report = metrics.filterConsistency(results)
%
% results is a single simulation.runScenario result run with the boundary
% EKF (estimation.makeBoundaryEkf), or a cell array of results (multi-seed
% Monte Carlo); samples are pooled across seeds. Truth is
% result.RawCasts{k}.Distances (MaxRange on no-return, matching the
% filter's no-return pseudo-measurement z = MaxRange). Scored over samples
% k >= 2 (after a Predict step; sample 1 has no prediction to score).
%
% Per beam: error = belief.Mean - Distances; NEES = error^2 / Sigma_ii
% (beams with Sigma_ii <= 0 are skipped); NIS and the gate decision come
% directly from the belief (methodology sec. 1.3). Beams are grouped by
% belief.Labels (the previous Predict's label for that beam) and pooled
% overall. report.PerLabel is a table with one row per label present
% (Count, RMSE, MeanNEES, MeanNIS, GateFailureRate); report.Overall holds
% the same four pooled over every beam. report.FirstMeasurementNIS is the
% mean of belief.FirstMeasurementNIS (the eta-calibration check, sec. 1.3)
% over disoccluded/new beams' first measurement. report.StepRatioMax and
% report.StepRatioExceedFraction summarise the plan sec. 4.8 step-size
% ratio logged by Predict.
%
% NEES and NIS near 1 (the chi-squared(1) mean) are the consistency
% targets: much above 1 means Q is too small (or a geometric case is
% mishandled) for that label, much below means Q is too large. This
% function only measures and reports the numbers; it does not tune Q.

if ~iscell(results)
    results = {results};
end

labelSet = ["blend", "overlap", "edge", "disoccluded", "no-return", "new", "measured"];
accumulators = repmat(zeroAccumulator(), numel(labelSet), 1);
overall = zeroAccumulator();
firstMeasurementNisSum = 0;
firstMeasurementNisCount = 0;
stepRatios = [];

for resultIndex = 1:numel(results)
    result = results{resultIndex};
    sampleCount = numel(result.Beliefs);
    for k = 2:sampleCount
        belief = result.Beliefs{k};
        truth = result.RawCasts{k}.Distances;
        err = belief.Mean - truth;
        variance = diag(belief.Covariance);
        scored = variance > 0;
        nees = nan(size(err));
        nees(scored) = err(scored) .^ 2 ./ variance(scored);

        overall = accumulate(overall, err, nees, belief.NIS, belief.Gated, scored, true(size(err)));
        for labelIndex = 1:numel(labelSet)
            mask = belief.Labels == labelSet(labelIndex);
            accumulators(labelIndex) = accumulate( ...
                accumulators(labelIndex), err, nees, belief.NIS, belief.Gated, scored, mask);
        end

        isFirstMeasurement = ~isnan(belief.FirstMeasurementNIS);
        firstMeasurementNisSum = firstMeasurementNisSum ...
            + sum(belief.FirstMeasurementNIS(isFirstMeasurement));
        firstMeasurementNisCount = firstMeasurementNisCount + sum(isFirstMeasurement);

        if ~isnan(belief.StepRatio)
            stepRatios(end + 1) = belief.StepRatio; %#ok<AGROW>
        end
    end
end

present = arrayfun(@(a) a.Count > 0, accumulators);
report = struct();
report.PerLabel = accumulatorsToTable(accumulators(present), labelSet(present).');
report.Overall = accumulatorToStruct(overall);
report.FirstMeasurementNIS = firstMeasurementNisSum / max(firstMeasurementNisCount, 1);
if firstMeasurementNisCount == 0
    report.FirstMeasurementNIS = NaN;
end
if isempty(stepRatios)
    report.StepRatioMax = NaN;
    report.StepRatioExceedFraction = NaN;
else
    report.StepRatioMax = max(stepRatios);
    report.StepRatioExceedFraction = mean(stepRatios > 1);
end
end

function acc = zeroAccumulator()
%ZEROACCUMULATOR A fresh running sum for one label (or overall).

acc = struct('Count', 0, 'SumSquaredError', 0, ...
    'NeesSum', 0, 'NeesCount', 0, 'NisSum', 0, 'NisCount', 0, 'GateSum', 0);
end

function acc = accumulate(acc, err, nees, nis, gated, scored, mask)
%ACCUMULATE Fold one belief's masked beams into a running sum.

acc.Count = acc.Count + sum(mask);
acc.SumSquaredError = acc.SumSquaredError + sum(err(mask) .^ 2);
neesMask = mask & scored;
acc.NeesSum = acc.NeesSum + sum(nees(neesMask));
acc.NeesCount = acc.NeesCount + sum(neesMask);
nisMask = mask & ~isnan(nis);
acc.NisSum = acc.NisSum + sum(nis(nisMask));
acc.NisCount = acc.NisCount + sum(nisMask);
acc.GateSum = acc.GateSum + sum(gated(mask));
end

function row = accumulatorToStruct(acc)
%ACCUMULATORTOSTRUCT Reduce a running sum to the reported Count/RMSE/... row.

row = struct();
row.Count = acc.Count;
row.RMSE = sqrt(acc.SumSquaredError / max(acc.Count, 1));
row.MeanNEES = acc.NeesSum / max(acc.NeesCount, 1);
row.MeanNIS = acc.NisSum / max(acc.NisCount, 1);
row.GateFailureRate = acc.GateSum / max(acc.Count, 1);
if acc.Count == 0
    row.RMSE = NaN;
    row.GateFailureRate = NaN;
end
if acc.NeesCount == 0
    row.MeanNEES = NaN;
end
if acc.NisCount == 0
    row.MeanNIS = NaN;
end
end

function reportTable = accumulatorsToTable(accumulators, labels)
%ACCUMULATORSTOTABLE One row per present label, as metrics.printFilterConsistency expects.

count = zeros(numel(accumulators), 1);
rmse = zeros(numel(accumulators), 1);
meanNees = zeros(numel(accumulators), 1);
meanNis = zeros(numel(accumulators), 1);
gateFailureRate = zeros(numel(accumulators), 1);
for i = 1:numel(accumulators)
    row = accumulatorToStruct(accumulators(i));
    count(i) = row.Count;
    rmse(i) = row.RMSE;
    meanNees(i) = row.MeanNEES;
    meanNis(i) = row.MeanNIS;
    gateFailureRate(i) = row.GateFailureRate;
end
reportTable = table(labels, count, rmse, meanNees, meanNis, gateFailureRate, ...
    'VariableNames', {'Label', 'Count', 'RMSE', 'MeanNEES', 'MeanNIS', 'GateFailureRate'});
end
