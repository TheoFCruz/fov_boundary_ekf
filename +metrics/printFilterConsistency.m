function printFilterConsistency(report)
%PRINTFILTERCONSISTENCY Print a metrics.filterConsistency report.
%
%   metrics.printFilterConsistency(report)
%
% Thin text summary so scripts stay thin: the per-label table, the overall
% row, the first-measurement (eta-calibration) NIS, and the step-ratio
% stats. NEES and NIS near 1 are the consistency targets (sec. 8.1); this
% only prints the numbers metrics.filterConsistency already computed.

disp(report.PerLabel);
fprintf('Overall: Count=%d RMSE=%.4f MeanNEES=%.3f MeanNIS=%.3f GateFailureRate=%.4f\n', ...
    report.Overall.Count, report.Overall.RMSE, report.Overall.MeanNEES, ...
    report.Overall.MeanNIS, report.Overall.GateFailureRate);
fprintf('First-measurement NIS (disoccluded/new, eta calibration): %.3f\n', ...
    report.FirstMeasurementNIS);
fprintf('StepRatio: max=%.3f, fraction > 1 = %.4f\n', ...
    report.StepRatioMax, report.StepRatioExceedFraction);
end
