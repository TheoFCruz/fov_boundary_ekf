%GENERATERESULTS Run every gradient source on each scenario and save results.
%
% For each scenario, runs the gradient sources of the methodology (sec. 1.5)
% with the same settings and sensor noise over several noise seeds, and
% writes into results/ (ignored by Git). The box compares all four sources;
% the cluttered field is a validation of the EKF (iii) against the oracle:
%
%   results/runs.csv                  one row per scenario, source and seed
%                                     (metrics.runSummary)
%   results/results.csv, results.tex  mean and std over the seeds per scenario
%                                     and source; the .tex file is a booktabs
%                                     tabular with mean +- std cells
%   results/<scenario>/<source>_summary.{pdf,png}       trajectories and h
%   results/<scenario>/<source>_qp.{pdf,png}            QP diagnostics
%   results/<scenario>/comparison_t<time>.{pdf,png}     the scenario's
%       sources side by side at its key times (viz.animateComparison)
%
% Figures come from the first seed only. PDFs are vector graphics for the
% paper; PNGs are for quick viewing. A complete run (3 seeds) takes about
% 20 minutes. scenariosToRun reruns only some scenarios; the rows of the
% others are kept from the previous results/runs.csv. Each scenario's settings mirror its run script
% (scripts/runBoxInFrontOfWall.m, scripts/runClutteredField.m).

run('startup.m');

% --- Scenarios to (re)run ---
scenariosToRun = ["boxInFrontOfWall", "clutteredField"];

% --- Settings shared by every run ---
rangeNoiseStd = 0.1;             % range noise (m); the filters assume the same
seeds = 1:3;                     % noise seeds; figures use the first
sources = ["oracle", "latest", "filter", "filter-conservative"];
sourceLabels = ["Oracle (i)", "Latest reading (ii)", "EKF (iii)", "EKF, eta = 0 (iv)"];
latexLabels = ["Oracle (i)", "Latest reading (ii)", "EKF (iii)", "EKF, $\eta = 0$ (iv)"];

% --- Settings per scenario (controller as in each run script) ---
% Sources: which of the sources above to run. KeyTimes: comparison frames,
% as fractions of the run (1 = the final frame, which shows the full
% trajectories); empty for none.
settings = struct('Name', {}, 'Factory', {}, 'Label', {}, 'StopTime', {}, ...
    'RateFactor', {}, 'DistanceMargin', {}, 'Sources', {}, 'KeyTimes', {});
settings(end + 1) = struct('Name', "boxInFrontOfWall", ...
    'Factory', @scenarios.boxInFrontOfWall, 'Label', "Box", ...
    'StopTime', 18, 'RateFactor', 6, 'DistanceMargin', 0.1, ...
    'Sources', sources, 'KeyTimes', [0.25, 0.5, 0.75]);
settings(end + 1) = struct('Name', "clutteredField", ...
    'Factory', @scenarios.clutteredField, 'Label', "Cluttered", ...
    'StopTime', 30, 'RateFactor', 6, 'DistanceMargin', 0.1, ...
    'Sources', ["oracle", "filter"], 'KeyTimes', []);
inputWeights = diag([1, 1, 0.25]);
params = estimation.defaultTransportParams();

outputRoot = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
previousRuns = table();
if any(~ismember([settings.Name], scenariosToRun))
    previousRuns = readtable(fullfile(outputRoot, 'runs.csv'), 'TextType', 'string');
end
runs = table();
for scenarioIndex = 1:numel(settings)
    setting = settings(scenarioIndex);
    if ~ismember(setting.Name, scenariosToRun)
        runs = [runs; previousRuns(previousRuns.Scenario == setting.Label, :)]; %#ok<AGROW>
        continue
    end
    [~, sourceIndices] = ismember(setting.Sources, sources);
    scenarioFolder = fullfile(outputRoot, setting.Name);
    if ~isfolder(scenarioFolder)
        mkdir(scenarioFolder);
    end
    policyConfig = struct( ...
        'InputWeights', inputWeights, ...
        'InputRatePenalty', setting.RateFactor * inputWeights, ...
        'DistanceMargin', setting.DistanceMargin, ...
        'GradientDirections', "axes+diagonals");

    firstSeedResults = cell(1, numel(sources));
    for sourceIndex = sourceIndices
        for seed = seeds
            fprintf('%s, %s, seed %d ...\n', setting.Name, sources(sourceIndex), seed);
            scenario = setting.Factory();
            scenario.Time.Stop = setting.StopTime;
            scenario.Sensor.RangeNoiseStd = rangeNoiseStd;
            scenario.Sensor.Seed = seed;
            scenario = scenarios.withGradientSource( ...
                scenario, sources(sourceIndex), params, policyConfig);
            runTimer = tic;
            result = simulation.runScenario(scenario);
            runTime = toc(runTimer);

            row = struct2table(metrics.runSummary(result));
            row = addvars(row, setting.Label, latexLabels(sourceIndex), seed, runTime, ...
                'Before', 1, 'NewVariableNames', {'Scenario', 'Source', 'Seed', 'RunTime'});
            runs = [runs; row]; %#ok<AGROW>
            if seed == seeds(1)
                firstSeedResults{sourceIndex} = result;
            end
        end

        % Per-run figures, first seed only.
        result = firstSeedResults{sourceIndex};
        stem = fullfile(scenarioFolder, sources(sourceIndex));
        handles = viz.plotSimulationSummary(result, 'Visible', false);
        saveFigure(handles.Figure, stem + "_summary", [0, 0, 1100, 480]);
        close(handles.Figure);
        handles = viz.plotControlDiagnostics(result, 'Visible', false);
        saveFigure(handles.Figure, stem + "_qp", [0, 0, 700, 600]);
        close(handles.Figure);
    end

    % Side-by-side frames of the scenario's sources at the key times.
    if isempty(setting.KeyTimes)
        continue
    end
    comparison = viz.animateComparison(firstSeedResults(sourceIndices), ...
        sourceLabels(sourceIndices), 'Visible', false, 'AutoPlay', false);
    time = firstSeedResults{sourceIndices(1)}.Time;
    for fraction = setting.KeyTimes
        [~, frame] = min(abs(time - (time(1) + fraction * (time(end) - time(1)))));
        comparison.UpdateFrame(frame);
        saveFigure(comparison.Figure, fullfile(scenarioFolder, ...
            sprintf('comparison_t%05.2f', time(frame))), [0, 0, 1600, 520]);
    end
    close(comparison.Figure);
end

summary = summarizeSeeds(runs);
writetable(runs, fullfile(outputRoot, 'runs.csv'));
writetable(summary, fullfile(outputRoot, 'results.csv'));
writeLatexTable(summary, fullfile(outputRoot, 'results.tex'), numel(seeds));
disp(summary);
fprintf('Results written to %s\n', outputRoot);

function summary = summarizeSeeds(runs)
%SUMMARIZESEEDS Mean and std over seeds per scenario and source, in run order.

metricNames = ["VisiblePercent", "HiddenTime", "InfeasibleSteps", "ActivePercent", ...
    "MinBarrier", "InputChangeRate", "BoundaryRmse", "SurfaceNees", "RunTime"];
groups = unique(runs(:, {'Scenario', 'Source'}), 'stable');
summary = groups;
for name = metricNames
    means = zeros(height(groups), 1);
    stds = zeros(height(groups), 1);
    for g = 1:height(groups)
        inGroup = runs.Scenario == groups.Scenario(g) & runs.Source == groups.Source(g);
        means(g) = mean(runs.(name)(inGroup));
        stds(g) = std(runs.(name)(inGroup));
    end
    summary.(name + "Mean") = means;
    summary.(name + "Std") = stds;
end
end

function saveFigure(figureHandle, stem, position)
%SAVEFIGURE Export a figure as vector PDF and PNG.

set(figureHandle, 'Position', position);
exportgraphics(figureHandle, stem + ".pdf", 'ContentType', 'vector');
exportgraphics(figureHandle, stem + ".png", 'Resolution', 150);
end

function writeLatexTable(summary, path, numSeeds)
%WRITELATEXTABLE Write mean +- std over seeds as a booktabs tabular.

file = fopen(path, 'w');
cleanup = onCleanup(@() fclose(file));
fprintf(file, ['%% Generated by scripts/generateResults.m (mean $\\pm$ std over %d ', ...
    'seeds); needs \\usepackage{booktabs}.\n'], numSeeds);
fprintf(file, '\\begin{tabular}{llrrrrr}\n\\toprule\n');
fprintf(file, ['Scenario & Source & Visible (\\%%) & Infeasible steps & ', ...
    'Input change (1/s) & RMSE (m) & NEES \\\\\n\\midrule\n']);
for k = 1:height(summary)
    scenarioCell = '';
    if k == 1 || summary.Scenario(k) ~= summary.Scenario(k - 1)
        scenarioCell = char(summary.Scenario(k));
        if k > 1
            fprintf(file, '\\midrule\n');
        end
    end
    nees = '--';
    if ~isnan(summary.SurfaceNeesMean(k))
        nees = meanStdCell(summary.SurfaceNeesMean(k), summary.SurfaceNeesStd(k), '%.2f');
    end
    fprintf(file, '%s & %s & %s & %s & %s & %s & %s \\\\\n', ...
        scenarioCell, summary.Source(k), ...
        meanStdCell(summary.VisiblePercentMean(k), summary.VisiblePercentStd(k), '%.1f'), ...
        meanStdCell(summary.InfeasibleStepsMean(k), summary.InfeasibleStepsStd(k), '%.1f'), ...
        meanStdCell(summary.InputChangeRateMean(k), summary.InputChangeRateStd(k), '%.2f'), ...
        meanStdCell(summary.BoundaryRmseMean(k), summary.BoundaryRmseStd(k), '%.3f'), nees);
end
fprintf(file, '\\bottomrule\n\\end{tabular}\n');
end

function text = meanStdCell(meanValue, stdValue, format)
%MEANSTDCELL One "mean +- std" table cell.

text = sprintf(['$' format ' \\pm ' format '$'], meanValue, stdValue);
end
