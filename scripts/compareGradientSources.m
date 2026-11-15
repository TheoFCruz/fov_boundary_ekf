%COMPAREGRADIENTSOURCES Compare source (ii) against the oracle gradient.
%
% Runs scenarios.boxInFrontOfWall once with its default oracle-boundary CBF
% and once with the transport predictor (source (ii), estimation.
% predictBoundaryByTransport with the default params) swapped in, both
% noiseless. For each run, control.compareGradients re-evaluates source (ii)
% against the oracle predictor built from that run's own config, at every
% logged pose (offline only, decision D7: no oracle data enters the source
% (ii) run itself). Prints the measures of plan sec. 8.2 (TrueVisible
% fraction, status counts, median/90th-percentile angle to the oracle
% gradient) and plots them with viz.plotGradientComparison.

run('startup.m');
params = estimation.defaultTransportParams();

oracleResult = simulation.runScenario(scenarios.boxInFrontOfWall());

transportScenario = scenarios.boxInFrontOfWall();
transportPredictor = @(belief, pose) estimation.predictBoundaryByTransport(belief, pose, params);
transportScenario.Observer.Policy = control.makeSignedDistanceCbfPolicy( ...
    struct('BoundaryPredictor', transportPredictor));
transportResult = simulation.runScenario(transportScenario);

series = struct('Name', {}, 'Time', {}, 'PositionAngleDeg', {}, 'TrueVisible', {});
results = {oracleResult, transportResult};
names = ["Oracle run", "Source (ii) run"];
for i = 1:numel(results)
    result = results{i};
    oraclePredictor = @(belief, pose) fov.visibleBoundaryAt( ...
        pose, result.Config.Observer.Fov, result.Config.Obstacles, ...
        result.Config.Sensor.NumRays);
    comparison = control.compareGradients(result, transportPredictor, oraclePredictor, struct());

    intervalCount = numel(comparison.PositionAngleDeg);
    trueVisible = result.Metrics.TrueVisible(1:intervalCount);
    fprintf('--- %s ---\n', names(i));
    fprintf('TrueVisible fraction: %.3f\n', mean(trueVisible));
    statusNames = unique(comparison.Status);
    statusCounts = arrayfun(@(name) sum(comparison.Status == name), statusNames);
    fprintf('Status counts (source ii): %s\n', ...
        strjoin(compose("%s=%d", statusNames, statusCounts), ', '));
    fprintf('Median / 90th pct angle to oracle gradient (deg): %.3f / %.3f\n', ...
        median(comparison.PositionAngleDeg, 'omitnan'), ...
        prctile(comparison.PositionAngleDeg, 90));

    series(end + 1) = struct('Name', names(i), 'Time', result.Time(1:intervalCount), ...
        'PositionAngleDeg', comparison.PositionAngleDeg, 'TrueVisible', trueVisible); %#ok<SAGROW>
end

viz.plotGradientComparison(series);
