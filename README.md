# Occluded FOV Metrics

MATLAB project for studying metrics derived from a field of view (FOV)
occluded by convex polygonal obstacles.

The project is being built incrementally. Checkpoint 1 provides a reproducible,
headless moving-observer/moving-follower simulation with ray-cast sensing, a
pass-through boundary estimator, and replay. Static signed-distance analysis
remains available as a separate workflow.

## Structure

```text
+fov/          FOV and obstacle model plus visibility computation
  +internal/   Testable low-level geometry helpers
+metrics/      Metrics computed from visibility results
+simulation/   Headless kinematic scenario runner and validation
+sensing/      Synthetic scan adapter
+estimation/   Replaceable boundary-estimation callbacks
+control/      Replaceable observer-policy callbacks
+viz/          Visualization helpers
+scenarios/    Reusable scenario definitions
scripts/       Experiment entry points
tests/         Automated tests
```

Angles use radians and positions use Cartesian coordinates. Simulation poses are
world-frame `[x, y, yaw]`; applied velocity inputs are body-frame
`[vxBody, vyBody, omega]`.

## Moving-pair simulation (checkpoint 1)

Run the scripted example from MATLAB:

```matlab
run('startup.m');
scenario = scenarios.movingPair();
result = simulation.runScenario(scenario);
viz.plotSimulationSummary(result);
% Full ray segments (default):
viz.animateSimulation(result, 'ShowRays', true, 'ShowHitPoints', false);

% Obstacle-impact markers only (used by scripts/runMovingPair.m):
viz.animateSimulation(result, 'ShowRays', false, 'ShowHitPoints', true);
```

`simulation.runScenario` never creates figures or advances with wall-clock
time. It returns K+1 synchronized pose, scan, and posterior samples for K
input intervals, so `result` can be saved directly to a MAT file. Replay speed
and frame rate affect only displayed samples. `scripts/runMovingPair.m` runs
the same sequence.

`ShowRays` draws each sampled ray from the observer to its endpoint.
`ShowHitPoints` marks only endpoints that intersect an obstacle; it does not
mark maximum-range no-return endpoints. Both options are logical scalars and
may be enabled together or disabled independently.

The default estimator is deliberately a pass-through placeholder: its mean is
the current scan ranges and its covariance is an exactly zero sparse matrix.
This is not an EKF or confidence guarantee. Replace
`scenario.Estimator` with callbacks named `Initialize`, `Predict`, and
`Correct`, or replace `scenario.Observer.Policy` with a function accepting the
current posterior, poses, base position, and reference velocity. Neither seam
receives obstacle polygons or raw ray-cast oracle data.

The follower state is intentionally known to the policy in this checkpoint.
There is no pursuit, collision response, base-link constraint, or continuous
visibility guarantee. The logged polygon visibility and signed distance are
sampled-polygon approximations; invalid sampled polygons produce invalid metric
diagnostics instead of fabricated distances.

### Relationship to the semester roadmap

The [semester roadmap](docs/probabilistic_fov_project_roadmap%284%29.pdf)
targets motion-aware **first-boundary estimation**, not observer-pose or target
tracking and not controller development. Checkpoint 1 is its deterministic
simulation foundation: independent synthetic body-frame velocity schedules,
noiseless scans, equal input/output angular grids, and no-op prediction.

`HasReturn=false` denotes a range cap, not an obstacle at maximum range.
Here `IsSupported=IsValid` means a sampled direction is available for the
baseline; it does not certify physical-surface support or visibility between
rays. Copying capped ranges with zero covariance is a placeholder, not Gaussian
assimilation of censored no-return information. Offline `result.Scans` logs are
for evaluation/replay only, never estimator map memory.

Later roadmap work adds noisy sparse sensing, segmentation, within-segment
motion transport, EKF covariance and forgetting/reset policies, then an existing
controller demonstration. No global smoothing through depth jumps or visibility
confidence guarantee is implemented here. The current polygon diagnostics use
belief geometry: check `IsSampledPolygonValid` before interpreting
`SampledPolygonVisible`. Degenerate or unsupported polygons have `NaN` distance
and `false` validity (their `false` visibility entry is not a classification).
Replay hides the estimated closed boundary if any direction is unsupported;
segmented rendering remains future work.

## Visible-region and signed-distance contract

`fov.castRays` returns `result.VisibleBoundary` as the ordered, closed
polygonal representation of its sampled visible region. For a partial FOV the
boundary is ordered as `observer -> first endpoint -> ... -> last endpoint ->
observer`; for a full-circle FOV it is ordered as `first endpoint -> ... ->
last endpoint -> first endpoint`.

The initial signed-distance benchmark measures shortest Euclidean distance to
this sampled boundary: values are negative strictly inside the visible polygon,
positive outside it, and zero on its boundary. It accepts only finite,
nondegenerate, simple polygon boundaries; consecutive duplicate vertices are
removed, while zero-area or self-intersecting inputs are rejected. A full-circle
cast with two rays remains valid ray-casting output but is too degenerate for
the signed-distance metric.

This is exact for the sampled polygon rather than for ideal continuous
visibility. Ray count controls the visible-geometry approximation; spatial grid
resolution independently controls contour interpolation.

## Current model API

The initial models, geometry, ray casting, scenarios, plotting, signed
Euclidean distance, metric-field sampling, and contour rendering are
implemented. The original scalar metrics and comparison experiments remain
placeholders:

```matlab
spec = fov.FovSpec(12, deg2rad(100));
observer = fov.Observer( ...
    'Position', [0, 0], ...
    'Heading', deg2rad(20), ...
    'Fov', spec);

wall = fov.polygonObstacle('wall', ...
    [4, -2; 4, 2; 4.3, 2; 4.3, -2]);
```

`fov.polygonObstacle` validates that vertices are finite, unique, ordered,
nondegenerate, and convex. Vertices may be clockwise or counterclockwise.

Low-level geometry helpers are available under `fov.internal` so they can be
tested directly without being part of the higher-level FOV API.

```matlab
rayAngles = fov.sampleRayAngles(observer, 181);
[edgeStarts, edgeEnds] = fov.internal.polygonEdges(wall);
[distances, points, parameters, isHit] = ...
    fov.internal.raySegmentIntersection( ...
    observer.Position, [cos(rayAngles(1)), sin(rayAngles(1))], ...
    edgeStarts, edgeEnds);
```

## Running tests

From MATLAB, run the startup file once and then execute:

```matlab
run('startup.m');
results = runtests('tests');
table(results)
```

## Signed-distance contours

Run the complete single-wall contour demonstration with:

```matlab
run('scripts/runSingleScenario.m')
```

It evaluates `metrics.signedEuclideanDistance` against the sampled
`result.VisibleBoundary`, plots signed contours, and labels the
negative-inside convention. The same workflow can be reproduced explicitly:

```matlab
scenario = scenarios.singleWall();
result = fov.castRays( ...
    scenario.Observer, scenario.Obstacles, ...
    'NumRays', scenario.NumRays);

distance = @(points) metrics.signedEuclideanDistance( ...
    result.VisibleBoundary, points, 'Tolerance', result.Tolerance);
field = metrics.sampleField(distance, ...
    'Bounds', [-0.5, 10.5, -7.6, 7.6], ...
    'GridSize', [241, 241], ...
    'Name', 'Signed Euclidean distance', ...
    'Units', 'coordinate units');

[~, ax] = viz.plotScenario(scenario, result, ...
    'ShowRays', false, ...
    'ShowNominalFov', true, ...
    'ShowHitPoints', false, ...
    'MetricField', field, ...
    'MetricContourLevels', -5:0.5:5);
title(ax, 'Signed Euclidean distance (negative inside)');
```

Other demonstration scenarios are available as
`scenarios.emptyField()` and `scenarios.clutteredField()`.

## Legacy interactive heading explorer

The slider explorer remains available for static signed-distance exploration:

```matlab
run('scripts/runInteractiveScenario.m')
```

Move the heading slider for a fast visible-FOV preview. Releasing it computes
the signed-distance field and contour overlay for the selected heading. It is
legacy/deprecated for new work; use the moving-pair scripted workflow above
instead of extending interactive controls.
