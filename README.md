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
scenario.Sensor.RangeNoiseStd = 0.05; % m; set to zero for noiseless scans
scenario.Sensor.Seed = 17;
scenario.Observer.Policy = control.makeSignedDistanceCbfPolicy();
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
the CBF-QP `scenarios.controlTest()` configuration for controller diagnostics.

`ShowRays` draws each sampled ray from the observer to its endpoint.
`ShowHitPoints` marks only endpoints that intersect an obstacle; it does not
mark maximum-range no-return endpoints. Both options are logical scalars and
may be enabled together or disabled independently. Disabled ray and impact
displays are not updated during replay, which reduces graphics transfer work.
Replay pacing accounts for graphics-render time, so slow rendering reduces the
remaining inter-frame pause rather than extending it.

### Selective replay frame export

Export completed replay logs as static PNG key frames with
`viz.saveSimulationFrames`. By default it writes to the repository-root
`frames/` directory and selects no more than five evenly spaced logged samples,
including the first and final samples:

```matlab
manifest = viz.saveSimulationFrames(result);

% Export only these logged samples with explicit display options:
manifest = viz.saveSimulationFrames(result, ...
    'FrameIndices', [1, 25, 50], ...
    'Resolution', 150, ...
    'ShowRays', true, ...
    'ShowHitPoints', false);
```

`FrameIndices` must contain valid logged sample indices; they are sorted and
deduplicated. `OutputRoot` changes the output root, `Resolution` defaults to
120 DPI, `ShowRays` defaults to `false`, and `ShowHitPoints` defaults to
`true`. Each call creates a directory of the form
`frames/<sanitized-scenario-name>/<yyyy-mm-dd_HH-MM-SS>[/suffix]/` (or the
corresponding `OutputRoot`) and writes `frame_*.png` files there. The returned
manifest contains `Directory`, `FrameIndices`, `Times`, and `FilePaths`, which
respectively identify the output directory, selected logged samples, their
logged times, and the corresponding PNG paths.

Only explicitly selected frames are rendered and exported, so small selections
should generally finish in seconds rather than minutes, subject to graphics
hardware. Export uses one hidden persistent replay figure, performs no playback
pauses, and closes that figure afterward. It consumes the completed `result`
only; it does not advance the simulation or alter its state or logs. This is
static key-frame export, not video export. It requires base-MATLAB
`exportgraphics` (MATLAB R2020a or newer). For lower-level offline rendering,
`viz.animateSimulation(result, 'AutoPlay', false)` returns replay handles
without automatically replaying the log.

The export feature has not had a final MATLAB or desktop rerun after this
feature; no successful verification is claimed here.

`scenario.Sensor.RangeNoiseStd` defaults to zero. A positive value adds
zero-mean Gaussian noise to first-return ranges only; values outside
`[0, MaxRange]` are clipped. `scenario.Sensor.Seed` selects the run-local random
stream, so repeated simulations with the same configuration reproduce the same
logged scans without changing MATLAB's global random stream. Capped no-return
rays remain unchanged and retain `HasReturn=false`. Replay shows the noiseless
oracle boundary/range, noisy measurement boundary/range, and belief estimate as
separate artists; it never resamples noise.

The default estimator is deliberately a pass-through placeholder: its mean is
the current scan ranges and its covariance is an exactly zero sparse matrix.
This is not an EKF or confidence guarantee. Replace
`scenario.Estimator` with callbacks named `Initialize`, `Predict`, and
`Correct`, or replace `scenario.Observer.Policy` with a function accepting the
current posterior, poses, base position, and reference velocity. Neither seam
receives obstacle polygons or raw ray-cast oracle data.

`control.makeSignedDistanceCbfPolicy` is the Milestone-B continuous-time CBF-QP
baseline and requires MATLAB Optimization Toolbox (`quadprog`). It minimally
modifies the configured observer reference using the current belief-derived
sampled polygon and the current known follower reference velocity. Its default
configuration uses a `0.1 m` interior margin, `CbfRate = 4 1/s`, body-input
bounds `[-0.5,-0.5,-1]` to `[0.5,0.5,1]`, and quadratic slack. Inspect
`result.PolicyDiagnostics` for the barrier, gradients, target drift, slack, QP
exit flag, and fallback status. This constrains the estimated sampled polygon at
controller samples only; it is not a true-FoV, intersample, collision, or safety
guarantee. Its pose finite-difference step defaults to `[1e-3, 1e-3, 1e-5]`
for observer `[x, y, yaw]` (m, m, rad); the smaller yaw perturbation avoids
spurious gradient rejection near the sampled range-cap boundary.

For a deterministic closed-loop control demonstration, use
`scenarios.controlTest()`. It gives the follower a constant forward body-frame
reference of `[0.4, 0, 0]` from an initial position near the range cap, while
the observer has a zero nominal reference and uses the CBF-QP policy. The
obstacle-free setup intentionally isolates belief-derived range-cap containment:
any observer motion comes from the policy's visibility correction, not its
nominal reference. It is an experiment scenario, not a pursuit or safety
benchmark.

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
optional deterministic return-range noise, equal input/output angular grids,
and no-op prediction.

`HasReturn=false` denotes a range cap, not an obstacle at maximum range.
Here `IsSupported=IsValid` means a sampled direction is available for the
baseline; it does not certify physical-surface support or visibility between
rays. Copying capped ranges with zero covariance is a placeholder, not Gaussian
assimilation of censored no-return information. Offline `result.Scans` logs are
for evaluation/replay only, never estimator map memory.

Later roadmap work adds sparse sensing, segmentation, within-segment motion
transport, EKF covariance and forgetting/reset policies, then an existing
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
