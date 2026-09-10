# FOV Metrics Implementation Plan

This document is the main implementation plan for extending the repository
from sampled occluded-FOV ray casting to signed-distance metrics, spatial
fields, contour plots, and scripted simulation workflows.

Agents and contributors should keep this document current while implementing
the plan. Each checklist item should be marked `[x]` only after the work it
describes is implemented and verified. If a step is only partially complete,
leave it unchecked and add a short note describing the remaining work.

## Active direction: checkpoint 1 scripted simulation

The active development direction is the moving-agent checkpoint described in
[`fov_metrics_checkpoint_1_codex_plan.md`](fov_metrics_checkpoint_1_codex_plan.md).
It supersedes further interactive-heading development. Existing static metric
and contour phases below remain historical functionality and are preserved.
`viz.interactiveScenario` and `scripts/runInteractiveScenario.m` are legacy
static-analysis tools, not the recommended workflow.

- [ ] Reconcile the checkpoint with repository guidance and retire interactive
  development from the recommended workflow.
- [ ] Implement validated body-frame references and forward-Euler kinematics.
- [ ] Implement the scan adapter, pass-through belief lifecycle, and shared
  boundary conversion.
- [ ] Implement the headless runner, causal policy seam, logs, and sampled
  diagnostics.
- [ ] Implement scripted replay, summary rendering, and the moving-pair
  scenario.
- [ ] Add and run focused checkpoint-one tests; document outcomes.

**Checkpoint status (2026-09-09):** The interfaces, scenario, plotting, and
focused test class have been added. The checklist remains unchecked because no
MATLAB runtime or graphics checks were run in this change. Run
`run('startup.m'); results = runtests('tests'); table(results)` in MATLAB
before marking checkpoint items complete. The checkpoint does not add an EKF,
pursuit/controller optimization, collision response, or a base-link constraint.

### Roadmap alignment review

The [revised semester roadmap](probabilistic_fov_project_roadmap%284%29.pdf)
defines the later first-boundary estimation research; checkpoint 1 remains the
active implementation scope. Noisy sparse scans/segmentation come next, followed
by deterministic transport, EKF covariance and support/reset handling, and then
history-assisted reconstruction and an existing-controller demonstration.
Controller development/proofs, persistent maps, and estimator scan archives are
outside that roadmap. Offline replay logs are not estimator memory.

The review tightened same-grid belief validation and covariance display inputs,
made unsupported closed-polygon diagnostics invalid rather than claiming
visibility, and added actual first/final-frame graphics checks using altered
estimator output. No-return range caps and zero covariance remain explicit
checkpoint-only placeholders, not Gaussian surface observations or confidence
claims. Segment construction and motion compensation have not been added.

**Verification update:** The user reported three failures in the initial
checkpoint tests. The scalar replay-index bug and two test setup/expectation
errors were subsequently corrected; no passing rerun has been reported here.
This alignment review also adds tests, but runs no MATLAB/runtime/visual checks.
Checkpoint acceptance remains open pending the focused and full MATLAB suites.

**Replay display extension (2026-09-10):** `viz.animateSimulation` now reuses
the existing static-plot `ShowRays` and `ShowHitPoints` semantics. The
moving-pair script selects obstacle-impact markers without full ray segments.
Focused coverage was added but not run; leave the checkpoint checklist unchecked
until the focused and full MATLAB suites, plus a desktop replay check, pass.

## Initial benchmark contract

The first metric is a direct signed Euclidean distance to the sampled visible
region produced by `fov.castRays`.

The initial contract is:

- **Geometry:** `result.VisibleBoundary` from `fov.castRays`.
- **Distance:** shortest Euclidean distance to that polygon boundary.
- **Sign:** negative inside, positive outside, and zero on the boundary.
- **Domain:** any finite Cartesian query point, including points outside the
  nominal FOV and points inside obstacles.
- **Units:** the same units used by observer and obstacle coordinates.
- **Approximation:** the metric is exact relative to the sampled polygon, not
  relative to the ideal continuous visibility region.

For a visible region `V`, the contract is:

\[
d_V(p) =
\begin{cases}
-\min_{q \in \partial V}\|p-q\|_2, & p \in V \\
0, & p \in \partial V \\
\phantom{-}\min_{q \in \partial V}\|p-q\|_2, & p \notin V.
\end{cases}
\]

The scalar signed-distance function is continuous and 1-Lipschitz. Its
closest boundary feature and gradient can change discontinuously at corners,
medial axes, and occlusion transitions. Those changes are meaningful parts of
the benchmark rather than bugs.

The two discretizations in the initial workflow must remain conceptually
separate:

1. **Visibility-geometry approximation:** `fov.castRays` samples a finite set
   of angles and connects the returned endpoints into a polygon.
2. **Field/contour approximation:** the signed-distance function is sampled
   on an XY grid and contour lines are interpolated between grid nodes.

Ray-count studies should vary only the first approximation. Grid-resolution
studies should vary only the second.

---

## Phase 1: Formalize the visible-region contract

- [x] Document `result.VisibleBoundary` as a computational representation,
  not merely a plotting convenience.
- [x] Document the partial-FOV boundary order:
  `observer -> first endpoint -> ... -> last endpoint -> observer`.
- [x] Document the full-circle boundary order:
  `first endpoint -> ... -> last endpoint -> first endpoint`.
- [x] Require or validate that the boundary supplied to distance metrics is
  finite, real, ordered, closed or closable, and nondegenerate.
- [x] Define behavior for consecutive duplicate vertices.
- [x] Define behavior for fewer than three unique vertices, zero-area
  boundaries, nonfinite coordinates, and self-intersections.
- [x] Decide whether full-circle casts with only two rays remain legal in
  `fov.castRays` while being rejected by the metric as degenerate. The initial
  recommendation is to leave ray casting unchanged and reject the degenerate
  boundary in the metric.
- [x] Add the visible-region, sign-convention, and finite-ray approximation
  documentation to the project documentation.

### Boundary invariants

The boundary used by the metric should be:

- finite and real;
- closed;
- ordered;
- nondegenerate;
- free of consecutive duplicate vertices;
- suitable for `inpolygon` and point-to-segment calculations.

---

## Phase 2: Implement direct signed Euclidean distance

Add:

```text
+metrics/signedEuclideanDistance.m
```

### Public API

Use a pointwise API that accepts a boundary directly:

```matlab
[distance, details] = metrics.signedEuclideanDistance( ...
    boundary, queryPoints, varargin)
```

Example:

```matlab
points = [0, 0; 4, 1; 8, 3];

[d, details] = metrics.signedEuclideanDistance( ...
    result.VisibleBoundary, points, ...
    'Tolerance', result.Tolerance);
```

### Inputs

- `boundary`: `M x 2` polygon vertices, open or closed.
- `queryPoints`: `N x 2` finite Cartesian points.
- `Tolerance`: optional positive geometric tolerance.

Accepting a boundary directly keeps the metric independent of ray casting and
makes it easy to test against analytically known polygons.

### Outputs

`distance` is an `N x 1` vector. An optional `details` output may contain:

```matlab
details.UnsignedDistance
details.IsInside
details.IsOnBoundary
details.ClosestPoint
details.ClosestEdgeIndex
```

Diagnostic fields are useful for investigating medial axes and changes in the
closest boundary feature. They should only be constructed when requested so a
large contour grid does not require unnecessary diagnostic memory.

### Internal helpers

Add helpers under:

```text
+metrics/+internal/normalizeBoundary.m
+metrics/+internal/pointToSegments.m
```

#### `normalizeBoundary`

Responsibilities:

1. Validate shape and numeric values.
2. Add the closing vertex if it is missing.
3. Remove consecutive duplicate vertices within tolerance.
4. Reject degenerate edges and zero-area polygons.
5. Return segment starts and ends.

Do not convert the boundary to `polyshape` in the core calculation. `polyshape`
may simplify or repair geometry, which could silently change the sampled
polygon being benchmarked.

#### `pointToSegments`

For each query point `p` and segment from `a` to `b`, calculate:

\[
t = \operatorname{clamp}\left(
\frac{(p-a)\cdot(b-a)}{\|b-a\|^2}, 0, 1\right)
\]

\[
q = a + t(b-a), \qquad \delta = \|p-q\|_2.
\]

Choose the segment with minimum `delta`.

Process query points in chunks so a large contour grid does not allocate an
`N points x M edges` matrix all at once.

### Sign calculation

Use MATLAB's polygon classification:

```matlab
[inside, on] = inpolygon( ...
    queryPoints(:,1), queryPoints(:,2), ...
    boundary(:,1), boundary(:,2));
```

Then apply the signed-distance convention:

```matlab
isOnBoundary = on | unsignedDistance <= tolerance;

distance = unsignedDistance;
distance(inside & ~isOnBoundary) = ...
    -unsignedDistance(inside & ~isOnBoundary);
distance(isOnBoundary) = 0;
```

When no tolerance is supplied, use a scale-aware default based on coordinate
magnitude and floating-point precision.

---

## Phase 3: Establish a generic metric-field interface

The grid-generation layer should not know which metric contract it evaluates.
Add:

```text
+metrics/sampleField.m
```

### Suggested API

```matlab
evaluator = @(points) metrics.signedEuclideanDistance( ...
    result.VisibleBoundary, points, ...
    'Tolerance', result.Tolerance);

field = metrics.sampleField(evaluator, ...
    'Bounds', [xmin, xmax, ymin, ymax], ...
    'GridSize', [201, 241], ...
    'Name', "Signed Euclidean distance", ...
    'Units', "m");
```

### Field structure

Return a structure containing:

```matlab
field.Name
field.Units
field.Bounds
field.GridSize
field.X
field.Y
field.Values
field.ZeroLevel
field.SignConvention
```

The initial metadata should include:

```matlab
field.ZeroLevel = 0;
field.SignConvention = "negative-inside";
```

### Grid convention

For `GridSize = [ny, nx]`:

```matlab
x = linspace(xmin, xmax, nx);
y = linspace(ymin, ymax, ny);
[X, Y] = meshgrid(x, y);
```

Therefore:

```matlab
size(field.X)      == [ny, nx]
size(field.Y)      == [ny, nx]
size(field.Values) == [ny, nx]
```

Explicit bounds are preferred initially. This avoids hiding plotting-domain
decisions inside the metric layer.

### Evaluator design

Use a pointwise function handle so future metric contracts can share the same
field sampler:

```matlab
euclidean = @(p) metrics.signedEuclideanDistance(boundary, p);
radial    = @(p) metrics.signedRadialDistance(result, p);
weighted  = @(p) metrics.weightedVisibilityDistance(result, p);
```

Do not create a formal metric class or descriptor system yet. A function handle
is sufficient until at least two real contracts exist.

---

## Phase 4: Add contour visualization

Add:

```text
+viz/plotMetricContours.m
```

### Suggested API

```matlab
handles = viz.plotMetricContours(field, ...
    'Parent', ax, ...
    'Levels', -4:0.5:4, ...
    'ShowZeroContour', true, ...
    'ShowColorbar', true);
```

### Initial visualization scope

Start with line contours rather than filled contours. This avoids obscuring
obstacles and the visible-region patch.

The function should:

- draw regular contours colored by signed value;
- draw the zero contour separately with a thicker line;
- distinguish negative and positive values through the colormap;
- preserve the axes' previous hold state;
- support a supplied axes for future UI integration;
- return graphics handles.

Example:

```matlab
[fig, ax] = viz.plotScenario(scenario, result, ...
    'ShowRays', false);

viz.plotMetricContours(field, ...
    'Parent', ax, ...
    'Levels', -5:0.5:5, ...
    'ShowZeroContour', true);
```

The zero contour should approximately overlay `result.VisibleBoundary`.
Differences of approximately one grid cell are expected.

Potential later options, not required for the first implementation:

- `contourf` mode;
- transparency;
- masks for obstacle interiors;
- labels via `clabel`;
- symmetric automatic levels;
- nearest-edge or gradient visualization.

---

## Phase 5: Automated testing

Use the existing placeholder:

```text
tests/TestMetrics.m
```

### A. Exact polygon tests

Use the known square:

```matlab
boundary = [
   -1, -1
    1, -1
    1,  1
   -1,  1
   -1, -1
];
```

Verify:

| Query point | Expected distance |
| --- | ---: |
| `[0, 0]` | `-1` |
| `[0.5, 0]` | `-0.5` |
| `[1, 0]` | `0` |
| `[2, 0]` | `1` |
| `[2, 2]` | `sqrt(2)` |

Also verify the closest point and edge where unambiguous.

### B. Boundary normalization

Test:

- open versus closed input;
- clockwise versus counterclockwise order;
- repeated closing point;
- consecutive duplicate rejection or removal;
- invalid dimensions;
- `NaN` and `Inf`;
- zero-area polygons.

### C. Concave geometry

Although obstacles are convex, the visible FOV polygon can be concave. Include
a simple concave polygon and test points:

- inside the main region;
- inside the concavity but outside the polygon;
- near the reflex vertex.

### D. FOV integration tests

Using actual ray-cast results, verify:

- the observer is inside or on the visible region;
- every `VisibleBoundary` vertex has zero distance;
- a distant point outside the nominal FOV is positive;
- empty partial FOV works;
- empty full-circle FOV works;
- the occluded wall scenario produces positive values behind the wall.

### E. Mathematical property test

For representative point pairs, verify the 1-Lipschitz property:

\[
|d(p)-d(q)| \leq \|p-q\|_2 + \epsilon.
\]

### F. Field tests

Verify:

- field array dimensions;
- exact requested bounds;
- mesh orientation;
- correct values at known nodes;
- invalid bounds and grid sizes;
- consistent output from equivalent evaluators.

Graphics tests may initially be limited to checking that valid graphics handles
are produced using an invisible test figure. Pixel-level plot testing is too
brittle for the first implementation.

**Status (2026-08-17):** Completed in `tests/TestMetrics.m`, including exact
square distances, diagnostics, boundary validation, concavity, FOV integration,
the Lipschitz property, field sampling, contour handles, and the static demo.

---

## Phase 6: Demonstration and documentation

Add:

```text
scripts/runDistanceContours.m
```

The script should:

1. Create `scenarios.singleWall`.
2. Cast rays.
3. Select bounds around the nominal FOV.
4. Evaluate the signed-distance field.
5. Plot the scenario without dense ray lines.
6. Overlay signed contours.
7. Emphasize the zero contour.
8. Display the sign convention in the title or colorbar label.

Update the README with a minimal usage example and, if useful, a generated
figure.

**Status (2026-08-17):** Complete. `scripts/runSingleScenario.m` performs
items 1--8 using the sampled visible boundary and
`metrics.signedEuclideanDistance`. It replaces the proposed separate
`runDistanceContours.m`, and the README now includes the equivalent usage.

---

## Phase 7: Add an interactive heading explorer

Add a moderate-complexity programmatic MATLAB interface that lets the user
change observer heading with a slider. Visibility should update while the
slider moves, but the signed-distance field and contours should be recomputed
only after the user releases the slider.

Prefer a text-based programmatic UI over an App Designer `.mlapp` file so the
interface remains easy to review and version. Add:

```text
+viz/interactiveScenario.m
scripts/runInteractiveScenario.m
```

`viz.interactiveScenario` should own the figure, controls, callback state, and
rendering workflow. The script should remain a small entry point that runs
`startup.m`, creates `scenarios.singleWall`, and launches the interface.

Use a reusable API such as:

```matlab
ui = viz.interactiveScenario(scenario, ...
    'GridSize', [241, 241], ...
    'ContourLevels', -5:0.5:5, ...
    'Padding', 0.5, ...
    'Visible', true);
```

Validate these options consistently with the existing visualization and field
functions. The `Visible` option permits headless graphics smoke tests.

### Initial scope

- [x] Create a `uifigure` containing a `uiaxes`, heading slider, numeric
  heading label, and update-status label.
- [x] Use heading degrees in the UI and convert to radians only when creating
  `fov.Observer` objects.
- [x] Give the slider limits `[-180, 180]` degrees and normalize the initial
  observer heading into that interval.
- [x] Recreate the immutable observer on each update while preserving its
  position and `Fov` object.
- [x] Keep observer position, maximum range, opening angle, obstacles, ray
  count, contour levels, and grid resolution fixed in this first interface.
- [x] Return a structure containing the figure, axes, and principal controls
  so the interface can be inspected and tested programmatically.

Observer dragging, obstacle editing, continuous contour updates, and a general
App Designer application are explicitly outside this phase.

### Fixed view and field domain

- [x] Compute one square domain centered on the observer using
  `MaxRange + padding` in each direction.
- [x] Use that domain for both metric sampling and fixed axes limits for every
  heading.
- [x] Do not derive bounds from the rotated nominal boundary; fixed bounds
  prevent the plot from jumping while the heading changes.
- [x] Use the existing full-quality defaults initially: the scenario ray count,
  a `[241, 241]` field grid, and contour levels `-5:0.5:5` unless options
  override them.

### Slider callback behavior

Use both slider callbacks with separate quality levels:

#### `ValueChangingFcn`: fast preview while dragging

1. Read `event.Value` in degrees and update the heading label.
2. Create the replacement observer and cast visibility rays.
3. Clear and redraw the nominal and visible FOV without a metric field,
   contours, or dense ray lines.
4. Restore the fixed axes limits.
5. Use `drawnow limitrate` to keep interaction responsive.

Do not call `metrics.sampleField` from `ValueChangingFcn`.

#### `ValueChangedFcn`: full update after release

1. Set the status label to indicate that contours are being computed.
2. Recreate or reuse the visibility result for the final slider value.
3. Create an evaluator using `metrics.signedEuclideanDistance` and the final
   `result.VisibleBoundary`.
4. Evaluate the full fixed-domain field with `metrics.sampleField`.
5. Redraw the scenario and metric contours through `viz.plotScenario`.
6. Restore fixed axes limits and update the title with the final heading and
   negative-inside convention.
7. Return the status label to ready even if rendering fails; use cleanup or
   guarded callback logic where appropriate.

The initial render should use the same full-update path as
`ValueChangedFcn`, ensuring startup and post-drag output cannot diverge.

### Graphics lifecycle

- [x] Start with a clear-and-redraw implementation rather than mutating every
  patch and contour object individually.
- [x] Remove or reuse an existing colorbar before redraw so repeated slider
  updates do not accumulate colorbars or axes.
- [x] Ensure legends contain scenario objects but not individual metric
  contour groups.
- [x] Prevent stale callbacks from leaving the UI in a partially updated state.
- [x] Keep UI orchestration in `viz.interactiveScenario`; continue to use
  `fov.castRays`, `metrics.signedEuclideanDistance`, `metrics.sampleField`,
  `viz.plotScenario`, and `viz.plotMetricContours` for their existing roles.

### Verification and acceptance criteria

- [x] Add a graphics smoke test that constructs the UI invisibly and verifies
  its returned figure, axes, slider, and labels are valid graphics objects.
- [x] Verify the initial view includes signed-distance contours.
- [x] Verify dragging changes heading and visible geometry without evaluating
  or drawing contours during `ValueChangingFcn`.
- [x] Verify releasing the slider recomputes contours for the final heading.
- [x] Verify axes limits remain fixed across multiple headings.
- [x] Verify repeated updates do not accumulate colorbars, legends, or stale
  graphics objects.
- [ ] Manually check that dragging is responsive and record any performance
  limitation without adding a machine-dependent timing assertion.
- [x] Document how to launch the interactive explorer in the README.

**Status (2026-08-17):** Implemented and covered by invisible-UI smoke tests.
The release callback queues its final heading if a preview is still rendering,
so the full contour update cannot be discarded by callback interleaving. Manual
responsiveness while dragging remains to be checked in an interactive MATLAB
desktop session.

---

## Phase 8: Quantify the two approximations separately

### Ray-count study

Implement the existing placeholder:

```text
scripts/compareRayCounts.m
```

Use fixed query points and ray counts such as:

```matlab
[31, 61, 121, 241, 481, 961]
```

For each ray count:

1. Cast the same scenario.
2. Build the sampled visible boundary.
3. Evaluate distance at exactly the same query points.
4. Compare against an analytical reference for an empty full-circle FOV or a
   very high-ray-count reference for an occluded scenario.
5. Record maximum error, RMSE, and runtime.

For an empty full-circle FOV, the analytical reference is:

\[
d(p) = \|p-o\| - R.
\]

The sampled polygon is inscribed inside the true circle, so convergence is
straightforward to visualize and quantify.

### Grid-resolution study

Add:

```text
scripts/compareGridResolutions.m
```

Keep one fixed visibility boundary and vary grid sizes such as:

```matlab
[51, 51]
[101, 101]
[201, 201]
[401, 401]
```

Compare interpolated field values against direct distance evaluations at fixed
off-grid probe points. This isolates grid interpolation from ray geometry.

Do not change ray count and grid size in the same convergence result.

---

## Phase 9: Improve visibility geometry after establishing the benchmark

Once the baseline is working, address occlusion-transition artifacts without
changing the signed-distance contract.

### Recommended ray-casting refactor

Separate angular selection from ray intersection:

```matlab
result = fov.castRayAngles(observer, obstacles, rayAngles);
```

Retain `fov.castRays` as the uniform-sampling wrapper:

```matlab
angles = fov.sampleRayAngles(observer, numRays);
result = fov.castRayAngles(observer, obstacles, angles);
```

This enables alternative samplers without duplicating intersection logic.

### Critical-angle sampling

For each obstacle vertex:

1. Compute its bearing from the observer.
2. Keep bearings inside the FOV.
3. Add rays at or immediately to either side of the bearing.
4. Merge them with uniform samples.
5. Sort and remove near-duplicates.

This should represent shadow boundaries more accurately than uniform sampling
alone. The direct Euclidean metric does not need to change; only the sampled
visible polygon improves.

---

## Recommended implementation sequence

- [x] Document the signed-distance contract.
- [x] Implement boundary normalization and point-to-segment distance.
- [x] Implement `metrics.signedEuclideanDistance`.
- [x] Complete exact metric tests.
- [x] Implement generic `metrics.sampleField`.
- [x] Implement `viz.plotMetricContours`.
- [x] Add the contour demonstration and README example.
- [ ] Add the interactive heading explorer with contours recomputed after
  slider release.
- [ ] Implement ray-count and grid-resolution studies.
- [ ] Add critical-angle sampling.
- [ ] Introduce additional metric contracts through the same field evaluator
  interface.

The order is intended to produce a usable benchmark quickly while preserving a
clean path toward alternative distance definitions and an eventual interactive
interface.

### Implementation note (2026-08-17)

Phases 1 through 6 are complete and the programmatically verifiable parts of
Phase 7 are complete. The full MATLAB suite passed on 2026-08-17. Manual
desktop responsiveness testing for slider dragging remains before Phase 7 can
be marked fully complete. Phase 8 convergence studies and later work have not
started.
