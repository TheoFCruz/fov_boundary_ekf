# Occluded FOV Metrics

MATLAB project for studying metrics derived from a field of view (FOV)
occluded by convex polygonal obstacles.

The project is being built incrementally. Ray-cast visibility, signed Euclidean
distance fields, contour visualization, and an interactive heading explorer are
implemented; additional metric contracts and convergence experiments are next.

## Structure

```text
+fov/          FOV and obstacle model plus visibility computation
  +internal/   Testable low-level geometry helpers
+metrics/      Metrics computed from visibility results
+viz/          Visualization helpers
+scenarios/    Reusable scenario definitions
scripts/       Experiment entry points
tests/         Automated tests
```

Angles will use radians and positions will use Cartesian `[x, y]` coordinates.

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

## Interactive heading explorer

Launch the heading explorer with:

```matlab
run('scripts/runInteractiveScenario.m')
```

Move the heading slider for a fast visible-FOV preview. Releasing it computes
the signed-distance field and contour overlay for the selected heading.
