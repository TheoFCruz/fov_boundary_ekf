# Occluded FOV Metrics

MATLAB project for studying metrics derived from a field of view (FOV)
occluded by convex polygonal obstacles.

The project is being built incrementally. The initial models, ray-casting
visibility, scenarios, and visualization are implemented; metrics are next.

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

## Current model API

The initial models, geometry, ray casting, scenarios, and plotting are
implemented. Metrics and comparison experiments remain placeholders:

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

## First visualization

The complete first-pass workflow is available in
`scripts/runSingleScenario.m`. It can also be reproduced interactively:

```matlab
scenario = scenarios.singleWall();
result = fov.castRays( ...
    scenario.Observer, scenario.Obstacles, ...
    'NumRays', scenario.NumRays);
viz.plotScenario(scenario, result, ...
    'ShowRays', true, ...
    'ShowNominalFov', true, ...
    'ShowHitPoints', true);
```

Other demonstration scenarios are available as
`scenarios.emptyField()` and `scenarios.clutteredField()`.
