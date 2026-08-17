# Agent Instructions

## Main implementation plan

Read [`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md) before making
changes related to signed-distance metrics, spatial fields, contour plots,
visibility sampling, or the eventual interface. That file is the main plan for
this repository.

When completing work from the plan:

1. Identify the relevant phase and checklist items before editing.
2. Implement the smallest coherent step.
3. Run the relevant MATLAB tests or checks when possible.
4. Update `docs/IMPLEMENTATION_PLAN.md` in the same change:
   - mark every completed checklist item as `[x]`;
   - leave partially completed items as `[ ]`;
   - add a brief note for deviations, known gaps, or verification that could
     not be performed.
5. Do not mark a step complete based only on intent; implementation and
   verification must be finished first.

If new work changes the design, update the plan rather than allowing the code
and plan to diverge.

## Repository structure

This is a small MATLAB package for studying metrics derived from a field of
view occluded by convex polygonal obstacles.

```text
+fov/          FOV models, obstacle validation, and visibility computation
  +internal/   Low-level ray/segment and polygon-edge geometry helpers
+metrics/      Metric contracts and spatial metric-field computation
+viz/          Plotting and contour visualization helpers
+scenarios/    Reusable observer/obstacle scenario factories
scripts/       Demonstration and comparison entry points
tests/         MATLAB unit tests
README.md      Project overview and currently supported usage
startup.m      Adds the repository and tests to the MATLAB path
```

Important existing entry points include:

- `fov.FovSpec` and `fov.Observer` for FOV model data.
- `fov.polygonObstacle` for validated convex polygon obstacles.
- `fov.sampleRayAngles` for uniform angular sampling.
- `fov.castRays` for sampled visibility and `result.VisibleBoundary`.
- `viz.plotScenario` for plotting nominal and visible FOV geometry.
- `scenarios.emptyField`, `scenarios.singleWall`, and
  `scenarios.clutteredField` for reusable examples.

The current metric files and comparison scripts may be placeholders. Preserve
the existing package boundaries: geometry belongs under `+fov`, metric
calculation under `+metrics`, rendering under `+viz`, and experiments under
`scripts`.

## Development practices

- Inspect relevant files before editing.
- Prefer small, focused changes and avoid unrelated churn.
- Keep computational metrics separate from plotting and UI code.
- Preserve the initial signed-distance convention unless the plan is explicitly
  updated: negative inside, positive outside, and zero on the boundary.
- Keep ray-count approximation and grid/contour approximation conceptually
  separate in APIs, tests, and comparison scripts.
- Reuse existing validation and MATLAB package naming conventions.
- Add or update tests for new behavior, especially geometry edge cases and
  metric contracts.
- Do not build or compile changes as part of routine work; leave that to the
  user. Run focused tests when useful and report any verification gap.
- Do not commit, push, or create pull requests unless explicitly requested.

## Verification

From MATLAB, the repository's documented test workflow is:

```matlab
run('startup.m');
results = runtests('tests');
table(results)
```

For a focused change, run the smallest relevant test class or test selection,
then report what was and was not verified. If MATLAB is unavailable, do not
claim that tests passed; document the gap in the plan when the work corresponds
to a plan item.
