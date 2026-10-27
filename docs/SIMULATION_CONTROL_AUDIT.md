# Simulation and control audit (2026-10-27)

Read-only audit of the checkpoint-one simulation, CBF-QP policy, FoV
construction, and replay, done before starting the boundary EKF. Nothing in the
code was changed. Numbers below come from headless MATLAB probes (scratch
scripts, not committed) run on `scenarios.controlTest` and
`scenarios.movingPair` with `DistanceMargin = 1.0`, `CbfRate = 2`.

## Summary

The jagged controlled motion comes from the barrier gradient in
`+control/makeSignedDistanceCbfPolicy.m`, not from the integrator or runner:

1. The finite-difference step (`PoseFiniteDifferenceStep = [1e-3; 1e-3; 1e-5]`)
   is far below one beam spacing, so the gradient sees the sampled polygon's
   artifacts instead of the geometry.
2. The gradient holds the local boundary frozen in the body frame, so obstacle
   edges translate and rotate rigidly with the observer. This is only correct
   for aperture side edges and range-cap edges.

Integration (`simulation.stepPose`), the causal runner order, ray casting,
noise handling, and `dt = 0.05` were checked and are fine.

## Evidence

### Range-arc chatter (`controlTest`)

The target sits on the max-range edge, which the sampled polygon draws as
chords between ray endpoints. With a `1e-5` rad step, dh/dpsi is about ±0.09
and changes sign each time the target crosses a chord vertex (true value on a
continuous arc: 0). The QP follows it.

| FD steps (x, y, psi) | omega sign flips (199 steps) | mean abs(d omega) | min h |
|---|---|---|---|
| `[1e-3, 1e-3, 1e-5]` (current) | 98 | 0.120 | -0.023 |
| `[0.05, 0.05, 0.02]` (~1 beam) | 0 | 0.000 | 0.000 |

### Wrong gradient sign near occluders

Observer at the origin, wall of `movingPair` (x in [4, 4.3], y in [-2, 2]),
81 rays over 100 deg, target at (8, 3.9), just inside the shadow of corner
(4, 2):

| Method | dh/dx | dh/dy | dh/dpsi |
|---|---|---|---|
| Current policy (frozen body frame, step 1e-3) | 0.48 | -0.88 | -8.9 |
| Oracle re-cast, step 1e-3 | 0.47 | -0.88 | -8.9 |
| Oracle re-cast, step 0.1 m / 0.0125 rad | -0.51 | +1.06 | -1.1 |
| Continuous geometry (shadow line through corner) | about -0.45 | about +0.89 | 0 |

Even the oracle sampled polygon has the wrong sign at small steps: its shadow
edge is a slanted chord from the near wall hit to the far range-cap endpoint,
and that far endpoint moves rigidly with the body. With steps of about one
beam spacing at the target range, the sign recovers.

Target directly behind the wall face, (5, 0.5): the frozen-body gradient gives
dh/dx = 1 (drive toward the wall); the true value is 0.

### Scenarios do not exercise occlusion

In `movingPair` the nearest polygon edge is always an aperture side edge and
the follower never approaches the wall. Runs with `RangeNoiseStd = 0` and
`0.1` produce identical closed-loop trajectories and statuses.

## Recommended changes (priority order)

1. **Gradient step and method.** Scale perturbations to the beam spacing
   (about dtheta in psi, about rho_target * dtheta in position) and fit the
   gradient by least squares over several displacements, as specified in
   `docs/metodologia.pdf` (eq. 17-18). Remove the 2 % forward/backward
   consistency rejection in `finiteDifference`; it rejects exactly the kinks
   least squares is meant to handle and triggers the zero-command fallback.
   Fixes the chatter at aperture and range edges.
2. **Boundary model under perturbation.** Replace the frozen body-frame
   boundary with the methodology's approach: run the prediction (mean
   transport) for each perturbation and rebuild the visible set. Build the
   deterministic mean transport first (it is the EKF predict step anyway) and
   reuse it in the policy. Until then, occlusion-driven control is not
   trustworthy. For the oracle boundary source, re-casting against obstacles
   (as in Zhou et al.) is an acceptable alternative.
3. **No polygon bridges across depth jumps.** `fov.boundaryFromRanges` and
   `fov.castRays` (`VisibleBoundary`) connect consecutive endpoints across range
   jumps, which `AGENTS.md` forbids. Add the conservative step construction
   from `docs/matlab-implementation-plan.md` section 6: at a jump, go radially
   out along the farther ray at the nearer range.
4. **True-visibility scoring.** `Metrics.SampledPolygonVisible` in
   `simulation.runScenario` scores against the belief polygon. The four-source
   comparison needs an exact check: line of sight from observer to target
   against obstacle edges, plus aperture and max-range tests.
5. **Slack status label.** With the quadratic slack penalty, slack is about
   4e-5 on every step where the constraint is active, so
   `"relaxed-with-slack"` is reported on 161 of 200 steps and carries no
   information. Use a relative threshold, or report slack only when the input
   bounds make the hard constraint infeasible.
6. **Fallback command.** The zero-command fallback stops the observer whenever
   the nominal is non-zero, giving stop-go motion. Prefer holding the previous
   command or the clipped nominal. Less important once item 1 is done.
7. **Occlusion scenario.** Add plan scenario S2 (box in front of a wall) or S3
   (corridor corner) with the target passing behind the occluder before any
   controller tuning.

Minor replay QoL: show h and the policy status in the world-panel title of
`viz.animateSimulation`.

## Not verified

- No fix above has been implemented or tested.
- The focused and full MATLAB test suites were not run as part of this audit.
- Gradient comparisons were spot checks at a few target positions, not a
  sweep.

## Implementation status (2026-10-27)

All seven recommended changes above, plus the replay QoL item, were
implemented and headlessly verified. `runtests('tests')` passed 70/70.

1. **Gradient step and method (done).** `control.makeSignedDistanceCbfPolicy`
   now scales observer-pose and target perturbations to the sensor's own
   `dtheta` (via `config.GradientBeamMultiples`, default `[1, 2]`) and fits
   `dh/dpose` and `dh/dtarget` by least squares over the finite samples. The
   2% forward/backward consistency rejection is gone.
2. **Boundary model under perturbation (done, oracle variant).** Added
   `config.BoundarySource` and `control.makeOracleBoundarySource`, which
   re-casts against the true obstacles at each queried pose (Zhou et al.
   style), used by `scenarios.controlTest` and the new
   `scenarios.boxInFrontOfWall`. The default (no `BoundarySource`) still uses
   the frozen belief polygon, now evaluated directly in world frame instead
   of the observer-local frame. EKF mean transport is not implemented; it can
   plug into the same seam later.
3. **No polygon bridges across depth jumps (done).** `fov.boundaryFromRanges`
   takes an optional `isJump` flag vector and inserts a single conservative
   step vertex (nearer range, farther ray direction) instead of a diagonal
   chord; `fov.castRays` computes `IsDepthJump` from obstacle-identity changes
   and uses it to build `VisibleBoundary`. Belief polygons are unaffected
   (still built without jump flags; segmentation is future work).
4. **True-visibility scoring (done).** Added `fov.isPointVisible` (range,
   aperture, and edge-occlusion check) and `simulation.runScenario`'s
   `Metrics.TrueVisible`.
5. **Slack status label (done).** Status is now classified from the
   constraint geometry (`coefficient * clippedReference` vs. `rhs`, and
   `maximumContribution` vs. `rhs`), not slack magnitude.
6. **Fallback command (done).** Every fallback path (invalid geometry,
   unusable gradient, zero control authority, QP failure) now returns the
   reference clipped to the input bounds.
7. **Occlusion scenario (done).** Added `scenarios.boxInFrontOfWall` (plan
   scenario S2): a box occluder in front of a far wall, zero observer
   reference, follower walking through the box's shadow.

Replay QoL: `viz.animateSimulation`'s world-panel title appends
`h = %.3f | <status>` from the displayed interval's `PolicyDiagnostics` when
present (the final logged sample reuses the last interval's diagnostics).

### Measured gradient checks (oracle source, `fov.FovSpec(10, deg2rad(100))`,
81 rays, `movingPair` wall `[4,-2;4,2;4.3,2;4.3,-2]`, default `DistanceMargin`)

| Case | dh/dx | dh/dy | dh/dpsi |
|---|---|---|---|
| Target (8, 3.9), corner shadow | -0.538 | 0.809 | n/a |
| Continuous geometry (for reference) | about -0.45 | about +0.89 | n/a |
| Target (5, 0.5), behind wall face | ~0.0000 (|·| < 0.2 bound; true value 0) | — | — |
| No-obstacle target near heading, range ~10 | — | — | ~0.0000 (|·| < 0.05 bound) |

All three match the continuous-geometry sign/magnitude expectations from the
original audit far better than the old sub-beam finite difference.

### Closed-loop numbers (not tuned)

| Scenario | omega sign flips | fraction `TrueVisible` | status counts | min `BarrierValue` | wall-clock runtime |
|---|---|---|---|---|---|
| `scenarios.controlTest` (empty obstacles, oracle source) | 0 / 200 | 1.0000 | nominal-feasible 88, cbf-corrected 112 | -0.0001 | 23.0 s |
| `scenarios.boxInFrontOfWall` | 0 / 320 | 0.8380 | nominal-feasible 97, cbf-corrected 94, relaxed-with-slack 129 | -0.2923 | 50.0 s |

In `scenarios.boxInFrontOfWall` the target is truly hidden for t = 7.65 to
10.20 s (52 samples). Those steps report `relaxed-with-slack`, meaning the
hard CBF constraint is infeasible within the input bounds; this is a
controller-authority/tuning question, not a geometry failure. A first draft of
the step construction produced self-intersecting polygons (out-and-back spikes
where a far ray sat at the aperture edge or between two jumps to nearer rays);
`fov.boundaryFromRanges` now pulls such rays in to their farther near
neighbour, and no invalid-geometry fallbacks remain. Runtime is dominated by
the ~13 oracle re-casts per controller step.

### Not verified

- Desktop graphics replay of `scenarios.movingPair()` and
  `scenarios.boxInFrontOfWall()` (headless session only).
- The gradient and closed-loop numbers above are spot checks and two full
  closed-loop runs each, not a parameter sweep.
