# Agent Instructions

## Project direction

This MATLAB project is primarily a probabilistic robotics testbed for
motion-aware first-occlusion-boundary field-of-view estimation. The target is
the observer-frame first-boundary range profile `B_t(theta)`, not observer
pose, follower/target pose, occupancy, or a persistent world map.

Checkpoint 1 is a deterministic foundation: a moving observer/follower, static
convex polygon obstacles, noiseless ray-cast scans, a pass-through estimator, a
replaceable policy, a headless deterministic runner, and offline replay.
Static signed-distance and contour functionality remains supported, but is
secondary/legacy for new development. Do not extend `viz.interactiveScenario`
or `scripts/runInteractiveScenario.m` as the primary workflow.

## Sources of truth and documentation

Read the relevant sources before changing implementation, tests, or docs:

- `README.md` — current supported usage and limitations.
- `docs/probabilistic_fov_project_roadmap(4).pdf` — research objective and
  stage order.
- `docs/fov_metrics_checkpoint_1_codex_plan.md` — detailed checkpoint-one
  contracts.
- `docs/IMPLEMENTATION_PLAN.md` — progress and history. Retain its historical
  static phases, but keep active-direction and status notes aligned with code.

No one document replaces the others. If a change affects a contract,
architecture, roadmap/status, or verification state, update the relevant docs
in the same change. Mark checklist items complete only after implementation
and actual verification; record unavailable or unrun MATLAB and graphics
verification honestly. The current checkpoint tests have not had a reported
passing rerun after fixes/additions, so do not imply that they have passed.

## Repository and package ownership

```text
+fov/          FOV models, obstacle validation, ray geometry, boundary conversion
  +internal/   Low-level geometry helpers
+sensing/      Measurement-only adapters, currently raycastScan
+estimation/   Replaceable boundary-estimator lifecycle
+simulation/   Scenario validation, schedules, kinematics, causal runner/logs
+control/      Observer-policy callbacks/seam
+metrics/      Computational metric contracts and spatial fields
+viz/          Summary, replay, and static rendering
+scenarios/    Reusable static and dynamic scenario factories
scripts/       Thin experiment/demo entry points
tests/         MATLAB unit tests
```

Keep geometry and FOV conversion under `+fov` (including
`fov.boundaryFromRanges`), sensing adapters under `+sensing`, estimation under
`+estimation`, simulation and logging under `+simulation`, policy seams under
`+control`, computation under `+metrics`, and rendering under `+viz`.
`scripts` should remain thin. Prefer plain structs, small functions, and
callbacks over abstract frameworks, plugin registries, or middleware. Useful
entry points include `simulation.runScenario`, `sensing.raycastScan`,
`estimation.makePassThroughEstimator`, `control.referencePolicy`, and
`scenarios.movingPair`.

## Non-negotiable conventions and contracts

- Use SI units where applicable and radians for angles. World poses are
  `[x,y,yaw]`; applied inputs are body-frame `[vxBody,vyBody,omega]`. Keep yaw
  unwrapped in state.
- Use zero-order-held forward Euler. Both agents advance from the same
  pre-step snapshot.
- Keep `simulation.runScenario` deterministic and headless: no figures,
  pauses/timers, unseeded randomness, or wall-clock physics. For `K` intervals,
  logs contain `K+1` synchronized poses/scans/beliefs and `K` interval inputs.
  The policy uses the current posterior before stepping; estimator `Predict`
  receives completed observer motion and the preceding applied input, then the
  next scan is passed to `Correct`.
- Rendering/replay consumes logs. Playback settings only change displayed
  samples and must always include the final frame. Offline logs are not
  estimator memory.
- The estimator lifecycle remains `Initialize`, `Predict`, `Correct`.
  Estimators receive scans and known motion through that lifecycle, not obstacle
  polygons, raw cast/oracle visibility, future inputs, or full log history.
  Reconstruct geometry/rendering from belief fields rather than copying raw
  casts.
- The current belief is boundary ranges on a relative angular grid with
  `Mean`, `Covariance`, `IsSupported`, `HasReturn`, and pose/FoV metadata. The
  pass-through implementation has `Mean = scan ranges` and sparse exact-zero
  covariance; it is not an EKF or a confidence claim. Its `Predict` is
  deliberately a no-op.
- `HasReturn = false` means censored range-cap/no-return information. It is not
  an obstacle at maximum range and is not initially a Gaussian equality.
  `IsSupported` means only that a direction has justified boundary information;
  it does not certify a physical return or visibility between rays. Do not
  fabricate a closed visible polygon from unsupported geometry.
- Raw casts are oracle data for simulation, evaluation, and rendering only.
- Preserve the signed-distance convention: negative inside, positive outside,
  and zero on the boundary, exact only for the sampled polygon. Keep ray-count
  geometry approximation separate from XY grid/contour interpolation.
- Preserve invalid-geometry behavior: unsupported, degenerate, or
  self-intersecting constructed boundaries produce invalid diagnostics/`NaN`
  distance rather than fabricated visibility. Unexpected errors should
  propagate.
- Checkpoint 1 requires equal scan and posterior angular grids. A future sparse
  sensor may deliberately separate `M` physical rays from `N` output bins, but
  only through deliberate contract, validation, test, and documentation
  changes—not accidental relaxation.

## Research guardrails

The roadmap sequence is: (1) dense ground truth plus noisy sparse sensing,
filtering/segmentation, and no-return handling; (2) deterministic within-
segment motion transport, then EKF covariance and support/reset/forgetting; (3)
reduced rays and history-assisted reconstruction, demonstrated with an existing
controller.

- Never interpolate or smooth across depth discontinuities. Associations and
  covariance coupling stay within supported surface segments.
- Newly exposed or unmeasured directions remain unsupported until observed or
  justified by within-segment interpolation. Forget surfaces that leave the
  current FoV; do not create persistent map memory.
- Confidence is pointwise and model-dependent, not a simultaneous
  visibility/safety guarantee.
- Unless explicitly planned and documented, do not add SLAM/occupancy grids,
  persistent maps or scan archives, hidden-surface models, controller redesign
  or proofs, pursuit optimization, CBF/QP, collision or actuator dynamics,
  base-link guarantees, ROS/CrazySim, App Designer/interactive physics, global
  smoothing, or video export.
- Do not add a required toolbox or dependency without explicit justification
  and corresponding documentation/tests.

## Development workflow

Inspect relevant implementation, tests, and docs first. Make the smallest
coherent change, preserve unrelated user modifications and existing package
APIs unless deliberately migrating them, and reuse current validation and
MATLAB error-naming conventions. Keep computation separate from plotting.
Test the relevant seams: synchronization, coordinate/angle conversion,
no-return/support semantics, segmentation and depth jumps, covariance
dimensions/validity, exposure/reset behavior, deterministic/headless behavior,
and estimator-bypass prevention as applicable.

For focused simulation changes, the documented MATLAB command is:

```matlab
run('startup.m');
results = runtests('tests/TestSimulation.m');
table(results)
```

The full suite is `runtests('tests')`. For graphics-affecting changes, also
manually replay `scenarios.movingPair()` on a desktop. State exactly what ran
and passed; if MATLAB or graphics are unavailable, do not claim success.
Do not build or compile routinely, and do not commit, push, or create a pull
request unless explicitly requested.
