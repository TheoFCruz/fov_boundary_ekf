# FOV Metrics Implementation Plan

This is the execution plan for the MATLAB first-boundary field-of-view (FoV)
testbed. The active target is the observer-frame first-occlusion-boundary range
profile `B_t(theta)`, together with its support and uncertainty—not observer or
follower estimation, occupancy, a persistent world map, or a new controller
research program.

The repository already contains the checkpoint-one plumbing and the historical
static signed-distance workflow. Milestone A's implementation is now present:
noisy measurements and the distinction between oracle, measurement, and belief
are visible before adding filtering. Milestone-A verification remains pending;
Milestone B is the next planned implementation. Each checklist item is marked
`[x]` only when the described implementation exists **and** the relevant
verification has actually been run. An implemented but unverified item stays
unchecked with an honest note.

## 1. Active direction and current status

The active workflow is the headless moving-pair simulation, followed by replay.
It supersedes further development of the interactive heading explorer. The
static signed-distance, field, contour, and UI phases remain supported as
historical/legacy material in Section 8. Milestone A is implementation-complete
but awaiting its focused and desktop verification; Milestone B is the next
planned milestone. Before Milestone B implementation, the concrete
signed-distance control law, reference-combination rule, finite input bounds,
and invalid-case fallback still require an explicit design decision.

### Current checkpoint-one state

The current checkout contains the following foundation:

- `simulation.runScenario` is a headless, causal, zero-order-held forward-
  Euler runner for a moving observer and follower.
- `sensing.raycastScan` adapts first-return ray casts into a measurement
  structure. It supports return-only Gaussian range noise clamped to
  `[0, MaxRange]`; no-return caps remain unchanged. `fov.castRays` remains the
  noiseless geometry/oracle layer.
- `estimation.makePassThroughEstimator` implements `Initialize`, `Predict`,
  and `Correct`. Its mean copies scan ranges and its sparse covariance is
  exactly zero; it is not an EKF or a confidence claim.
- `control.referencePolicy` is the replaceable policy seam and currently
  returns the configured observer reference.
- Logs contain synchronized `K+1` poses, scans, beliefs, and raw casts for `K`
  intervals. Replay consumes those logs and does not feed history back into an
  estimator. Replay maintains distinct persistent oracle, measurement, and
  belief boundary and range artists.
- `viz.animateSimulation` updates persistent graphics handles and always
  includes the final sample. The shipped replay pacing optimization avoids
  populating disabled ray/impact artists and subtracts render time from the
  requested inter-frame delay. The user has manually observed substantially
  faster replay; automated MATLAB verification of this optimization is still
  unreported.

The Milestone-A implementation is present, but checkpoint acceptance remains
open. Focused tests were added for sensor validation, deterministic same-seed
scans, preservation of the global RNG, preservation of oracle and no-return
data, explicit clipping, replay without resampling, oracle/measurement/belief
source separation, labels and styles, and fixed artist count. Those tests have
not been run. No current focused-test, full-suite, or formal desktop graphics
pass should be inferred from the existence of the code. The user's comment
“That looks great” is recorded as positive manual visual observation/approval
of the Milestone-A result only; it is not a formal desktop graphics or
final-frame verification.

### Foundation acceptance checklist

- [ ] Run the focused checkpoint tests after the latest fixes/additions and
  record the actual result.
- [ ] Run the full MATLAB suite and record the actual result.
- [ ] Manually replay `scenarios.movingPair()` on a desktop and record the
  graphics result, including final-frame inclusion and replay responsiveness.
- [ ] Keep the checkpoint limits explicit: no EKF, pursuit optimization,
  collision response, base-link guarantee, map, occupancy representation, scan
  archive, or hidden-surface model.

## 2. Project contracts that remain in force

### Units, state, and causal order

- Use SI units where applicable and radians for angles.
- World poses are finite `[x, y, yaw]` rows with unwrapped yaw. Applied inputs
  are body-frame `[vxBody, vyBody, omega]` rows.
- Use zero-order-held forward Euler. Both agents advance from the same
  pre-step snapshot.
- For `K` intervals, logs contain `K+1` synchronized poses, scans, and beliefs
  plus `K` interval inputs. The policy sees the current posterior before the
  step. `Predict` receives completed observer motion and the preceding applied
  input; the next scan then goes to `Correct`.
- The runner is deterministic and headless: no figures, timers, pauses, wall-
  clock physics, or unseeded randomness.

### Measurement, belief, and geometry

- The current boundary state is a range vector on a relative angular grid. A
  belief contains `Mean`, `Covariance`, `IsSupported`, `HasReturn`,
  `ObserverPose`, `MaxRange`, `OpeningAngle`, and method/time metadata.
- `HasReturn=false` is censored range-cap/no-return information. It is not an
  obstacle at maximum range and is not initially a Gaussian equality.
- `IsSupported` means only that boundary information is justified in that
  direction. It does not certify a physical return or visibility between rays.
  Unsupported directions must not be silently closed into a visible polygon.
- Checkpoint one keeps the scan and posterior angular grids equal. A later
  sparse sensor may deliberately use `M` physical rays and `N` output bins, but
  only through a separately validated and documented contract change.
- Raw casts are oracle data for simulation, evaluation, and rendering only.
  Estimators receive scans and known motion through the lifecycle, never
  obstacle polygons, raw casts, future inputs, or full logs.
- Belief-derived geometry uses `fov.boundaryFromRanges`; rendering and policy
  code must not bypass the estimator by copying a raw cast boundary.
- Signed distance keeps the convention negative inside, positive outside, and
  zero on the sampled polygon. Invalid, unsupported, degenerate, or
  self-intersecting constructed boundaries produce invalid diagnostics/`NaN`
  distance rather than fabricated visibility. Unexpected errors propagate.

### Ownership map

Keep responsibilities in the existing packages:

| Package | Ownership |
| --- | --- |
| `+fov` | FOV models, obstacle validation, ray geometry, and boundary conversion |
| `+fov/internal` | Low-level geometry helpers |
| `+sensing` | Measurement-only adapters, currently `raycastScan` |
| `+estimation` | `Initialize`/`Predict`/`Correct` estimators and current boundary belief |
| `+simulation` | Scenario validation, schedules, kinematics, causal runner, and logs |
| `+control` | Observer-policy callbacks and the policy seam |
| `+metrics` | Computational metric contracts and spatial fields |
| `+viz` | Summary, replay, and static rendering |
| `+scenarios` | Reusable static and dynamic scenario factories |
| `scripts` | Thin experiment/demo entry points |
| `tests` | MATLAB unit and graphics-smoke tests |

Prefer plain structs, small functions, and callbacks. Do not add a required
toolbox or dependency.

## 3. Roadmap alignment and intentional sequence

The revised semester roadmap in
[`probabilistic_fov_project_roadmap(4).pdf`](probabilistic_fov_project_roadmap%284%29.pdf)
defines three research stages:

1. dense ground truth with noisy sparse sensing, filtering/segmentation, and
   no-return handling;
2. deterministic within-segment motion transport followed by EKF covariance,
   support, reset, and forgetting behavior; and
3. reduced rays, history-assisted reconstruction, and a demonstration with the
   existing controller.

The implementation sequence below inserts a minimal signed-distance controller
baseline between sensing visualization and the EKF. This is a deliberate,
slight deviation from the controller-last ordering: it validates the complete
belief-to-policy seam early and makes the value of filtering visible. It does
not change the research goal or authorize controller redesign. After the EKF,
baseline control behavior must be compared using pass-through versus EKF
beliefs; only then should sparse/history-assisted reconstruction and the final
existing-controller demonstration proceed.

## 4. Active implementation sequence

All roadmap acceptance work below remains unchecked until implementation and
the relevant verification are both complete. The substeps are ordered
requirements, not optional suggestions.

### Milestone A — deterministic range noise and clear replay visualization

**Purpose.** Add the first experimental variation without adding a new
persistent/evaluation-metrics program. The implemented result makes the
difference among truth, sensed data, and estimated boundary unambiguous in an
animation.

#### Implemented sensor contract

The sensor configuration now has a zero-noise default and an explicit seed:

```matlab
scenario.Sensor.RangeNoiseStd = 0;
scenario.Sensor.Seed = 0;       % explicit run-local seed
```

The implemented field names and validation follow the existing MATLAB
conventions. The behavior is:

- `RangeNoiseStd=0` reproduces the current noiseless measurement behavior.
- A positive standard deviation adds zero-mean Gaussian measurement noise only
  to actual first returns (`HasReturn=true`). A no-return sample remains
  censored range-cap data with `HasReturn=false`; it must not receive a
  Gaussian equality or become an artificial surface at the cap.
- The raw result from `fov.castRays` remains noiseless oracle data. It is kept
  separate from the noisy `scan` and from the belief.
- The simulation owns the run-local random stream. A seed makes repeated runs
  reproducible, and sampling must not mutate MATLAB's global RNG state. Noise
  is sampled exactly once while the simulation creates each scan. Replay and
  `handles.UpdateFrame` only display logged values and never resample.
- The equal scan/output angular-grid contract remains unchanged. No sparse-grid
  interpolation, smoothing, or cross-ray coupling is introduced by this
  milestone.
- Additive Gaussian samples on returns are clipped to `[0, MaxRange]`, keeping
  scan values finite and physically usable. This policy does not alter
  no-return semantics.

Noise belongs in `+sensing/raycastScan` (or its measurement-only helper), not in
`+fov/castRays`. `simulation.runScenario` passes a seeded run-local
`RandStream`, causing one sample per scan without exposing oracle geometry to
the estimator.

#### Visualization contract

Use persistent handles and the existing replay frame-selection/final-frame
behavior. Clearly distinguish all three data products in both views:

- **World view:** noiseless oracle boundary/rays from the raw cast; noisy scan
  endpoints/boundary reconstructed from `scan.Angles` and `scan.Ranges`; and
  belief-estimated boundary reconstructed from `belief.Mean`. Use distinct
  colors, line styles, and legend labels. Do not hide coincidence when noise is
  zero.
- **Boundary view:** show separate curves for the noiseless oracle, noisy scan
  measurement, and belief mean, with distinct labels/styles; retain the
  covariance annotation/band when a future estimator provides one.
- Unsupported belief portions remain hidden or segmented rather than joined
  into a closed polygon. A noisy scan itself may be rendered only according to
  its valid measurement contract.

Keep `ShowRays`/`ShowHitPoints` semantics, fixed graphics handles, and the
shipped optimization. Do not add video export, persistent scan archives, or
new evaluation metrics. Existing checkpoint diagnostics may remain; this
milestone adds only transient display state needed to explain the three curves.

#### Milestone-A checklist and acceptance

- [ ] Validate zero-noise default, positive-noise validation, explicit seed,
  and run-local reproducibility without global-RNG mutation.
- [ ] Verify noise is applied only to first returns and no-return flags/range
  caps remain censored data.
- [ ] Verify raw casts are unchanged/noiseless and replay does not resample.
- [ ] Verify noisy scan and belief geometry are both sourced from their own
  fields, not raw-cast geometry.
- [ ] Verify world and boundary legends/styles distinguish oracle, noisy scan,
  and belief estimate while artist counts remain fixed across frames.
- [ ] Verify zero-noise replay is backward-compatible with the current
  checkpoint display.

**User-run verification after implementation:**

```matlab
run('startup.m');
scenario = scenarios.movingPair();
scenario.Sensor.RangeNoiseStd = 0;
scenario.Sensor.Seed = 17;
first = simulation.runScenario(scenario);
second = simulation.runScenario(scenario);
assert(isequal(first.Scans, second.Scans));
scenario.Sensor.RangeNoiseStd = 0.05;
noisy = simulation.runScenario(scenario);
viz.animateSimulation(noisy, 'ShowRays', true, 'ShowHitPoints', false);
```

Also run the focused test command in Section 5, inspect a desktop replay with
noise enabled and disabled, and record actual outcomes. Do not treat this
manual visualization check as an automated or full graphics pass.

**Implementation status (2026-09-10):** `RangeNoiseStd` and `Seed` are now
validated sensor fields with zero-noise defaults in `scenarios.movingPair`.
`simulation.runScenario` owns a seeded local `RandStream`; `sensing.raycastScan`
adds clipped Gaussian noise only to first returns and leaves capped no-returns
unchanged. Replay distinguishes oracle, measurement, and belief boundaries and
ranges with persistent artists. The focused tests listed above were added but
not run, so all Milestone-A checklist items remain unchecked pending the
focused commands and desktop replay. The positive user observation “That looks
great” does not establish a formal graphics, final-frame, focused-test, or
full-suite result.

### Milestone B — minimal signed-distance controller baseline

Implement a deliberately small end-to-end controller through the existing
`+control` policy seam, before the EKF. It should use the current belief-derived
geometry and current observation only. In particular, it must never consume raw
casts, obstacle polygons, future inputs, or full logs. The known follower-pose
assumption remains explicit: in this baseline the policy may use the current
follower pose supplied by the checkpoint observation, but this is an idealized
known-target assumption, not target estimation.

The baseline should evaluate signed Euclidean distance to the estimated FoV
constructed from the belief. The sampled-polygon distance is only an
approximation to continuous visibility. Distance and control diagnostics are
transient policy/visualization diagnostics; they are not a request to begin a
new persistent metrics or evaluation program.

The concrete signed-distance control law, reference-combination rule, finite
input bounds, and invalid-case fallback must be selected and documented
**before coding**. Signed distance alone is a scalar diagnostic, not a control
input. The policy contract must specify what happens for each invalid case:

- if the belief has unsupported directions or the constructed boundary is
  degenerate, self-intersecting, or otherwise invalid, return an explicit safe
  fallback (the initial baseline should use a bounded zero observer command)
  and diagnostics that identify the fallback;
- do not fabricate a closed polygon or silently substitute raw visibility;
- describe the fallback as an operational fail-safe for this experiment, not a
  collision, visibility, or safety guarantee.

The baseline is not a visibility/safety proof, CBF/QP effort, pursuit
optimizer, actuator/collision model, or controller redesign. It must not add
base-link guarantees. Preserve the known follower-pose and sampled-polygon
limitations in its user-facing diagnostics.

#### Milestone-B checklist and acceptance

- [ ] Select and document the control law, reference use, bounds, and fallback
  before implementation.
- [ ] Implement the policy under `+control`; keep the runner's observation and
  callback seam replaceable.
- [ ] Prove by injected-estimator tests that policy geometry follows belief
  output rather than raw casts.
- [ ] Test valid, unsupported, degenerate, and self-intersecting geometry
  behavior and explicit bounded fallback diagnostics.
- [ ] Confirm no policy input includes obstacles, raw casts, future inputs, or
  full logs.
- [ ] Show the baseline operating with the current pass-through belief without
  adding persistent metrics.

**User-run verification after implementation:**

```matlab
run('startup.m');
scenario = scenarios.movingPair();
scenario.Observer.Policy = @control.signedDistanceBaseline;
result = simulation.runScenario(scenario);
viz.plotSimulationSummary(result);
viz.animateSimulation(result, 'ShowRays', false, 'ShowHitPoints', true);
results = runtests('tests/TestSimulation.m');
table(results)
```

The planned entry point above is only to make the handoff command concrete;
the control law and bounds must be selected and documented before that policy
is coded. Manually inspect a valid run and an invalid/unsupported-belief
fallback; record that inspection separately from MATLAB test results.

### Milestone C — motion-aware first-boundary EKF

The EKF is the third major milestone and must be implemented in the following
ordered substeps. Do not collapse them into one untestable estimator rewrite.

1. **Surface segmentation and depth jumps.** Identify supported first-surface
   segments and discontinuities before interpolation. Never interpolate,
   smooth, or couple covariance across an obstacle-silhouette/depth jump.
2. **Explicit support and no-return semantics.** Keep support separate from
   `HasReturn`. A censored no-return is not initially a Gaussian equality;
   newly exposed or unjustified directions stay unsupported.
3. **Fixed-view Gaussian measurement correction.** With the observer view held
   fixed, correct valid first-return bins using a documented measurement noise
   model and dimensions. Restrict associations/covariance coupling to a single
   supported segment.
4. **Deterministic within-segment rotation/translation transport.** Transport
   supported boundary points into the new observer frame using known completed
   motion. Resample only within the same segment, with deterministic behavior
   at valid associations.
5. **Jacobians and covariance propagation.** Add local transition Jacobians,
   process noise, and covariance propagation only where the segment and
   interpolation associations remain unchanged. Validate symmetry, dimensions,
   finiteness, and nonnegative variances.
6. **Exposure, FoV exit, reset, and forgetting.** Newly exposed directions get
   an explicit unsupported/reset policy; surfaces leaving the current FoV are
   forgotten. Remove stale cross-correlations instead of retaining hidden
   surfaces for later reuse.

The EKF state remains only the current observer-frame boundary belief and its
support/uncertainty. Do not add a world map, occupancy grid, persistent map
memory, scan archive, hidden-surface model, or global smoothing. Keep equal
scan/output grids through this milestone; a later sparse-grid contract requires
its own validation, tests, and documentation.

#### Milestone-C checklist and acceptance

- [ ] Implement and test segmentation/depth-jump boundaries before any
  cross-ray interpolation or covariance coupling.
- [ ] Implement and test support, return, and censored no-return behavior.
- [ ] Implement fixed-view correction and verify measurement-noise dimensions
  and covariance validity.
- [ ] Implement deterministic within-segment rotation and translation
  transport, including pure rotation, translation toward/along a wall, and
  corner exposure cases.
- [ ] Add local Jacobians and covariance propagation with tests for symmetry,
  finite values, and correct dimensions.
- [ ] Implement newly exposed-surface reset, FoV exit, and forgetting tests.
- [ ] Verify the estimator receives only scans and motion through
  `Initialize`/`Predict`/`Correct`, never oracle geometry or log history.
- [ ] Verify unsupported portions are not rendered or evaluated as a closed
  visible polygon.

**User-run verification after implementation:**

```matlab
run('startup.m');
results = runtests('tests/TestSimulation.m');
table(results)
scenario = scenarios.movingPair();
scenario.Sensor.RangeNoiseStd = 0.05;
scenario.Sensor.Seed = 17;
result = simulation.runScenario(scenario);
viz.animateSimulation(result, 'ShowRays', false, 'ShowHitPoints', true);
```

Add focused EKF tests to the appropriate test class before using this command;
record the actual focused result and the desktop replay result.

### Milestone D — belief comparison, sparse rays, and final demonstration

After Milestone C, compare the signed-distance baseline's behavior when driven
by pass-through belief versus EKF belief. Keep comparison diagnostics scoped to
the experiment and visualization; do not turn them into a new metrics program.
Only after that comparison:

- deliberately reduce physical ray count and introduce history-assisted
  reconstruction with a documented `M`-ray/`N`-output contract;
- report unsupported directions explicitly rather than improving apparent
  error by withholding predictions; and
- supply the resulting first boundary to the existing controller for the final
  demonstration. This is an interface demonstration, not controller
  redesign, proof, pursuit optimization, or a safety guarantee.

#### Milestone-D checklist and acceptance

- [ ] Compare pass-through and EKF belief under the same scenario and control
  law before reducing rays.
- [ ] Add sparse-grid validation, segmentation-aware history reconstruction,
  and explicit unsupported-direction reporting.
- [ ] Demonstrate the existing controller using the final estimated boundary.
- [ ] Record finite-return error, supported fraction, exposure recovery, and
  runtime only if the project later explicitly authorizes that evaluation
  study; these are not current work.

## 5. Verification plan and commands

No verification command has been run as part of this documentation update.
After each implementation milestone, run the smallest relevant focused suite,
inspect the output, and update this document with the actual date and result.
If MATLAB or desktop graphics are unavailable, say so and leave the checklist
unchecked.

### Focused simulation verification

```matlab
run('startup.m');
results = runtests('tests/TestSimulation.m');
table(results)
```

This suite should cover synchronization, body/world angle conversion,
no-return/support semantics, deterministic seeded sensing, estimator-bypass
prevention, policy fallback, and fixed-handle replay as those features are
implemented.

### Full verification

```matlab
run('startup.m');
results = runtests('tests');
table(results)
```

For graphics-affecting changes, also run on a desktop:

```matlab
run('startup.m');
scenario = scenarios.movingPair();
result = simulation.runScenario(scenario);
viz.plotSimulationSummary(result);
viz.animateSimulation(result);
```

Changing replay speed or frame rate must not change logged arrays. The final
frame must be shown. These manual checks are not substitutes for the MATLAB
suite, and no graphics pass should be claimed without actually performing it.

## 6. Risks, guardrails, and open decisions

- **Noise and reproducibility:** `RangeNoiseStd`/`Seed` names, seed validation,
  run-local random-stream construction, and clipping returns to `[0, MaxRange]`
  are resolved and implemented. Milestone-A verification remains pending; the
  future statistical interpretation of noise and filter semantics remains open.
  Never use an unseeded/global RNG or add noise in `fov`.
- **Censoring:** no-return observations must remain separate from returns in
  sensing, correction, plotting, diagnostics, and tests.
- **Depth discontinuities:** no global interpolation, smoothing, or covariance
  coupling across segments. A jump is a structural boundary, not noise to be
  blurred away.
- **Support and forgetting:** high covariance alone does not create
  information. Newly exposed directions need a justified observation or
  within-segment rule; surfaces outside the current FoV are forgotten.
- **Controller law:** select the concrete signed-distance control law,
  reference-combination behavior, finite input bounds, and invalid-case
  fallback before Milestone-B coding. The policy fallback is an explicit
  bounded command, not a safety proof.
- **Geometry approximation:** sampled-polygon signed distance is exact only for
  the constructed sampled polygon, not ideal continuous visibility. Invalid
  geometry must remain invalid rather than repaired silently.
- **Estimator boundaries:** the estimator must not receive polygons, raw
  casts, future inputs, or full replay logs. Offline logs are not memory.
- **Scope:** do not add SLAM, occupancy, persistent maps, hidden surfaces,
  scan archives, CBF/QP, pursuit optimization, actuator/collision dynamics,
  base-link guarantees, ROS/CrazySim, interactive physics, video export, or a
  required dependency.

## 7. Checkpoint-one handoff and current acceptance record

The supported handoff remains:

```matlab
run('startup.m');
scenario = scenarios.movingPair();
result = simulation.runScenario(scenario);
viz.plotSimulationSummary(result);
viz.animateSimulation(result);
```

Configuration can change time bounds, step, independent velocity schedules,
obstacles, FoV parameters, ray count, and initial poses without editing the
runner. The follower pose is intentionally known to the policy in this
checkpoint. There is no pursuit, collision response, continuous visibility
guarantee, or base-link guarantee.

**Acceptance record:** the current code additions and replay optimization are
documented above, but the focused checkpoint tests have no reported passing
rerun after the fixes/additions. Automated MATLAB verification and the required
desktop graphics replay remain open. The reported faster replay is a manual
performance observation only.

## 8. Historical static signed-distance, contour, and UI phases

These phases document completed or intentionally retained legacy work. They do
not override the active probabilistic-FoV sequence in Section 4.

### Historical benchmark contract

The original benchmark computes shortest Euclidean distance to
`result.VisibleBoundary` from `fov.castRays`:

- negative strictly inside the sampled visible polygon;
- positive outside;
- zero on its boundary;
- valid for finite Cartesian query points, including points outside the nominal
  FoV and points inside obstacles; and
- exact only relative to the finite sampled polygon, not ideal continuous
  visibility.

Ray-count geometry approximation and XY field/contour interpolation remain
separate approximations. For partial FoV, the boundary order is
`observer -> first endpoint -> ... -> last endpoint -> observer`; for full FoV,
it is `first endpoint -> ... -> last endpoint -> first endpoint`.

The metric accepts finite, real, ordered, closed-or-closable, nondegenerate
boundaries. Consecutive duplicate vertices are removed within tolerance;
fewer than three unique vertices, zero area, nonfinite coordinates, degenerate
edges, and self-intersections are rejected. Full-circle ray casts with two
rays remain legal ray-casting output but are rejected as degenerate by the
metric. The scalar distance is continuous and 1-Lipschitz, although closest
features can change at corners, medial axes, and occlusion transitions.

### Historical phases 1–6: static implementation

- [x] Formalize the visible-region boundary contract, ordering, sign
  convention, duplicate handling, and invalid-geometry behavior.
- [x] Implement `metrics.signedEuclideanDistance` with direct boundary input,
  point-to-segment distance, polygon sign classification, optional diagnostics,
  and scale-aware tolerance behavior.
- [x] Implement generic `metrics.sampleField` with explicit bounds, grid size,
  mesh orientation, values, zero level, units, and sign metadata. Keep field
  sampling independent of visibility ray count.
- [x] Implement `viz.plotMetricContours` with regular contours, a distinct zero
  contour, supplied-parent support, held-state preservation, and graphics
  handles.
- [x] Complete the historical static metric tests: exact polygons,
  normalization, concavity, FOV integration, the Lipschitz property, field
  sampling, contour handles, and the static demo.
- [x] Complete the historical demonstration as
  `scripts/runSingleScenario.m` (rather than the originally proposed
  `runDistanceContours.m`) and document the equivalent README workflow.

The prior historical record reports a full MATLAB suite pass on 2026-08-17 for
that static phase. That old result is not a passing rerun of the current
checkpoint after later fixes/additions.

### Historical Phase 7: interactive heading explorer

`viz.interactiveScenario` and `scripts/runInteractiveScenario.m` remain
available for static exploration and are legacy/deprecated for new work.

- [x] Provide a programmatic `uifigure`/`uiaxes` explorer with heading slider,
  degree/radian conversion, fixed observer/FoV/scenario settings, returned
  graphics handles, and invisible construction for smoke tests.
- [x] Keep a fixed square domain and axes limits while the heading changes.
- [x] Use a fast `ValueChangingFcn` preview without metric fields and a full
  `ValueChangedFcn` contour update after release.
- [x] Preserve graphics lifecycle behavior: clear/redraw safely, avoid stale
  callbacks, avoid accumulated colorbars/legends, and delegate geometry,
  distance, field, and contour work to existing package functions.
- [x] Add invisible-UI smoke coverage for controls, preview/release behavior,
  fixed limits, and repeated updates.
- [ ] Manually verify desktop slider responsiveness and record limitations.

Do not extend this UI as the primary simulation workflow. No App Designer
application, observer dragging, obstacle editing, or continuous contour update
is planned.

### Historical phases 8–9: not active

- [ ] Separate ray-count convergence studies from grid-resolution convergence
  studies.
- [ ] Add critical-angle sampling only if a later static-geometry task
  explicitly reactivates it; retain `fov.castRays` uniform-sampling behavior.
- [ ] Add additional metric contracts only through the existing function-handle
  field interface and only after an explicitly approved scope change.

These items are retained for history and are not prerequisites for the active
noise, controller-baseline, or EKF sequence.

## 9. Historical static API notes

The original static workflow remains usable with `fov.FovSpec`,
`fov.Observer`, `fov.polygonObstacle`, `fov.castRays`,
`metrics.signedEuclideanDistance`, `metrics.sampleField`, and
`viz.plotMetricContours`. Static signed-distance is shortest distance to the
sampled polygon boundary, with negative-inside sign; ray count and field-grid
resolution must not be conflated in studies.

The moving-pair workflow is the recommended new entry point. Interactive
heading controls are retained only for legacy static signed-distance analysis.
