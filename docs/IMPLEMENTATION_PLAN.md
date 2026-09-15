# Motion-Aware FOV Boundary Estimation Plan

This is the execution plan for the MATLAB first-boundary field-of-view (FoV)
testbed. The active target is the observer-frame first-occlusion-boundary range
profile `B_t(theta)`, together with its support and uncertainty—not observer or
follower estimation, occupancy, a persistent world map, or a controller
redesign research program. A narrowly scoped belief-derived controller
baseline is explicitly authorized in Milestone B only to exercise this
interface.

The repository contains the checkpoint-one plumbing and retains sampled-polygon
signed distance as a computational primitive. Milestone A's implementation is
now present: noisy measurements and the distinction between oracle, measurement,
and belief are visible before adding filtering. Milestone-A and Milestone-B
verification remain pending. Each checklist item is marked
`[x]` only when the described implementation exists **and** the relevant
verification has actually been run. An implemented but unverified item stays
unchecked with an honest note.

## 1. Active direction and current status

The active workflow is the headless moving-pair simulation, followed by replay.
Static Cartesian field sampling, contour rendering, static scenario plotting,
and the interactive heading explorer are retired; Section 8 preserves their
historical record only. Milestone A is implementation-complete but awaiting its
focused and desktop verification; Milestone B is also implementation-complete
but awaiting focused and desktop verification. Its approved design is a
belief-derived continuous-time CBF-QP baseline evaluated at controller samples,
using the current noisy pass-through belief before the EKF. The known current
follower reference velocity, initial baseline configuration, and invalid-case
behavior are selected for implementation and verification; they must not be
inferred from wall-clock time or hidden oracle data. The runner sample interval
remains documented for zero-order-hold interpretation, but is not an explicit
term in the continuous CBF inequality.

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
open. The focused `tests/TestSimulation.m` suite was rerun on 2026-09-15 after
the latest frame-export assertion fix and passed all 22 tests. It covers sensor
validation, deterministic same-seed scans, preservation of the global RNG,
preservation of oracle and no-return data, explicit clipping, replay without
resampling, oracle/measurement/belief source separation, labels and styles,
and fixed artist count. The focused controller suite, full MATLAB suite, and
formal desktop graphics checks remain unrun. The user's comment “That looks
great” is recorded as positive manual visual observation/approval of the
Milestone-A result only; it is not a formal desktop graphics or final-frame
verification.

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
| `+metrics` | Sampled-polygon signed-distance computation |
| `+viz` | Summary, replay, diagnostics, and static frame export |
| `+scenarios` | Reusable dynamic scenario factories |
| `scripts` | Thin experiment/demo entry points |
| `tests` | MATLAB unit and graphics-smoke tests |

Prefer plain structs, small functions, and callbacks. MATLAB Optimization
Toolbox is an intentional required dependency beginning with Milestone B:
`quadprog` is available, and using it avoids adding a custom QP solver and its
non-research implementation/testing burden. Use direct `quadprog`, not
problem-based optimization abstractions, and do not provide or require a
custom fallback QP backend. This is the explicit exception to the general
rule against introducing required dependencies.

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

The implementation sequence below inserts a narrowly scoped, belief-derived
sampled-polygon continuous-time CBF-QP baseline between sensing visualization
and the EKF. This
is a deliberate, slight deviation from the controller-last ordering: it
validates the complete belief-to-policy seam early and intentionally exposes
how the noisy pass-through estimator causes controller problems before the EKF.
It does not authorize controller redesign, pursuit optimization, collision or
actuator dynamics, base-link guarantees, or claims about true-FoV safety. After
the EKF, the identical policy must be compared using pass-through versus EKF
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

### Milestone B — belief-derived sampled-polygon continuous-time CBF-QP baseline

Implement a deliberately small continuous-time CBF-QP policy through the
existing `+control` seam, before the EKF. This is explicitly authorized for
Milestone B. Its experimental purpose is to observe how the noisy pass-through
estimator causes controller problems before filtering; it is not a claim that
the pass-through belief is reliable.

The policy uses only the current observation, current belief, configured
reference, and the causally available current follower reference velocity. It
must never consume raw casts, obstacle polygons, future inputs, or full logs.
The known follower-pose assumption remains explicit: `p` is the current known
target/follower position supplied by the checkpoint observation, not an
estimated target state. The policy's geometry is constructed only from the
current belief, never directly from a scan, raw cast, obstacle polygon, future
input, or full log. Unsupported directions remain unsupported; the policy must
not fabricate a closed visible polygon or silently substitute oracle
visibility.

#### Barrier and continuous sampled-controller contract

Let `F` be the sampled polygon constructed only from the current belief fields.
It is represented and frozen in the observer-local frame for the entire
control calculation. Reconstruct it neither from obstacles, a map, nor raw
casts, and never resample sensing noise. Refresh `F` only from the next
posterior. Let `d(p,F)` be the existing signed Euclidean distance, with
negative values inside the polygon, positive values outside, and zero on its
boundary. Use a named configuration field `DistanceMargin` rather than `eps`
(`eps` is a MATLAB builtin and this margin has metre units). The barrier is

```text
h = -(d(p,F) + DistanceMargin)
  = -d(p,F) - DistanceMargin.
```

Thus `h >= 0` means that the known target/follower point is at least
`DistanceMargin` inside the sampled estimated polygon. This is an interior
condition for the estimated polygon, not a clearance guarantee for continuous
visibility or the oracle FoV.

Numerically calculate the barrier gradients with respect to observer world pose
`x_o = [x,y,yaw]` and target world position `p`, using the frozen local belief
polygon and the coordinate transforms. For an observer-pose perturbation,
transform the fixed local polygon/point relationship through the perturbed
observer pose; do not rebuild the polygon from any external geometry. Use
central differences where possible, with the actual unequal displacements near
a domain boundary and a one-sided difference when only one side is usable.
Separate position and yaw perturbation units. The initial configuration is
`PoseFiniteDifferenceStep = [1e-3, 1e-3, 1e-5]`, where the first two entries
are metres and the third is radians. Nonfinite or otherwise unusable gradient
results use the documented fallback.

Observer control is body-frame `u = [vxBody; vyBody; omega]`. Map it to the
observer world-pose rate with

```text
G_o(yaw) = [[cos(yaw), -sin(yaw), 0],
            [sin(yaw),  cos(yaw), 0],
            [0,         0,        1]].
```

The target-motion model is resolved: use the known current follower reference
velocity. Convert its body-frame translational components to world velocity
using the current follower yaw,
`v_p = R(yawFollower) * [vxFollowerBody; vyFollowerBody]`. The follower yaw
rate does not directly move the target point instantaneously. The runner must
make this current-interval reference available causally to the policy; it is
not a future input. Both agents still advance from the same pre-step snapshot.

At each controller sample, enforce the continuous sampled-controller CBF
inequality

```text
grad_xo(h) * G_o * u + grad_p(h) * v_p + CbfRate*h + delta >= 0,
```

where `CbfRate = alpha > 0` has units `1/s` and `delta >= 0` is slack in
barrier-rate units. Define

```text
a = grad_xo(h) * G_o;
targetDrift = grad_p(h) * v_p;
b = -CbfRate*h - targetDrift;
```

so the affine constraint is `a*u + delta >= b`. The inequality is evaluated
at sample instants under the runner's zero-order hold. It does not by itself
guarantee intersample behavior or safety for the true/oracle FoV.

The Python formulation's additional gamma scaling of both `h` and its gradient
is intentionally omitted: absent slack, that common scaling largely cancels
from the inequality. Python `alpha` corresponds to `CbfRate` here, keeping the
policy in continuous barrier-rate units.

#### QP variables, objective, and direct `quadprog` contract

Use `z = [u; delta]` and minimize

```text
0.5*(u-u_ref)'*W*(u-u_ref) + 0.5*rho*delta^2
```

subject to the finite input bounds, `delta >= 0`, and the continuous affine
CBF inequality. Assemble the direct `quadprog` inputs as

```matlab
H = blkdiag(W, rho);
f = [-W*u_ref; 0];
A = [-a, -1];
bIneq = -b;                 % equivalently CbfRate*h + targetDrift
lb = [uMin; 0];
ub = [uMax; Inf];
```

`W` must be symmetric positive definite, `rho` must be positive, and all
dimensions and values must be finite apart from the intentional `Inf` upper
bound for slack. Translational and angular weights must account for their
different units/scales. Validate a finite three-component nominal reference,
finite ordered bounds, an admissible bounded zero command for the fallback,
finite nonnegative `DistanceMargin`, positive finite `CbfRate`, and positive
finite `PoseFiniteDifferenceStep` entries. The initial baseline defaults are:

```matlab
Policy.DistanceMargin           = 0.1;             % metres
Policy.CbfRate                  = 4;               % 1/s
Policy.InputLower               = [-0.5; -0.5; -1];
Policy.InputUpper               = [ 0.5;  0.5;  1];
Policy.InputWeights             = diag([1, 1, 0.25]);
Policy.SlackPenalty             = 1e4;
Policy.PoseFiniteDifferenceStep = [1e-3; 1e-3; 1e-5]; % m, m, rad
```

These are initial baseline defaults, not universal guarantees. The known
current follower reference velocity is the resolved target input model and is
provided causally by the runner. The value `CbfRate=4` is first-order
equivalent to the prior `gamma` of about `0.2` at a `0.05 s` sample interval,
but the continuous inequality does not require that interval as a parameter.
The sample interval must remain documented for zero-order-hold experiments
and intersample limitations. An equivalent `W` name may be used if the
surrounding configuration convention requires it, but its stored/documented
meaning must remain explicit.

Call `quadprog` directly with

```matlab
options = optimoptions('quadprog', 'Algorithm', 'active-set', ...
                       'Display', 'off');
[z, ~, exitFlag, output] = quadprog(H, f, A, bIneq, [], [], ...
                                    lb, ub, x0, options);
```

Use the bounded nominal input
`uInitial = min(max(u_ref, uMin), uMax)` and
`initialSlack = max(0, b - a*uInitial)`, then `x0 = [uInitial;
initialSlack]`. This makes the starting point feasible for the affine CBF
constraint. Positive slack means the estimated barrier-rate condition was
relaxed. MATLAB Optimization Toolbox is a required dependency beginning with
this milestone because `quadprog` is available and a custom solver would add
implementation and testing burden unrelated to the research contribution.
Do not add a custom or problem-based fallback backend. If `quadprog` is
unavailable, report a clear missing-Optimization-Toolbox dependency error;
this is distinct from a solver failure after the dependency is available.

If `exitFlag <= 0` or the returned solution/diagnostics are nonfinite, use the
documented bounded zero-command fallback and record the failure. The fallback
is an operational experiment fail-safe, not a collision, visibility, or safety
guarantee. A fixed-zero or otherwise authority-free bound configuration is
reported as `zero-control-authority`, not as successful CBF correction.

#### Numerical-gradient and post-solve diagnostics

The transient policy diagnostics are:

- `SignedDistance`, `BarrierValue`, `ObserverPoseGradient`,
  `TargetPositionGradient`, and `BodyInputCoefficient`/`a`;
- `TargetDrift`, `NominalInput`, `AppliedInput`, and `Slack`;
- the continuous affine `ConstraintResidual = a*u + delta - b`;
- `ActiveBounds`, `QpExitFlag`, and `QpIterations`;
- finite-difference mode/quality information where useful; and
- a status such as `nominal-feasible`, `cbf-corrected`,
  `relaxed-with-slack`, `zero-control-authority`,
  `invalid-belief-fallback`, `unusable-gradient-fallback`, or
  `qp-failure-fallback`.

No nonlinear post-solve barrier diagnostic is computed in this formulation.
First run the identical policy
with the noisy pass-through belief so chattering, nearest-edge switching,
slack activation/QP relaxation, fallback behavior, and true-FoV violations can
be visualized. Later run the identical policy against the EKF belief. This is
a continuous sampled-controller CBF for the estimated sampled polygon only;
it is not a guarantee for the oracle/true occluded FoV, intersample
visibility, collision avoidance, or base-link safety.

#### Implemented architecture and checklist

Keep the existing policy replaceable and use plain structs, small functions,
and callbacks. `+control/makeSignedDistanceCbfPolicy.m` implements the policy
and direct `quadprog` assembly without a framework, registry, middleware, or
custom QP backend. Geometry remains in `+fov`, the policy and QP seam in
`+control`, and computation separate from plotting.

**Implementation status (2026-09-10):**
`control.makeSignedDistanceCbfPolicy` now constructs frozen belief-local
geometry, calculates continuous observer-pose and target-position finite-
difference gradients, incorporates the causal known follower reference velocity,
and solves the bounded slack QP with direct `quadprog`. `simulation.runScenario`
now exposes `FollowerReference` in the current policy observation. Focused
`TestControl` and simulation-seam tests were added but not run. Milestone B
remains unchecked pending focused/full MATLAB verification and zero/noise desktop
inspection; no true-FoV or safety result is claimed.

**Acceptance criteria.** Milestone B is accepted only after the complete
belief-only continuous barrier/QP contract, validation, numerical-gradient
handling, diagnostics, and bounded fallbacks are implemented; focused tests
cover the cases below; deterministic zero/noise pass-through runs have been
inspected; and the results are recorded without claiming true-FoV safety.
Until then every item below remains unchecked.

- [ ] Implement the belief-derived sampled-polygon barrier, frozen observer-
  local geometry, continuous sampled-controller inequality, and causal known
  follower-reference-velocity input.
- [ ] Implement direct `quadprog` use with the stated options, matrices,
  bounded nominal feasible start, finite bounds, slack, and bounded zero-command
  fallback; do not add a custom QP backend.
- [ ] Validate Optimization Toolbox/`quadprog` availability and clear missing-
  dependency error messaging.
- [ ] Validate `DistanceMargin`, `CbfRate`, input bounds, `InputWeights`,
  `SlackPenalty`, and `PoseFiniteDifferenceStep` dimensions, units,
  finiteness, and positivity/definiteness requirements.
- [ ] Test body-to-world observer mapping and known follower translational
  drift, including the fact that follower yaw rate has no instantaneous target
  position contribution.
- [ ] Test central and one-sided/unequal observer-pose finite differences,
  finite-difference quality reporting, and rejection of unusable gradients.
- [ ] Test continuous CBF/QP assembly and signs, nominal-feasible unchanged
  control, corrective control, input bounds, slack activation, solver exit
  handling, and fallback behavior where testable.
- [ ] Test invalid and unsupported belief geometry, including degenerate and
  self-intersecting boundaries, with `invalid-belief-fallback` diagnostics.
- [ ] Prove with injected-belief tests that geometry comes from belief output,
  never raw casts, obstacle polygons, future inputs, or full logs; verify that
  sensing is not resampled during control or replay.
- [ ] Verify deterministic behavior with explicit sample configuration and
  repeated noisy/pass-through runs.
- [ ] Run the policy with zero-noise and noisy pass-through beliefs and record
  chattering, switching, slack, fallback, and true-FoV observations without
  treating them as safety results.
- [ ] Keep the controller limited to the approved baseline; do not add
  controller redesign/proofs, pursuit optimization, collision/actuator
  dynamics, base-link guarantees, or true-FoV safety claims.

**User-run verification after implementation:**

```matlab
run('startup.m');
scenario = scenarios.movingPair();
scenario.Observer.Policy = control.makeSignedDistanceCbfPolicy();
scenario.Sensor.RangeNoiseStd = 0;
scenario.Sensor.Seed = 17;
zeroNoise = simulation.runScenario(scenario);
scenario.Sensor.RangeNoiseStd = 0.05;
noisy = simulation.runScenario(scenario);
viz.plotSimulationSummary(noisy);
viz.animateSimulation(noisy, 'ShowRays', false, 'ShowHitPoints', true);
results = runtests('tests/TestControl.m');
table(results)
results = runtests('tests/TestSimulation.m');
table(results)
```

The constructor above uses the documented initial defaults; an optional scalar
configuration struct overrides those fields. Focused controller tests now exist
in `tests/TestControl.m` but have not been run. Separately inspect nominal-
feasible, corrected, slack, invalid-belief, unusable-gradient, and solver-
fallback runs; record actual MATLAB and desktop outcomes rather than inferring
them from code or plotted appearance.

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

After Milestone C, compare the sampled-polygon continuous-time CBF-QP
baseline's behavior when driven by pass-through belief versus EKF belief. Keep
comparison diagnostics scoped to the experiment and visualization; do not turn
them into a new metrics program.
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
implemented. Milestone-B focused coverage should also cover body-to-world
observer mapping, known follower translational drift,
central and one-sided observer-pose gradients, continuous CBF/QP assembly and
signs, `quadprog` availability and clear dependency errors, nominal-feasible
unchanged control, corrective control, bounds, slack, solver exit handling and
bounded fallback, invalid/unsupported geometry, belief-not-oracle sourcing,
deterministic noisy/pass-through runs, and no sensing resampling. Add those
tests to the appropriate controller test class; none of these new Milestone-B
checks are currently verified.

### Milestone-B controller verification

Before marking Milestone B accepted, run commands such as:

```matlab
run('startup.m');
assert(~isempty(which('quadprog')), ...
       'Milestone B requires MATLAB Optimization Toolbox and quadprog.');
results = runtests('tests/TestControl.m');
table(results)
results = runtests('tests/TestSimulation.m');
table(results)
```

The controller tests should exercise the full planned checklist, including
missing-`quadprog` error messaging in an isolated test where available, rather
than installing a custom QP backend. Repeat the deterministic zero-noise and
noisy pass-through scenario runs from Milestone B and inspect diagnostics and
replay separately. Record actual results and leave every new checklist item
unchecked until implementation and verification both exist.

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
- **CBF-QP limitations:** Milestone B is explicitly authorized to use the
  continuous sampled-controller barrier and direct `quadprog` contract above.
  The polygon is frozen in the current observer-local belief frame during each
  calculation, the target drift is based on the known current follower
  reference velocity, and gradients are numerical. Sample-instant enforcement
  under zero-order hold does not guarantee intersample behavior or safety for
  the true/oracle FoV; do not present it as a safety proof.
- **QP dependency and failure:** MATLAB Optimization Toolbox/`quadprog` is a
  deliberate required dependency beginning with Milestone B. Missing toolbox
  support must produce clear error messaging, not a custom solver or silent
  backend substitution. Solver failure, nonfinite output, invalid belief, or
  unusable gradients use the bounded zero-command fallback and diagnostics.
- **Controller reference and authority:** the policy minimally modifies the
  bounded nominal reference. The initial baseline defaults are
  `InputLower=[-0.5;-0.5;-1]`, `InputUpper=[0.5;0.5;1]`,
  `DistanceMargin=0.1 m`, `CbfRate=4 1/s`,
  `InputWeights=diag([1,1,0.25])`, `SlackPenalty=1e4`, and
  `PoseFiniteDifferenceStep=[1e-3;1e-3;1e-5]` in m, m, rad. They are
  configuration baselines, not universal guarantees. Positive slack is a
  relaxed estimated-set condition, not a guarantee; lack of authority is
  reported explicitly.
- **Resolved target model and remaining tuning:** use the known current
  follower reference velocity, converted from body-frame translation with the
  current follower yaw; follower yaw rate has no instantaneous target-point
  contribution. The target model and initial defaults are resolved. Remaining
  implementation tuning and verification are open. No explicit controller time
  step is required in the continuous inequality, but the simulation sample
  interval must remain documented and intersample limitations must remain
  explicit. Never infer causal inputs from wall-clock timing, raw casts, or
  replay history.
- **Geometry approximation:** sampled-polygon signed distance is exact only for
  the constructed sampled polygon, not ideal continuous visibility. Invalid
  geometry must remain invalid rather than repaired silently.
- **Estimator boundaries:** the estimator must not receive polygons, raw
  casts, future inputs, or full replay logs. Offline logs are not memory.
- **Scope:** do not add SLAM, occupancy, persistent maps, hidden surfaces,
  scan archives, pursuit optimization, actuator/collision dynamics, base-link
  guarantees, ROS/CrazySim, interactive physics, video export, or claims about
  true-FoV safety. A narrowly scoped Milestone-B continuous-time CBF-QP baseline
  and its Optimization Toolbox dependency are the explicit approved
  exceptions; do not expand them into controller redesign/proofs, pursuit
  optimization, a custom QP backend, or a general safety program.

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

**Acceptance record:** `tests/TestSimulation.m` passed 22/22 on 2026-09-15
after the latest frame-export assertion fix. The focused controller suite, full
MATLAB suite, and required desktop graphics replay remain open. The reported
faster replay is a manual performance observation only.

## 8. Retired static field, contour, and UI phases

These phases document work that was completed and then retired during the
pre-EKF cleanup. They do not override the active probabilistic-FoV sequence in
Section 4 and do not describe supported APIs.

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

For partial FoV, the boundary order is
`observer -> first endpoint -> ... -> last endpoint -> observer`; for full FoV,
it is `first endpoint -> ... -> last endpoint -> first endpoint`.

The metric accepts finite, real, ordered, closed-or-closable, nondegenerate
boundaries. Consecutive duplicate vertices are removed within tolerance;
fewer than three unique vertices, zero area, nonfinite coordinates, degenerate
edges, and self-intersections are rejected. Full-circle ray casts with two
rays remain legal ray-casting output but are rejected as degenerate by the
metric. The scalar distance is continuous and 1-Lipschitz, although closest
features can change at corners, medial axes, and occlusion transitions.

### Historical phases 1–6: static implementation and retirement

- [x] Formalize the visible-region boundary contract, ordering, sign
  convention, duplicate handling, and invalid-geometry behavior.
- [x] Implement `metrics.signedEuclideanDistance` with direct boundary input,
  point-to-segment distance, polygon sign classification, optional diagnostics,
  and scale-aware tolerance behavior.
- [x] Implement and later retire generic Cartesian field sampling and contour
  rendering. They are not part of the current boundary-estimation workflow.
- [x] Complete the historical static metric tests and demonstration, then remove
  their field, contour, static-plot, and UI coverage with the retired APIs.

The prior historical record reports a full MATLAB suite pass on 2026-08-17 for
that static phase. That old result is not a passing rerun of the current
checkpoint after later fixes/additions.

### Historical Phase 7: interactive heading explorer

`viz.interactiveScenario` and `scripts/runInteractiveScenario.m` were removed
with the static field and contour workflow.

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

Do not restore this UI as the primary simulation workflow. No App Designer
application, observer dragging, obstacle editing, or continuous contour update
is planned.

### Historical phases 8–9: not active

- [ ] Separate ray-count convergence studies from unrelated spatial-grid studies
  only if a future task explicitly restores a field-analysis workflow.
- [ ] Add critical-angle sampling only if a later static-geometry task
  explicitly reactivates it; retain `fov.castRays` uniform-sampling behavior.
- [ ] Add additional metric contracts only after an explicitly approved scope
  change.

These items are retained for history and are not prerequisites for the active
noise, controller-baseline, or EKF sequence.

## 9. Retired static API notes

The retired static workflow used `metrics.sampleField`,
`viz.plotMetricContours`, `viz.plotScenario`, and the interactive heading
explorer. Those APIs and their static scenario factories are no longer present.
`metrics.signedEuclideanDistance` remains supported for sampled-polygon
diagnostics and the controller baseline, with the negative-inside convention.

The moving-pair workflow is the supported entry point.
