# Code architecture

This document describes how the simulation is put together: what runs, in which
order, where the oracle ray cast and the CBF gradient are computed, how
components are swapped, and what each file does. It reflects `main` after
PR #3 (2026-11). For the research method itself see `docs/metodologia.pdf`;
for status and history see `docs/IMPLEMENTATION_PLAN.md`.

**Naming:** as in the methodology, the **leader** is the agent at position `y`
that must stay visible, and the **observer** is the robot carrying the sensor
and running the controller. A few policy diagnostics still say "target" for the
leader (`TargetPositionGradient`, `TargetDrift`).

## 1. The big picture

```
scenarios.<name>()            plain struct: geometry, agents, sensor,
        │                     estimator handles, policy handle, playback
        ▼
simulation.runScenario(s)     the only place time advances (headless, deterministic)
        │   per step k:
        │   1. Policy(observation)        → body input u       +control
        │   2. stepPose(observer, u)      → new poses          +simulation
        │      stepPose(leader, ref)
        │   3. Estimator.Predict(motion)  → transported belief +estimation
        │   4. raycastScan(new pose)      → noisy scan         +sensing → +fov
        │   5. Estimator.Correct(scan)    → corrected belief   +estimation
        ▼
result struct                 full log + metrics (TrueVisible, ...)
        │
        ▼
viz.*                         plots and replay read the log only
```

There are three swappable parts, each a function handle stored in the
scenario struct:

| Slot | Default | What it decides |
|---|---|---|
| `scenario.Observer.Policy` | `@control.referencePolicy` or a CBF-QP policy | The observer's input each step |
| `scenario.Estimator` (`Initialize`/`Predict`/`Correct`) | `estimation.makePassThroughEstimator()` | The belief `μ, Σ` about the boundary |
| `BoundaryPredictor` (inside the CBF policy config, required) | Oracle predictor (`fov.visibleBoundaryAt`) | How the boundary looks from a hypothetical pose, used for the barrier and gradient |

The runner never changes between experiments. Changing the scenario, the
estimator, or the boundary predictor is how you run a different experiment.

## 2. One simulation run, step by step

### Before the loop (`simulation.runScenario`)

1. `simulation.validateScenario` checks the scenario struct once.
2. Allocate the logs. For `K` intervals there are `K+1` samples of poses,
   scans, beliefs and raw casts, and `K` inputs and policy diagnostics.
3. Create a local `RandStream` from `Sensor.Seed`, so sensor noise is
   reproducible and MATLAB's global RNG is untouched.
4. `Estimator.Initialize(config)`, then the first scan
   (`sensing.raycastScan`) and `Estimator.Correct`. A belief exists before the
   first decision.

### Each step `k` (the causal loop)

1. **Observation.** The runner builds `observation` with the current
   `Belief`, `ObserverPose`, `LeaderPose`, `LeaderReference` (the
   leader's known velocity), `Time` and `BasePosition`. It also samples the
   observer's nominal input from `Observer.Reference`.
2. **Policy.** `[u, policyState, diagnostics] = Policy(observation,
   nominalInput, policyState)`. Here `u = [vxBody, vyBody, omega]`.
3. **Motion.** `simulation.stepPose` advances both agents by one forward-Euler
   step from the same starting snapshot. The observer uses `u`; the leader
   uses its scheduled reference.
4. **Predict.** `Estimator.Predict(state, motion, dt)`, where `motion` holds
   the previous pose, the new pose and the applied `u`. Δp and Δψ in the old
   frame follow from these.
5. **Sense.** `sensing.raycastScan(newPose, Fov, Obstacles, Sensor, t, rng)`
   returns the noisy `scan` and the noiseless `rawCast` (see §3).
6. **Correct.** `Estimator.Correct(state, scan)` returns the new `belief`. The
   runner checks it matches the scan's grid, time and pose.

### After the loop

`calculateMetrics` (local function in `runScenario.m`) adds, per sample:
- `TrueVisible`: `fov.isPointVisible`, the oracle line of sight to the leader;
- `SampledPolygonSignedDistance` / `SampledPolygonVisible`: distance of the
  leader to the belief polygon;
- distances to the observer and base, and the pass-through range residual.

The `result` struct holds `Config` (the scenario), `Time`, all poses and
inputs, `Scans`, `Beliefs`, `RawCasts`, `PolicyDiagnostics` and `Metrics`.

## 3. Where the oracle ray cast is computed

All true-geometry ray casting goes through one function, **`fov.castRays`**.
It intersects every ray with every obstacle edge in one vectorized
rays-by-edges solve (`fov.internal.rayEdgeDistances`, whose header derives the
intersection formulas) and keeps the nearest hit, saturated at `MaxRange`. Its outputs are:
- per-ray `Distances`, `EndPoints`, `HitObstacleId` (0 = no return);
- `IsDepthJump` (hit identity changes between neighbouring rays);
- `VisibleBoundary`, the polygon built by `fov.boundaryFromRanges` with radial
  steps at depth jumps.

It is called from three places, for three different purposes:

| Caller | Purpose | Who sees the result |
|---|---|---|
| `sensing.raycastScan` (each step, from the runner) | The simulated LiDAR: true ranges, plus noise in `scan` | The estimator sees only `scan`; the noiseless `rawCast` goes to the log and replay |
| `fov.visibleBoundaryAt`, wrapped as the oracle predictor (inside the policy, ~13 times per step) | Oracle boundary (i): true boundary at hypothetical poses | Only the policy that was built with it |
| *(not `castRays`)* `fov.isPointVisible` | Exact line-of-sight test for scoring | Only `Metrics.TrueVisible` |

The estimator never calls `castRays` and never receives obstacles. That is
what makes it an estimator.

## 4. Where the gradient is computed

The policy **`+control/makeSignedDistanceCbfPolicy.m`** runs each step as
predict → barrier → gradient → solve. The gradient lives in
**`control.barrierGradient`** and the QP in **`control.solveCbfQp`**; both
are pure numerics with no FoV geometry, so they are tested on their own.

1. **`boundaryAt(pose) = BoundaryPredictor(belief, pose)`**, the world-frame
   visible polygon predicted from the current belief as seen from `pose`.
   The baseline is the oracle predictor, which ignores the belief and re-casts
   the true obstacles at `pose` (source (i)).
2. **Barrier now:** `h = -metrics.signedEuclideanDistance(boundaryAt(currentPose),
   leaderPosition) - DistanceMargin` (local `barrierOrNaN`, which turns known
   invalid-geometry failures into `NaN`).
3. **Gradient (`control.barrierGradient`, eq. 12):** the policy hands it two
   barrier closures, one re-predicting the boundary at a displaced pose and
   one displacing the leader against the current boundary.
   - Steps: δψ = Δθ (beam spacing, from `belief.Angles`) and
     δx = δy = ‖y − p‖·Δθ.
   - Observer: for each axis in {x, y, ψ}, each multiple ℓ in
     `GradientBeamMultiples` (default {1, 2}) and each sign, it calls
     `boundaryAt` at the displaced pose (12 calls) and evaluates h. A
     least-squares fit gives ∂h/∂q, which equals eq. 12's weighted central
     differences. With `GradientDirections = "axes+diagonals"` it also
     samples every pair of axes at once ((x, y), (x, ψ), (y, ψ), all sign
     combinations; 36 calls), so moves that only help in combination, such
     as translating and rotating together, register in the fitted gradient.
     Default `"axes+diagonals"`; `"axes"` is eq. 12 as written.
   - Leader: it displaces the leader by ±ℓ steps in x and y against the
     fixed current polygon (8 evaluations, no new boundary), giving ∂h/∂y.
   - Samples whose polygon is invalid geometry are dropped. If too few
     remain, the gradient is marked unusable.
4. **CBF terms:** `coefficient = ∂h/∂q · G(ψ)` maps body inputs to ḣ.
   `TargetDrift = ∂h/∂y · ẏ` is ∂h/∂t from the leader's known velocity.
5. **QP (`control.solveCbfQp`, direct `quadprog`):** minimize
   `(u − u_nom)ᵀW(u − u_nom) + (u − u_prev)ᵀR(u − u_prev) + λδ²` subject to
   `coefficient·u + δ ≥ −γh − drift`, input bounds and `δ ≥ 0`. The rate term
   (`InputRatePenalty` R, default 6W = diag(6, 6, 1.5), tuned at dt = 0.025 s) damps input changes
   between steps; `u_prev` is the previous applied input, kept in the policy
   state. It also handles the zero-authority and QP-failure
   fallbacks.
6. **Status and fallback:** `nominal-feasible`, `cbf-corrected` or
   `relaxed-with-slack`, based on whether the constraint can be met within the
   bounds. Any failure (invalid geometry and unusable gradient in the policy;
   zero authority and QP failure in the solver) returns the nominal input
   clipped to the bounds.
7. Everything is written to `diagnostics`, one per step:
   - `BarrierValue`, `SignedDistance`;
   - `ObserverPoseGradient`, `TargetPositionGradient`, `GradientSampleCount`,
     `GradientQuality`;
   - `BodyInputCoefficient`, `TargetDrift`, `NominalInput`, `AppliedInput`;
   - `Slack`, `ConstraintResidual`, `ActiveBounds`, `QpExitFlag`,
     `QpIterations`;
   - `Status`.

The only thing that differs between boundary sources is the predictor.
Everything from step 2 onward is shared.

## 5. How the modular parts plug in

### Policy

Signature: `[u, state, diagnostics] = policy(observation, nominalInput, state)`.

- `control.referencePolicy` returns `nominalInput` unchanged (no control).
- `control.makeSignedDistanceCbfPolicy(config)` builds the CBF-QP callback.
  Its settings (margin, γ, bounds, weights, slack penalty, gradient
  multiples, `BoundaryPredictor`) are fixed when the callback is built.

### Estimator

A struct with three handles:
- `state = Initialize(config)`, where `config` has `NumRays`, `MaxRange` and
  `OpeningAngle`;
- `state = Predict(state, motion, dt)`;
- `[state, belief] = Correct(state, scan)`.

The estimator owns its `state` (e.g. μ, Σ, labels). The runner only passes it
back in. `estimation.makePassThroughEstimator` copies the scan into the belief:
`Mean = Ranges`, zero `Covariance`, and `Predict` does nothing.

### Boundary predictor

Signature: `boundary = predictor(belief, pose)`, which returns the closed
world-frame polygon (N-by-2) the observer would see from `pose`.

The runner never calls it. Only the CBF policy does, from its config.

- **Oracle (baseline):** the scenario factories build
  `@(belief, pose) fov.visibleBoundaryAt(pose, fov, obstacles, numRays)`.
  It ignores the belief and captures the obstacles when it is built. If you
  change `scenario.Obstacles`, `Observer.Fov` or `Sensor.NumRays`, rebuild
  the policy (see `+scenarios/template.m`). It must never reach an estimator.
- **Methodology transport (source ii):** `estimation.predictBoundaryByTransport`
  calls `estimation.transportBoundary` (below) to move the belief mean to
  `pose`, then builds the polygon from the predicted mean on **every** beam
  (`fov.boundaryFromRanges`, regardless of `IsSupported`: plan §4.10, since
  the CBF gradient needs exactly the hypothesis beams eq. 11 produces). A
  predicted pair is a jump (decision D6) when its means differ by more than
  `max(JumpThreshold, JumpRelative * nearer mean)` or its `HasReturn` flags
  differ. A scenario or script plugs it into a policy as a closure over its
  params, the same pattern as the oracle closure:
  `@(belief, pose) estimation.predictBoundaryByTransport(belief, pose, params)`.
  The barrier uses the expected boundary, not a confidence bound such as
  μ − κσ (§1.4).

`estimation.transportBoundary(belief, pose, params)` is the mean-only
transport core (`docs/metodologia.pdf` §1.2, eqs. 1-2, 7-11): it transports
every old beam's endpoint to the hypothetical `pose` and resamples onto the
same fixed grid, labelling each new beam `blend`, `overlap`, `edge`,
`disoccluded`, `no-return`, or `new`. It takes the same `(belief, pose)`
pair as a `BoundaryPredictor` so `predictBoundaryByTransport` (building the
polygon) and the future EKF's `Predict` step (with the real motion instead
of a hypothetical displacement) can both call it directly. Each stage is
one file in `estimation.transport` (listed in §8), so later rules (edges,
disocclusion, covariance) are added as stages rather than growing one
function. `IsSupported` is false only for `disoccluded` and `new` beams
(decision D3): hypothesis, not evidence. `ShadowRange` (s_i) and `FarRange`
(r_far,i) are set on disoccluded beams only (NaN elsewhere), for diagnostics
and the free-space blend `mu_bar_i = s_i + eta * (r_far,i - s_i)` (eq. 11,
`params.FreeSpaceFraction`, baseline η = 0.5; η = 0 is source (iv)).

Alongside `Mean`, each stage also records per-beam provenance, `FromIndex`
(N-by-2 old-grid indices) and `FromWeight` (N-by-1): `[a, b]` and the
weight on `a` for a blend/overlap beam, `[j, NaN]` for an edge beam or an
exact-bearing match, `[k, NaN]` for a "new" beam's fill point, and `NaN`
otherwise (no-return and disoccluded beams, which have no cross-covariance
with the old estimate). `estimation.transport.propagateCovariance(belief,
prediction, params)` uses that provenance to build the eq. 3-5 Jacobian `F`
and the eq. 6/11 process noise `Q`, returning `Sigma_bar = F*Sigma*F' + Q`
(symmetrised, kept full). It is a separate function, called only by the
future EKF's `Predict` step; `transportBoundary` and
`predictBoundaryByTransport` stay mean-only and do not call it.

### How the methodology's sources map onto the slots

| Source (§1.5) | `BoundaryPredictor` | `Estimator` | Exists? |
|---|---|---|---|
| (i) Oracle | `fov.visibleBoundaryAt` wrapper | pass-through (unused for geometry) | Yes |
| (ii) Latest reading | `estimation.predictBoundaryByTransport` | pass-through | Yes |
| (iii) EKF | `estimation.predictBoundaryByTransport` | `estimation.makeBoundaryEkf` | Yes |
| (iv) EKF, η = 0 | same, with η = 0 | `estimation.makeBoundaryEkf`, η = 0 | Yes |

All four are built consistently by `scenarios.withGradientSource(scenario,
source, params)` (`source` one of `"oracle"`, `"latest"`, `"filter"`,
`"filter-conservative"`), which sets both `scenario.Estimator` and a fresh
`control.makeSignedDistanceCbfPolicy` with only `BoundaryPredictor` set (every
other policy setting, including the rate penalty, stays default).
`"filter-conservative"` forces `params.FreeSpaceFraction = 0` in both the
filter and the predictor closure.

The transport inside `predictBoundaryByTransport` is the prediction model of
§1.2. The EKF's `Predict` uses the same transport with the real motion; the
policy calls the predictor with hypothetical displacements.

### The boundary EKF (`estimation.makeBoundaryEkf`)

`estimator = estimation.makeBoundaryEkf(params)` implements the methodology's
filter (§1.3), with `params` = `estimation.defaultTransportParams()`'s fields
plus `MeasurementNoiseStd` (the sensor's `RangeNoiseStd`; floored at 1 mm
inside the filter only, `sigma = max(params.MeasurementNoiseStd, 1e-3)`) and
an optional `GateConfidence` (default 0.99; the chi-squared(1) threshold is
`(sqrt(2)*erfinv(GateConfidence))^2`, no Statistics Toolbox dependency).

- `Predict(state, motion, dt)` calls `estimation.transportBoundary` and
  `estimation.transport.propagateCovariance` with the real
  `motion.CurrentObserverPose` (the same core source (ii) uses with
  hypothetical poses), and holds the result (`mu_bar`, `Sigma_bar`, labels,
  `HasReturn`, `IsSupported`) as a pending prediction until the next
  `Correct`. It also logs `StepRatio = (‖Δp‖/ρ_min + |Δψ|)/Δθ` (plan §4.8);
  the compact scenarios' `dt = 0.025` keeps this well under 1 without
  substepping (checked, not substepped, per the dated note below).
- `Correct(state, scan)` is the first scan (`Mean = scan.Ranges`,
  `Covariance = sigma^2 * I`) or, afterwards, the linear correction on the
  pending prediction: per measured beam, gate the normalised innovation
  `(z_i - mu_bar_i)^2/(Sigma_bar_ii + sigma^2)`; a beam that fails is
  re-initialised (mean `z_i`, variance `sigma^2`, zero cross-covariance) and
  excluded from the joint update that follows; the remaining beams get one
  Joseph-form Kalman update (`H` selects them; all beams in every scenario
  here). The mean is clamped to `[0, MaxRange]`.
- The belief has the usual checkpoint-one fields (`Method = "boundary-ekf"`)
  plus `Labels` (the previous `Predict`'s per-beam label), `Innovation`,
  `NIS`, `Gated`, `FirstMeasurementNIS` (the eta-calibration check of §1.3,
  on disoccluded/new beams' first measurement) and `StepRatio`.
  `simulation.runScenario`'s `validateBelief` only requires the checkpoint-one
  fields, so these extra ones pass through untouched.

`metrics.filterConsistency(results)` (one result or several, seeds pooled)
reports, over samples `k >= 2`, per-beam RMSE/NEES/NIS and gate-failure rate
against `RawCasts{k}.Distances`, grouped by `Labels` and overall, plus the
mean first-measurement NIS and the `StepRatio` max/exceed-fraction;
`metrics.printFilterConsistency` prints it and
`scripts/checkFilterConsistency.m` runs it over several seeds on `openRoom`
and a scripted `boxInFrontOfWall` slide. It only measures consistency
(NEES/NIS near the chi-squared(1) mean of 1); see the dated note in
`docs/IMPLEMENTATION_PLAN.md` for the measured numbers, not acted on here.

`control.compareGradients(result, predictor, referencePredictor,
policyConfig)` re-evaluates two predictors (typically source (ii) and the
oracle built from `result.Config`) against every logged belief and pose of
an existing run, offline, for the angle between their observer-pose
gradients, the heading-gradient difference, and each one's barrier value and
status; `scripts/compareGradientSources.m` and `viz.plotGradientComparison`
use it to compare source (ii) against the oracle on `boxInFrontOfWall`.

## 6. Data structures

| Struct | Created by | Main fields |
|---|---|---|
| scenario | `scenarios.*` | `Name`, `Time{Start,Stop,Step}`, `Observer{InitialPose,Fov,Reference,Policy}`, `Leader{InitialPose,Reference}`, `Base{Position}`, `Obstacles`, `Sensor{NumRays,RangeNoiseStd,Seed,Tolerance}`, `Estimator`, `Playback{Bounds,FrameRate,Speed}` |
| Reference schedule | scenario | `Times` (breakpoints), `Values` (rows of `[vx, vy, omega]`, held until the next breakpoint) |
| obstacle | `fov.polygonObstacle` | `Name`, `Vertices` (convex, ordered) |
| scan | `sensing.raycastScan` | `Time`, `ObserverPose`, `Angles` (relative to heading), `Ranges` (noisy), `HasReturn`, `IsValid`, `MaxRange`, `OpeningAngle` |
| rawCast | `fov.castRays` | noiseless `Distances`, `EndPoints`, `HitObstacleId`, `IsDepthJump`, `VisibleBoundary` |
| belief | `Estimator.Correct` | `Time`, `Angles`, `Mean`, `Covariance`, `IsSupported`, `HasReturn`, `ObserverPose`, `MaxRange`, `OpeningAngle`, `Method`; the boundary EKF adds `Labels`, `Innovation`, `NIS`, `Gated`, `FirstMeasurementNIS`, `StepRatio` |
| motion | runner | `PreviousObserverPose`, `CurrentObserverPose`, `AppliedInput` |
| observation | runner | `Time`, `Belief`, `ObserverPose`, `LeaderPose`, `LeaderReference`, `BasePosition` |
| result | `runScenario` | `Config`, `Time`, `ObserverPose`, `LeaderPose`, `ObserverInput`, `LeaderInput`, `Scans`, `Beliefs`, `RawCasts`, `PolicyDiagnostics`, `Metrics` |

**Conventions:**
- SI units and radians.
- World poses are `[x, y, yaw]`, with yaw not wrapped.
- Inputs are in the body frame.
- Beam angles are relative to the heading, ascending counter-clockwise.

## 7. Who sees the true geometry

| Component | Obstacles / truth? | Reason |
|---|---|---|
| `fov.castRays`, `sensing.raycastScan` | yes | they are the simulated sensor |
| Estimator | **no**: only `scan` and `motion` | it must estimate |
| CBF policy with oracle source | yes, through its own handle | baseline (i), labelled oracle |
| CBF policy with belief/prediction source | **no**: only `observation.Belief` | sources (ii)–(iv) |
| `fov.isPointVisible`, `Metrics.TrueVisible` | yes | scoring only |
| `viz.*` | reads `RawCasts` and `Config.Obstacles` from the log | drawing truth next to the estimate |

## 8. File reference

### Entry and scripts

| File | Role |
|---|---|
| `startup.m` | Adds the repo root, `tests/` and `scripts/` to the MATLAB path. Run first. |
| `scripts/runBoxInFrontOfWall.m` | `boxInFrontOfWall` with a settings block: gradient source (oracle / latest / filter / filter-conservative), range noise, seed, gradient directions, prediction-model params; summary, diagnostics, replay. |
| `scripts/runClutteredField.m` | The same settings block for `clutteredField`. |
| `scripts/runMovingPair.m` | Moving-pair scenario with an oracle-predictor CBF, noise 0.1; summary and replay. |
| `scripts/plotControlDiagnostics.m` | Plots CBF diagnostics for a `result` already in the workspace. |
| `scripts/checkTransport.m` | Plots `estimation.transportBoundary` against truth in `scenarios.openRoom` (rotation, translation) and `scenarios.boxInFrontOfWall` (sideways slide: edges, overlap, disocclusion). |
| `scripts/compareGradientSources.m` | Runs `boxInFrontOfWall` with the oracle and with source (ii); reports and plots (`viz.plotGradientComparison`) the plan §8.2 measures via `control.compareGradients`. |
| `scripts/checkFilterConsistency.m` | Runs the boundary EKF over several seeds on `openRoom` and a scripted `boxInFrontOfWall` slide; prints `metrics.filterConsistency`'s per-label table. |

### `+scenarios`: experiment definitions (return a scenario struct, run nothing)

| File | Role |
|---|---|
| `movingPair.m` | Base scenario: 10 m / 100° FoV, 81 rays, one wall, scripted references, reference policy. Others start from it. |
| `controlTest.m` | No obstacles; the leader drives toward max range; oracle CBF with zero nominal input. |
| `boxInFrontOfWall.m` | Compact 2 m layout: a 0.2 m box in front of a wall; the leader crosses the box's shadow; oracle CBF. |
| `clutteredField.m` | Short-range layout (1 m FoV): posts on a staggered 0.6 m grid, thinned near an open exploring leader path (30 s, about 0.165 m/s); zero nominal, oracle CBF. |
| `openRoom.m` | Plan scenario S1: an enclosed room with no depth jumps; tests transport and newly-explored beams. |
| `template.m` | Every configurable field written out with units; copy it to make new scenarios. |
| `withGradientSource.m` | Sets `Estimator` and rebuilds the CBF policy consistently for one of the four gradient sources (`"oracle"`, `"latest"`, `"filter"`, `"filter-conservative"`); optional policy settings (e.g. `GradientDirections`) apply to every source alike. |

### `+simulation`: the runner

| File | Role |
|---|---|
| `runScenario.m` | The causal loop (§2), belief checks, and `calculateMetrics`. |
| `validateScenario.m` | One-time check of the scenario struct (time grid, schedules, sensor, estimator handles, playback). |
| `stepPose.m` | One forward-Euler step: body velocity rotated to world, yaw integrated. |
| `sampleVelocityReference.m` | Looks up the held velocity at time t from a `Times`/`Values` schedule. |

### `+sensing`

| File | Role |
|---|---|
| `raycastScan.m` | Simulated LiDAR. Calls `fov.castRays`, adds clipped Gaussian noise to returns only (local RNG), and returns `scan` plus the noiseless `rawCast`. |

### `+fov`: geometry

| File | Role |
|---|---|
| `FovSpec.m` | Value class: `MaxRange`, `OpeningAngle`. |
| `Observer.m` | Value class: `Position`, `Heading`, `Fov` (input to `castRays`). |
| `polygonObstacle.m` | Builds and validates a convex polygon obstacle struct. |
| `sampleRayAngles.m` | Evenly spaced world ray angles over the aperture (both edges included for a partial FoV). |
| `castRays.m` | **The oracle ray cast** (§3). Nearest hit per ray, depth-jump flags, `VisibleBoundary`. |
| `boundaryFromRanges.m` | Ranges plus angles at a pose → closed world polygon. With jump flags: radial steps at jumps and spike collapse. Used by `castRays` (oracle), metrics and replay. |
| `isPointVisible.m` | Exact line-of-sight, aperture and range test. Scoring only. |
| `visibleBoundaryAt.m` | Oracle visible polygon from a pose (`castRays` → `VisibleBoundary`); the CBF's oracle boundary predictor. |
| `+internal/obstacleEdges.m` | All obstacles' edges stacked as start/end arrays, with the owning obstacle index. |
| `+internal/rayEdgeDistances.m` | Rays-by-edges distance matrix (`Inf` on a miss; ray-parallel edges count as misses). Used by `castRays` and `isPointVisible`. |

### `+metrics`

| File | Role |
|---|---|
| `signedEuclideanDistance.m` | Signed distance from points to a polygon: negative inside, positive outside, zero on the edge. Invalid polygons raise errors. Used for h. |
| `+internal/normalizeBoundary.m` | Closes and validates a polygon (degenerate or self-intersecting → error); all nonadjacent edge pairs are tested in one vectorized pass. |
| `+internal/pointToSegments.m` | Closest point and distance to a set of segments. |
| `filterConsistency.m` | Per-beam RMSE/NEES/NIS/gate-failure-rate report for one or several boundary-EKF results, by label and overall, plus first-measurement NIS and `StepRatio` stats. |
| `printFilterConsistency.m` | Thin text printer for a `filterConsistency` report. |

### `+estimation`

| File | Role |
|---|---|
| `makePassThroughEstimator.m` | `Initialize`/`Predict`/`Correct` handles; belief = latest scan, zero covariance, no-op predict. |
| `makeBoundaryEkf.m` | The boundary EKF (methodology §1.3): `Predict` transports with the real motion (`transportBoundary`/`propagateCovariance`); `Correct` gates, re-initialises or Joseph-updates each beam. Gradient sources (iii)/(iv). |
| `transportBoundary.m` | Mean-only transport and resampling core (eqs. 1-2, 7-11): moves `belief.Mean` to a hypothetical pose and labels each new beam blend/overlap/edge/disoccluded/no-return/new; calls the `+transport` stages in order and records each beam's provenance (`FromIndex`, `FromWeight`). |
| `+transport/relativeMotion.m` | Δp, Δψ from one pose to another, in the old sensor frame. |
| `+transport/transportEndpoints.m` | Eq. 1: old endpoints into the new frame (ρ, φ), plus the eq. 4 range variance for the jump test. |
| `+transport/classifyPairs.m` | Eq. 7: each consecutive old pair is a same surface, an open (no-return) pair, or a jump (`IsJumpPair`), with each side's `HasReturn` carried along for the edge/disocclusion stage. |
| `+transport/resampleOntoBeams.m` | Eq. 2: blend within surfaces (nearest surface wins; `overlap` when candidates disagree beyond the jump bound), keep open space open, leave jump-only beams at the internal `gap` label for the next stage. |
| `+transport/edgeAndShadowCandidates.m` | Eqs. 8-10: per jump pair, the mid-beam corner and shadow edge; resolves each `gap` beam to `edge` (t < 0, the near range) or `disoccluded` (t ≥ 0, the seen-free limit s_i), or `new` if no jump pair gives a usable candidate. Decision D4's exact-bearing match runs first. |
| `+transport/applyFreeSpaceHypothesis.m` | Eq. 11: blends each disoccluded beam's s_i toward the background range r_far,i (extending the far surface through its same-surface neighbour) by `params.FreeSpaceFraction`. |
| `+transport/fillUncoveredBeams.m` | Newly explored (`new`) beams from the nearest covered beam (every other label): extend its surface, or stay open next to open space. |
| `+transport/propagateCovariance.m` | Eqs. 3-6, 11: `Sigma_bar = F*Sigma*F' + Q` from `prediction`'s provenance and labels (two-entry blend/overlap rows, single-entry edge rows, zero hypothesis rows) and the process noise (motion terms, the eq. 8 edge term, and each hypothesis's own variance). Called only by the future EKF's `Predict`; `transportBoundary` and `predictBoundaryByTransport` stay mean-only. |
| `defaultTransportParams.m` | The eq. 7 jump-test constants (`JumpThreshold`, `JumpRelative`), the eq. 11 free-space fraction (`FreeSpaceFraction`), the eq. 6 process-noise coefficients (`ProcessNoiseFloor`, `TranslationNoise`, `RotationNoise`) and the sec. 4.7 newly-explored variance terms (`NewBeamSigmaFraction`, `NewBeamSlope`), used by `transportBoundary` and `propagateCovariance`. |
| `predictBoundaryByTransport.m` | Gradient source (ii): wraps `transportBoundary`, then builds the polygon from the predicted mean on every beam, with jump flags from decision D6. |

### `+control`

| File | Role |
|---|---|
| `referencePolicy.m` | No-control policy: returns the nominal input. |
| `makeSignedDistanceCbfPolicy.m` | **The CBF-QP policy** (§4): predict, barrier, gradient, solve. |
| `barrierGradient.m` | Least-squares `dh/dpose` and `dh/dtarget` over beam-spacing samples (eq. 12). |
| `solveCbfQp.m` | The slack CBF-QP with `quadprog` (nominal tracking plus the input-rate penalty), its status and its fallbacks. |
| `compareGradients.m` | Offline, per-step comparison of two boundary predictors' gradients on an existing `result` log (angle between position gradients, heading-gradient difference, each one's barrier value and status). |

### `+viz`: offline, reads `result` only

| File | Role |
|---|---|
| `animateSimulation.m` | Replay. World panel (obstacles, oracle/measurement/belief polygons, optional rays, agents, title with h and status) and range panel with fixed limits (oracle, measured, posterior ±1σ, the leader's bearing, and for filter runs markers on edge/overlap/disoccluded/newly explored beams). Rays off by default; legends below the axes. Always shows the final frame. |
| `plotSimulationSummary.m` | Trajectories (one legend entry for all obstacles) and the barrier h over time, with the intervals where the leader is truly hidden shaded. |
| `plotControlDiagnostics.m` | The QP diagnostics: h, inputs and slack over time; prints status counts. |
| `saveSimulationFrames.m` | Exports selected replay frames as PNGs under `frames/` (ignored by Git). |
| `plotTransportCheck.m` | Plots `transportBoundary`'s predicted mean against a truth scan, beams marked by label, with s_i/r_far,i markers on disoccluded beams; replay-only diagnostic. |
| `animateComparison.m` | Synchronized side-by-side replay of several runs on the same time grid (e.g. oracle vs source ii): true visible polygon, trajectories, agents, and the direction of each run's applied gradient (dh/dx, dh/dy); titles give t, h, status, and true visibility. |
| `plotGradientComparison.m` | Plots `control.compareGradients`' angle-to-oracle and each run's true visibility over time, one line per run. |

### `tests`

| File | Covers |
|---|---|
| `TestModels.m` | `FovSpec`, `Observer`, `polygonObstacle` construction and validation. |
| `TestRayCasting.m` | Ray–segment intersection, `castRays`, depth jumps, step construction, `isPointVisible`. |
| `TestMetrics.m` | Signed distance and invalid-geometry behaviour; a `metrics.filterConsistency` smoke test. |
| `TestControl.m` | CBF policy: statuses, fallbacks, input mapping, oracle and source (ii) gradient signs and magnitudes, source (ii) closed-loop determinism; sources (iii)/(iv) closed-loop determinism and the conservative η = 0 behavioural check. |
| `TestSimulation.m` | Runner contracts: causal order, sample counts, determinism, noise, replay and export, metrics, scenarios. |
| `TestTransport.m` | `estimation.transportBoundary`'s stages: blend/overlap, edges, disocclusion (eq. 11), newly explored, zero-motion identity, hand-computed corners; `estimation.transport.propagateCovariance`'s Jacobian F (the permanent finite-difference check) and Q (eqs. 6, 11), including the long-run symmetry/PSD check. |
| `TestFilter.m` | `estimation.makeBoundaryEkf`: first-scan initialization and the 1 mm noise floor, gating vs. fusion, Joseph-form symmetry/PSD over a long closed-loop run, a full `runScenario` pass with determinism, and the stage-1 static-observer scalar variance recursion. |

Run with `run('startup.m'); runtests('tests')`.
