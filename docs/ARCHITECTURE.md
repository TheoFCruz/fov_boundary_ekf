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
     differences.
   - Leader: it displaces the leader by ±ℓ steps in x and y against the
     fixed current polygon (8 evaluations, no new boundary), giving ∂h/∂y.
   - Samples whose polygon is invalid geometry are dropped. If too few
     remain, the gradient is marked unusable.
4. **CBF terms:** `coefficient = ∂h/∂q · G(ψ)` maps body inputs to ḣ.
   `TargetDrift = ∂h/∂y · ẏ` is ∂h/∂t from the leader's known velocity.
5. **QP (`control.solveCbfQp`, direct `quadprog`):** minimize
   `(u − u_nom)ᵀW(u − u_nom) + λδ²` subject to `coefficient·u + δ ≥ −γh − drift`,
   input bounds and `δ ≥ 0`. It also handles the zero-authority and QP-failure
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
- **Methodology transport:** `estimation.predictBoundaryByTransport` is a
  placeholder that errors. It will call `estimation.transportBoundary`
  (below) to move the belief mean to `pose` and return the polygon built
  from the predicted mean. The barrier uses the expected boundary, not a
  confidence bound such as μ − κσ (§1.4).

`estimation.transportBoundary(belief, pose, params)` is the mean-only
transport core (`docs/metodologia.pdf` §1.2, eqs. 1-2, 7): it transports
every old beam's endpoint to the hypothetical `pose` and resamples onto the
same fixed grid, labelling each new beam `blend`, `no-return`, `jump-gap`,
or `new`. It takes the same `(belief, pose)` pair as a `BoundaryPredictor`
so the still-placeholder `predictBoundaryByTransport` wrapper (building the
polygon) and the future EKF's `Predict` step (with the real motion instead
of a hypothetical displacement) can both call it directly. It does not yet
propagate covariance, place a corner between a jump pair (eqs. 8-10), or
apply the free-space disocclusion fraction η (eq. 11); jump gaps are filled
by the newly-explored rule and labelled `jump-gap` until that lands.

### How the methodology's sources map onto the slots

| Source (§1.5) | `BoundaryPredictor` | `Estimator` | Exists? |
|---|---|---|---|
| (i) Oracle | `fov.visibleBoundaryAt` wrapper | any (unused for geometry) | Yes |
| (ii) Latest reading | `estimation.predictBoundaryByTransport` | pass-through | Placeholder only |
| (iii) EKF | `estimation.predictBoundaryByTransport` | EKF | No |
| (iv) EKF, η = 0 | same, with η = 0 | EKF with η = 0 | No |

The transport inside `predictBoundaryByTransport` is the prediction model of
§1.2. The EKF's `Predict` will use the same transport with the real motion;
the policy calls the predictor with hypothetical displacements.

## 6. Data structures

| Struct | Created by | Main fields |
|---|---|---|
| scenario | `scenarios.*` | `Name`, `Time{Start,Stop,Step}`, `Observer{InitialPose,Fov,Reference,Policy}`, `Leader{InitialPose,Reference}`, `Base{Position}`, `Obstacles`, `Sensor{NumRays,RangeNoiseStd,Seed,Tolerance}`, `Estimator`, `Playback{Bounds,FrameRate,Speed}` |
| Reference schedule | scenario | `Times` (breakpoints), `Values` (rows of `[vx, vy, omega]`, held until the next breakpoint) |
| obstacle | `fov.polygonObstacle` | `Name`, `Vertices` (convex, ordered) |
| scan | `sensing.raycastScan` | `Time`, `ObserverPose`, `Angles` (relative to heading), `Ranges` (noisy), `HasReturn`, `IsValid`, `MaxRange`, `OpeningAngle` |
| rawCast | `fov.castRays` | noiseless `Distances`, `EndPoints`, `HitObstacleId`, `IsDepthJump`, `VisibleBoundary` |
| belief | `Estimator.Correct` | `Time`, `Angles`, `Mean`, `Covariance`, `IsSupported`, `HasReturn`, `ObserverPose`, `MaxRange`, `OpeningAngle`, `Method` |
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
| `scripts/runMovingPair.m` | Moving-pair scenario with an oracle-predictor CBF, noise 0.1; summary and replay. |
| `scripts/runBoxInFrontOfWall.m` | Box occlusion scenario (oracle CBF); summary, diagnostics, replay. |
| `scripts/runClutteredField.m` | Cluttered-field scenario (oracle CBF); summary, diagnostics, replay. |
| `scripts/plotControlDiagnostics.m` | Plots CBF diagnostics for a `result` already in the workspace. |
| `scripts/checkTransport.m` | Plots `estimation.transportBoundary` against truth in `scenarios.openRoom`, for a rotation and a translation case. |

### `+scenarios`: experiment definitions (return a scenario struct, run nothing)

| File | Role |
|---|---|
| `movingPair.m` | Base scenario: 10 m / 100° FoV, 81 rays, one wall, scripted references, reference policy. Others start from it. |
| `controlTest.m` | No obstacles; the leader drives toward max range; oracle CBF with zero nominal input. |
| `boxInFrontOfWall.m` | Compact 2 m layout: a 0.2 m box in front of a wall; the leader crosses the box's shadow; oracle CBF. |
| `clutteredField.m` | Compact layout with 7 small posts; the leader visits waypoints over 32 s; oracle CBF. |
| `openRoom.m` | Plan scenario S1: an enclosed room with no depth jumps; tests transport and newly-explored beams. |
| `template.m` | Every configurable field written out with units; copy it to make new scenarios. |

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

### `+estimation`

| File | Role |
|---|---|
| `makePassThroughEstimator.m` | `Initialize`/`Predict`/`Correct` handles; belief = latest scan, zero covariance, no-op predict. The EKF will be a sibling `make...Estimator`. |
| `transportBoundary.m` | Mean-only transport and resampling core (eqs. 1-2, 7): moves `belief.Mean` to a hypothetical pose and labels each new beam blend/no-return/jump-gap/new. |
| `defaultTransportParams.m` | The eq. 7 jump-test constants (`JumpThreshold`, `JumpRelative`) used by `transportBoundary`. |
| `predictBoundaryByTransport.m` | Placeholder for the methodology boundary predictor (wraps `transportBoundary`, then builds the polygon from the predicted mean); errors until implemented. |

### `+control`

| File | Role |
|---|---|
| `referencePolicy.m` | No-control policy: returns the nominal input. |
| `makeSignedDistanceCbfPolicy.m` | **The CBF-QP policy** (§4): predict, barrier, gradient, solve. |
| `barrierGradient.m` | Least-squares `dh/dpose` and `dh/dtarget` over beam-spacing samples (eq. 12). |
| `solveCbfQp.m` | The slack CBF-QP with `quadprog`, its status and its fallbacks. |

### `+viz`: offline, reads `result` only

| File | Role |
|---|---|
| `animateSimulation.m` | Replay. World panel (obstacles, oracle/measurement/belief polygons, rays, agents, title with h and status) and range panel (oracle, measured, posterior). Always shows the final frame. |
| `plotSimulationSummary.m` | Trajectories and the sampled-polygon distance over time. |
| `plotControlDiagnostics.m` | h, inputs, slack and step sizes over time; prints status counts. |
| `saveSimulationFrames.m` | Exports selected replay frames as PNGs under `frames/` (ignored by Git). |
| `plotTransportCheck.m` | Plots `transportBoundary`'s predicted mean against a truth scan, beams marked by label; replay-only diagnostic. |

### `tests`

| File | Covers |
|---|---|
| `TestModels.m` | `FovSpec`, `Observer`, `polygonObstacle` construction and validation. |
| `TestRayCasting.m` | Ray–segment intersection, `castRays`, depth jumps, step construction, `isPointVisible`. |
| `TestMetrics.m` | Signed distance and invalid-geometry behaviour. |
| `TestControl.m` | CBF policy: statuses, fallbacks, input mapping, oracle gradient signs and magnitudes. |
| `TestSimulation.m` | Runner contracts: causal order, sample counts, determinism, noise, replay and export, metrics, scenarios. |

Run with `run('startup.m'); runtests('tests')`.
