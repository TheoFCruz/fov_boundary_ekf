# Codex implementation brief: fov_metrics checkpoint 1

## 1. Task and scope

Refactor and extend the MATLAB repository [TheoFCruz/fov_metrics](https://github.com/TheoFCruz/fov_metrics) into a scripted, reproducible 2D simulation testbed. Implement **only checkpoint 1**:

- Observer and follower motion over a configured time interval.
- Ray casting from the moving observer against static polygonal obstacles.
- A replaceable boundary-estimation interface with a pass-through implementation: posterior mean equals the measured range vector and posterior covariance is zero.
- Replay animation showing both agents, the static base, raycasts, and the estimator output.
- An observer-policy interface that can later host visibility/base-link control, but currently returns a prescribed velocity reference.
- Logs, basic diagnostics, tests, and clear usage documentation.

Both agents must exist and move in this checkpoint. The follower is the observed target; the observer will eventually follow it through the user's existing FoV controller. For now, **both follow configured inputs independently**. Do not implement pursuit logic, a CBF/QP, or any base-link constraint.

Do not implement a real EKF, covariance transport, segmentation, uncertainty resets, confidence-region guarantees, occupancy mapping, persistent map memory, ROS, CrazySim, an App Designer application, sliders, timers, or interactive geometry editing. No new required toolbox dependencies. Do not solve the future research problems inside the placeholder.

## 2. Grounding and repository instructions

This brief is based on the inspected commit `0c8ff48a4234716fd1d27a5205058f953abfa3c1`. Inspect the actual checkout and adapt small naming details if it has changed. Read `AGENTS.md` and the complete `docs/IMPLEMENTATION_PLAN.md` before editing; preserve unrelated user changes.

Existing useful code:

| Existing item | Treatment |
| --- | --- |
| `+fov/Observer.m`, `FovSpec.m` | Keep geometry objects; construct observer snapshots from simulation poses. No need to make them mutable. |
| `+fov/castRays.m` and geometry helpers | Reuse for synthetic sensing. Preserve the existing API and static tests. |
| `+fov/sampleRayAngles.m` | Currently returns **world-frame** angles. Convert explicitly at the sensor interface. |
| `+metrics/signedEuclideanDistance.m` | Preserve negative-inside convention and static polygon contract. |
| `+metrics/sampleField.m`, contour plotting | Keep for optional static analysis; do not calculate a full XY field every simulation step. |
| Existing scenario factories | Preserve; add a dynamic factory rather than breaking static demos. |
| `+viz/plotScenario.m` | Preserve static plotting. Avoid repeatedly calling it inside animation because it creates new objects. |
| `+viz/interactiveScenario.m`, interactive script | Mark legacy/deprecated in docs, remove from the recommended workflow; do not delete merely for cleanup. |

Update `docs/IMPLEMENTATION_PLAN.md` to make this checkpoint the active direction and supersede further interactive development. Preserve historical completed items. Add a linked detailed checkpoint document if useful. Update README examples. Follow AGENTS.md's rules for focused tests, honest verification notes, and no unsolicited commits/pushes/PRs.

## 3. Design: separate simulation, estimator, policy, and playback

The main numerical runner is headless: it returns a result and never opens figures, pauses, or uses wall-clock timing to advance physics. Animation consumes the completed result. This makes replay speed independent of dynamics and enables later batch experiments.

Suggested additions (small functions and plain structs are sufficient; no abstract-class framework):

| Package | Responsibilities / suggested entry points |
| --- | --- |
| `+simulation` | `runScenario`, `validateScenario`, `stepPose`, `sampleVelocityReference` |
| `+sensing` | `raycastScan`: wrap existing ray casting and produce a clean measurement structure |
| `+estimation` | `makePassThroughEstimator`: return initialization/prediction/correction callbacks |
| `+control` | `referencePolicy`: observer policy returning the scenario reference, with no optimization |
| `+fov` | `boundaryFromRanges`: deterministic conversion from relative-angle ranges to a world polygon |
| `+viz` | `animateSimulation`, `plotSimulationSummary`; internal create/update graphics helpers if needed |
| `+scenarios` | `movingPair`: complete scripted example with static obstacles and base |
| `scripts` | `runMovingPair.m`: startup, scenario, run, summary, replay |
| `tests` | Focused tests for dynamics, references, scan conversion, estimator, runner, and playback |

Keep geometry independent of estimation; plotting independent of policy and estimator internals. Do not build a plugin registry or generic multi-robot middleware. Two named moving agents and one fixed base are enough.

## 4. Scenario configuration and motion convention

Use SI units and radians. Store each pose as a finite row vector `[x, y, yaw]` in the world frame. The follower uses the same pose representation even if its heading is not relevant to visibility yet. The base has a fixed world `Position`; no state integration.

Use one unambiguous input convention throughout checkpoint 1: `[vxBody, vyBody, omega]`. Translational velocities are in each agent's own body frame. Do not assume they are world-frame velocities. Keep yaw unwrapped in the state; wrap only display angles if necessary.

For zero-order-held input on `[t_k, t_(k+1))`, use documented forward Euler:

```text
p_(k+1)   = p_k + dt * Rot(yaw_k) * [vxBody; vyBody]
yaw_(k+1) = yaw_k + dt * omega
```

Both agents advance from the same pre-step snapshot. This checkpoint is kinematic; it does not model collision response or actuator tracking. Demo paths must avoid physical obstacles even though their lines of sight may become occluded.

Example scenario contract (the implementer should make this a working example):

```matlab
scenario.Name = "Moving pair around a wall";
scenario.Time.Start = 0;
scenario.Time.Stop = 12;
scenario.Time.Step = 0.05;
scenario.Observer.InitialPose = [0, 0, 0];
scenario.Observer.Fov = fov.FovSpec(10, deg2rad(100));
scenario.Follower.InitialPose = [2, -1, 0];
scenario.Base.Position = [-1, 0];
scenario.Obstacles = fov.polygonObstacle('wall', ...
    [4, -2; 4, 2; 4.3, 2; 4.3, -2]);
scenario.Sensor.NumRays = 81;

scenario.Observer.Reference.Times = [0; 4; 8];
scenario.Observer.Reference.Values = [ ...
    0.15, 0, 0; ...
    0, 0, 0.15; ...
    0.10, 0, 0];
scenario.Follower.Reference.Times = [0; 6];
scenario.Follower.Reference.Values = [0, 0.2, 0; 0, -0.2, 0];

scenario.Observer.Policy = @control.referencePolicy;
scenario.Estimator = estimation.makePassThroughEstimator();
scenario.Playback.Bounds = [-2, 12, -7, 7];
scenario.Playback.FrameRate = 20;
scenario.Playback.Speed = 1;
```

Reference semantics: each row is active from its time until the next row; final row holds until Stop. At an exact breakpoint use the **new** row. Require the first time to equal Start, strictly increasing times, finite values, three velocity columns, and breakpoints aligned with the simulation step. Require `(Stop-Start)/Step` to be integral within a tolerance. Reject malformed inputs instead of silently truncating time. Function-handle trajectory references are an optional later addition, not required here.

## 5. Scan, belief, and geometry contracts

### Measurement structure

`sensing.raycastScan(pose, fovSpec, obstacles, sensorConfig, time)` returns a clean scan and, separately, the raw cast result for evaluation/rendering.

```matlab
scan.Time                  % scalar
scan.ObserverPose          % 1x3 world pose
scan.Angles                % Nx1 sensor-relative angles, ordered
scan.Ranges                % Nx1 finite first-hit / range-capped distances
scan.HasReturn             % Nx1 logical: true for an obstacle return
scan.IsValid               % Nx1 logical: sensor sample valid
scan.MaxRange
scan.OpeningAngle
```

Checkpoint 1 uses noiseless rays, all valid, and one scan per simulation step. Compute relative angles from current world ray angles minus observer heading, preserving ordering. For full-circle scans, retain the existing convention of not duplicating the end direction. Do not individually wrap angles in a way that reverses ordering at +/-pi.

Pass only this scan to the estimator. Obstacle polygons, `HitObstacleId`, surface identities, and oracle visibility results must not be available through the estimator API. The raw cast result belongs to the simulation/evaluation layer.

### Belief structure

Use `Mean` and `Covariance` fields (corresponding to mu and Sigma); the state is the boundary range vector, not the observer pose.

```matlab
belief.Time
belief.Angles              % Nx1 relative output grid
belief.Mean                % Nx1 boundary-range estimate
belief.Covariance          % NxN, use sparse zeros for the placeholder
belief.IsSupported         % Nx1
belief.HasReturn           % Nx1, distinguishes physical surface vs range cap
belief.ObserverPose        % pose for which this posterior is defined
belief.MaxRange
belief.OpeningAngle
belief.Method              % "pass-through", never label this a working EKF
```

The requested `bel(x)=(x,0)` means: `belief.Mean = scan.Ranges`, `belief.Covariance = sparse(N,N)`. This is a deterministic/degenerate posterior used to validate plumbing, not a probabilistic confidence claim. `IsSupported = scan.IsValid` at sampled directions; valid no-return samples still carry their separate `HasReturn=false` flag. A maximum-range sample is not a detected surface.

Use the same input and output angular grid for checkpoint 1 (`N=M`). Do not implement inter-grid interpolation, smoothing, segment detection, confidence margins, or covariance propagation. In particular, zero covariance at samples does not certify the polygon between them.

### Estimator lifecycle

Use an explicit replaceable interface, for example a struct of handles:

```matlab
state = estimator.Initialize(config);
state = estimator.Predict(state, motion, dt);
[state, belief] = estimator.Correct(state, scan);
```

`motion` contains the previous and current observer poses and the actually applied input for the preceding interval. Future motion compensation can therefore use relative displacement without receiving a world map.

The pass-through `Predict` is intentionally a no-op and must not publish a supposedly transported boundary. `Correct` overwrites the output with the current scan. The runner requests output only after correction. Do not add fake process noise, small nonzero covariance, or an EKF equation just to fill the interface. The estimator stores only its current state; run history is stored by the logger, never fed back as a map.

### Geometry conversion

Provide a reusable `fov.boundaryFromRanges(observerPose, angles, ranges, openingAngle)` helper. Build world endpoints using `yaw + angles`. Match the current `castRays.VisibleBoundary` ordering and closure exactly for equivalent inputs:

- Partial FoV: observer, ordered endpoints, observer.
- Full FoV: ordered endpoints, first endpoint.

The renderer and future controller must construct estimated geometry from **belief fields**, not copy `rawCast.VisibleBoundary`. Verify they match in this placeholder. Keep the direct linear connection baseline; document that it is exact only for the constructed polygon, not true sparse-ray visibility. Segment-aware jumps, unsupported wedges, and unions are later work; do not introduce them in checkpoint 1. Preserve current degeneracy rules instead of silently repairing invalid polygons.

## 6. Causal simulation order and future control hook

For K intervals use K+1 samples, including Start and Stop. At Start: initialize poses, cast the first scan, initialize/correct estimator, and log the posterior.

At each `t_k < Stop`:

1. Build a controller observation from the **current posterior**, observer pose, follower pose, base position, and time. Follower pose is exactly known in this checkpoint, explicitly an ideal-state assumption.
2. Sample observer and follower velocity references at `t_k`.
3. Call `[uObserver, policyState, diagnostics] = scenario.Observer.Policy(observation, observerReference, policyState)`. The default returns the reference unchanged. Use followerReference directly as follower input.
4. Log the inputs applied over this interval and policy diagnostics.
5. Advance both agents from their current poses to `t_(k+1)`.
6. Call estimator prediction using this completed observer motion; cast the scan at `t_(k+1)` and correct; then log both poses, scan, and posterior at that same time.

No additional control/integration after the terminal sample. No one-step mismatch between scans, poses, and rendered boundaries. Use the previous interval's applied observer input for prediction, not the next requested input.

The policy observation must not contain obstacle polygons, raw oracle cast results, future target inputs, or the complete logged trajectory. It can access current target state and the reference explicitly supplied to it. This will later allow the observer policy to use estimated FoV geometry while respecting the static-base link.

For now, draw the base marker and an optional dashed observer-base line labeled as a geometric reference, **not proof of a valid link**. Whether that link means distance, line of sight, FoV, or communication connectivity is deliberately not chosen in this checkpoint.

## 7. Results, animation, and simple diagnostics

Suggested result fields:

```matlab
result.Config              % effective parameters; include schema version
result.Time                % (K+1)x1
result.ObserverPose        % (K+1)x3
result.FollowerPose        % (K+1)x3
result.ObserverInput       % Kx3, interval-aligned
result.FollowerInput       % Kx3
result.Scans               % K+1 cells
result.Beliefs             % K+1 cells
result.PolicyDiagnostics   % K cells
result.Metrics
```

Keep polygons/static data once in Config. Raw raycast outputs may be logged for replay if useful; avoid unnecessarily duplicating dense covariance and boundary arrays. Use sparse zero covariance. Diagnostics/history are offline logs, not estimator memory.

`viz.animateSimulation(result, ...)` uses a normal MATLAB `figure`, fixed equal-axis bounds, and graphics handles created once. Update `XData`/`YData`/patch vertices rather than clearing axes or recreating legends each frame. No sliders, `uifigure`, callbacks driving physics, or timer infrastructure. Closing the playback window exits gracefully.

Required views:

- World axes: static obstacles/base, distinct observer/follower markers, heading arrows, trajectories, observer rays, raw sampled visible polygon, and estimated boundary with a distinct style. Coincidence of raw and estimated curves is expected.
- Boundary axes: range versus **relative angle** for scan and posterior mean. Read covariance diagonals for a standard-deviation display; hide or annotate the zero-width band in pass-through mode. Label it as a placeholder.
- Current simulation time in the title.

Playback speed and render frame rate only select/delay displayed samples, never change numerical simulation. Always include the final sample. A static summary function must work without animation.

Basic metrics: sampled-polygon signed distance of follower (negative inside), polygon visibility indicator, observer-follower distance, observer-base distance, and pass-through range residual. Clearly label polygon visibility as a **sampled approximation**. Do not claim oracle continuous visibility or safety. A separate exact line-of-sight evaluator may be added later. For polygon degeneracy, return documented invalid diagnostics rather than fabricated distance values.

Return a result that the user can `save` to MAT. Video export, full Monte Carlo reporting, contour animations, and performance benchmarking are optional later work, not completion requirements.

## 8. Implementation sequence and tests

Complete in small coherent stages; all boxes below start unchecked:

- [ ] Reconcile this brief with repository instructions; document the active checkpoint and deprecated interactive workflow.
- [ ] Implement/validate scenario and velocity-reference contracts plus kinematic stepping.
- [ ] Implement scan adapter, belief contract, pass-through lifecycle, and shared polygon conversion.
- [ ] Implement headless runner, causal policy hook, logs, and simple metrics.
- [ ] Implement replay/summary plotting and a moving-pair example.
- [ ] Add focused tests, run available checks, update README and plan with actual outcomes.

Required tests:

1. Zero input leaves either agent unchanged. Constant translation at zero yaw gives expected displacement. At yaw=pi/2, positive body-x moves along positive world-y. Pure rotation leaves position fixed.
2. Reference breakpoints select the new row; final row holds. Reject invalid times, dimensions, nonfinite values, or nonaligned schedule boundaries.
3. Exactly K+1 state/scan/belief samples and K inputs. Both agents advance once per interval; final time is correct.
4. With nonzero observer heading, relative/world angle conversion reconstructs the same endpoints as `castRays` (including wrap-near headings and full circle).
5. Pass-through mean equals scan ranges, covariance has correct shape and is exactly zero, and no-return flags survive. Predictor is no-op, not fake motion compensation.
6. Estimated polygon from belief equals raw sampled polygon for supported test cases. Retain existing geometry and metric tests.
7. Inject a test estimator that deliberately changes range output: estimated rendering geometry and policy observation must reflect the modified output, proving no bypass around the estimator.
8. Inject a custom observer policy returning zero velocity while the follower moves. Observer stays fixed and the follower continues, proving the policy seam is active.
9. Repeated runs are deterministic. The runner creates no figures. Changing playback settings does not change result arrays.
10. Invisible-figure smoke tests for first/final frames: expected artists exist, update successfully, and do not grow in number with frame count. Clean up figures even if tests fail.

Use the repository's MATLAB test workflow. If MATLAB is unavailable, report that explicitly; do not mark runtime or visual checks as passed. Do not add dependencies or perform a build/compile contrary to AGENTS.md.

## 9. Acceptance and handoff

A user can run:

```matlab
run('startup.m');
scenario = scenarios.movingPair();
result = simulation.runScenario(scenario);
viz.plotSimulationSummary(result);
viz.animateSimulation(result);
```

They can change time bounds, step, velocity schedules, obstacles, FoV parameters, ray count, and initial poses by editing configuration only. Both agents move; rays follow observer translation/rotation; raw and estimated boundaries overlap; replacing the estimator or observer policy does not require editing the runner.

Keep the checkpoint small and working. Final implementation report should list changed files, runnable commands, tests actually run, remaining limitations, and the exact extension points for a real EKF and the user's controller. Do not implement later stages or claim closed-loop visibility/base connectivity is already enforced.
