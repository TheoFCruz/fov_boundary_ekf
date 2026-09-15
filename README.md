# Motion-Aware FOV Boundary Estimation

MATLAB testbed for estimating the observer-frame first-occlusion-boundary range
profile `B_t(theta)` from simulated range scans. The target is the current
boundary belief, its support, and its uncertainty—not observer or follower pose
estimation, occupancy, or a persistent map.

The current pre-EKF foundation provides a deterministic moving-observer/moving-
follower simulation, ray-cast sensing with optional seeded range noise, a
pass-through boundary estimator, a belief-only CBF-QP baseline, and offline
replay. The EKF, segmentation, motion transport, covariance propagation, and
support/reset/forgetting policies are not implemented yet.

## Quick start

```matlab
run('startup.m');
scenario = scenarios.movingPair();
scenario.Sensor.RangeNoiseStd = 0.05; % m; use zero for noiseless scans
scenario.Sensor.Seed = 17;
result = simulation.runScenario(scenario);

viz.plotSimulationSummary(result);
viz.animateSimulation(result, 'ShowRays', false, 'ShowHitPoints', true);
```

For the belief-derived CBF-QP experiment, use `scenarios.controlTest()` or
configure `control.makeSignedDistanceCbfPolicy()` on a moving-pair scenario.
It requires MATLAB Optimization Toolbox (`quadprog`) and is an experimental
sampled-polygon baseline, not a safety guarantee or controller redesign.

## Structure

```text
+fov/          FOV models, obstacle validation, ray geometry, boundary conversion
  +internal/   Low-level geometry helpers
+sensing/      Measurement-only ray-cast scan adapter
+estimation/   Replaceable boundary-estimator lifecycle
+simulation/   Scenario validation, kinematics, causal runner, and logs
+control/      Observer-policy callbacks and CBF-QP baseline
+metrics/      Sampled-polygon signed distance
+viz/          Summary, replay, diagnostics, and static frame export
+scenarios/    Reusable dynamic scenario factories
scripts/       Thin experiment entry points
tests/         MATLAB unit tests
```

## Experiment configuration

Start from `scenarios.movingPair()` and edit its plain-struct fields:

| Concern | Fields |
| --- | --- |
| Time | `scenario.Time.Start`, `Stop`, `Step` |
| Initial poses | `Observer.InitialPose`, `Follower.InitialPose` |
| FOV | `Observer.Fov = fov.FovSpec(maxRange, openingAngle)` |
| Body-frame references | `Observer.Reference`, `Follower.Reference` |
| Sensor | `Sensor.NumRays`, `RangeNoiseStd`, `Seed` |
| Estimator | `Estimator = estimation.makePassThroughEstimator()` |
| Policy | `Observer.Policy` |
| Replay | `Playback.Bounds`, `FrameRate`, `Speed` |

Angles are radians. World poses are `[x, y, yaw]`; held body-frame inputs are
`[vxBody, vyBody, omega]`. The runner uses synchronized zero-order-held forward
Euler: both agents step from the same pre-step snapshot.

## Scan, belief, and causal contracts

`sensing.raycastScan` produces first-return measurements on a sensor-relative
angular grid. `HasReturn=false` is censored range-cap information, not an
obstacle at maximum range. Positive `RangeNoiseStd` adds clipped Gaussian noise
only to actual returns, using the simulation-owned local `RandStream`; the
oracle ray cast remains noiseless and MATLAB's global RNG is unchanged.

The pass-through belief has `Mean = scan.Ranges`, exactly-zero sparse
`Covariance`, `IsSupported`, `HasReturn`, pose, and FOV metadata. It is a
plumbing placeholder, not an EKF or a confidence claim. Checkpoint 1 uses equal
scan and posterior angular grids.

For `K` input intervals, `simulation.runScenario` logs `K+1` synchronized poses,
scans, and beliefs plus `K` applied inputs. At each sample, the policy uses the
current posterior; after both agents advance, the estimator receives completed
observer motion through `Predict` before the next scan goes to `Correct`.
Estimators and policies do not receive obstacle polygons, raw casts, future
inputs, or complete logs.

## Replay and diagnostics

`simulation.runScenario` is headless and deterministic. It returns `Config`,
`Time`, poses, inputs, `Scans`, `Beliefs`, `RawCasts`, `PolicyDiagnostics`, and
sampled-polygon diagnostics in `Metrics`. Replay consumes completed logs only;
frame rate and speed affect display samples, never simulation state. It always
includes the final frame.

```matlab
viz.plotSimulationSummary(result);
viz.animateSimulation(result, 'ShowRays', true, 'ShowHitPoints', false);
viz.plotControlDiagnostics(result);
manifest = viz.saveSimulationFrames(result, ...
    'FrameIndices', [1, 25, 50], 'Resolution', 150);
```

`viz.saveSimulationFrames` exports selected logged samples to ignored
`frames/` PNG directories. It does not advance the simulation, resample noise,
or mutate the completed result.

## Sampled-polygon signed distance

`metrics.signedEuclideanDistance` remains the computational primitive used by
the simulation diagnostics and CBF baseline. It is negative inside a valid
sampled polygon, positive outside, and zero on its boundary. Unsupported,
degenerate, or self-intersecting constructed boundaries remain invalid rather
than being repaired into fabricated visibility.

Static Cartesian field sampling, contour rendering, static scenario plotting,
and the interactive heading explorer have been retired. They are not supported
workflows for this boundary-estimation project.

## Roadmap and limitations

The [semester roadmap](docs/probabilistic_fov_project_roadmap%284%29.pdf)
orders the remaining work as segmentation/no-return handling, deterministic
within-segment motion transport, EKF covariance and reset/forgetting behavior,
then sparse-ray reconstruction and an existing-controller demonstration.

Do not infer true-FoV, intersample visibility, collision, base-link, or safety
guarantees from the current sampled-polygon diagnostics or controller baseline.
There is no SLAM, occupancy grid, persistent map, hidden-surface model, or scan
archive.

## Verification status

`tests/TestSimulation.m` passed 22/22 on 2026-09-15 after the frame-export
assertion fix. The focused controller suite, full MATLAB suite, and desktop
replay/export checks remain unrun.

The standard full-suite command, when verification is desired, is:

```matlab
run('startup.m');
results = runtests('tests');
table(results)
```
