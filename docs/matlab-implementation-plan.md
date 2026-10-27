# Implementation plan: boundary EKF + FoV-maintenance CBF in MATLAB

This plan is meant to be handed to a Claude Code session working inside Theo's existing MATLAB simulation. It is qualitative on purpose: it says *what* each piece must do, *how to check it*, and *in which order to build it*. The equations live in the methodology PDF (the compiled Metodologia section, the agreed model) and are only referenced here by name. When this plan and the PDF disagree, the PDF wins.

The only other file the code session needs is the methodology PDF. Everything else it needs (default numbers from the earlier prototype, the sparse-LiDAR pitfalls) is summarised in this plan.

---

## 0. How to use this plan in the code session

1. Give the session this file plus the methodology PDF (nothing else), and tell it to **audit the existing code first** (section 2) before writing anything new.
2. Build in the stage order of section 9. Each stage ends with a test that must pass before the next one starts. Do not let the session jump to the controller before the filter is validated.
3. Ask for small, separately testable functions (one per box in section 3), with all tuning numbers in one parameter struct, and a fixed random seed per run.
4. After each stage, ask for the plots listed in that stage. Most bugs in this project are geometric and are obvious in a picture and invisible in a number.

---

## 1. What the finished system does

Every simulation step:

1. **Truth.** The target moves along its predefined trajectory. The observer moves by the command from the previous step. The simulator ray-casts the true first-occlusion boundary `B_true(θ_i)` for the N beams of the observer's sector (aperture β, range R). This truth is used only to generate measurements and to score results.
2. **Sensor.** The oracle LiDAR returns `B_true` plus Gaussian noise of std σ_r. Beams with no return report R (pseudo-measurement).
3. **Filter, predict.** The belief `N(μ, Σ)` over the boundary vector (from the last step, in the old sensor frame) is transported to the new sensor frame using the *known* motion (Δp, Δψ) and resampled onto the fixed beam angles. This produces `μ̄`, `F`, `Q`, and `Σ̄ = F Σ Fᵀ + Q`, plus a label per beam (blend, edge, overlap/new occlusion, disoccluded, new in FoV).
4. **Filter, correct.** Each measured beam is gated by its innovation. Consistent beams go into a linear Kalman update (H selects measured beams). Inconsistent beams are re-initialised from their measurement.
5. **Confidence boundary.** `B_ε(θ_i) = μ_i − κ_ε σ_i`, clamped to [0, R] (and, for disoccluded beams before they are measured, at the shadow bound s). From it, build the estimated visible polygon `V_ε` in the sensor frame.
6. **Control.** The barrier is `h = −d(ỹ, V_ε)`, the negative signed distance of the target (in the sensor frame) to the visible set. Its gradient with respect to the observer's input is found numerically. A CBF-QP minimally modifies a nominal tracking command subject to `ḣ + γh ≥ −δ` and velocity limits.
7. **Log** everything needed for the metrics of section 8.

The same controller is run with four boundaries (oracle, raw LiDAR, EKF mean, EKF B_ε), which is the core comparison of the project.

---

## 2. Audit of the existing simulation (do this first)

The repo already has agents, scenarios, FoV ray-casting, noise addition, and an initial control attempt. Before adding the filter, the session should read it and write a short summary (in a markdown file in the repo) answering:

- **Agent model (confirmed by Theo).** The observer is a planar single integrator **with heading**: state (x, y, ψ), input (v_x, v_y, ω) expressed in the body frame, `ṗ = R(ψ) v`, `ψ̇ = ω`. Check the existing code matches this; Δp and Δψ per step feed the prediction.
- **Frames and angle conventions.** Where is the sensor origin, is θ measured from the body x-axis, counter-clockwise, in radians, in (−β/2, β/2]? Are beam angles fixed in the body frame? Write the convention down once and make every new function follow it.
- **Ray-casting.** Does it return the first-hit range per beam, saturated at R? Does it handle polygon vertices, grazing rays, and the observer standing very close to a wall? It will be the ground truth for everything, so it must be trusted (test in stage 0).
- **Noise.** Is noise added per beam, Gaussian, with a known σ_r? Is it clamped (negative ranges, ranges above R)? What happens to no-return beams? The filter must use the same σ_r the noise generator uses (a mismatch study is a later, separate experiment).
- **Target.** Is its trajectory predefined with a known velocity? The barrier's time derivative needs that velocity.
- **Existing control attempt.** What does it compute as the barrier, how does it get the gradient, which QP solver does it use (quadprog needs the Optimization Toolbox), and what nominal controller does it modify? Keep whatever is reusable; section 7 says what the target design is.
- **Time step and rates.** One dt for physics, LiDAR, filter and control? The prediction model assumes small steps (per-step point shift at most about one beam spacing), see section 4.8.

Output of this stage: the summary file and a list of mismatches with the agreed model. Fix the mismatches before stage 1.

---

## 3. Suggested structure

Keep it modular so each piece can be unit-tested and swapped. Names are suggestions; follow the existing repo's style if it has one.

- **Parameters**: one struct with every number (N, β, R, σ_r, dt, ε, jump threshold τ, Q coefficients, gating confidence, σ_min, new-FoV prior, CBF γ, slack weight λ, velocity limits, seed). Every experiment is a parameter file plus a scenario.
- **Scenario**: obstacle polygons, initial observer pose, target trajectory (function of time returning position and velocity).
- **Truth / sensor**: existing ray-caster, plus a measurement function that adds noise and handles no-return beams, and optionally a mask of which beams are measured (for the sparse extension later).
- **Filter state**: μ (N×1), Σ (N×N, full), beam labels from the last prediction, and the auxiliary per-beam info the predictor needs (e.g. which beams were edges).
- **Predict**: takes (μ, Σ, Δp, Δψ, params), returns (μ̄, Σ̄, F, Q, labels, shadow bounds s). Internally split into: transport old endpoints; jump test between old neighbours; candidate generation per new beam (blend, edge, disoccluded); nearest-candidate selection; new-FoV fill; Q construction.
- **Correct**: takes (μ̄, Σ̄, z, measured mask, params), returns (μ, Σ, gate decisions, innovations, NIS values).
- **Confidence boundary and visible set**: B_ε per beam, polygon of V_ε, signed distance of a point to it.
- **Barrier and gradient**: h for a given pose and boundary, numerical gradient, ∂h/∂t.
- **CBF-QP**: nominal command, QP assembly and solve, slack, fallback when the solver fails.
- **Metrics and plots**: filter metrics, control metrics, the standard figures (section 8.3).
- **Runner**: one function that runs a scenario with a parameter set and a boundary source (oracle / raw / mean / B_ε) and returns a log struct. A second function runs Monte Carlo over seeds.

---

## 4. The filter: prediction step

All rules below are the ones already agreed in the methodology PDF. The session should implement them exactly, in this order, and keep each case labelled so it can be debugged and scored separately.

### 4.1 Transport of old endpoints

Each old beam j has an endpoint at range μ_j along its old direction. Move that point into the new sensor frame using the known translation and rotation (both expressed in the old frame), and record its new range ρ_j and bearing φ_j. Also record the derivatives of ρ_j and φ_j with respect to μ_j (the tilt angle α_j = θ_j − Δψ − φ_j gives both: cos α_j for range, sin α_j / ρ_j for bearing).

Watch for: angle wrapping of φ_j; beams whose old endpoint is at R because there was no return (they are not real surfaces; see 4.9).

### 4.2 Jump (edge) test between old neighbours

For each consecutive old pair (j, j+1), decide whether they belong to the same surface: the range difference must be below max(τ, 3·combined σ of the pair), where the σ's are the old variances projected onto range. An optional relative term (8 % of the nearer range) can be added as a parameter. Reasonable starting value: τ = 0.35 m. Store the result as a boolean per pair; it drives everything else.

### 4.3 Resampling onto the fixed beams (blend)

For each new beam angle θ_i, find the old pairs whose transported bearings bracket it. For a pair with no jump, the candidate is the linear interpolation of the two ranges at θ_i with weight w_i, and its F row has two entries given by the formula in the methodology (weight times the range derivative, minus the slope term that couples the bearing derivative with the local boundary slope m_i). Keep the slope term; it is part of the agreed model and backed by Matthies App. C. Make it switchable for an ablation.

### 4.4 Edges (pair with a jump, no crossing into the shadow)

Never interpolate across a jump. If the bracketing pair failed the jump test and the beam does not fall into the old shadow (4.6), the candidate is the nearer point, with a single-entry F row. The extra edge variance q_e was judged unnecessary in the prototype (the disocclusion rule covers it); keep it as a parameter defaulting to zero, so it can be revisited during NEES tuning.

### 4.5 New occlusions (several candidates for one beam)

After motion, several pairs can bracket the same new beam (a near surface sliding in front of a far one). Keep the candidate with the smallest range, take its F row, and discard the farther one. Label the beam "overlap" if a farther candidate existed clearly behind it; this label is useful when reading NEES by beam type.

### 4.6 Disocclusions (seeing behind a corner)

When the beam falls in the angular gap of a jump pair, use the shadow-line construction from the methodology: the near (corner) endpoint, transported, plus the old corner beam's direction in the new frame define the old shadow edge. Intersect the new beam with it to get s_i (the region before s_i was seen free). Then:

- if the crossing parameter t is negative, the beam still hits the near surface: treat it as an edge (nearest point);
- if the beam is nearly parallel to the shadow edge, treat it as a new-FoV beam (4.7);
- otherwise the prior is the midpoint of s_i and the far neighbour's range, with σ chosen so that B_ε falls exactly on s_i (σ = (μ̄ − s_i)/κ_ε, floored at σ_min). Cap s_i at the far range. The F row is zero (no cross-covariance with the old estimate).

Store s_i per beam so B_ε can be clamped at it until the beam is measured.

### 4.7 New-FoV beams

Beams that no pair brackets (they entered through the sector sides during a rotation) get a zero F row, a mean equal to the nearest covered beam's range (or R), and a large variance σ_0², so the first measurement defines them.

### 4.8 Process noise Q and the small-step condition

A starting point that worked in the prototype: √q = σ_q + 0.05·|Δp| + 0.2·|Δψ| with σ_q ≈ 0.1 m (|Δp| in m, |Δψ| in rad), new-FoV prior mean R/2 with σ = R/3. Diagonal Q per beam: a floor q_0 for interpolation error, plus terms growing with |Δp|² and Δψ², plus the edge term (default zero). The prototype also used spatially correlated Q within a surface (Gaussian kernel over angle, zeroed across jumps); treat that as an option, not part of the baseline. All Q coefficients are tuned empirically and checked by NEES (stage 6).

The linearisation assumes that per step, |Δp|/ρ + |Δψ| is at most about one beam spacing. If the commanded motion over one control step violates this near close obstacles, split the step into substeps and run predict (and correct, if a scan is available) per substep. Ask the session to log how often substepping triggers.

### 4.9 Covariance and housekeeping

- Σ̄ = F Σ Fᵀ + Q, kept full. Symmetrise after every operation; check positive semi-definiteness in debug mode.
- Clamp means to [0, R]. Decide explicitly how no-return beams (μ near R) take part in the jump test and the shadow construction: an endpoint at R is "free space up to R", not a wall. The safest first version treats them as a surface at R and checks in a scenario with open space that nothing absurd happens at the sector's far edge.
- Validate F numerically: perturb each μ_j slightly, rerun the mean prediction, and compare the finite-difference column with F (excluding beams whose label changed). This test caught a missing slope term in the prototype and should be a permanent unit test.

---

## 5. The filter: correction step

- **Measurement model.** z_i = B_i + noise, H selects the measured beams (all beams in the baseline; a subset in the sparse extension). No-return beams enter as z = R. Consider a slightly larger variance for them if they turn out to cause issues; record the choice.
- **Gating.** For each measured beam, compare the normalised innovation squared with a χ²(1) threshold. A failed gate means a visibility change the geometry did not predict (missed edge, occluder appearing). Re-initialise that beam: mean z_i, variance σ_r², all cross-covariances zero. Do not also fuse that measurement in the update.
- **Update.** Standard Kalman update on the gated beams, Joseph form for numerical robustness. With full Σ, a measurement on one beam also corrects its correlated neighbours; this is intended.
- **Logs.** Innovations, NIS per beam, gate decisions, number of re-initialisations per step and per beam label.

---

## 6. Confidence boundary and visible set

- B_ε(θ_i) = μ_i − κ_ε σ_i with κ_ε = Φ⁻¹(1 − ε) (1.645 at 95 %). This is a per-beam marginal bound, not a joint one; say so in the results.
- Clamp at 0, at R, and at s_i for disoccluded beams not yet measured.
- The visible polygon V_ε in the sensor frame is the sensor origin plus the points B_ε(θ_i)·u(θ_i) in angle order. Between beams the polygon interpolates linearly, which can be slightly optimistic at concave corners; a conservative option is to use the min of the two neighbours' B_ε over each angular interval. Make it a parameter and compare.
- Signed distance d(point, V): negative inside (minus the distance to the polygon boundary), positive outside. Unit-test it on simple shapes (inside, outside, on a vertex, behind an occluder within the sector, outside the aperture, beyond R).
- Oracle visible set: same construction from B_true. Raw-LiDAR visible set: from z directly (optionally z − κ_ε σ_r).

---

## 7. FoV-maintenance control (CBF-QP)

### 7.1 Barrier

h = −d(ỹ, V), with ỹ the target position in the sensor frame. h ≥ 0 means the target is inside the (estimated) visible set. Add a margin for the target's size if the target is not a point.

### 7.2 Gradient and time derivative

There is no map, so h cannot be differentiated analytically with respect to the observer's motion. The methodology's choice is numerical: for each input direction (v_x, v_y, ω), apply a small displacement of the observer, *run the prediction model* (means and variances) to get the boundary that would be seen from there, rebuild V_ε, recompute h, and take finite differences. Three (or six, for central differences) extra predictions per step are cheap at N ≈ 100.

A cheaper fallback, if this is slow or noisy: freeze the current B_ε endpoints as world points and only move the sector. It ignores how the boundary itself shifts, so use it only as an ablation or a backup.

∂h/∂t comes from the target's known velocity: move the target by v_target·δt with the observer fixed and difference.

h is only Lipschitz (it has kinks at polygon vertices and when the closest feature switches), so expect gradient jumps. Use a perturbation size comparable to a fraction of the beam spacing at the target's range, and log the gradient to spot chattering.

### 7.3 QP

Minimise ‖u − u_nom‖² + λδ² subject to gᵀu + ∂h/∂t + γh ≥ −δ, δ ≥ 0, and box limits on (v_x, v_y, ω). The slack keeps the QP feasible; its value is a metric. Use quadprog if available; otherwise this QP is small enough for a closed-form projection or a tiny active-set routine. Always have a fallback (e.g. stop, or the previous command) if the solver fails, and log it.

### 7.4 Nominal controller

Something simple that would lose the target without the CBF: e.g. go to a waypoint or follow a fixed path while keeping a heading toward the target, or track the target at a standoff distance with a pure-pursuit heading. The CBF should be visibly active only when occluders threaten visibility. Reuse the existing initial control attempt if it fits.

### 7.5 What the controller is fed

Four boundary sources, same everything else: oracle (B_true), raw LiDAR, EKF mean μ, EKF B_ε. Visibility violations are always scored against the *true* visible set.

Out of scope (state it in the write-up): guarantees for jumps in h caused by filter updates (the safe set is time-varying and discontinuous), obstacle-collision barrier (add one only if the observer hits walls in the experiments).

---

## 8. Validation, metrics and figures

### 8.1 Filter metrics (scored against ray-cast truth, per beam label)

- RMSE of μ vs B_true, overall and per label (blend, edge, overlap, disoccluded, new-FoV, recently re-initialised).
- NEES per beam (scalar, (μ_i − B_i)²/Σ_ii), averaged over time and Monte Carlo runs, compared with the χ²(1) mean of 1 and its confidence band. Also a joint NEES over all beams when Σ is well conditioned. Consistent filter: averages near 1; much above 1 means Q is too small or a geometric case is wrong; much below means over-inflated Q.
- NIS of the innovations, same idea, available without ground truth.
- Calibration of B_ε: the fraction of beams with B_ε > B_true (false-visible rate) should be at most ε. Plot it against ε for several values (90/95/99 %).
- Conservatism: area of the true visible set lost by V_ε, per step.
- Number of gate failures and re-initialisations per step, per label (a high rate means the prediction is missing a geometric case).

### 8.2 Control metrics (scored against the true visible set)

- Fraction of time the target is truly visible.
- Number and total duration of true visibility violations.
- Minimum true h along the run.
- Control effort, slack usage, QP failures, substep count.
- Conservatism of motion (e.g. extra path length or distance kept from occluders compared with the oracle).

### 8.3 Standard figures (ask for them as reusable plot functions)

1. World view animation: obstacles, observer, sector, target, true visible set, V_ε, measured endpoints.
2. Polar view per step: B_true, z, μ, μ ± κσ, B_ε, beams coloured by label, s_i markers on disoccluded beams.
3. Time series: per-label RMSE and NEES with χ² bounds; NIS.
4. Calibration plot: false-visible rate vs ε.
5. Control: true h and estimated h over time, u vs u_nom, slack.
6. Comparison table over Monte Carlo runs: oracle / raw / mean / B_ε.

---

## 9. Staged build and test order

Each stage lists what to build and what must be true before moving on.

**Stage 0. Ground truth and sensor.**
Build: audit (section 2), parameter struct, scenario library (below), seeded noise.
Pass when: ray-casting matches hand-computed ranges in a simple room; noise histogram has the right σ; no-return handling is as intended; the observer and target move as commanded.

**Stage 1. Static observer, correction only.**
Build: filter state, correction with Joseph form; prediction = identity with a small Q.
Pass when: variance per beam decreases as in a scalar Kalman filter toward the steady state for that Q and σ_r; NEES ≈ 1; NIS ≈ 1.

**Stage 2. Pure rotation.**
Build: transport and blend (rotation makes α = 0, so F is just the weights), new-FoV beams.
Pass when: *prediction-only test*: start from the true boundary with tiny variance, no noise, no correction, rotate a few steps; μ̄ tracks B_true on all non-new beams up to interpolation error. Then with correction: NEES ≈ 1.

**Stage 3. Translation in an open room.**
Build: full F including the slope term; finite-difference F unit test.
Pass when: F matches finite differences; prediction-only drift in a room without interior obstacles is small and grows smoothly with step size; with correction, NEES ≈ 1 for blend beams.

**Stage 4. Edges, new occlusions, disocclusions.**
Build: jump test, nearest-candidate rule, shadow construction with s_i and ε-matched prior, labels.
Pass when: in "box in front of wall" and "pass a corner" scenarios, no fake surfaces appear between box and wall in the prediction plot; disoccluded beams have B_ε exactly on s_i before measurement; false-visible rate on disoccluded beams is at most ε; overlap beams take the near surface.

**Stage 5. Gating and re-initialisation.**
Build: χ² gate and re-init.
Pass when: deliberately disabling the disocclusion rule causes gate failures exactly at the disoccluded beams (shows the gate is a safety net), and with the rule enabled gate failures are rare.

**Stage 6. Tuning Q by consistency.**
Build: Monte Carlo runner (e.g. 20 to 50 seeds per scenario), NEES/NIS per label.
Tune q_0, q_v, q_ω (and q_e if needed) so that NEES per label is inside the χ² band, starting from the blend beams. Record the final values and the tuning procedure; it goes into the report.

**Stage 7. Confidence boundary and visible set.**
Build: B_ε with clamps, polygon, signed distance, both interpolation options.
Pass when: signed-distance unit tests pass; false-visible rate is at most ε for each ε tested.

**Stage 8. CBF-QP.**
Build in this order: (a) oracle boundary, check the controller keeps the target visible in all scenarios; (b) numerical gradient checked against a brute-force finite difference of the full closed loop for one step; (c) EKF mean; (d) EKF B_ε; (e) raw LiDAR.
Pass when: oracle run has zero true violations (or only ones explained by velocity limits, visible as slack); the four sources run end to end on all scenarios.

**Stage 9. Experiments.**
Monte Carlo over seeds for each scenario × boundary source × ε × σ_r. Produce the comparison table and figures. Expected story: raw LiDAR jitters and violates; mean is less jittery but still violates near edges and disocclusions about half the time the target sits near the boundary; B_ε is calibrated with fewer violations at the cost of conservatism; oracle is the reference.

**Stage 10. Ablations and extensions (only after stage 9).**
- Ablations: no slope term in F; diagonal Σ; naive interpolation across jumps; no disocclusion rule (beam gets new-FoV prior instead); no gating; correlated vs diagonal Q; linear vs min-of-neighbours polygon.
- Sparse LiDAR: measure every S-th beam. Known pitfalls: shifts that are multiples of S never fill the gaps (aliasing), and translation does not sweep along the heading. Remedies: interleave the ray offset per scan, or BLUE weights in the blend. Keep the number of state bins M separate from the number of rays from the start, so this is a mask change and not a refactor.
- Noise mismatch: filter σ_r different from the true σ_r.

### Scenario library (suggested)

- **S1 open room**: walls only. Tests transport and new-FoV beams.
- **S2 box in front of a wall**: one strong jump. Tests edges, overlap, disocclusion when moving sideways.
- **S3 corridor corner**: the target turns a corner; the observer must reposition to keep it visible. Main control showcase.
- **S4 pillars**: target walks behind a row of pillars. Repeated occlusions and disocclusions, CBF activity.
- **S5 cluttered**: several polygons, for Monte Carlo statistics.

---

## 10. Pitfalls to tell the code session about

- Frames: Δp and Δψ in the prediction are expressed in the *old* sensor frame. Mixing world and body frames here is the most likely bug.
- Angle wrapping in φ_j and in the bracket search, especially for β close to 360°.
- Interpolating across a jump creates walls that do not exist; any visible "bridge" between a near and a far surface in the polar plot means the jump test or the candidate selection is wrong.
- Zero F rows (disoccluded, new-FoV) must have zero cross-covariance; do not copy neighbour covariances into them.
- Σ losing symmetry or positive definiteness after many steps; symmetrise and use Joseph form.
- Using the same step for physics and prediction with large velocities near obstacles breaks the small-step assumption; substep.
- Gate failures are a symptom, not a fix. If they are frequent in a beam category, the prediction for that category is wrong.
- The barrier is non-smooth; small perturbations for the gradient, and do not be surprised by chattering at polygon vertices.
- Score visibility violations against the true visible set, never against the estimated one.

---

## 11. Open decisions to settle with Theo during implementation

1. Sector size: aperture β and N (e.g. 90° to 120° with about 1° spacing, or a 360° LiDAR with the FoV as a sub-sector). The filter can run on the full LiDAR while the visible set uses only the FoV sector.
2. Gradient method: full prediction under perturbation (methodology) vs frozen world points (fallback).
3. Polygon interpolation between beams: linear vs conservative min-of-neighbours.
4. Whether an obstacle-collision barrier is needed in the experiments.
5. Whether the sparse-LiDAR extension is part of the final report or future work.
