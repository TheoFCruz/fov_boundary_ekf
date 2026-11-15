# Implementation plan: boundary EKF + FoV-maintenance CBF in MATLAB

This plan is meant to be handed to a Claude Code session working inside Theo's existing MATLAB simulation. It is qualitative on purpose: it says *what* each piece must do, *how to check it*, and *in which order to build it*. The equations live in the methodology PDF (`docs/metodologia.pdf`, the agreed model) and are only referenced here by name or equation number. When this plan and the PDF disagree, the PDF wins.

Revised 2026-11-12 to match the updated methodology: the controller uses the **expected** boundary (the filter mean), not a confidence bound `μ − κσ`, and disocclusions use the free-space hypothesis η (eq. 11) instead of a prior matched to a confidence level. There is no `B_ε`/`V_ε` anywhere in the pipeline.

Decisions settled with Theo on 2026-11-12, before stage 1 (details in the sections cited):

- **No-return beams** are free space up to R, never a wall: they stay at the cap after transport, a pair with one return and one no-return is always a jump, and a no-return beam is never the near surface of an edge (4.9).
- **Disocclusion variance**: keep the PDF's `(r_far − s)²/12 + q_0` for now; it may be revisited after stage 6 (4.6).
- **Gradient steps**: the samples at displaced poses use only the transported mean, which is exact geometry, so steps of one or two beam spacings are fine; the small-step condition only limits the linearised F and Q (7.2).
- **Jump threshold**: start with τ plus the relative term (8 % of the nearer range); a slope-aware test comes later (4.2).
- **Support**: hypothesis beams (disoccluded, newly explored) stay unsupported, but the transport predictor builds its polygon from the mean on every beam (4.10).
- **State grid**: one state bin per ray (equal scan and belief grids); the sparse extension enters as a measurement mask (10).
- **Baseline η = 0.5**; η = 0 is source (iv) (11).

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

1. **Truth.** The target (leader) moves along its predefined trajectory. The observer moves by the command from the previous step. The simulator ray-casts the true first-occlusion boundary `B_true(θ_i)` for the N beams of the observer's sector (aperture β, range R). This truth is used only to generate measurements and to score results.
2. **Sensor.** The oracle LiDAR returns `B_true` plus Gaussian noise of std σ_r. Beams with no return report R (pseudo-measurement).
3. **Filter, predict.** The belief `N(μ, Σ)` over the boundary vector (from the last step, in the old sensor frame) is transported to the new sensor frame using the *known* motion (Δp, Δψ) and resampled onto the fixed beam angles. This produces `μ̄`, `F`, `Q`, and `Σ̄ = F Σ Fᵀ + Q`, plus a label per beam (blend, edge, overlap/new occlusion, disoccluded, new in FoV).
4. **Filter, correct.** Each measured beam is gated by its innovation. Consistent beams go into a linear Kalman update (H selects measured beams). Inconsistent beams are re-initialised from their measurement.
5. **Visible set.** Build the estimated visible polygon `V(μ)` from the corrected mean `μ`: the sensor origin plus the points `μ_i·u(θ_i)` in angle order. No confidence bound is applied (methodology §1.4): disoccluded and newly explored beams have high variance by construction, so a bound like `μ − κσ` would collapse the visible set exactly where the gradient has to point toward the leader.
6. **Control.** The barrier is `h = −d(ỹ, V(μ)) − m`, the negative signed distance of the target (in the sensor frame) to the visible set, minus a margin. Its gradient with respect to the observer's pose is found by running the prediction model at displaced poses (eq. 12). A CBF-QP minimally modifies a nominal tracking command subject to `ḣ + γh ≥ −δ` and velocity limits (eq. 13).
7. **Log** everything needed for the metrics of section 8.

The same controller is run with four gradient sources (methodology §1.5), which is the core comparison of the project:

- **(i) oracle**: re-cast the true geometry at each displaced pose;
- **(ii) latest reading**: the prediction model applied to the latest scan, with no memory and no covariance;
- **(iii) filter**: the prediction model applied to the corrected filter mean `μ_k`;
- **(iv) filter, η = 0**: as (iii) with the conservative free-space hypothesis.

(ii) vs (iii) isolates the effect of fusion on the gradient; (iii) vs (iv) isolates the effect of the free-space hypothesis.

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

- **Parameters**: one struct with every number (N, β, R, σ_r, dt, jump threshold τ, free-space fraction η, Q coefficients, gating confidence γ, new-FoV prior σ_0 and κ, CBF γ, margin m, slack weight λ, velocity limits, seed). Every experiment is a parameter file plus a scenario.
- **Scenario**: obstacle polygons, initial observer pose, target trajectory (function of time returning position and velocity).
- **Truth / sensor**: existing ray-caster, plus a measurement function that adds noise and handles no-return beams, and optionally a mask of which beams are measured (for the sparse extension later).
- **Filter state**: μ (N×1), Σ (N×N, full), beam labels from the last prediction, and the auxiliary per-beam info the predictor needs (e.g. which beams were edges, shadow bounds s_i).
- **Predict**: takes (μ, Σ, Δp, Δψ, params), returns (μ̄, Σ̄, F, Q, labels, shadow bounds s). Internally split into: transport old endpoints; jump test between old neighbours; corner estimate per jump pair; candidate generation per new beam (blend, edge, disoccluded); nearest-candidate selection; new-FoV fill; Q construction. The same mean transport is what the controller runs at displaced poses.
- **Correct**: takes (μ̄, Σ̄, z, measured mask, params), returns (μ, Σ, gate decisions, innovations, NIS values).
- **Visible set**: polygon `V(μ)` from a range vector, signed distance of a point to it.
- **Barrier and gradient**: h for a given pose and boundary, the least-squares gradient over predicted boundaries at displaced poses, ∂h/∂t.
- **CBF-QP**: nominal command, QP assembly and solve, slack, fallback when the solver fails.
- **Metrics and plots**: filter metrics, control metrics, the standard figures (section 8.3).
- **Runner**: one function that runs a scenario with a parameter set and a gradient source (oracle / latest reading / filter / filter η = 0) and returns a log struct. A second function runs Monte Carlo over seeds.

---

## 4. The filter: prediction step

All rules below are the ones agreed in the methodology PDF (§1.2). The session should implement them exactly, in this order, and keep each case labelled so it can be debugged and scored separately.

### 4.1 Transport of old endpoints (eq. 1)

Each old beam j has an endpoint at range μ_j along its old direction. Move that point into the new sensor frame using the known translation and rotation (both expressed in the old frame), and record its new range ρ_j and bearing φ_j. Also record the derivatives of ρ_j and φ_j with respect to μ_j (eq. 4: the tilt angle α_j = θ_j − Δψ − φ_j gives both: cos α_j for range, sin α_j / ρ_j for bearing).

Watch for: angle wrapping of φ_j; beams whose old endpoint is at R because there was no return (they are not real surfaces; see 4.9).

### 4.2 Jump (edge) test between old neighbours (eq. 7)

For each consecutive old pair (j, j+1), decide whether they belong to the same surface: |ρ_a − ρ_b| ≤ max(τ, 3·√(σ_a² + σ_b²)), with σ_j² = cos²α_j Σ_jj (the old variances projected onto range). The uncertainty term keeps noise from splitting a surface into false edges; this is why the mean prediction needs Σ even though the barrier does not. For source (ii), which has no covariance, only τ applies. The baseline replaces τ by max(τ, 0.08·min(ρ_a, ρ_b)), a relative term of 8 % of the nearer range, so that grazing surfaces far away are not split into false edges; keep both numbers as parameters. A slope-aware test (comparing against the range change the local slope predicts) comes later, after this one is tested in S3. Reasonable starting value: τ = 0.35 m. Store the result as a boolean per pair; it drives everything else.

### 4.3 Resampling onto the fixed beams (blend, eqs. 2 and 5)

For each new beam angle θ_i, find the old pairs whose transported bearings bracket it. For a pair with no jump, the candidate is the linear interpolation of the two ranges at θ_i with weight w_i, and its F row has two entries given by eq. 5 (weight times the range derivative, minus the slope term that couples the bearing derivative with the local boundary slope m_i). Keep the slope term; it is part of the agreed model and backed by Matthies App. C. Make it switchable for an ablation.

### 4.4 Edges and the corner position (eqs. 8–10)

Never interpolate across a jump. A jump pair (n, f) has a near point n and a far point f, and the reading does not say where the near surface's corner is between them. Use a single estimate for every decision about the pair: the corner at the mid-beam angle θ* = (θ_n + θ_f)/2, at the near range μ_n, transported as **c**, with **d** the direction of the old ray through it in the new frame (eq. 8).

A new beam between φ_n and φ_f is intersected with the old shadow edge **c** + t**d** (eqs. 9–10), giving s_i (distance to the shadow edge) and t_i:

- t_i < 0: the beam still hits the near surface. Candidate μ̄_i = ρ_n, single-entry F row F_in = cos α_n.
- t_i ≥ 0: the beam enters the old shadow; use the disocclusion rule (4.6).
- beam nearly parallel to **d**: no useful crossing; treat it as newly explored (4.7).

Edge variance q_e,i (in Q, eq. 6): with the corner at mid-beam the maximum tangential error is half a beam; on beams within half a beam of **c**, which can fall on either side of the edge, q_e,i is of the order (ρ_f − ρ_n)²/4, so the next measurement decides the side.

### 4.5 New occlusions (several candidates for one beam)

After motion, several pairs can bracket the same new beam (a near surface sliding in front of a far one). Keep the candidate with the smallest range, take its F row, and discard the farther one (the polar version of the collision rule of Engel et al.). Label the beam "overlap" if a farther candidate existed clearly behind it; this label is useful when reading NEES by beam type.

### 4.6 Disocclusions (seeing behind a corner, eq. 11)

A beam with t_i ≥ 0 crosses seen-free space up to s_i and then enters a region no earlier reading saw. The conservative estimate μ̄_i = s_i would cancel the gradient (the seen-free set is fixed in the world, so the leader's distance to it does not change as the observer moves). Use the free-space hypothesis instead, a constant fraction η ∈ [0, 1] of the interval between the seen-free limit and the background:

- μ̄_i = s_i + η (r_far,i − s_i), σ̄_i² = (r_far,i − s_i)²/12 + q_0 (the PDF's variance, kept for now; revisit after stage 6 if the first-measurement innovations say otherwise);
- r_far,i is the background range along the new beam, found by extending the far surface: the intersection of the beam with the line through the transported points f and f′ (f's neighbour on the same surface), or ρ_f if f has no valid neighbour. Cap s_i at r_far,i;
- the F row is zero (the element starts with no cross-covariance), since **c**×**d** does not depend on μ_n.

η = 0 recovers the conservative limit (source (iv)); η = 1 assumes nothing lies between the corner and the background. Store s_i and r_far,i per beam for diagnostics and plots. Later extensions (not baseline): η varying per beam with the crossing distance, η_i = η_∞(1 − e^{−t_i/ℓ}), or using the leader's known position and predicted path to mark free space.

### 4.7 Newly explored (new-FoV) beams

Beams that no pair brackets (they entered through the sector sides during a rotation) get a zero F row and a mean that assumes the nearest covered surface continues: the intersection of the beam with the line through the last two covered points of that surface (or the nearest covered beam's range if there are not two points on the same surface), saturated at R. The variance grows with the angular distance δ_i to the last covered beam: σ̄_i² = σ_0² + (κ μ̄_i δ_i)². The same free-space hypothesis as for disocclusions can be applied when the leader is in that direction.

### 4.8 Process noise Q and the small-step condition (eq. 6)

Diagonal Q per beam: Q_ii = q_0 + q_v‖Δp‖² + q_ω Δψ² + q_e,i, with q_0 for interpolation error, terms growing with the step, and the edge term of 4.4. A starting point that worked in the prototype: √q = σ_q + 0.05·|Δp| + 0.2·|Δψ| with σ_q ≈ 0.1 m (|Δp| in m, |Δψ| in rad). The prototype also used spatially correlated Q within a surface (Gaussian kernel over angle, zeroed across jumps); treat that as an option, not part of the baseline. All Q coefficients are tuned empirically so that the normalised error per beam stays near one (stage 6).

The linearisation assumes that per step, ‖Δp‖/ρ + |Δψ| is at most about one beam spacing. If the commanded motion over one control step violates this near close obstacles, split the step into substeps and run predict (and correct, if a scan is available) per substep. Ask the session to log how often substepping triggers.

### 4.9 Covariance and housekeeping

- Σ̄ = F Σ Fᵀ + Q, kept full (interpolation correlates neighbouring beams; treating them as independent would artificially shrink the variance). Symmetrise after every operation; check positive semi-definiteness in debug mode.
- Clamp means to [0, R]. No-return beams (`HasReturn = false`) mean "free space up to R", not a wall at R. Decided rule: a no-return beam is not transported as an endpoint; it stays at the cap R after transport. A pair with one return and one no-return is always a jump, so no surface is interpolated toward the cap. A no-return beam is never the near surface of an edge: it creates no corner and no shadow. Check in S1 that nothing absurd happens at the sector's far edge.
- Validate F numerically: perturb each μ_j slightly, rerun the mean prediction, and compare the finite-difference column with F (excluding beams whose label changed). This test caught a missing slope term in the prototype and should be a permanent unit test.

### 4.10 Support and the predicted polygon

`IsSupported` stays honest: it is true only for beams backed by a measurement or by interpolation within one surface. Disoccluded and newly explored beams carry a hypothesis, not evidence, so they keep `IsSupported = false` until measured, and the per-beam label says which hypothesis they hold. With dense scans every step, `Correct` measures them right after `Predict`, so the posterior is unaffected; the label matters for the prior, for sparse sensing, and for the predictor.

The CBF gradient needs exactly those hypothesis beams (it is what eq. 11 is for), so the transport predictor (`estimation.predictBoundaryByTransport`) builds its polygon from the predicted mean on **every** beam, ignoring the support gate. That polygon is a predicted boundary for the gradient, not a visibility claim. Everywhere else (metrics, replay, the estimator's own outputs) the rule stands: no closed polygon from unsupported geometry. The polygon construction will need an explicit option for this; its default stays support-gated.

---

## 5. The filter: correction step (methodology §1.3)

- **Measurement model.** z_i = B_i + noise, H selects the measured beams (all beams in the baseline; a subset in the sparse extension). No-return beams enter as z = R. Consider a slightly larger variance for them if they turn out to cause issues; record the choice.
- **Gating.** For each measured beam, test (z_i − μ̄_i)²/(Σ̄_ii + σ_r²) > χ²_{1,1−γ}. A failed gate means a visibility change the geometry did not predict (missed edge, occluder appearing). Re-initialise that beam: mean z_i, variance σ_r², all cross-covariances zero. Do not also fuse that measurement in the update.
- **Update.** Standard Kalman update on the gated beams, Joseph form for numerical robustness. With full Σ, a measurement on one beam also corrects its correlated neighbours; this is intended. On disoccluded and newly explored beams the prior variance is large, so the gain is near one and the measurement replaces the free-space hypothesis.
- **η calibration.** The normalised innovation of the *first* measurement of a disoccluded or newly explored beam, (z_i − μ̄_i)²/(Σ̄_ii + σ_r²), shows whether η is calibrated for the scene. Log it per label.
- **Logs.** Innovations, NIS per beam, gate decisions, number of re-initialisations per step and per beam label.

---

## 6. Visible set and signed distance

- The visible polygon V(**b**) in the sensor frame is the sensor origin plus the points b_i·u(θ_i) in angle order, for a range vector **b**. The barrier uses **b** = μ_k (the corrected mean, in which no beam is disoccluded); the gradient samples use the predicted means μ̄ at displaced poses. No confidence bound is applied (see section 1, step 5).
- Between beams the polygon interpolates linearly, which can be slightly optimistic at concave corners; a conservative option is to use the min of the two neighbours' range over each angular interval. Make it a parameter and compare. Never bridge a depth jump with a chord into the shadow.
- Signed distance d(point, V): negative inside (minus the distance to the polygon boundary), positive outside. Unit-test it on simple shapes (inside, outside, on a vertex, behind an occluder within the sector, outside the aperture, beyond R).
- Oracle visible set: same construction from B_true (re-cast at each pose). Latest-reading visible set: from z directly.

---

## 7. FoV-maintenance control (CBF-QP, methodology §1.4)

### 7.1 Barrier

h(**q**, **b**) = −d(ỹ, V(**b**)) − m, with ỹ = R(−ψ)(**y** − **p**) the target position in the sensor frame and m a margin (for the target's size, if it is not a point). h ≥ 0 means the target is inside the estimated visible set. The value at the current pose is h_k = h(**q**_k, μ_k).

### 7.2 Gradient and time derivative (eq. 12)

There is no map, so h cannot be differentiated analytically with respect to the observer's motion. The methodology's choice: for each pose axis e ∈ {x, y, ψ} and ℓ ∈ {1, 2}, transport μ_k to the displaced pose **q**_k ± ℓδ_e**e**_e with the prediction model, rebuild the polygon from the predicted mean μ̄^(e,±ℓ), and evaluate h^(e,±ℓ). The gradient component is the least-squares fit of the central differences, g_e = Σ_ℓ ℓ(h^(e,+ℓ) − h^(e,−ℓ)) / (2δ_e Σ_ℓ ℓ²), with steps of the order of the beam spacing: δ_ψ = Δθ, δ_x = δ_y = ‖**y** − **p**_k‖Δθ.

Smaller steps would capture the steps of the sampled polygon; with steps of this size, g_k approximates the gradient of a version of h smoothed at the scale of one beam. Because each sample uses the predicted mean, g_k approximates, to first order, the gradient of the expected barrier under the prediction model, including the free-space hypothesis behind obstacles. The samples use only the transported mean, never the linearised covariance: mean transport is exact geometry (move the endpoints, resample within surfaces), so the small-step condition of 4.8, which limits F and Q, does not limit the gradient steps, even at ℓ = 2 for a distant target. The predicted covariance could weight the fit (e.g. by the inverse linearised variance of each h sample); that is an extension.

∂h/∂t = ∇_**y** h · **ẏ**, from the leader's known velocity, evaluated on μ_k.

h is only piecewise smooth (the nearest point on the polygon switches edges, the boundary is sampled and updated with every reading), so expect gradient jumps; log the gradient to spot chattering. Freezing the current boundary and only moving the sector is not one of the four sources: it ignores how the boundary shifts and gives wrong-sign gradients near occluders.

### 7.3 QP (eq. 13)

Minimise ‖**u** − **u**_nom‖² + λδ² subject to **g**_kᵀ**G**(ψ_k)**u** + ∂h/∂t + γh_k ≥ −δ, δ ≥ 0, and box limits on (v_x, v_y, ω), with **G**(ψ) = diag(**R**(ψ), 1). The slack keeps the QP feasible; its value is a metric. Use quadprog if available; otherwise this QP is small enough for a closed-form projection or a tiny active-set routine. Always have a fallback if the solver fails, and log it; the repo uses the nominal command clipped to the bounds (never a stop command, which causes stop-go motion).

The result is a visibility-maintenance constraint evaluated empirically, without a formal invariance guarantee: the barrier is only piecewise smooth, the boundary is sampled and updated with every reading, and control is applied in discrete time.

### 7.4 Nominal controller

Something simple that would lose the target without the CBF: e.g. go to a waypoint or follow a fixed path while keeping a heading toward the target, or track the target at a standoff distance with a pure-pursuit heading. The CBF should be visibly active only when occluders threaten visibility. Reuse the existing initial control attempt if it fits.

### 7.5 What the controller is fed

The four gradient sources of section 1, same scenario, limits and noise seeds for all: oracle, latest reading, filter, filter with η = 0. Visibility violations are always scored against the *true* visible set.

Out of scope (state it in the write-up): guarantees for jumps in h caused by filter updates (the safe set is time-varying and discontinuous), obstacle-collision barrier (add one only if the observer hits walls in the experiments).

---

## 8. Validation, metrics and figures

### 8.1 Filter metrics (scored against ray-cast truth, per beam label)

- Error of μ vs B_true (RMSE), overall and per label (blend/interpolated, edge, overlap, disoccluded, newly explored, recently re-initialised).
- Normalised error per beam ((μ_i − B_i)²/Σ_ii, NEES), averaged over time and Monte Carlo runs, compared with the χ²(1) mean of 1 and its confidence band. Also a joint NEES over all beams when Σ is well conditioned. Consistent filter: averages near 1; much above 1 means Q is too small or a geometric case is wrong; much below means over-inflated Q.
- NIS of the innovations, same idea, available without ground truth.
- First-measurement normalised innovation on disoccluded and newly explored beams, as a check on η (section 5).
- Number of gate failures and re-initialisations per step, per label (a high rate means the prediction is missing a geometric case).

### 8.2 Control metrics (scored against the true visible set)

- Fraction of time the target is truly visible.
- Time with the target hidden beyond a tolerance; number and duration of true visibility violations.
- Angle between the gradient and the oracle gradient at the same step.
- Minimum true h along the run.
- Control effort, slack usage, QP failures, substep count.
- Conservatism of motion (e.g. extra path length or distance kept from occluders compared with the oracle).

### 8.3 Standard figures (ask for them as reusable plot functions)

1. World view animation: obstacles, observer, sector, target, true visible set, estimated visible set V(μ), measured endpoints.
2. Polar view per step: B_true, z, μ, μ ± 2σ (display only), beams coloured by label, s_i and r_far,i markers on disoccluded beams.
3. Time series: per-label error and NEES with χ² bounds; NIS.
4. Gradient: angle to the oracle gradient over time, per source.
5. Control: true h and estimated h over time, u vs u_nom, slack.
6. Comparison table over Monte Carlo runs: oracle / latest reading / filter / filter η = 0.

---

## 9. Staged build and test order

Each stage lists what to build and what must be true before moving on.

**Build-order deviation (2026-11-13):** the mean-only boundary-transport
model (this section's "P0-P2" checkpoint 1: `estimation.defaultTransportParams`,
`scenarios.openRoom`, `estimation.transportBoundary` covering eqs. 1, 2, and
the eq. 7 jump test) is being built before stage 1's correction-only filter,
ahead of the staged order below. Edges/disocclusion/the predictor wrapper
(checkpoint 2) and covariance (stage 1's P6) follow later. This gives the
CBF gradient source (ii), the latest-reading transport predictor, without
any EKF, which is useful on its own and exercises the transport core the
EKF's `Predict` will reuse.

**Stage 0. Ground truth and sensor.**
Build: audit (section 2), parameter struct, scenario library (below), seeded noise.
Pass when: ray-casting matches hand-computed ranges in a simple room; noise histogram has the right σ; no-return handling is as intended; the observer and target move as commanded.

**Stage 1. Static observer, correction only.**
Build: filter state, correction with Joseph form; prediction = identity with a small Q.
Pass when: variance per beam decreases as in a scalar Kalman filter toward the steady state for that Q and σ_r; NEES ≈ 1; NIS ≈ 1.

**Stage 2. Pure rotation.**
Build: transport and blend (rotation makes α = 0, so F is just the weights), newly explored beams.
Pass when: *prediction-only test*: start from the true boundary with tiny variance, no noise, no correction, rotate a few steps; μ̄ tracks B_true on all non-new beams up to interpolation error. Then with correction: NEES ≈ 1.

**Stage 3. Translation in an open room.**
Build: full F including the slope term; finite-difference F unit test.
Pass when: F matches finite differences; prediction-only drift in a room without interior obstacles is small and grows smoothly with step size; with correction, NEES ≈ 1 for blend beams.

**Stage 4. Edges, new occlusions, disocclusions.**
Build: jump test, mid-beam corner estimate, nearest-candidate rule, shadow construction with s_i and r_far,i, free-space prior with η, labels.
Pass when: in "box in front of wall" and "pass a corner" scenarios, no fake surfaces appear between box and wall in the prediction plot; before their first measurement, disoccluded beams have μ̄ = s + η(r_far − s) (exactly s with η = 0); overlap beams take the near surface; the first-measurement normalised innovation on disoccluded beams is near one for a calibrated η.

**Stage 5. Gating and re-initialisation.**
Build: χ² gate and re-init.
Pass when: deliberately disabling the disocclusion rule causes gate failures exactly at the disoccluded beams (shows the gate is a safety net), and with the rule enabled gate failures are rare.

**Stage 6. Tuning Q by consistency.**
Build: Monte Carlo runner (e.g. 20 to 50 seeds per scenario), NEES/NIS per label.
Tune q_0, q_v, q_ω (and the edge term if needed) so that NEES per label is inside the χ² band, starting from the blend beams. Record the final values and the tuning procedure; it goes into the report.

**Stage 7. Visible set and barrier from the mean.**
Build: polygon V(μ) from a range vector, signed distance, both between-beam interpolation options, h and the prediction-based gradient (eq. 12) for sources (ii)–(iv).
Pass when: signed-distance unit tests pass; with dense noiseless readings, the gradients of (ii) and (iii) match the oracle gradient's sign and rough magnitude at hand-checked poses (corner shadow, behind a wall face, on the range arc).

**Stage 8. CBF-QP.**
Build in this order: (a) oracle source, check the controller keeps the target visible in all scenarios; (b) numerical gradient checked against a brute-force finite difference of the full closed loop for one step; (c) latest reading; (d) filter; (e) filter with η = 0.
Pass when: oracle run has zero true violations (or only ones explained by velocity limits, visible as slack); the four sources run end to end on all scenarios.

**Stage 9. Experiments.**
Monte Carlo over seeds for each scenario × gradient source × η × reading resolution and frequency (dense, sparse beams, intermittent readings) × σ_r. Produce the comparison table and figures. Expected story: with dense, low-noise readings, (ii) and (iii) are close; they separate with sparse beams and intermittent readings, where the prediction has to carry the boundary between measurements; (iii) vs (iv) shows what the free-space hypothesis buys near corners, and where it fails (obstacles behind the corner); the oracle is the reference.

**Stage 10. Ablations and extensions (only after stage 9).**
- Ablations: no slope term in F; diagonal Σ; naive interpolation across jumps; corner at the near or far beam instead of mid-beam; no disocclusion rule (beam gets the newly explored prior instead); no gating; correlated vs diagonal Q; linear vs min-of-neighbours polygon.
- η per beam (η_i from the crossing distance t_i) and leader-informed free space (section 4.6).
- Covariance-weighted gradient fit (section 7.2).
- Sparse LiDAR: measure every S-th beam. Known pitfalls: shifts that are multiples of S never fill the gaps (aliasing), and translation does not sweep along the heading. Remedies: interleave the ray offset per scan, or BLUE weights in the blend. The baseline keeps one state bin per ray (equal scan and belief grids, as `AGENTS.md` requires); the sparse extension enters as a measurement mask on that grid (H selects the measured beams), so it needs no refactor. Separating M state bins from the physical rays stays a deliberate contract change, only if the mask proves insufficient.
- Noise mismatch: filter σ_r different from the true σ_r.

### Scenario library (suggested)

The distance from the leader to the visible set should be defined by obstacle edges, not by the sector's side edges.

- **S1 open room**: walls only. Tests transport and newly explored beams.
- **S2 box in front of a wall**: one isolated obstacle with a strong jump. Tests edges, overlap, disocclusion when moving sideways.
- **S3 corridor corner**: the target turns a corner; the observer must reposition to keep it visible. Main control showcase.
- **S3b obstacles behind the corner**: as S3 with clutter behind the corner, where the free-space hypothesis can fail.
- **S4 pillars**: target walks behind a row of pillars. Repeated occlusions and disocclusions, CBF activity.
- **S5 cluttered**: several polygons, for Monte Carlo statistics.

---

## 10. Pitfalls to tell the code session about

- Frames: Δp and Δψ in the prediction are expressed in the *old* sensor frame. Mixing world and body frames here is the most likely bug.
- Angle wrapping in φ_j and in the bracket search, especially for β close to 360°.
- Interpolating across a jump creates walls that do not exist; any visible "bridge" between a near and a far surface in the polar plot means the jump test or the candidate selection is wrong.
- Zero F rows (disoccluded, newly explored) must have zero cross-covariance; do not copy neighbour covariances into them.
- Σ losing symmetry or positive definiteness after many steps; symmetrise and use Joseph form.
- Using the same step for physics and prediction with large velocities near obstacles breaks the small-step assumption; substep.
- Gate failures are a symptom, not a fix. If they are frequent in a beam category, the prediction for that category is wrong.
- The barrier is non-smooth; use gradient steps of about one beam spacing (smaller steps see the polygon's sampling steps), and do not be surprised by chattering at polygon vertices.
- With η = 0 the disoccluded boundary is fixed in the world, so the gradient behind an obstacle vanishes; that is expected for source (iv), not a bug.
- Score visibility violations against the true visible set, never against the estimated one.

---

## 11. Open decisions to settle with Theo during implementation

1. Sector size: aperture β and N (e.g. 90° to 120° with about 1° spacing, or a 360° LiDAR with the FoV as a sub-sector). The filter can run on the full LiDAR while the visible set uses only the FoV sector.
2. Whether the per-beam or leader-informed η variants enter the report. (Settled: baseline η = 0.5, with η = 0 as source (iv).)
3. Polygon interpolation between beams: linear vs conservative min-of-neighbours.
4. Whether an obstacle-collision barrier is needed in the experiments.
5. Whether the sparse-LiDAR extension is part of the final report or future work.
