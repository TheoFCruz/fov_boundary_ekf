classdef TestControl < matlab.unittest.TestCase
    %TESTCONTROL Focused tests for the world-frame CBF-QP policy seam.

    methods (Test)
        function requiresQuadprog(testCase)
            verifyNotEmpty(testCase, which('quadprog'));
        end

        function nominalFeasibleReferenceIsUnchanged(testCase)
            policy = emptyFieldPolicy();
            [input, ~, diagnostics] = policy(validObservation(), [0, 0, 0], struct());

            verifyEqual(testCase, input, zeros(1, 3), 'AbsTol', 1e-10);
            verifyEqual(testCase, diagnostics.Status, "nominal-feasible");
            verifyGreaterThanOrEqual(testCase, diagnostics.ConstraintResidual, -1e-10);
        end

        function policyMapsBodyInputAndLeaderDrift(testCase)
            observation = validObservation();
            observation.ObserverPose = [0, 0, pi/2];
            observation.LeaderPose = [-0.5, 2, pi/2];
            observation.LeaderReference = [1, 0, 3];
            policy = emptyFieldPolicy();
            [~, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            inputMap = [0, -1, 0; 1, 0, 0; 0, 0, 1];
            verifyEqual(testCase, diagnostics.BodyInputCoefficient, ...
                diagnostics.ObserverPoseGradient * inputMap, 'AbsTol', 1e-9);
            verifyEqual(testCase, diagnostics.TargetDrift, ...
                diagnostics.TargetPositionGradient * [0; 1], 'AbsTol', 1e-9);
            verifyEqual(testCase, diagnostics.GradientQuality, "usable");
            verifyGreaterThan(testCase, diagnostics.GradientSampleCount, 0);
        end

        function policyCorrectsNearBoundaryAndRespectsBounds(testCase)
            observation = validObservation();
            observation.LeaderPose = [4.93, 0.3, 0];
            policy = emptyFieldPolicy();
            [input, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyGreaterThan(testCase, input(1), 0);
            verifyGreaterThanOrEqual(testCase, input(:), [-0.5; -0.5; -1]);
            verifyLessThanOrEqual(testCase, input(:), [0.5; 0.5; 1]);
            % With beam-spacing-scaled least-squares gradients the hard
            % constraint is feasible within bounds here (unlike the stale
            % sub-beam finite-difference gradient), so the QP corrects the
            % nominal without needing slack.
            verifyEqual(testCase, diagnostics.Status, "cbf-corrected");
            verifyLessThanOrEqual(testCase, diagnostics.Slack, 1e-4);
        end

        function policyUsesSlackWhenBoundsCannotCounterTargetDrift(testCase)
            observation = validObservation();
            observation.LeaderPose = [4.93, 0.3, 0];
            observation.LeaderReference = [100, 0, 0];
            policy = emptyFieldPolicy();
            [input, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyGreaterThan(testCase, diagnostics.Slack, 0);
            verifyEqual(testCase, diagnostics.Status, "relaxed-with-slack");
            verifyGreaterThanOrEqual(testCase, input(:), [-0.5; -0.5; -1] - 1e-9);
            verifyLessThanOrEqual(testCase, input(:), [0.5; 0.5; 1] + 1e-9);
        end

        function invalidGeometryUsesClippedNominalFallback(testCase)
            degeneratePredictor = @(belief, pose) zeros(4, 2);
            policy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundaryPredictor', degeneratePredictor));
            reference = [0.3, -0.2, 0.1];
            [input, ~, diagnostics] = policy(validObservation(), reference, struct());

            % A nonzero nominal must survive the fallback (clipped, not zeroed).
            verifyEqual(testCase, input, reference, 'AbsTol', 1e-12);
            verifyEqual(testCase, diagnostics.Status, "invalid-geometry-fallback");
        end

        function fallbackClipsReferenceToInputBounds(testCase)
            degeneratePredictor = @(belief, pose) zeros(4, 2);
            policy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundaryPredictor', degeneratePredictor));
            [input, ~, diagnostics] = policy(validObservation(), [10, -10, 0], struct());

            verifyEqual(testCase, input, [0.5, -0.5, 0], 'AbsTol', 1e-12);
            verifyEqual(testCase, diagnostics.Status, "invalid-geometry-fallback");
        end

        function policyIsDeterministicAndFollowsPredictedGeometry(testCase)
            observation = validObservation();
            observation.LeaderPose = [3, 0.5, 0];
            nominal = zeros(1, 3);
            policy = emptyFieldPolicy();
            [firstInput, ~, firstDiagnostics] = policy(observation, nominal, struct());
            [secondInput, ~, secondDiagnostics] = policy(observation, nominal, struct());

            % The same observation through a predictor that sees a wall.
            wall = fov.polygonObstacle('wall', [2, -2; 2, 2; 2.3, 2; 2.3, -2]);
            wallPredictor = oraclePredictor(fov.FovSpec(5, pi/2), wall, 5);
            wallPolicy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundaryPredictor', wallPredictor));
            [~, ~, wallDiagnostics] = wallPolicy(observation, nominal, struct());

            verifyEqual(testCase, firstInput, secondInput, 'AbsTol', 1e-12);
            verifyEqual(testCase, firstDiagnostics.SignedDistance, ...
                secondDiagnostics.SignedDistance, 'AbsTol', 1e-12);
            verifyNotEqual(testCase, wallDiagnostics.SignedDistance, ...
                firstDiagnostics.SignedDistance);
        end

        function zeroAuthorityBoundsUseZeroAuthorityStatus(testCase)
            noAuthority = struct('InputLower', zeros(3, 1), 'InputUpper', zeros(3, 1));
            policy = emptyFieldPolicy(noAuthority);
            [input, ~, diagnostics] = policy(validObservation(), [0, 0, 0], struct());

            verifyEqual(testCase, input, zeros(1, 3));
            verifyEqual(testCase, diagnostics.Status, "zero-control-authority");
        end

        function rejectsInvalidPolicyConfiguration(testCase)
            oracle = oraclePredictor(fov.FovSpec(5, pi/2), [], 5);
            invalidConfigs = { ...
                struct('BoundaryPredictor', oracle, 'CbfRate', 0), ...
                struct('BoundaryPredictor', oracle, 'InputWeights', diag([1, 1, -1])), ...
                struct('BoundaryPredictor', oracle, 'GradientBeamMultiples', [0, 1]), ...
                struct('BoundaryPredictor', 5), ...
                struct('CbfRate', 4)};
            for index = 1:numel(invalidConfigs)
                verifyError(testCase, ...
                    @() control.makeSignedDistanceCbfPolicy(invalidConfigs{index}), ...
                    'control:makeSignedDistanceCbfPolicy:InvalidConfig');
            end
            verifyError(testCase, @() control.makeSignedDistanceCbfPolicy(), ...
                'control:makeSignedDistanceCbfPolicy:InvalidConfig');
        end

        function solverClassifiesConstraintOutcomes(testCase)
            [config, options] = solverSetup();
            moveForward = [1, 0, 0];
            still = zeros(1, 3);

            [input, qp] = control.solveCbfQp(moveForward, -1, still, config, options);
            verifyEqual(testCase, input, zeros(1, 3), 'AbsTol', 1e-9);
            verifyEqual(testCase, qp.Status, "nominal-feasible");

            % Minimal change under W = I. The soft slack leaves a tiny gap:
            % minimizing u^2 + lambda*(0.2 - u)^2 gives u = 0.2*lambda/(lambda + 1).
            [input, qp] = control.solveCbfQp(moveForward, 0.2, still, config, options);
            lambda = config.SlackPenalty;
            expected = [0.2 * lambda / (lambda + 1), 0, 0];
            verifyEqual(testCase, input, expected, 'AbsTol', 1e-9);
            verifyEqual(testCase, qp.Status, "cbf-corrected");

            % Beyond the 0.5 bound: saturate and carry the rest as slack.
            [input, qp] = control.solveCbfQp(moveForward, 1, still, config, options);
            verifyEqual(testCase, input, [0.5, 0, 0], 'AbsTol', 1e-6);
            verifyEqual(testCase, qp.Slack, 0.5, 'AbsTol', 1e-3);
            verifyEqual(testCase, qp.Status, "relaxed-with-slack");
        end

        function solverReturnsClippedNominalWithoutAuthority(testCase)
            [config, options] = solverSetup();
            noAuthority = zeros(1, 3);
            nominal = [10, -10, 0];
            [input, qp] = control.solveCbfQp(noAuthority, 1, nominal, config, options);

            verifyEqual(testCase, input, [0.5, -0.5, 0]);
            verifyEqual(testCase, qp.Status, "zero-control-authority");
        end

        function gradientRecoversLinearBarrierExactly(testCase)
            pose = [1; 2; 0.3];
            target = [4; 5];
            barrier = 0.7;
            poseGradient = [2, -3, 0.5];
            targetGradient = [-1, 4];
            barrierAtPose = @(q) barrier + poseGradient * (q - pose);
            barrierAtTarget = @(y) barrier + targetGradient * (y - target);

            [gPose, gTarget, sampleCount, quality] = control.barrierGradient( ...
                barrierAtPose, barrierAtTarget, pose, target, barrier, 0.02, [1, 2]);

            verifyEqual(testCase, gPose, poseGradient, 'AbsTol', 1e-9);
            verifyEqual(testCase, gTarget, targetGradient, 'AbsTol', 1e-9);
            verifyEqual(testCase, sampleCount, 12);
            verifyEqual(testCase, quality, "usable");
        end

        function gradientIsUnusableWhenSamplesAreInvalid(testCase)
            invalid = @(q) NaN;
            valid = @(y) 0;
            [gPose, ~, sampleCount, quality] = control.barrierGradient( ...
                invalid, valid, [0; 0; 0], [1; 0], 0, 0.02, [1, 2]);

            verifyTrue(testCase, all(isnan(gPose)));
            verifyEqual(testCase, sampleCount, 0);
            verifyEqual(testCase, quality, "unusable");
        end

        function transportPredictorMatchesZeroMotionBoundaryFromRanges(testCase)
            % Decision D6: at zero motion (the predictor queried at the
            % belief's own pose) estimation.predictBoundaryByTransport must
            % equal fov.boundaryFromRanges on the belief's own mean, with
            % the same jump flags the predictor builds from it, since
            % transportBoundary reproduces the belief exactly there.
            [belief, scenario] = boxInFrontOfWallBelief();
            params = estimation.defaultTransportParams();
            pose = scenario.Observer.InitialPose;

            boundary = estimation.predictBoundaryByTransport(belief, pose, params);

            meanA = belief.Mean(1:end - 1);
            meanB = belief.Mean(2:end);
            jumpBound = max(params.JumpThreshold, params.JumpRelative .* min(meanA, meanB));
            isJump = abs(meanA - meanB) > jumpBound ...
                | belief.HasReturn(1:end - 1) ~= belief.HasReturn(2:end);
            expected = fov.boundaryFromRanges( ...
                pose, belief.Angles, belief.Mean, belief.OpeningAngle, isJump);
            verifyEqual(testCase, boundary, expected, 'AbsTol', 1e-9);
        end

        function transportPredictorGradientMatchesOracleSignsNearCornerShadow(testCase)
            % Hand-checked case (a): a noiseless pass-through belief at
            % boxInFrontOfWall's initial pose, leader just inside the
            % box corner's shadow (silhouette corner at (0.8, 0.1), leader
            % at range 1.5 m, 6 deg). Source (ii) and the oracle must agree
            % in sign on both position-gradient components; the angle
            % between them is bounded after measuring it directly (17.55
            % deg).
            [belief, scenario] = boxInFrontOfWallBelief();
            params = estimation.defaultTransportParams();
            leaderPose = 1.5 * [cos(deg2rad(6)), sin(deg2rad(6)), 0];
            observation = boxObservation(belief, scenario, leaderPose);

            transportPredictor = @(b, p) estimation.predictBoundaryByTransport(b, p, params);
            oraclePredictorHandle = boxOraclePredictor(scenario);
            transportPolicy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundaryPredictor', transportPredictor));
            oraclePolicy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundaryPredictor', oraclePredictorHandle));
            [~, ~, transportDiagnostics] = transportPolicy(observation, [0, 0, 0], struct());
            [~, ~, oracleDiagnostics] = oraclePolicy(observation, [0, 0, 0], struct());

            transportPosition = transportDiagnostics.ObserverPoseGradient(1:2);
            oraclePosition = oracleDiagnostics.ObserverPoseGradient(1:2);
            verifyEqual(testCase, sign(transportPosition), sign(oraclePosition));
            angleDeg = acosd(min(max(dot(transportPosition, oraclePosition) ...
                / (norm(transportPosition) * norm(oraclePosition)), -1), 1));
            testCase.log(sprintf('angle to oracle gradient near corner shadow = %.3f deg', angleDeg));
            verifyLessThanOrEqual(testCase, angleDeg, 30);
        end

        function transportPredictorGradientIsSmallBehindBoxFace(testCase)
            % Hand-checked case (b): leader directly behind the box's front
            % face (the box occludes the wall entirely dead ahead), where
            % dh/dx should be small since the nearest occluder does not
            % change character as the observer moves forward a beam or two.
            [belief, scenario] = boxInFrontOfWallBelief();
            params = estimation.defaultTransportParams();
            leaderPose = [0.95, 0, 0];
            observation = boxObservation(belief, scenario, leaderPose);

            transportPredictor = @(b, p) estimation.predictBoundaryByTransport(b, p, params);
            policy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundaryPredictor', transportPredictor));
            [~, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyEqual(testCase, diagnostics.GradientQuality, "usable");
            % Measured magnitude 0.1557; report and bound loosely.
            testCase.log(sprintf('abs(dh/dx) behind box face (source ii) = %.6g', ...
                abs(diagnostics.ObserverPoseGradient(1))));
            verifyLessThanOrEqual(testCase, abs(diagnostics.ObserverPoseGradient(1)), 0.3);
        end

        function transportPredictorGradientIsSmallOnMaxRangeArcNoObstacles(testCase)
            % Hand-checked case (c): no obstacles, leader near the range
            % arc; dh/dpsi should be small (rotating in place barely moves
            % the open boundary's distance to a point near the cap).
            scenario = scenarios.boxInFrontOfWall();
            emptyObstacles = struct('Name', {}, 'Vertices', {});
            pose0 = scenario.Observer.InitialPose;
            scan = sensing.raycastScan(pose0, scenario.Observer.Fov, ...
                emptyObstacles, scenario.Sensor, 0);
            estimator = estimation.makePassThroughEstimator();
            state = estimator.Initialize(struct());
            [~, belief] = estimator.Correct(state, scan);
            params = estimation.defaultTransportParams();
            leaderPose = 1.9 * [cos(0.1), sin(0.1), 0];
            observation = struct('Belief', belief, 'ObserverPose', pose0, ...
                'LeaderPose', leaderPose, 'LeaderReference', [0, 0, 0]);

            transportPredictor = @(b, p) estimation.predictBoundaryByTransport(b, p, params);
            policy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundaryPredictor', transportPredictor));
            [~, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyEqual(testCase, diagnostics.GradientQuality, "usable");
            testCase.log(sprintf('abs(dh/dpsi) on max-range arc, no obstacles (source ii) = %.6g', ...
                abs(diagnostics.ObserverPoseGradient(3))));
            verifyLessThanOrEqual(testCase, abs(diagnostics.ObserverPoseGradient(3)), 0.05);
        end

        function sourceTwoRunsEndToEndDeterministically(testCase)
            % Closed-loop smoke test: source (ii) (the transport predictor,
            % default params, pass-through estimator, noiseless) must run
            % boxInFrontOfWall headless end to end, and twice from the same
            % scenario must agree exactly (no hidden randomness or state).
            params = estimation.defaultTransportParams();
            transportPredictor = @(b, p) estimation.predictBoundaryByTransport(b, p, params);
            buildScenario = @() setPolicyField(scenarios.boxInFrontOfWall(), ...
                control.makeSignedDistanceCbfPolicy( ...
                    struct('BoundaryPredictor', transportPredictor)));

            firstResult = simulation.runScenario(buildScenario());
            secondResult = simulation.runScenario(buildScenario());

            verifyEqual(testCase, firstResult.ObserverPose, secondResult.ObserverPose);
            verifyEqual(testCase, firstResult.LeaderPose, secondResult.LeaderPose);
        end

        function oraclePredictorGradientMatchesContinuousGeometrySign(testCase)
            % Observer facing a wall with a corner, per the simulation/control
            % audit's hand-checked case: target just inside the corner shadow.
            fovSpec = fov.FovSpec(10, deg2rad(100));
            wall = fov.polygonObstacle('wall', [4, -2; 4, 2; 4.3, 2; 4.3, -2]);
            numRays = 81;
            predictor = oraclePredictor(fovSpec, wall, numRays);
            observation = oracleObservation(fovSpec, numRays, [8, 3.9, 0]);

            policy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundaryPredictor', predictor));
            [~, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyEqual(testCase, diagnostics.GradientQuality, "usable");
            verifyLessThan(testCase, diagnostics.ObserverPoseGradient(1), 0);
            verifyGreaterThan(testCase, diagnostics.ObserverPoseGradient(2), 0);
        end

        function oraclePredictorGradientIsSmallBehindWallFace(testCase)
            fovSpec = fov.FovSpec(10, deg2rad(100));
            wall = fov.polygonObstacle('wall', [4, -2; 4, 2; 4.3, 2; 4.3, -2]);
            numRays = 81;
            predictor = oraclePredictor(fovSpec, wall, numRays);
            observation = oracleObservation(fovSpec, numRays, [5, 0.5, 0]);

            policy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundaryPredictor', predictor));
            [~, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyEqual(testCase, diagnostics.GradientQuality, "usable");
            % True value is 0 (straight behind a flat face); report the actual
            % magnitude rather than assume a tight bound holds.
            testCase.log(sprintf('abs(dh/dx) behind wall face = %.6g', ...
                abs(diagnostics.ObserverPoseGradient(1))));
            verifyLessThanOrEqual(testCase, ...
                abs(diagnostics.ObserverPoseGradient(1)), 0.2);
        end

        function oraclePredictorGradientIsSmallOnMaxRangeArc(testCase)
            fovSpec = fov.FovSpec(10, deg2rad(100));
            emptyObstacles = struct('Name', {}, 'Vertices', {});
            numRays = 81;
            predictor = oraclePredictor(fovSpec, emptyObstacles, numRays);
            target = 10 * [cos(0.1), sin(0.1)];
            observation = oracleObservation(fovSpec, numRays, [target, 0]);

            policy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundaryPredictor', predictor));
            [~, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyEqual(testCase, diagnostics.GradientQuality, "usable");
            testCase.log(sprintf('abs(dh/dpsi) on max-range arc = %.6g', ...
                abs(diagnostics.ObserverPoseGradient(3))));
            verifyLessThanOrEqual(testCase, ...
                abs(diagnostics.ObserverPoseGradient(3)), 0.05);
        end
    end
end

function observation = validObservation()
angles = linspace(-pi/4, pi/4, 5).';
belief = struct( ...
    'Angles', angles, ...
    'Mean', 5 * ones(size(angles)), ...
    'OpeningAngle', pi/2, ...
    'IsSupported', true(size(angles)));
observation = struct( ...
    'Belief', belief, ...
    'ObserverPose', [0, 0, 0], ...
    'LeaderPose', [2, 0.5, 0], ...
    'LeaderReference', [0, 0, 0]);
end

function observation = oracleObservation(fovSpec, numRays, leaderPose)
%ORACLEOBSERVATION Build an observation with the sensor-grid Belief.Angles
% that the policy still needs for the beam-spacing step size.
angles = fov.sampleRayAngles(fov.Observer('Position', [0, 0], 'Heading', 0, ...
    'Fov', fovSpec), numRays);
belief = struct('Angles', angles, 'Mean', nan(size(angles)), ...
    'OpeningAngle', fovSpec.OpeningAngle, 'IsSupported', true(size(angles)));
observation = struct( ...
    'Belief', belief, ...
    'ObserverPose', [0, 0, 0], ...
    'LeaderPose', leaderPose, ...
    'LeaderReference', [0, 0, 0]);
end

function predictor = oraclePredictor(fovSpec, obstacles, numRays)
%ORACLEPREDICTOR Oracle boundary predictor, as the scenario factories build it.
predictor = @(belief, pose) fov.visibleBoundaryAt(pose, fovSpec, obstacles, numRays);
end

function [belief, scenario] = boxInFrontOfWallBelief()
%BOXINFRONTOFWALLBELIEF Noiseless pass-through belief at S2's initial pose.

scenario = scenarios.boxInFrontOfWall();
pose = scenario.Observer.InitialPose;
scan = sensing.raycastScan(pose, scenario.Observer.Fov, ...
    scenario.Obstacles, scenario.Sensor, 0);
estimator = estimation.makePassThroughEstimator();
state = estimator.Initialize(struct());
[~, belief] = estimator.Correct(state, scan);
end

function observation = boxObservation(belief, scenario, leaderPose)
%BOXOBSERVATION Observation at S2's initial pose with a chosen leader pose.

observation = struct('Belief', belief, ...
    'ObserverPose', scenario.Observer.InitialPose, ...
    'LeaderPose', leaderPose, 'LeaderReference', [0, 0, 0]);
end

function predictor = boxOraclePredictor(scenario)
%BOXORACLEPREDICTOR Oracle predictor over S2's own obstacles.

predictor = @(belief, pose) fov.visibleBoundaryAt( ...
    pose, scenario.Observer.Fov, scenario.Obstacles, scenario.Sensor.NumRays);
end

function scenario = setPolicyField(scenario, policy)
%SETPOLICYFIELD Swap a scenario's observer policy (for a fresh closure each run).

scenario.Observer.Policy = policy;
end

function policy = emptyFieldPolicy(overrides)
%EMPTYFIELDPOLICY Policy whose oracle sees the bare 5 m, 90 deg sector.
% It matches validObservation's 5-ray grid; overrides adds config fields.
config = struct('BoundaryPredictor', oraclePredictor(fov.FovSpec(5, pi/2), [], 5));
if nargin > 0
    for name = fieldnames(overrides).'
        config.(name{1}) = overrides.(name{1});
    end
end
policy = control.makeSignedDistanceCbfPolicy(config);
end

function [config, options] = solverSetup()
%SOLVERSETUP Policy-default bounds with W = I for hand-checkable QP solutions.
config = struct( ...
    'InputLower', [-0.5; -0.5; -1], ...
    'InputUpper', [0.5; 0.5; 1], ...
    'InputWeights', eye(3), ...
    'SlackPenalty', 1e4);
options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
end
