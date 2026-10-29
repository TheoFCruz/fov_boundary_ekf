classdef TestControl < matlab.unittest.TestCase
    %TESTCONTROL Focused tests for the world-frame CBF-QP policy seam.

    methods (Test)
        function requiresQuadprog(testCase)
            verifyNotEmpty(testCase, which('quadprog'));
        end

        function nominalFeasibleReferenceIsUnchanged(testCase)
            policy = control.makeSignedDistanceCbfPolicy();
            [input, ~, diagnostics] = policy(validObservation(), [0, 0, 0], struct());

            verifyEqual(testCase, input, zeros(1, 3), 'AbsTol', 1e-10);
            verifyEqual(testCase, diagnostics.Status, "nominal-feasible");
            verifyEqual(testCase, diagnostics.BoundarySource, "belief-frozen");
            verifyGreaterThanOrEqual(testCase, diagnostics.ConstraintResidual, -1e-10);
        end

        function policyMapsBodyInputAndFollowerDrift(testCase)
            observation = validObservation();
            observation.ObserverPose = [0, 0, pi/2];
            observation.FollowerPose = [-0.5, 2, pi/2];
            observation.FollowerReference = [1, 0, 3];
            policy = control.makeSignedDistanceCbfPolicy();
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
            observation.FollowerPose = [4.93, 0.3, 0];
            policy = control.makeSignedDistanceCbfPolicy();
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
            observation.FollowerPose = [4.93, 0.3, 0];
            observation.FollowerReference = [100, 0, 0];
            policy = control.makeSignedDistanceCbfPolicy();
            [input, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyGreaterThan(testCase, diagnostics.Slack, 0);
            verifyEqual(testCase, diagnostics.Status, "relaxed-with-slack");
            verifyGreaterThanOrEqual(testCase, input(:), [-0.5; -0.5; -1] - 1e-9);
            verifyLessThanOrEqual(testCase, input(:), [0.5; 0.5; 1] + 1e-9);
        end

        function unsupportedBeliefUsesClippedNominalFallback(testCase)
            observation = validObservation();
            observation.Belief.IsSupported(:) = false;
            reference = [0.3, -0.2, 0.1];
            policy = control.makeSignedDistanceCbfPolicy();
            [input, ~, diagnostics] = policy(observation, reference, struct());

            % A nonzero nominal must survive the fallback (clipped, not zeroed).
            verifyEqual(testCase, input, reference, 'AbsTol', 1e-12);
            verifyEqual(testCase, diagnostics.Status, "invalid-belief-fallback");
        end

        function degenerateBeliefUsesClippedNominalFallback(testCase)
            observation = validObservation();
            observation.Belief.Mean(:) = 0;
            reference = [0.3, -0.2, 0.1];
            policy = control.makeSignedDistanceCbfPolicy();
            [input, ~, diagnostics] = policy(observation, reference, struct());

            verifyEqual(testCase, input, reference, 'AbsTol', 1e-12);
            verifyEqual(testCase, diagnostics.Status, "invalid-belief-fallback");
        end

        function fallbackClipsReferenceToInputBounds(testCase)
            observation = validObservation();
            observation.Belief.IsSupported(:) = false;
            reference = [10, -10, 0];
            policy = control.makeSignedDistanceCbfPolicy();
            [input, ~, diagnostics] = policy(observation, reference, struct());

            verifyEqual(testCase, input, [0.5, -0.5, 0], 'AbsTol', 1e-12);
            verifyEqual(testCase, diagnostics.Status, "invalid-belief-fallback");
        end

        function policyRespondsToBeliefGeometryDeterministically(testCase)
            observation = validObservation();
            observation.FollowerPose = [3, 0.5, 0];
            policy = control.makeSignedDistanceCbfPolicy();
            [firstInput, ~, firstDiagnostics] = policy(observation, [0, 0, 0], struct());
            [secondInput, ~, secondDiagnostics] = policy(observation, [0, 0, 0], struct());
            observation.Belief.Mean = 2 * ones(size(observation.Belief.Mean));
            [~, ~, changedDiagnostics] = policy(observation, [0, 0, 0], struct());

            verifyEqual(testCase, firstInput, secondInput, 'AbsTol', 1e-12);
            verifyEqual(testCase, firstDiagnostics.SignedDistance, ...
                secondDiagnostics.SignedDistance, 'AbsTol', 1e-12);
            verifyNotEqual(testCase, changedDiagnostics.SignedDistance, ...
                firstDiagnostics.SignedDistance);
        end

        function zeroAuthorityBoundsUseZeroAuthorityStatus(testCase)
            config = struct('InputLower', zeros(3, 1), 'InputUpper', zeros(3, 1));
            policy = control.makeSignedDistanceCbfPolicy(config);
            [input, ~, diagnostics] = policy(validObservation(), [0, 0, 0], struct());

            verifyEqual(testCase, input, zeros(1, 3));
            verifyEqual(testCase, diagnostics.Status, "zero-control-authority");
        end

        function rejectsInvalidPolicyConfiguration(testCase)
            verifyError(testCase, @() control.makeSignedDistanceCbfPolicy( ...
                struct('CbfRate', 0)), ...
                'control:makeSignedDistanceCbfPolicy:InvalidConfig');
            verifyError(testCase, @() control.makeSignedDistanceCbfPolicy( ...
                struct('InputWeights', diag([1, 1, -1]))), ...
                'control:makeSignedDistanceCbfPolicy:InvalidConfig');
            verifyError(testCase, @() control.makeSignedDistanceCbfPolicy( ...
                struct('GradientBeamMultiples', [0, 1])), ...
                'control:makeSignedDistanceCbfPolicy:InvalidConfig');
            verifyError(testCase, @() control.makeSignedDistanceCbfPolicy( ...
                struct('BoundarySource', 5)), ...
                'control:makeSignedDistanceCbfPolicy:InvalidConfig');
        end

        function oracleSourceGradientMatchesContinuousGeometrySign(testCase)
            % Observer facing a wall with a corner, per the simulation/control
            % audit's hand-checked case: target just inside the corner shadow.
            fovSpec = fov.FovSpec(10, deg2rad(100));
            wall = fov.polygonObstacle('wall', [4, -2; 4, 2; 4.3, 2; 4.3, -2]);
            numRays = 81;
            source = control.makeOracleBoundarySource(wall, fovSpec, numRays);
            observation = oracleObservation(fovSpec, numRays, [8, 3.9, 0]);

            policy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundarySource', source));
            [~, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyEqual(testCase, diagnostics.GradientQuality, "usable");
            verifyLessThan(testCase, diagnostics.ObserverPoseGradient(1), 0);
            verifyGreaterThan(testCase, diagnostics.ObserverPoseGradient(2), 0);
        end

        function oracleSourceGradientIsSmallBehindWallFace(testCase)
            fovSpec = fov.FovSpec(10, deg2rad(100));
            wall = fov.polygonObstacle('wall', [4, -2; 4, 2; 4.3, 2; 4.3, -2]);
            numRays = 81;
            source = control.makeOracleBoundarySource(wall, fovSpec, numRays);
            observation = oracleObservation(fovSpec, numRays, [5, 0.5, 0]);

            policy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundarySource', source));
            [~, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyEqual(testCase, diagnostics.GradientQuality, "usable");
            % True value is 0 (straight behind a flat face); report the actual
            % magnitude rather than assume a tight bound holds.
            testCase.log(sprintf('abs(dh/dx) behind wall face = %.6g', ...
                abs(diagnostics.ObserverPoseGradient(1))));
            verifyLessThanOrEqual(testCase, ...
                abs(diagnostics.ObserverPoseGradient(1)), 0.2);
        end

        function oracleSourceGradientIsSmallOnMaxRangeArc(testCase)
            fovSpec = fov.FovSpec(10, deg2rad(100));
            emptyObstacles = struct('Name', {}, 'Vertices', {});
            numRays = 81;
            source = control.makeOracleBoundarySource(emptyObstacles, fovSpec, numRays);
            target = 10 * [cos(0.1), sin(0.1)];
            observation = oracleObservation(fovSpec, numRays, [target, 0]);

            policy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundarySource', source));
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
    'FollowerPose', [2, 0.5, 0], ...
    'FollowerReference', [0, 0, 0]);
end

function observation = oracleObservation(fovSpec, numRays, followerPose)
%ORACLEOBSERVATION Build an observation with the sensor-grid Belief.Angles
% that an oracle BoundarySource still needs for the beam-spacing step size.
angles = fov.sampleRayAngles(fov.Observer('Position', [0, 0], 'Heading', 0, ...
    'Fov', fovSpec), numRays);
belief = struct('Angles', angles, 'Mean', nan(size(angles)), ...
    'OpeningAngle', fovSpec.OpeningAngle, 'IsSupported', true(size(angles)));
observation = struct( ...
    'Belief', belief, ...
    'ObserverPose', [0, 0, 0], ...
    'FollowerPose', followerPose, ...
    'FollowerReference', [0, 0, 0]);
end
