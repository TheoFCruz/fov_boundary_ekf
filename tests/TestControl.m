classdef TestControl < matlab.unittest.TestCase
    %TESTCONTROL Focused tests for the belief-derived CBF-QP policy seam.

    methods (Test)
        function requiresQuadprog(testCase)
            verifyNotEmpty(testCase, which('quadprog'));
        end

        function nominalFeasibleReferenceIsUnchanged(testCase)
            policy = control.makeSignedDistanceCbfPolicy();
            [input, ~, diagnostics] = policy(validObservation(), [0, 0, 0], struct());

            verifyEqual(testCase, input, zeros(1, 3), 'AbsTol', 1e-10);
            verifyEqual(testCase, diagnostics.Status, "nominal-feasible");
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
            verifyEqual(testCase, diagnostics.FiniteDifferenceMode, "central");
        end

        function policyCorrectsNearBoundaryAndRespectsBounds(testCase)
            observation = validObservation();
            observation.FollowerPose = [4.93, 0.3, 0];
            policy = control.makeSignedDistanceCbfPolicy();
            [input, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyGreaterThan(testCase, input(1), 0);
            verifyGreaterThanOrEqual(testCase, input(:), [-0.5; -0.5; -1]);
            verifyLessThanOrEqual(testCase, input(:), [0.5; 0.5; 1]);
            verifyEqual(testCase, diagnostics.Status, "relaxed-with-slack");
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
            verifyGreaterThanOrEqual(testCase, input(:), [-0.5; -0.5; -1]);
            verifyLessThanOrEqual(testCase, input(:), [0.5; 0.5; 1]);
        end

        function unsupportedBeliefUsesZeroFallback(testCase)
            observation = validObservation();
            observation.Belief.IsSupported(:) = false;
            policy = control.makeSignedDistanceCbfPolicy();
            [input, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyEqual(testCase, input, zeros(1, 3));
            verifyEqual(testCase, diagnostics.Status, "invalid-belief-fallback");
        end

        function degenerateBeliefUsesZeroFallback(testCase)
            observation = validObservation();
            observation.Belief.Mean(:) = 0;
            policy = control.makeSignedDistanceCbfPolicy();
            [input, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyEqual(testCase, input, zeros(1, 3));
            verifyEqual(testCase, diagnostics.Status, "invalid-belief-fallback");
        end

        function nondifferentiableBeliefGeometryUsesGradientFallback(testCase)
            observation = validObservation();
            observation.FollowerPose = [2, 0, 0];
            policy = control.makeSignedDistanceCbfPolicy();
            [input, ~, diagnostics] = policy(observation, [0, 0, 0], struct());

            verifyEqual(testCase, input, zeros(1, 3));
            verifyEqual(testCase, diagnostics.Status, "unusable-gradient-fallback");
            verifyEqual(testCase, diagnostics.FiniteDifferenceQuality, "unusable");
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
