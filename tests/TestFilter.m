classdef TestFilter < matlab.unittest.TestCase
    %TESTFILTER Tests for estimation.makeBoundaryEkf (checkpoint E1).

    methods (Test)
        function firstScanInitializesMeanAndIndependentVariance(testCase)
            % No prior exists before the first scan: the posterior copies the
            % measurement exactly, with an independent sigma_r^2 per beam.
            scan = shortScan();
            params = estimation.defaultTransportParams();
            params.MeasurementNoiseStd = 0.03;
            estimator = estimation.makeBoundaryEkf(params);
            state = estimator.Initialize(struct());
            [~, belief] = estimator.Correct(state, scan);

            verifyEqual(testCase, belief.Mean, scan.Ranges);
            verifyEqual(testCase, belief.Covariance, 0.03 ^ 2 * eye(numel(scan.Ranges)), ...
                'AbsTol', 1e-15);
            verifyEqual(testCase, belief.Labels, repmat("measured", numel(scan.Ranges), 1));
            verifyTrue(testCase, all(isnan(belief.Innovation)));
            verifyTrue(testCase, isnan(belief.StepRatio));
        end

        function firstScanFloorsZeroMeasurementNoiseAtOneMillimeter(testCase)
            % Decision 1: sigma = max(RangeNoiseStd, 1e-3), applied only
            % inside the filter, so a noiseless scenario stays well-posed.
            scan = shortScan();
            params = estimation.defaultTransportParams();
            params.MeasurementNoiseStd = 0;
            estimator = estimation.makeBoundaryEkf(params);
            state = estimator.Initialize(struct());
            [~, belief] = estimator.Correct(state, scan);

            verifyEqual(testCase, belief.Covariance, 1e-3 ^ 2 * eye(numel(scan.Ranges)), ...
                'AbsTol', 1e-18);
        end

        function inconsistentBeamIsGatedAndConsistentBeamsAreFused(testCase)
            % A beam whose measurement disagrees with the prediction well
            % beyond the gate is re-initialised to the measurement, with
            % variance sigma_r^2 and zero cross-covariance, and excluded
            % from the joint update; a consistent beam is fused normally.
            belief = shortBelief();
            params = estimation.defaultTransportParams();
            params.MeasurementNoiseStd = 0.05;
            estimator = estimation.makeBoundaryEkf(params);
            state = estimator.Initialize(struct());
            state.Belief = belief;
            motion = struct('PreviousObserverPose', belief.ObserverPose, ...
                'CurrentObserverPose', belief.ObserverPose, 'AppliedInput', [0, 0, 0]);
            state = estimator.Predict(state, motion, 0.025);

            scan = struct();
            scan.Time = 0.025;
            scan.Angles = belief.Angles;
            scan.Ranges = belief.Mean;
            scan.Ranges(6) = 0.05; % far beyond the gate for beam 6, within [0, MaxRange]
            scan.HasReturn = true(11, 1);
            scan.IsValid = true(11, 1);
            scan.ObserverPose = belief.ObserverPose;
            scan.MaxRange = belief.MaxRange;
            scan.OpeningAngle = belief.OpeningAngle;

            [~, corrected] = estimator.Correct(state, scan);

            verifyTrue(testCase, corrected.Gated(6));
            verifyEqual(testCase, corrected.Mean(6), 0.05, 'AbsTol', 1e-12);
            verifyEqual(testCase, corrected.Covariance(6, 6), 0.05 ^ 2, 'AbsTol', 1e-15);
            verifyEqual(testCase, nnz(corrected.Covariance(6, [1:5, 7:11])), 0);
            verifyEqual(testCase, nnz(corrected.Covariance([1:5, 7:11], 6)), 0);
            others = true(11, 1);
            others(6) = false;
            verifyFalse(testCase, any(corrected.Gated(others)));
            % A consistent beam measured at its predicted mean stays there.
            verifyEqual(testCase, corrected.Mean(1), belief.Mean(1), 'AbsTol', 1e-9);
        end

        function josephUpdateKeepsCovarianceSymmetricAndPsdOverLongRun(testCase)
            % Closed-loop boxInFrontOfWall run with the EKF, long enough to
            % exercise edges, disocclusions and re-initialisation; the
            % covariance must stay symmetric and (numerically) PSD at every
            % logged step.
            scenario = scenarios.boxInFrontOfWall();
            scenario.Sensor.RangeNoiseStd = 0.01;
            scenario.Sensor.Seed = 3;
            params = estimation.defaultTransportParams();
            params.MeasurementNoiseStd = scenario.Sensor.RangeNoiseStd;
            scenario.Estimator = estimation.makeBoundaryEkf(params);
            oraclePredictor = @(belief, pose) fov.visibleBoundaryAt( ...
                pose, scenario.Observer.Fov, scenario.Obstacles, scenario.Sensor.NumRays);
            scenario.Observer.Policy = control.makeSignedDistanceCbfPolicy( ...
                struct('BoundaryPredictor', oraclePredictor));
            result = simulation.runScenario(scenario);

            minEigenvalue = inf;
            maxMagnitude = 0;
            for k = 1:numel(result.Beliefs)
                covariance = result.Beliefs{k}.Covariance;
                verifyLessThan(testCase, norm(covariance - covariance.', 'fro'), 1e-9);
                minEigenvalue = min(minEigenvalue, min(eig((covariance + covariance.') / 2)));
                maxMagnitude = max(maxMagnitude, max(abs(covariance(:))));
            end
            verifyGreaterThanOrEqual(testCase, minEigenvalue, -1e-12 * maxMagnitude);
        end

        function fullRunThroughRunScenarioIsValidatedAndDeterministic(testCase)
            % The EKF must satisfy simulation.runScenario's belief contract
            % for every sample and reproduce the same run for the same seed.
            scenario = openRoomWithEkf(0.01, 11);
            result = simulation.runScenario(scenario);
            resultAgain = simulation.runScenario(scenario);

            intervalCount = round( ...
                (scenario.Time.Stop - scenario.Time.Start) / scenario.Time.Step);
            verifyEqual(testCase, numel(result.Beliefs), intervalCount + 1);
            verifyEqual(testCase, numel(result.Scans), intervalCount + 1);
            verifyEqual(testCase, result.ObserverPose, resultAgain.ObserverPose);
            verifyEqual(testCase, result.Beliefs{end}.Mean, resultAgain.Beliefs{end}.Mean);
            for k = 1:numel(result.Beliefs)
                verifyEqual(testCase, result.Beliefs{k}.Method, "boundary-ekf");
            end
        end

        function stage1StaticObserverVarianceRecursionIsExact(testCase)
            % Plan stage 1: a static observer (F = I, Q = q0 at zero
            % motion). Blend beams must follow the scalar Kalman recursion
            % Pminus = P + q0, P = Pminus*sigma^2/(Pminus + sigma^2) exactly;
            % no-return beams (zero F row, decision P6) follow
            % Pminus = q0, P = q0*sigma^2/(q0 + sigma^2). H = I (every beam
            % measured each step), so a diagonal prior stays diagonal. A
            % beam the 0.99 gate rejects (about 1% of consistent ones) is
            % re-initialised to sigma^2 instead.
            rangeNoiseStd = 0.02;
            scenario = openRoomWithEkf(rangeNoiseStd, 7);
            scenario.Observer.Reference = struct('Times', 0, 'Values', [0, 0, 0]);
            scenario.Time.Stop = 0.125; % 5 steps at the scenario's dt
            result = simulation.runScenario(scenario);

            sigma2 = rangeNoiseStd ^ 2;
            q0 = estimation.defaultTransportParams().ProcessNoiseFloor;
            priorVariance = diag(result.Beliefs{1}.Covariance);
            for k = 2:numel(result.Beliefs)
                belief = result.Beliefs{k};
                verifyEqual(testCase, belief.Covariance, ...
                    diag(diag(belief.Covariance)), 'AbsTol', 1e-9);

                isBlend = belief.Labels == "blend" & ~belief.Gated;
                isNoReturn = belief.Labels == "no-return" & ~belief.Gated;
                predictedBlend = priorVariance + q0;
                expectedBlend = predictedBlend .* sigma2 ./ (predictedBlend + sigma2);
                expectedNoReturn = q0 * sigma2 / (q0 + sigma2);

                actual = diag(belief.Covariance);
                verifyEqual(testCase, actual(isBlend), expectedBlend(isBlend), ...
                    'RelTol', 1e-12);
                verifyEqual(testCase, actual(isNoReturn), ...
                    expectedNoReturn * ones(sum(isNoReturn), 1), 'RelTol', 1e-12);
                verifyEqual(testCase, actual(belief.Gated), ...
                    sigma2 * ones(sum(belief.Gated), 1), 'RelTol', 1e-12);

                priorVariance = actual;
            end
        end
    end
end

function scan = shortScan()
%SHORTSCAN An 11-beam noiseless scan from boxInFrontOfWall's initial pose.

scenario = scenarios.boxInFrontOfWall();
scenario.Sensor.NumRays = 11;
scan = sensing.raycastScan(scenario.Observer.InitialPose, scenario.Observer.Fov, ...
    scenario.Obstacles, scenario.Sensor, 0);
end

function belief = shortBelief()
%SHORTBELIEF An 11-beam posterior-shaped belief for the gating test.

scan = shortScan();
belief = struct();
belief.Time = 0;
belief.Angles = scan.Angles;
belief.Mean = scan.Ranges;
belief.Covariance = 0.01 * eye(numel(scan.Ranges));
belief.IsSupported = true(numel(scan.Ranges), 1);
belief.HasReturn = scan.HasReturn;
belief.ObserverPose = scan.ObserverPose;
belief.MaxRange = scan.MaxRange;
belief.OpeningAngle = scan.OpeningAngle;
belief.Method = "boundary-ekf";
belief.Labels = repmat("measured", numel(scan.Ranges), 1);
end

function scenario = openRoomWithEkf(rangeNoiseStd, seed)
%OPENROOMWITHEKF scenarios.openRoom() with the boundary EKF, noise and seed.

scenario = scenarios.openRoom();
scenario.Sensor.RangeNoiseStd = rangeNoiseStd;
scenario.Sensor.Seed = seed;
params = estimation.defaultTransportParams();
params.MeasurementNoiseStd = rangeNoiseStd;
scenario.Estimator = estimation.makeBoundaryEkf(params);
end
