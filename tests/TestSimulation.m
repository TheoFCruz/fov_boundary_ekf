classdef TestSimulation < matlab.unittest.TestCase
    %TESTSIMULATION Focused tests for the checkpoint-one simulation seams.

    methods (Test)
        function stepPoseUsesBodyFrameEuler(testCase)
            verifyEqual(testCase, simulation.stepPose([1, 2, 0], [0, 0, 0], 0.5), ...
                [1, 2, 0]);
            verifyEqual(testCase, simulation.stepPose([1, 2, 0], [2, 0, 0], 0.5), ...
                [2, 2, 0]);
            verifyEqual(testCase, simulation.stepPose([1, 2, pi/2], [2, 0, 0], 0.5), ...
                [1, 3, pi/2], 'AbsTol', 1e-12);
            verifyEqual(testCase, simulation.stepPose([1, 2, 0], [0, 0, 2], 0.5), ...
                [1, 2, 1]);
        end

        function samplesReferencesAtBreakpoints(testCase)
            reference = struct('Times', [0; 1], ...
                'Values', [1, 0, 0; 0, 2, 0]);
            verifyEqual(testCase, simulation.sampleVelocityReference(reference, 0.5), ...
                [1, 0, 0]);
            verifyEqual(testCase, simulation.sampleVelocityReference(reference, 1), ...
                [0, 2, 0]);
            verifyEqual(testCase, simulation.sampleVelocityReference(reference, 3), ...
                [0, 2, 0]);
        end

        function rejectsMalformedScenarioReferences(testCase)
            scenario = shortScenario();
            scenario.Observer.Reference.Times = [0; 0.15];
            scenario.Observer.Reference.Values = [1, 0, 0; 1, 0, 0];
            verifyError(testCase, @() simulation.validateScenario(scenario), ...
                'simulation:validateScenario:UnalignedReference');
            scenario = shortScenario();
            scenario.Follower.Reference.Values = [0, 0];
            verifyError(testCase, @() simulation.validateScenario(scenario), ...
                'simulation:validateScenario:InvalidReference');
        end

        function runnerSynchronizesSamplesAndInputs(testCase)
            scenario = shortScenario();
            result = simulation.runScenario(scenario);

            verifyEqual(testCase, numel(result.Time), 4);
            verifySize(testCase, result.ObserverPose, [4, 3]);
            verifySize(testCase, result.FollowerPose, [4, 3]);
            verifySize(testCase, result.ObserverInput, [3, 3]);
            verifyEqual(testCase, numel(result.Scans), 4);
            verifyEqual(testCase, numel(result.Beliefs), 4);
            verifyEqual(testCase, result.Time(end), 0.3, 'AbsTol', 1e-12);
            verifyEqual(testCase, result.ObserverPose(end, 1), 0.3, 'AbsTol', 1e-12);
            verifyEqual(testCase, result.FollowerPose(end, 2), -0.7, 'AbsTol', 1e-12);
        end

        function scanAnglesReconstructRawBoundary(testCase)
            pose = [1, -2, pi - 0.01];
            spec = fov.FovSpec(5, 2*pi);
            sensor = struct('NumRays', 9);
            [scan, raw] = sensing.raycastScan(pose, spec, ...
                struct('Name', {}, 'Vertices', {}), sensor, 2);
            reconstructed = fov.boundaryFromRanges(pose, scan.Angles, ...
                scan.Ranges, scan.OpeningAngle);

            verifyEqual(testCase, reconstructed, raw.VisibleBoundary, 'AbsTol', 1e-12);
            verifyEqual(testCase, scan.Angles, raw.RayAngles - pose(3), ...
                'AbsTol', 1e-12);
            verifyFalse(testCase, any(scan.HasReturn));
            verifyTrue(testCase, all(scan.IsValid));
        end

        function validatesNoiseSensorConfiguration(testCase)
            scenario = shortScenario();
            scenario.Sensor.RangeNoiseStd = -0.1;
            verifyError(testCase, @() simulation.validateScenario(scenario), ...
                'simulation:validateScenario:InvalidSensor');
            scenario = shortScenario();
            scenario.Sensor.RangeNoiseStd = NaN;
            verifyError(testCase, @() simulation.validateScenario(scenario), ...
                'simulation:validateScenario:InvalidSensor');
            scenario = shortScenario();
            scenario.Sensor.Seed = 0.5;
            verifyError(testCase, @() simulation.validateScenario(scenario), ...
                'simulation:validateScenario:InvalidSensor');
            scenario = shortScenario();
            scenario.Sensor.RangeNoiseStd = 0.1;
            verifyError(testCase, @() sensing.raycastScan(scenario.Observer.InitialPose, ...
                scenario.Observer.Fov, scenario.Obstacles, scenario.Sensor, 0), ...
                'sensing:raycastScan:MissingRandomStream');
        end

        function seededRangeNoisePreservesOracleAndNoReturns(testCase)
            scenario = shortScenario();
            movingPair = scenarios.movingPair();
            scenario.Obstacles = movingPair.Obstacles;
            scenario.Sensor.RangeNoiseStd = 0.25;
            scenario.Sensor.Seed = 17;
            originalRng = rng;
            testCase.addTeardown(@() rng(originalRng));
            rng(41);
            expectedNextRandom = rand;
            rng(41);

            first = simulation.runScenario(scenario);
            second = simulation.runScenario(scenario);
            actualNextRandom = rand;

            verifyEqual(testCase, first.Scans, second.Scans);
            verifyEqual(testCase, actualNextRandom, expectedNextRandom);
            cleanScenario = scenario;
            cleanScenario.Sensor.RangeNoiseStd = 0;
            clean = simulation.runScenario(cleanScenario);
            verifyEqual(testCase, first.RawCasts, clean.RawCasts);
            for index = 1:numel(first.Scans)
                scan = first.Scans{index};
                raw = first.RawCasts{index};
                verifyEqual(testCase, scan.HasReturn, raw.IsOccluded);
                verifyEqual(testCase, scan.Ranges(~scan.HasReturn), ...
                    raw.Distances(~raw.IsOccluded));
                verifyGreaterThanOrEqual(testCase, scan.Ranges(scan.HasReturn), 0);
                verifyLessThanOrEqual(testCase, scan.Ranges(scan.HasReturn), scan.MaxRange);
            end
            firstScan = first.Scans{1};
            firstRaw = first.RawCasts{1};
            verifyTrue(testCase, any(firstScan.HasReturn));
            verifyTrue(testCase, any(~firstScan.HasReturn));
            verifyNotEqual(testCase, firstScan.Ranges(firstScan.HasReturn), ...
                firstRaw.Distances(firstRaw.IsOccluded));

            clippingSensor = movingPair.Sensor;
            clippingSensor.RangeNoiseStd = 1e6;
            clippingStream = RandStream('mt19937ar', 'Seed', 17);
            [clippedScan, clippedRaw] = sensing.raycastScan( ...
                movingPair.Observer.InitialPose, movingPair.Observer.Fov, ...
                movingPair.Obstacles, clippingSensor, 0, clippingStream);
            clippedReturns = clippedScan.Ranges(clippedScan.HasReturn);
            verifyEqual(testCase, clippedScan.HasReturn, clippedRaw.IsOccluded);
            verifyTrue(testCase, any(clippedReturns == 0));
            verifyTrue(testCase, any(clippedReturns == clippedScan.MaxRange));
        end

        function passThroughEstimatorCopiesScan(testCase)
            scenario = shortScenario();
            [scan, raw] = sensing.raycastScan(scenario.Observer.InitialPose, ...
                scenario.Observer.Fov, scenario.Obstacles, scenario.Sensor, 0);
            estimator = estimation.makePassThroughEstimator();
            state = estimator.Initialize(struct());
            predicted = estimator.Predict(state, struct(), 0.1);
            verifyEqual(testCase, predicted, state);
            [state, belief] = estimator.Correct(predicted, scan);

            verifyEqual(testCase, belief.Mean, scan.Ranges);
            verifyEqual(testCase, belief.Covariance, sparse(numel(scan.Ranges), numel(scan.Ranges)));
            verifyEqual(testCase, belief.HasReturn, scan.HasReturn);
            verifyEqual(testCase, belief.IsSupported, scan.IsValid);
            verifyEqual(testCase, state.CurrentBelief, belief);
            motion = struct('PreviousObserverPose', [0, 0, 0], ...
                'CurrentObserverPose', [1, 0, pi/4], 'AppliedInput', [1, 0, 1]);
            verifyEqual(testCase, estimator.Predict(state, motion, 0.1), state);
            verifyEqual(testCase, fov.boundaryFromRanges(belief.ObserverPose, ...
                belief.Angles, belief.Mean, belief.OpeningAngle), ...
                raw.VisibleBoundary, 'AbsTol', 1e-12);
        end

        function runnerUsesEstimatorBeliefForPolicyAndGeometry(testCase)
            scenario = shortScenario();
            scenario.Estimator = makeOffsetEstimator(0.1);
            scenario.Observer.Policy = @captureBeliefPolicy;
            result = simulation.runScenario(scenario);
            raw = result.RawCasts{1};
            belief = result.Beliefs{1};
            estimated = fov.boundaryFromRanges(belief.ObserverPose, belief.Angles, ...
                belief.Mean, belief.OpeningAngle);

            verifyNotEqual(testCase, estimated, raw.VisibleBoundary);
            verifyEqual(testCase, result.PolicyDiagnostics{1}.ObservedRange, ...
                belief.Mean(1));
            verifyGreaterThan(testCase, result.Metrics.PassThroughRangeResidual(1), 0);
        end

        function runnerProvidesCurrentFollowerReferenceToPolicy(testCase)
            scenario = shortScenario();
            scenario.Follower.Reference = struct('Times', 0, 'Values', [0.2, -0.1, 0.3]);
            scenario.Observer.Policy = @captureFollowerReferencePolicy;
            result = simulation.runScenario(scenario);

            verifyEqual(testCase, result.PolicyDiagnostics{1}.FollowerReference, ...
                [0.2, -0.1, 0.3]);
        end

        function customPolicyCanHoldObserverWhileFollowerMoves(testCase)
            scenario = shortScenario();
            scenario.Observer.Policy = @zeroPolicy;
            result = simulation.runScenario(scenario);

            verifyEqual(testCase, result.ObserverPose, ...
                repmat(result.ObserverPose(1, :), numel(result.Time), 1));
            verifyGreaterThan(testCase, result.FollowerPose(end, 2), ...
                result.FollowerPose(1, 2));
        end

        function rejectsChangedAngularGrid(testCase)
            scenario = shortScenario();
            scenario.Estimator = makeBeliefOverride('Angles', zeros(9, 1));
            verifyError(testCase, @() simulation.runScenario(scenario), ...
                'simulation:runScenario:InvalidBelief');
        end

        function unsupportedPolygonHasInvalidDiagnostics(testCase)
            scenario = shortScenario();
            scenario.Estimator = makeBeliefOverride('IsSupported', false(9, 1));
            result = simulation.runScenario(scenario);
            verifyFalse(testCase, any(result.Metrics.IsSampledPolygonValid));
            verifyTrue(testCase, all(isnan(result.Metrics.SampledPolygonSignedDistance)));
        end

        function rejectsInvalidCovariance(testCase)
            scenario = shortScenario();
            invalid = zeros(9);
            invalid(1, 2) = NaN;
            scenario.Estimator = makeBeliefOverride('Covariance', invalid);
            verifyError(testCase, @() simulation.runScenario(scenario), ...
                'simulation:runScenario:InvalidBelief');
            invalid = -eye(9);
            scenario.Estimator = makeBeliefOverride('Covariance', invalid);
            verifyError(testCase, @() simulation.runScenario(scenario), ...
                'simulation:runScenario:InvalidBelief');
        end

        function runnerIsHeadlessAndDeterministic(testCase)
            scenario = shortScenario();
            beforeFigures = findall(groot, 'Type', 'figure');
            first = simulation.runScenario(scenario);
            second = simulation.runScenario(scenario);
            scenario.Playback.Speed = 4;
            scenario.Playback.FrameRate = 8;
            changedPlayback = simulation.runScenario(scenario);

            verifyEqual(testCase, findall(groot, 'Type', 'figure'), beforeFigures);
            verifyEqual(testCase, first.Time, second.Time);
            verifyEqual(testCase, first.ObserverPose, second.ObserverPose);
            verifyEqual(testCase, first.FollowerPose, second.FollowerPose);
            verifyEqual(testCase, first.ObserverPose, changedPlayback.ObserverPose);
            verifyEqual(testCase, first.Metrics.SampledPolygonSignedDistance, ...
                changedPlayback.Metrics.SampledPolygonSignedDistance);
        end

        function animationUpdatesFixedArtists(testCase)
            scenario = shortScenario();
            scenario.Estimator = makeOffsetEstimator(0.1);
            result = simulation.runScenario(scenario);
            beforeFigures = findall(groot, 'Type', 'figure');
            testCase.addTeardown(@() deleteNewFigures(beforeFigures));
            handles = viz.animateSimulation(result, 'Visible', false, ...
                'FrameRate', 1e9, 'Speed', 1e9);
            artists = findall(handles.Figure);

            verifyTrue(testCase, isgraphics(handles.RawBoundary));
            verifyTrue(testCase, isgraphics(handles.MeasurementBoundary));
            verifyTrue(testCase, isgraphics(handles.EstimatedBoundary));
            verifyTrue(testCase, isgraphics(handles.OracleRange));
            verifyTrue(testCase, isgraphics(handles.ScanLine));
            for index = [1, numel(result.Time)]
                handles.UpdateFrame(index);
                belief = result.Beliefs{index};
                expected = fov.boundaryFromRanges(belief.ObserverPose, ...
                    belief.Angles, belief.Mean, belief.OpeningAngle);
                actualX = get(handles.EstimatedBoundary, 'XData');
                actualY = get(handles.EstimatedBoundary, 'YData');
                verifyEqual(testCase, [actualX(:), actualY(:)], expected, 'AbsTol', 1e-12);
                verifyEqual(testCase, findall(handles.Figure), artists);
            end
            verifyEqual(testCase, get(handles.Observer, 'XData'), result.ObserverPose(end, 1));

            scenario.Estimator = makeBeliefOverride('IsSupported', false(9, 1));
            unsupported = simulation.runScenario(scenario);
            hidden = viz.animateSimulation(unsupported, 'Visible', false, ...
                'FrameRate', 1e9, 'Speed', 1e9);
            hidden.UpdateFrame(1);
            verifyEqual(testCase, visibilityState(hidden.EstimatedBoundary), "off");
        end

        function animationCanDeferPlayback(testCase)
            result = simulation.runScenario(shortScenario());
            beforeFigures = findall(groot, 'Type', 'figure');
            testCase.addTeardown(@() deleteNewFigures(beforeFigures));
            handles = viz.animateSimulation(result, 'Visible', false, 'AutoPlay', false);

            verifyTrue(testCase, all(isnan(get(handles.Observer, 'XData'))));
            handles.UpdateFrame(1);
            verifyEqual(testCase, get(handles.Observer, 'XData'), result.ObserverPose(1, 1));
            verifyError(testCase, @() viz.animateSimulation(result, 'AutoPlay', 1), ...
                'viz:animateSimulation:InvalidOption');
        end

        function savesDefaultKeyFramesInScenarioTimestampDirectory(testCase)
            result = simulation.runScenario(shortScenario());
            result.Config.Name = "Frame demo / control?";
            resultBeforeExport = result;
            outputRoot = tempname;
            beforeFigures = findall(groot, 'Type', 'figure');
            testCase.addTeardown(@() removeTemporaryDirectory(outputRoot));

            manifest = viz.saveSimulationFrames(result, 'OutputRoot', outputRoot);

            verifyLessThanOrEqual(testCase, numel(manifest.FrameIndices), 5);
            verifyEqual(testCase, manifest.FrameIndices(1), 1);
            verifyEqual(testCase, manifest.FrameIndices(end), numel(result.Time));
            verifyEqual(testCase, fileparts(manifest.Directory), ...
                fullfile(outputRoot, 'frame-demo-control'));
            verifyTrue(testCase, all(cellfun(@isFile, manifest.FilePaths)));
            verifyEqual(testCase, result, resultBeforeExport);
            verifyEqual(testCase, findall(groot, 'Type', 'figure'), beforeFigures);
        end

        function savesExplicitKeyFramesOnly(testCase)
            result = simulation.runScenario(shortScenario());
            outputRoot = tempname;
            testCase.addTeardown(@() removeTemporaryDirectory(outputRoot));

            manifest = viz.saveSimulationFrames(result, 'OutputRoot', outputRoot, ...
                'FrameIndices', [numel(result.Time), 1, numel(result.Time)]);

            verifyEqual(testCase, manifest.FrameIndices, [1; numel(result.Time)]);
            verifyEqual(testCase, numel(manifest.FilePaths), 2);
            verifyError(testCase, @() viz.saveSimulationFrames(result, ...
                'OutputRoot', outputRoot, 'FrameIndices', 0), ...
                'viz:saveSimulationFrames:InvalidFrameIndices');
        end

        function animationDistinguishesOracleMeasurementAndBelief(testCase)
            scenario = shortScenario();
            movingPair = scenarios.movingPair();
            scenario.Obstacles = movingPair.Obstacles;
            scenario.Sensor.RangeNoiseStd = 0.25;
            scenario.Sensor.Seed = 17;
            result = simulation.runScenario(scenario);
            beforeFigures = findall(groot, 'Type', 'figure');
            testCase.addTeardown(@() deleteNewFigures(beforeFigures));
            scansBeforeReplay = result.Scans;
            handles = viz.animateSimulation(result, 'Visible', false, ...
                'FrameRate', 1e9, 'Speed', 1e9);
            artists = findall(handles.Figure);
            handles.UpdateFrame(1);
            raw = result.RawCasts{1};
            scan = result.Scans{1};
            belief = result.Beliefs{1};
            expectedMeasurement = fov.boundaryFromRanges(scan.ObserverPose, ...
                scan.Angles, scan.Ranges, scan.OpeningAngle);
            actualX = get(handles.MeasurementBoundary, 'XData');
            actualY = get(handles.MeasurementBoundary, 'YData');

            verifyEqual(testCase, get(handles.RawBoundary, 'DisplayName'), ...
                'Oracle sampled boundary');
            verifyEqual(testCase, get(handles.MeasurementBoundary, 'DisplayName'), ...
                'Measurement boundary');
            verifyEqual(testCase, get(handles.EstimatedBoundary, 'DisplayName'), ...
                'Estimated boundary');
            verifyEqual(testCase, get(handles.OracleRange, 'DisplayName'), 'Oracle range');
            verifyEqual(testCase, get(handles.ScanLine, 'DisplayName'), 'Measurement range');
            verifyEqual(testCase, get(handles.OracleRange, 'LineStyle'), '-');
            verifyEqual(testCase, get(handles.ScanLine, 'LineStyle'), '-.');
            verifyEqual(testCase, get(handles.MeanLine, 'LineStyle'), '--');
            verifyEqual(testCase, [actualX(:), actualY(:)], expectedMeasurement, ...
                'AbsTol', 1e-12);
            oracleRange = get(handles.OracleRange, 'YData');
            measurementRange = get(handles.ScanLine, 'YData');
            verifyEqual(testCase, oracleRange(:), raw.Distances, 'AbsTol', 1e-12);
            verifyEqual(testCase, measurementRange(:), scan.Ranges, 'AbsTol', 1e-12);
            meanRange = get(handles.MeanLine, 'YData');
            verifyEqual(testCase, meanRange(:), belief.Mean(:), ...
                'AbsTol', 1e-12);
            verifyNotEqual(testCase, scan.Ranges(scan.HasReturn), ...
                raw.Distances(raw.IsOccluded));
            verifyEqual(testCase, result.Scans, scansBeforeReplay);
            verifyEqual(testCase, findall(handles.Figure), artists);
        end

        function animationConfiguresRayAndImpactDisplays(testCase)
            scenario = shortScenario();
            movingPair = scenarios.movingPair();
            scenario.Obstacles = movingPair.Obstacles;
            result = simulation.runScenario(scenario);
            beforeFigures = findall(groot, 'Type', 'figure');
            testCase.addTeardown(@() deleteNewFigures(beforeFigures));

            defaultDisplay = viz.animateSimulation(result, 'Visible', false, ...
                'FrameRate', 1e9, 'Speed', 1e9);
            verifyEqual(testCase, visibilityState(defaultDisplay.Rays), "on");
            verifyEqual(testCase, visibilityState(defaultDisplay.HitPoints), "off");

            impactOnly = viz.animateSimulation(result, 'Visible', false, ...
                'FrameRate', 1e9, 'Speed', 1e9, ...
                'ShowRays', false, 'ShowHitPoints', true);
            verifyEqual(testCase, visibilityState(impactOnly.Rays), "off");
            verifyEqual(testCase, visibilityState(impactOnly.HitPoints), "on");
            hiddenRayX = get(impactOnly.Rays, 'XData');
            hiddenRayY = get(impactOnly.Rays, 'YData');
            artists = findall(impactOnly.Figure);
            for index = [1, numel(result.Time)]
                impactOnly.UpdateFrame(index);
                expected = result.RawCasts{index}.EndPoints( ...
                    result.RawCasts{index}.IsOccluded, :);
                hitX = get(impactOnly.HitPoints, 'XData');
                hitY = get(impactOnly.HitPoints, 'YData');
                verifyEqual(testCase, [hitX(:), hitY(:)], expected, 'AbsTol', 1e-12);
                verifyEqual(testCase, get(impactOnly.Rays, 'XData'), hiddenRayX);
                verifyEqual(testCase, get(impactOnly.Rays, 'YData'), hiddenRayY);
                verifyEqual(testCase, findall(impactOnly.Figure), artists);
            end

            both = viz.animateSimulation(result, 'Visible', false, ...
                'FrameRate', 1e9, 'Speed', 1e9, ...
                'ShowRays', true, 'ShowHitPoints', true);
            verifyEqual(testCase, visibilityState(both.Rays), "on");
            verifyEqual(testCase, visibilityState(both.HitPoints), "on");
            both.UpdateFrame(1);
            raw = result.RawCasts{1};
            expectedRayX = [repmat(raw.Origin(1), numel(raw.Distances), 1), ...
                raw.EndPoints(:, 1), nan(numel(raw.Distances), 1)].';
            expectedRayY = [repmat(raw.Origin(2), numel(raw.Distances), 1), ...
                raw.EndPoints(:, 2), nan(numel(raw.Distances), 1)].';
            actualRayX = get(both.Rays, 'XData');
            actualRayY = get(both.Rays, 'YData');
            verifyEqual(testCase, actualRayX(:), expectedRayX(:), ...
                'AbsTol', 1e-12);
            verifyEqual(testCase, actualRayY(:), expectedRayY(:), ...
                'AbsTol', 1e-12);
            neither = viz.animateSimulation(result, 'Visible', false, ...
                'FrameRate', 1e9, 'Speed', 1e9, ...
                'ShowRays', false, 'ShowHitPoints', false);
            verifyEqual(testCase, visibilityState(neither.Rays), "off");
            verifyEqual(testCase, visibilityState(neither.HitPoints), "off");
            hiddenRayX = get(neither.Rays, 'XData');
            hiddenRayY = get(neither.Rays, 'YData');
            hiddenHitX = get(neither.HitPoints, 'XData');
            hiddenHitY = get(neither.HitPoints, 'YData');
            neither.UpdateFrame(1);
            verifyEqual(testCase, get(neither.Rays, 'XData'), hiddenRayX);
            verifyEqual(testCase, get(neither.Rays, 'YData'), hiddenRayY);
            verifyEqual(testCase, get(neither.HitPoints, 'XData'), hiddenHitX);
            verifyEqual(testCase, get(neither.HitPoints, 'YData'), hiddenHitY);

            noImpactResult = simulation.runScenario(shortScenario());
            noImpact = viz.animateSimulation(noImpactResult, 'Visible', false, ...
                'FrameRate', 1e9, 'Speed', 1e9, 'ShowHitPoints', true);
            noImpact.UpdateFrame(1);
            verifyEmpty(testCase, get(noImpact.HitPoints, 'XData'));
            verifyEmpty(testCase, get(noImpact.HitPoints, 'YData'));
            verifyError(testCase, @() viz.animateSimulation(result, 'ShowRays', 1), ...
                'viz:animateSimulation:InvalidOption');
            verifyError(testCase, @() viz.animateSimulation(result, ...
                'ShowHitPoints', [true, false]), ...
                'viz:animateSimulation:InvalidOption');
        end
    end
end

function scenario = shortScenario()
scenario = scenarios.movingPair();
scenario.Time.Stop = 0.3;
scenario.Time.Step = 0.1;
scenario.Observer.Reference = struct('Times', 0, 'Values', [1, 0, 0]);
scenario.Follower.Reference = struct('Times', 0, 'Values', [0, 1, 0]);
scenario.Obstacles = struct('Name', {}, 'Vertices', {});
scenario.Sensor.NumRays = 9;
end

function estimator = makeOffsetEstimator(offset)
passThrough = estimation.makePassThroughEstimator();
estimator = passThrough;
estimator.Correct = @correct;

    function [state, belief] = correct(state, scan)
        [state, belief] = passThrough.Correct(state, scan);
        belief.Mean = belief.Mean + offset;
        state.CurrentBelief = belief;
    end
end

function [input, state, diagnostics] = captureBeliefPolicy(observation, reference, state)
input = reference;
diagnostics = struct('ObservedRange', observation.Belief.Mean(1));
end

function [input, state, diagnostics] = captureFollowerReferencePolicy(observation, reference, state)
input = reference;
diagnostics = struct('FollowerReference', observation.FollowerReference);
end

function estimator = makeBeliefOverride(field, value)
passThrough = estimation.makePassThroughEstimator();
estimator = passThrough;
estimator.Correct = @correct;
    function [state, belief] = correct(state, scan)
        [state, belief] = passThrough.Correct(state, scan);
        belief.(field) = value;
        state.CurrentBelief = belief;
    end
end

function [input, state, diagnostics] = zeroPolicy(~, ~, state)
input = [0, 0, 0];
diagnostics = struct('Method', "zero-test-policy");
end

function value = visibilityState(graphicsHandle)
value = string(get(graphicsHandle, 'Visible'));
end

function result = isFile(path)
result = exist(path, 'file') == 2;
end

function removeTemporaryDirectory(path)
if exist(path, 'dir')
    rmdir(path, 's');
end
end

function deleteNewFigures(beforeFigures)
delete(setdiff(findall(groot, 'Type', 'figure'), beforeFigures));
end
