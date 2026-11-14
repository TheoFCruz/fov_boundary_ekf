classdef TestTransport < matlab.unittest.TestCase
    %TESTTRANSPORT Tests for estimation.transportBoundary (checkpoint 1).

    methods (Test)
        function openRoomCastsFrontNoReturnThenSideWalls(testCase)
            % Sanity check on the plan-S1 geometry the transport tests rely
            % on: a front-wall band, two no-return wedges where the room
            % corners fall beyond the 2 m range, then side-wall returns.
            scenario = scenarios.openRoom();
            pose = scenario.Observer.InitialPose;
            scan = sensing.raycastScan(pose, scenario.Observer.Fov, ...
                scenario.Obstacles, scenario.Sensor, 0);

            frontReturn = scan.HasReturn & abs(scan.Angles) < deg2rad(30);
            noReturnWedge = ~scan.HasReturn & abs(scan.Angles) > deg2rad(33) ...
                & abs(scan.Angles) < deg2rad(38);
            sideReturn = scan.HasReturn & abs(scan.Angles) > deg2rad(43);

            verifyGreaterThan(testCase, sum(frontReturn), 10);
            verifyGreaterThan(testCase, sum(noReturnWedge), 1);
            verifyGreaterThan(testCase, sum(sideReturn), 1);
            % Two separate no-return wedges, one on each side of boresight.
            leftWedge = noReturnWedge & scan.Angles < 0;
            rightWedge = noReturnWedge & scan.Angles > 0;
            verifyTrue(testCase, any(leftWedge) && any(rightWedge));
        end

        function rotationByExactBeamMultipleMatchesTruthTightly(testCase)
            % Rotating by an integer number of beam spacings puts every
            % transported bearing exactly back on the fixed grid, so blend
            % beams should reproduce the truth to machine precision.
            scenario = scenarios.openRoom();
            beamSpacing = deg2rad(100) / (scenario.Sensor.NumRays - 1);
            poseA = scenario.Observer.InitialPose;
            poseB = poseA + [0, 0, 3 * beamSpacing];
            [belief, truth] = transportCase(scenario, poseA, poseB);

            prediction = estimation.transportBoundary( ...
                belief, poseB, estimation.defaultTransportParams());
            isBlend = prediction.Label == "blend";
            verifyGreaterThan(testCase, sum(isBlend), 50);
            err = abs(prediction.Mean(isBlend) - truth.Ranges(isBlend));
            % Measured max error 4.4e-16; bound with generous headroom.
            verifyLessThan(testCase, max(err), 1e-9);
            verifyEqual(testCase, prediction.HasReturn(isBlend), ...
                truth.HasReturn(isBlend));
        end

        function rotationByFractionalBeamHasSmallBlendError(testCase)
            % A sub-beam rotation leaves parallax between the transported
            % bearings and the fixed grid, so blend beams only match truth
            % up to the within-surface linear-interpolation error, of order
            % r''*dtheta^2/8 (sub-mm on these flat walls at 1.25 deg / 2 m).
            scenario = scenarios.openRoom();
            beamSpacing = deg2rad(100) / (scenario.Sensor.NumRays - 1);
            poseA = scenario.Observer.InitialPose;
            poseB = poseA + [0, 0, 0.4 * beamSpacing];
            [belief, truth] = transportCase(scenario, poseA, poseB);

            prediction = estimation.transportBoundary( ...
                belief, poseB, estimation.defaultTransportParams());
            isBlend = prediction.Label == "blend";
            % Exclude beams next to a no-return wedge: P1 has no jump test,
            % so a blend pair spanning return/no-return is not excluded by
            % construction there, and the wedge edge is not a flat surface.
            isNearWedge = nearNoReturnWedge(belief.HasReturn);
            scored = isBlend & ~isNearWedge;
            verifyGreaterThan(testCase, sum(scored), 50);
            err = abs(prediction.Mean(scored) - truth.Ranges(scored));
            % Measured max error 3.6e-4 m; bound with headroom.
            verifyLessThan(testCase, max(err), 1e-3);
        end

        function newBeamsExtendSideWallCloseToTruth(testCase)
            % A larger rotation uncovers beams through the sector side with
            % no old bracketing pair; they should extend the flat side wall
            % closely, since the fixed grid's existing blend points already
            % lie on that straight edge.
            scenario = scenarios.openRoom();
            beamSpacing = deg2rad(100) / (scenario.Sensor.NumRays - 1);
            poseA = scenario.Observer.InitialPose;
            poseB = poseA + [0, 0, 5 * beamSpacing];
            [belief, truth] = transportCase(scenario, poseA, poseB);

            prediction = estimation.transportBoundary( ...
                belief, poseB, estimation.defaultTransportParams());
            isNew = prediction.Label == "new";
            % Most newly-uncovered beams at this rotation bracket a
            % no-return old pair (label "no-return"); a handful at the
            % trailing sector edge have no bracketing pair at all.
            verifyGreaterThan(testCase, sum(isNew), 3);
            verifyEqual(testCase, prediction.HasReturn(isNew), ...
                truth.HasReturn(isNew));
            scored = isNew & truth.HasReturn;
            err = abs(prediction.Mean(scored) - truth.Ranges(scored));
            % Measured max error 2.2e-15 m (exact wall-line extension);
            % bound loosely in case the room geometry changes.
            verifyLessThan(testCase, max(err), 1e-2);
        end

        function newBeamsNextToOpenSpaceStayOpen(testCase)
            % Regression: a wall at x = 1 covers only |theta| <= 20 deg and
            % the rest of the 2 m sector is open. Rotating left uncovers two
            % beams whose covered neighbour is open, so they must stay open
            % (MaxRange, no return) instead of extending the distant wall
            % line across open space (which predicted a fake wall at 1.6 m).
            angles = linspace(-deg2rad(50), deg2rad(50), 81).';
            onWall = abs(angles) <= deg2rad(20);
            belief = struct();
            belief.Angles = angles;
            belief.Mean = 2 * ones(81, 1);
            belief.Mean(onWall) = 1 ./ cos(angles(onWall));
            belief.Covariance = sparse(81, 81);
            belief.HasReturn = onWall;
            belief.ObserverPose = [0, 0, 0];
            belief.MaxRange = 2;
            belief.OpeningAngle = deg2rad(100);
            beamSpacing = angles(2) - angles(1);

            prediction = estimation.transportBoundary( ...
                belief, [0, 0, 2 * beamSpacing], estimation.defaultTransportParams());
            isNew = prediction.Label == "new";

            verifyEqual(testCase, sum(isNew), 2);
            verifyEqual(testCase, prediction.Mean(isNew), [2; 2]);
            verifyFalse(testCase, any(prediction.HasReturn(isNew)));
        end

        function transportMatchesHandComputedEquation1(testCase)
            % Hand-checked eq. 1 (plus the eq. 2 blend) on a synthetic
            % 3-beam belief with a pure rotation of 0.03 rad: each
            % transported bearing is phi_j = theta_j - dpsi and each
            % transported range is rho_j = mean_j (rotation preserves
            % norm). With angles +-0.1 and 0, and means [1.8, 2.0, 2.2],
            % beam 1 (theta=-0.1) is bracketed by (1,2) with weight 0.7,
            % giving 0.7*1.8+0.3*2.0 = 1.86; beam 2 (theta=0) is bracketed
            % by (2,3) with weight 0.7, giving 0.7*2.0+0.3*2.2 = 2.06.
            belief = struct();
            belief.Angles = [-0.1; 0; 0.1];
            belief.Mean = [1.8; 2.0; 2.2];
            belief.Covariance = sparse(3, 3);
            belief.HasReturn = true(3, 1);
            belief.ObserverPose = [0, 0, 0];
            belief.MaxRange = 5;
            belief.OpeningAngle = pi / 2;
            newPose = [0, 0, 0.03];

            prediction = estimation.transportBoundary( ...
                belief, newPose, estimation.defaultTransportParams());

            verifyEqual(testCase, prediction.Label(1), "blend");
            verifyEqual(testCase, prediction.Mean(1), 1.86, 'AbsTol', 1e-9);
            verifyEqual(testCase, prediction.Label(2), "blend");
            verifyEqual(testCase, prediction.Mean(2), 2.06, 'AbsTol', 1e-9);
        end

        function translationErrorGrowsWithStepSize(testCase)
            % Pure translation (no rotation) at a flat wall, forward and
            % sideways: the within-surface blend error stays small and the
            % larger step's error exceeds the smaller step's. Beam arc
            % length at the ~2 m front wall is about 0.044 m.
            scenario = scenarios.openRoom();
            poseA = scenario.Observer.InitialPose;
            scanA = sensing.raycastScan(poseA, scenario.Observer.Fov, ...
                scenario.Obstacles, scenario.Sensor, 0);
            estimator = estimation.makePassThroughEstimator();
            state = estimator.Initialize(struct());
            [~, belief] = estimator.Correct(state, scanA);
            isNearWedge = nearNoReturnWedge(belief.HasReturn);

            for direction = [1, 0; 0, 1].'
                smallError = blendError(belief, poseA, direction.' * 0.2 * 0.04, ...
                    scenario, isNearWedge);
                largeError = blendError(belief, poseA, direction.' * 4 * 0.04, ...
                    scenario, isNearWedge);
                % Measured: forward 1.8e-4 (0.2 beam) to 3.8e-4 (4 beam);
                % sideways 2.0e-4 to 2.8e-4. Bound both with headroom.
                verifyLessThan(testCase, smallError, 1e-3);
                verifyLessThan(testCase, largeError, 1e-3);
                verifyGreaterThan(testCase, largeError, smallError);
            end
        end

        function mixedRotationAndTranslationHasSmallBlendError(testCase)
            % A step combining rotation and translation is the general case
            % the EKF's Predict will use every step.
            scenario = scenarios.openRoom();
            poseA = scenario.Observer.InitialPose;
            poseB = poseA + [0.1, 0.05, 0.03];
            [belief, truth] = transportCase(scenario, poseA, poseB);
            isNearWedge = nearNoReturnWedge(belief.HasReturn);

            prediction = estimation.transportBoundary( ...
                belief, poseB, estimation.defaultTransportParams());
            scored = prediction.Label == "blend" & ~isNearWedge;
            err = abs(prediction.Mean(scored) - truth.Ranges(scored));
            % Measured max error 3.9e-4 m.
            verifyLessThan(testCase, max(err), 1e-3);
        end

        function noReturnWedgeStaysAtCapWithoutBlending(testCase)
            % Deep inside a no-return wedge (away from its borders), beams
            % must stay at MaxRange with HasReturn false; nothing interpolates
            % a "blend" value between the wall and the cap.
            scenario = scenarios.openRoom();
            poseA = scenario.Observer.InitialPose;
            poseB = poseA + [0.04, 0, 0];
            [belief, ~] = transportCase(scenario, poseA, poseB);

            prediction = estimation.transportBoundary( ...
                belief, poseB, estimation.defaultTransportParams());
            deepWedge = abs(prediction.Angles) > deg2rad(33) ...
                & abs(prediction.Angles) < deg2rad(39);
            verifyGreaterThan(testCase, sum(deepWedge), 3);
            verifyTrue(testCase, all(prediction.Label(deepWedge) == "no-return"));
            verifyTrue(testCase, all(prediction.Mean(deepWedge) == belief.MaxRange));
            verifyFalse(testCase, any(prediction.HasReturn(deepWedge)));
        end

        function jumpGapAppearsOnlyNearReturnBoundary(testCase)
            % No pair inside one wall should fail the eq. 7 jump test: a
            % "jump-gap" label should appear only where an old pair spans a
            % return/no-return transition, not deep inside the front wall
            % or deep inside a side wall.
            scenario = scenarios.openRoom();
            poseA = scenario.Observer.InitialPose;
            poseB = poseA + [0.04, 0, 0];
            [belief, ~] = transportCase(scenario, poseA, poseB);

            prediction = estimation.transportBoundary( ...
                belief, poseB, estimation.defaultTransportParams());
            isJumpGap = prediction.Label == "jump-gap";
            % Measured jump-gap angles: +-32.5 and +-41.25 deg, at the
            % wedge borders. The deep front-wall and deep side-wall
            % regions must have none.
            deepFrontWall = abs(prediction.Angles) < deg2rad(25);
            deepSideWall = abs(prediction.Angles) > deg2rad(45);
            verifyFalse(testCase, any(isJumpGap & deepFrontWall));
            verifyFalse(testCase, any(isJumpGap & deepSideWall));
        end

        function nonzeroHeadingTranslatesInOldSensorFrame(testCase)
            % Start with a nonzero heading and translate along the
            % observer's own body x axis. Delta p and Delta psi are in the
            % old sensor frame, so a world/body mix-up would show up here.
            scenario = scenarios.openRoom();
            poseA = [0.1, 0.2, 0.3];
            heading = poseA(3);
            poseB = poseA + [0.1 * cos(heading), 0.1 * sin(heading), 0];
            [belief, truth] = transportCase(scenario, poseA, poseB);

            prediction = estimation.transportBoundary( ...
                belief, poseB, estimation.defaultTransportParams());
            isBlend = prediction.Label == "blend";
            err = abs(prediction.Mean(isBlend) - truth.Ranges(isBlend));
            % Measured max error 3.5e-3 m, at the single beam closest to
            % the front/side-wall corner: the true range has a cusp there
            % (continuous range, discontinuous slope) that the eq. 7
            % range-difference jump test does not catch; the corner
            % estimate (eqs. 8-10) that would handle it is a later
            % checkpoint. All other blend beams are far more accurate.
            verifyLessThan(testCase, max(err), 5e-3);
        end

        function fullFovBeliefErrorsWithIdentifier(testCase)
            belief = struct();
            belief.Angles = [-0.1; 0; 0.1];
            belief.Mean = [1; 1; 1];
            belief.Covariance = sparse(3, 3);
            belief.HasReturn = true(3, 1);
            belief.ObserverPose = [0, 0, 0];
            belief.MaxRange = 5;
            belief.OpeningAngle = 2 * pi;

            verifyError(testCase, @() estimation.transportBoundary( ...
                belief, [0.1, 0, 0], estimation.defaultTransportParams()), ...
                'estimation:transportBoundary:FullFovUnsupported');
        end

        function plotTransportCheckRendersInvisibly(testCase)
            scenario = scenarios.openRoom();
            poseA = scenario.Observer.InitialPose;
            poseB = poseA + [0.04, 0, 0];
            [belief, truth] = transportCase(scenario, poseA, poseB);
            prediction = estimation.transportBoundary( ...
                belief, poseB, estimation.defaultTransportParams());

            handles = viz.plotTransportCheck(prediction, truth, 'Visible', false);
            testCase.addTeardown(@() close(handles.Figure));

            verifyEqual(testCase, string(get(handles.Figure, 'Visible')), "off");
            verifyError(testCase, @() viz.plotTransportCheck(prediction, truth, ...
                'Visible', 1), 'viz:plotTransportCheck:InvalidOption');
        end
    end
end

function [belief, truthAtB] = transportCase(scenario, poseA, poseB)
%TRANSPORTCASE Build a noiseless belief at poseA and the truth scan at poseB.
%
% Shared by the transport tests (P1-P2): the belief comes from the
% checkpoint-one pass-through estimator (Initialize then Correct on a
% noiseless scan at poseA); the truth is an independent noiseless scan cast
% directly at poseB, never touched by the estimator.

scanA = sensing.raycastScan(poseA, scenario.Observer.Fov, ...
    scenario.Obstacles, scenario.Sensor, 0);
estimator = estimation.makePassThroughEstimator();
state = estimator.Initialize(struct('NumRays', scenario.Sensor.NumRays, ...
    'MaxRange', scenario.Observer.Fov.MaxRange, ...
    'OpeningAngle', scenario.Observer.Fov.OpeningAngle));
[~, belief] = estimator.Correct(state, scanA);

truthAtB = sensing.raycastScan(poseB, scenario.Observer.Fov, ...
    scenario.Obstacles, scenario.Sensor, 0);
end

function mask = nearNoReturnWedge(hasReturn)
%NEARNORETURNWEDGE Flag beams adjacent to a return/no-return transition.

mask = false(size(hasReturn));
transition = find(diff(hasReturn) ~= 0);
mask(transition) = true;
mask(transition + 1) = true;
end

function maxError = blendError(belief, poseA, offset, scenario, isNearWedge)
%BLENDERROR Max blend-beam error for a pure translation offset from poseA.

poseB = poseA + [offset(:).', 0];
truth = sensing.raycastScan(poseB, scenario.Observer.Fov, ...
    scenario.Obstacles, scenario.Sensor, 0);
prediction = estimation.transportBoundary( ...
    belief, poseB, estimation.defaultTransportParams());
scored = prediction.Label == "blend" & ~isNearWedge;
maxError = max(abs(prediction.Mean(scored) - truth.Ranges(scored)));
end
