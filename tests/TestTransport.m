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
            verifyGreaterThan(testCase, sum(isNew), 10);
            verifyEqual(testCase, prediction.HasReturn(isNew), ...
                truth.HasReturn(isNew));
            scored = isNew & truth.HasReturn;
            err = abs(prediction.Mean(scored) - truth.Ranges(scored));
            % Measured max error 2.2e-15 m (exact wall-line extension);
            % bound loosely in case the room geometry changes.
            verifyLessThan(testCase, max(err), 1e-2);
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
