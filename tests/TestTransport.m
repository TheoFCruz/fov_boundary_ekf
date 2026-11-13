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
