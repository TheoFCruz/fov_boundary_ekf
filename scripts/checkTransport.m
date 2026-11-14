%CHECKTRANSPORT Plot estimation.transportBoundary against truth in openRoom.

run('startup.m');
scenario = scenarios.openRoom();
poseA = scenario.Observer.InitialPose;
scanA = sensing.raycastScan(poseA, scenario.Observer.Fov, ...
    scenario.Obstacles, scenario.Sensor, 0);
estimator = estimation.makePassThroughEstimator();
state = estimator.Initialize(struct());
[~, belief] = estimator.Correct(state, scanA);
params = estimation.defaultTransportParams();

% A rotation case: 3 beam spacings in place.
beamSpacing = deg2rad(100) / (scenario.Sensor.NumRays - 1);
rotationPose = poseA + [0, 0, 3 * beamSpacing];
rotationTruth = sensing.raycastScan(rotationPose, scenario.Observer.Fov, ...
    scenario.Obstacles, scenario.Sensor, 0);
rotationPrediction = estimation.transportBoundary(belief, rotationPose, params);
viz.plotTransportCheck(rotationPrediction, rotationTruth);

% A translation case: forward by a few beam spacings' worth of range.
translationPose = poseA + [0.1, 0, 0];
translationTruth = sensing.raycastScan(translationPose, scenario.Observer.Fov, ...
    scenario.Obstacles, scenario.Sensor, 0);
translationPrediction = estimation.transportBoundary(belief, translationPose, params);
viz.plotTransportCheck(translationPrediction, translationTruth);

% A sideways-slide case in boxInFrontOfWall (eqs. 8-11): the box's near
% surface slides across the wall behind it, producing edge, overlap and
% disoccluded beams, with s_i and r_far,i markers on the disoccluded ones.
boxScenario = scenarios.boxInFrontOfWall();
boxPoseA = boxScenario.Observer.InitialPose;
boxScanA = sensing.raycastScan(boxPoseA, boxScenario.Observer.Fov, ...
    boxScenario.Obstacles, boxScenario.Sensor, 0);
boxState = estimator.Initialize(struct());
[~, boxBelief] = estimator.Correct(boxState, boxScanA);
slidePose = boxPoseA + [0, 0.06, 0];
slideTruth = sensing.raycastScan(slidePose, boxScenario.Observer.Fov, ...
    boxScenario.Obstacles, boxScenario.Sensor, 0);
slidePrediction = estimation.transportBoundary(boxBelief, slidePose, params);
viz.plotTransportCheck(slidePrediction, slideTruth);
