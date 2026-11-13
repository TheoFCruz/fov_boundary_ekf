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
