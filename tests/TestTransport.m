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

        function isolatedEdgeSurfaceIsInheritedByNewBeams(testCase)
            % Regression: only the outermost beam (-50 deg) hits a box at
            % 0.94 m; the rest of the 2 m sector is open. Moving back 3 cm
            % pulls that point inward between new beams, leaving the two
            % outermost beams uncovered. They must inherit the box point's
            % transported range (an isolated surface has no line to extend)
            % instead of being filled as open from the nearest resampled beam.
            angles = linspace(-deg2rad(50), deg2rad(50), 81).';
            belief = struct();
            belief.Angles = angles;
            belief.Mean = 2 * ones(81, 1);
            belief.Mean(1) = 0.94;
            belief.Covariance = sparse(81, 81);
            belief.HasReturn = false(81, 1);
            belief.HasReturn(1) = true;
            belief.ObserverPose = [0, 0, 0];
            belief.MaxRange = 2;
            belief.OpeningAngle = deg2rad(100);
            boxPoint = 0.94 * [cos(angles(1)), sin(angles(1))];
            expectedRange = norm(boxPoint - [-0.03, 0]);

            prediction = estimation.transportBoundary( ...
                belief, [-0.03, 0, 0], estimation.defaultTransportParams());
            % Moving back uncovers both sector edges: beams 1-2 next to the
            % box point and beam 81 next to open space.
            verifyEqual(testCase, find(prediction.Label == "new").', [1, 2, 81]);
            verifyEqual(testCase, prediction.Mean(1:2), ...
                expectedRange * [1; 1], 'AbsTol', 1e-12);
            verifyTrue(testCase, all(prediction.HasReturn(1:2)));
            verifyEqual(testCase, prediction.Mean(81), 2);
            verifyFalse(testCase, prediction.HasReturn(81));
        end

        function zeroMotionAddsNoEdgeTermAtRealEdges(testCase)
            % Regression: at zero motion beams n and f sit exactly half a beam
            % from the mid-beam corner; rounding must not add q_e there.
            scenario = scenarios.boxInFrontOfWall();
            pose = scenario.Observer.InitialPose;
            [belief, ~] = transportCase(scenario, pose, pose);
            belief.Covariance = 1e-4 * speye(numel(belief.Mean));
            params = estimation.defaultTransportParams();
            prediction = estimation.transportBoundary(belief, pose, params);
            [~, ~, Q] = estimation.transport.propagateCovariance(belief, prediction, params);

            verifyEqual(testCase, full(diag(Q)), ...
                params.ProcessNoiseFloor * ones(numel(belief.Mean), 1), 'AbsTol', 1e-15);
        end

        function zeroRangeReturnGivesFiniteMeanAndCovariance(testCase)
            % Regression: a noisy return clipped to 0 m (observer next to a
            % post) has no bearing; the transport must stay finite.
            angles = linspace(-deg2rad(50), deg2rad(50), 81).';
            belief = struct();
            belief.Angles = angles;
            belief.Mean = 0.14 * ones(81, 1);
            belief.Mean(80) = 0;
            belief.Covariance = 0.05 ^ 2 * eye(81);
            belief.HasReturn = true(81, 1);
            belief.ObserverPose = [0, 0, 0];
            belief.MaxRange = 1;
            belief.OpeningAngle = deg2rad(100);
            params = estimation.defaultTransportParams();

            for pose = {[0, 0, 0], [0.01, 0.005, 0.01]}
                prediction = estimation.transportBoundary(belief, pose{1}, params);
                covariance = estimation.transport.propagateCovariance( ...
                    belief, prediction, params);
                verifyTrue(testCase, all(isfinite(prediction.Mean)));
                verifyTrue(testCase, all(isfinite(covariance(:))));
            end
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

        function noGapLabelEverAndWedgeBordersResolveToEdgeOrDisoccluded(testCase)
            % P3 replaces the P1/P2 "jump-gap" placeholder: every beam a
            % jump pair alone brackets is now resolved into "edge" or
            % "disoccluded" (eqs. 8-10), never left as "jump-gap" (or the
            % internal "gap" label). The wedge borders in openRoom (a
            % return/no-return transition, a jump pair by decision D2)
            % should resolve to one of those two labels; the deep front-wall
            % and deep side-wall regions, bracketed only by same-surface
            % pairs, must have none.
            scenario = scenarios.openRoom();
            poseA = scenario.Observer.InitialPose;
            poseB = poseA + [0.04, 0, 0];
            [belief, ~] = transportCase(scenario, poseA, poseB);

            prediction = estimation.transportBoundary( ...
                belief, poseB, estimation.defaultTransportParams());
            verifyTrue(testCase, all(prediction.Label ~= "jump-gap"));
            verifyTrue(testCase, all(prediction.Label ~= "gap"));

            isWedgeBorder = prediction.Label == "edge" | prediction.Label == "disoccluded";
            verifyGreaterThan(testCase, sum(isWedgeBorder), 0);
            deepFrontWall = abs(prediction.Angles) < deg2rad(25);
            deepSideWall = abs(prediction.Angles) > deg2rad(45);
            verifyFalse(testCase, any(isWedgeBorder & deepFrontWall));
            verifyFalse(testCase, any(isWedgeBorder & deepSideWall));
        end

        function zeroMotionReproducesMeanAndHasReturnExactly(testCase)
            % Decision D4: at zero motion every transported bearing lands
            % exactly back on the fixed grid, so the predicted mean and
            % HasReturn must reproduce the belief exactly, in both a
            % no-jump scenario (openRoom) and one full of edges
            % (boxInFrontOfWall). Labels stay within the categories a
            % zero-displacement transport can produce; "disoccluded"/
            % "overlap" need actual parallax and cannot appear here.
            for scenario = [scenarios.openRoom(), scenarios.boxInFrontOfWall()]
                pose = scenario.Observer.InitialPose;
                scan = sensing.raycastScan(pose, scenario.Observer.Fov, ...
                    scenario.Obstacles, scenario.Sensor, 0);
                estimator = estimation.makePassThroughEstimator();
                state = estimator.Initialize(struct());
                [~, belief] = estimator.Correct(state, scan);

                prediction = estimation.transportBoundary( ...
                    belief, pose, estimation.defaultTransportParams());
                verifyEqual(testCase, prediction.Mean, belief.Mean, 'AbsTol', 1e-9);
                verifyEqual(testCase, prediction.HasReturn, belief.HasReturn);
                verifyTrue(testCase, all(ismember(prediction.Label, ...
                    ["blend", "no-return", "edge", "new"])));
            end
        end

        function zeroMotionIsolatedSingleRaySurfaceIsExact(testCase)
            % Decision D4: a thin post hit by exactly one beam, flanked by
            % no-return on both sides, is bracketed only by two jump pairs
            % (mixed return / no-return) and would otherwise be lost as
            % "new". At zero motion its own transported bearing equals its
            % own angle exactly, so it must come back as "edge" at its own
            % range, not interpolated or dropped.
            belief = isolatedPostBelief();
            prediction = estimation.transportBoundary( ...
                belief, belief.ObserverPose, estimation.defaultTransportParams());

            verifyEqual(testCase, prediction.Label(6), "edge");
            verifyEqual(testCase, prediction.Mean(6), 1, 'AbsTol', 1e-12);
            verifyTrue(testCase, prediction.HasReturn(6));
            verifyEqual(testCase, prediction.Mean([5, 7]), [5; 5], 'AbsTol', 1e-12);
            verifyFalse(testCase, any(prediction.HasReturn([5, 7])));
        end

        function handBuiltCornerMatchesHandComputedShadowAndCrossing(testCase)
            % Hand-computed check of eqs. 8-10 on the isolated-post belief
            % (a near "surface" of one beam between two open regions,
            % decision D2's mixed-pair case) translated sideways by 0.2 m.
            % Independently re-deriving the pair (6,7) corner (theta* =
            % 0.05, c = [0.99875, -0.15002], d = [0.99875, 0.04998], from
            % eq. 8 with mu_n = 1, Delta p = [0; 0.2], Delta psi = 0) and
            % crossing each beam (eq. 10) gives s = 1.336674 at theta =
            % -0.1 rad and s = 3.996666 at theta = 0 rad, both with t > 0
            % (disoccluded); verified against the function's ShadowRange.
            % Both beams' far side (old index 7) has no return, so
            % r_far = MaxRange = 5 (eq. 11) and, with the baseline
            % eta = 0.5, mu_bar = s + 0.5 * (5 - s): 3.168337 and 4.498333.
            belief = isolatedPostBelief();
            pose = [0, 0.2, 0];
            prediction = estimation.transportBoundary( ...
                belief, pose, estimation.defaultTransportParams());

            verifyEqual(testCase, prediction.Label(5), "disoccluded");
            verifyEqual(testCase, prediction.ShadowRange(5), 1.336674, 'AbsTol', 1e-5);
            verifyEqual(testCase, prediction.FarRange(5), 5, 'AbsTol', 1e-9);
            verifyEqual(testCase, prediction.Mean(5), 3.168337, 'AbsTol', 1e-5);
            verifyEqual(testCase, prediction.Label(6), "disoccluded");
            verifyEqual(testCase, prediction.ShadowRange(6), 3.996666, 'AbsTol', 1e-5);
            verifyEqual(testCase, prediction.FarRange(6), 5, 'AbsTol', 1e-9);
            verifyEqual(testCase, prediction.Mean(6), 4.498333, 'AbsTol', 1e-5);
        end

        function freeSpaceFractionBlendsExactlyAtEtaExtremes(testCase)
            % Eq. 11 on the isolated-post belief (same corner as the
            % hand-computed test above): with f (old index 7) no-return,
            % r_far = MaxRange = 5 for beam 5's disocclusion (s = 1.336674).
            % eta = 0 must give exactly s (the conservative seen-free
            % limit, source (iv)); eta = 1 must give exactly r_far (nothing
            % between the corner and the background); eta = 0.5 the
            % midpoint. Eta = 0 depending only on s, never on r_far, is the
            % formal sense in which the conservative hypothesis is blind to
            % what lies behind the corner (methodology sec. 1.2,
            % "Desoclusões": a fixed seen-free limit cannot reveal the
            % background, so it carries no gradient).
            belief = isolatedPostBelief();
            pose = [0, 0.2, 0];
            baseParams = estimation.defaultTransportParams();
            s = 1.336674;
            rFar = 5;

            for eta = [0, 0.5, 1]
                params = baseParams;
                params.FreeSpaceFraction = eta;
                prediction = estimation.transportBoundary(belief, pose, params);
                expected = s + eta * (rFar - s);
                verifyEqual(testCase, prediction.Mean(5), expected, 'AbsTol', 1e-5);
            end
        end

        function farRangeMatchesFlatWallGeometry(testCase)
            % Hand-checked r_far: in boxInFrontOfWall the wall's front face
            % is the flat line x = 1.6, so along a beam at relative angle
            % theta the true range to it is 1.6 / cos(theta). Beam 44
            % (3.75 deg) disoccludes behind the box when slid 0.06 m
            % sideways; its r_far must match that flat-wall formula.
            [belief, scenario] = boxInFrontOfWallBelief();
            pose = scenario.Observer.InitialPose + [0, 0.06, 0];
            prediction = estimation.transportBoundary( ...
                belief, pose, estimation.defaultTransportParams());

            verifyEqual(testCase, prediction.Label(44), "disoccluded");
            expectedFarRange = 1.6 / cos(prediction.Angles(44));
            verifyEqual(testCase, prediction.FarRange(44), expectedFarRange, 'AbsTol', 1e-3);
        end

        function boxInFrontOfWallGapNeverBridgedExceptDisoccluded(testCase)
            % Decision D1: a far-surface pair cannot bracket a beam inside
            % a jump gap, so no beam's predicted range should fall strictly
            % between the box's own range (~0.80-0.82 m across its angular
            % width) and the wall's (~1.6-1.8 m) unless it is explicitly
            % "disoccluded" (the free-space hypothesis, eq. 11).
            [belief, scenario] = boxInFrontOfWallBelief();
            params = estimation.defaultTransportParams();
            offsets = [0, 0.03, 0; 0, 0.06, 0; 0, -0.03, 0; 0, -0.06, 0; ...
                0.03, 0, 0; 0.06, 0, 0];
            for row = 1:size(offsets, 1)
                pose = scenario.Observer.InitialPose + offsets(row, :);
                prediction = estimation.transportBoundary(belief, pose, params);
                inGap = prediction.Mean > 0.85 & prediction.Mean < 1.55;
                verifyTrue(testCase, all(prediction.Label(inGap) == "disoccluded"));
            end
        end

        function boxInFrontOfWallSidewaysSlideYieldsOverlapAtBoxRange(testCase)
            % A 0.06 m sideways slide is enough for the box's near surface
            % to bracket a beam together with another box pair whose
            % interpolated range differs by more than the jump bound: that
            % beam is "overlap", at the box's own (nearest) range.
            [belief, scenario] = boxInFrontOfWallBelief();
            pose = scenario.Observer.InitialPose + [0, 0.06, 0];
            prediction = estimation.transportBoundary( ...
                belief, pose, estimation.defaultTransportParams());

            isOverlap = prediction.Label == "overlap";
            verifyGreaterThan(testCase, sum(isOverlap), 0);
            verifyTrue(testCase, all(prediction.Mean(isOverlap) > 0.79 ...
                & prediction.Mean(isOverlap) < 0.85));
            verifyTrue(testCase, all(prediction.HasReturn(isOverlap)));
        end

        function boxInFrontOfWallTinyMotionGivesEdgeEqualToNearRange(testCase)
            % A sub-cm sideways motion is too small to open a disocclusion
            % window (t stays negative): the box/wall jump pairs' beams all
            % resolve to "edge", at the near pair's own transported range
            % rho_n, never interpolated toward the wall.
            [belief, scenario] = boxInFrontOfWallBelief();
            params = estimation.defaultTransportParams();
            pose = scenario.Observer.InitialPose + [0, 0.005, 0];
            prediction = estimation.transportBoundary(belief, pose, params);

            isEdge = prediction.Label == "edge";
            boxEdge = isEdge & abs(prediction.Angles) < deg2rad(15);
            verifyGreaterThan(testCase, sum(boxEdge), 0);
            verifyTrue(testCase, all(prediction.Mean(boxEdge) > 0.79 ...
                & prediction.Mean(boxEdge) < 0.82));

            % Cross-check against the lower-level stage directly: the near
            % pair's own transported range (eq. 10, t < 0 branch).
            angles = belief.Angles;
            [deltaP, deltaPsi] = estimation.transport.relativeMotion( ...
                belief.ObserverPose, pose);
            endpoints = estimation.transport.transportEndpoints( ...
                angles, belief.Mean, full(diag(belief.Covariance)), deltaP, deltaPsi);
            pairs = estimation.transport.classifyPairs(endpoints, belief.HasReturn, params);
            boxEdgeIndex = find(boxEdge, 1);
            jumpPairIndex = find(pairs.IsJumpPair ...
                & pairs.Lower < angles(boxEdgeIndex) & angles(boxEdgeIndex) < pairs.Upper, 1);
            nearRange = min(pairs.RhoA(jumpPairIndex), pairs.RhoB(jumpPairIndex));
            verifyEqual(testCase, prediction.Mean(boxEdgeIndex), nearRange, 'AbsTol', 1e-9);
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

        function finiteDifferenceJacobianMatchesPredictionGradient(testCase)
            % P6, the permanent test of plan sec. 4.9: perturb each old mean
            % mu_j by +-epsilon and compare the central-difference column of
            % prediction.Mean against estimation.transport.propagateCovariance's
            % F, on surface beams (blend/overlap/edge) whose Label,
            % FromIndex and HasReturn are unaffected by the perturbation and
            % whose Mean stays strictly inside (0, MaxRange) throughout.
            % Hypothesis beams (disoccluded, new) are excluded here: the
            % methodology deliberately sets their F row to zero (no
            % cross-covariance) even though their filled mean does depend
            % on a neighbouring mu_j; that is checked separately
            % (hypothesisRowsAreZeroOnDisocclusionAndNewBeams). Uses the
            % pass-through belief (zero covariance) so the eq. 7 jump test's
            % uncertainty term is unaffected by epsilon.
            params = estimation.defaultTransportParams();
            epsilon = 1e-6;
            beamSpacing = deg2rad(100) / 80;
            motions = { ...
                [0, 0, 0.5 * beamSpacing]; [0, 0, 2 * beamSpacing]; ...
                [0.02, 0, 0]; [0.08, 0, 0]; ...
                [0.03, 0.02, 0.3 * beamSpacing]; [0.08, -0.04, 1.5 * beamSpacing]};

            overallMax = 0;
            for scenario = [scenarios.openRoom(), scenarios.boxInFrontOfWall()]
                pose = scenario.Observer.InitialPose;
                scan = sensing.raycastScan(pose, scenario.Observer.Fov, ...
                    scenario.Obstacles, scenario.Sensor, 0);
                estimator = estimation.makePassThroughEstimator();
                state = estimator.Initialize(struct());
                [~, belief] = estimator.Correct(state, scan);

                for m = 1:numel(motions)
                    poseB = pose + motions{m};
                    err = finiteDifferenceError(belief, poseB, params, epsilon);
                    overallMax = max(overallMax, err);
                end
            end
            % Measured max error 3.7e-10 m across openRoom and
            % boxInFrontOfWall, rotation-only, translation-only and mixed
            % motions up to ~2 beam spacings; bound with headroom. This also
            % settles decision Q3's sign convention: eq. 5 as derived
            % matches the finite difference without flipping any sign.
            verifyLessThan(testCase, overallMax, 1e-5);
        end

        function pureRotationBlendRowsAreInterpolationWeightsSummingToOne(testCase)
            % At alpha = 0 (pure rotation, no translation), eq. 5 reduces to
            % F_ia = w, F_ib = 1 - w: a blend row is exactly the eq. 2
            % interpolation weights, which sum to 1.
            params = estimation.defaultTransportParams();
            scenario = scenarios.openRoom();
            pose = scenario.Observer.InitialPose;
            scan = sensing.raycastScan(pose, scenario.Observer.Fov, ...
                scenario.Obstacles, scenario.Sensor, 0);
            estimator = estimation.makePassThroughEstimator();
            state = estimator.Initialize(struct());
            [~, belief] = estimator.Correct(state, scan);
            beamSpacing = deg2rad(100) / 80;
            poseB = pose + [0, 0, 0.7 * beamSpacing];

            prediction = estimation.transportBoundary(belief, poseB, params);
            [~, F] = estimation.transport.propagateCovariance(belief, prediction, params);
            isBlend = find(prediction.Label == "blend");
            verifyGreaterThan(testCase, numel(isBlend), 50);
            for n = 1:numel(isBlend)
                i = isBlend(n);
                a = prediction.FromIndex(i, 1);
                verifyEqual(testCase, full(F(i, a)), prediction.FromWeight(i), 'AbsTol', 1e-12);
                verifyEqual(testCase, full(sum(F(i, :))), 1, 'AbsTol', 1e-12);
            end
        end

        function zeroMotionGivesIdentityRowsOnSurfaceBeams(testCase)
            % At zero motion every surface beam (blend/overlap/edge with a
            % return) is its own transported bearing, so its F row must be
            % the unit vector e_i; every other row (no-return, and "new"
            % beams that can still appear at the sector edge, decision D1)
            % is zero.
            params = estimation.defaultTransportParams();
            for scenario = [scenarios.openRoom(), scenarios.boxInFrontOfWall()]
                pose = scenario.Observer.InitialPose;
                scan = sensing.raycastScan(pose, scenario.Observer.Fov, ...
                    scenario.Obstacles, scenario.Sensor, 0);
                estimator = estimation.makePassThroughEstimator();
                state = estimator.Initialize(struct());
                [~, belief] = estimator.Correct(state, scan);

                prediction = estimation.transportBoundary(belief, pose, params);
                [~, F] = estimation.transport.propagateCovariance(belief, prediction, params);
                isSurface = prediction.HasReturn ...
                    & ismember(prediction.Label, ["blend", "overlap", "edge"]);
                numBeams = numel(belief.Mean);
                identity = speye(numBeams);
                verifyEqual(testCase, full(F(isSurface, :)), full(identity(isSurface, :)), ...
                    'AbsTol', 1e-9);
                verifyEqual(testCase, full(F(~isSurface, :)), ...
                    zeros(sum(~isSurface), numBeams), 'AbsTol', 1e-9);
            end
        end

        function hypothesisRowsAreZeroOnDisocclusionAndNewBeams(testCase)
            % Disoccluded (c x d does not depend on mu_n, eq. 11) and newly
            % explored (sec. 4.7) beams both get a zero F row by model
            % construction; so does a no-return beam.
            params = estimation.defaultTransportParams();
            [belief, scenario] = boxInFrontOfWallBelief();
            pose = scenario.Observer.InitialPose + [0, 0.06, 0];
            prediction = estimation.transportBoundary(belief, pose, params);
            [~, F] = estimation.transport.propagateCovariance(belief, prediction, params);

            isHypothesis = ismember(prediction.Label, ["disoccluded", "new", "no-return"]);
            verifyGreaterThan(testCase, sum(prediction.Label == "disoccluded"), 0);
            verifyEqual(testCase, full(F(isHypothesis, :)), ...
                zeros(sum(isHypothesis), numel(belief.Mean)));
        end

        function edgeVarianceLandsOnBeamsWithinHalfABeamOfTheTransportedCorner(testCase)
            % Hand-check of eq. 8's qe,i in boxInFrontOfWall with a 5 mm
            % sideways motion (too small to open a disocclusion window, so
            % the box/wall jump pairs resolve to "edge", as in
            % boxInFrontOfWallTinyMotionGivesEdgeEqualToNearRange above).
            % Independently recomputing each jump pair's corner bearing and
            % (rho_f - rho_n)^2/4 and comparing against Qdiag - qBase
            % confirms qe,i on the beams next to the box/wall corners
            % (pairs 35, 46) and shows it vanishes on the far corners near
            % the sector edges (pairs 20, 61, nearly in line with the wall).
            params = estimation.defaultTransportParams();
            [belief, scenario] = boxInFrontOfWallBelief();
            pose = scenario.Observer.InitialPose + [0, 0.005, 0];
            prediction = estimation.transportBoundary(belief, pose, params);
            [~, ~, Q] = estimation.transport.propagateCovariance(belief, prediction, params);
            Qdiag = full(diag(Q));

            [deltaP, deltaPsi] = estimation.transport.relativeMotion(belief.ObserverPose, pose);
            qBase = params.ProcessNoiseFloor + params.TranslationNoise * (deltaP.' * deltaP) ...
                + params.RotationNoise * deltaPsi ^ 2;

            isEdge = prediction.Label == "edge";
            boxEdge = find(isEdge & abs(prediction.Angles) < deg2rad(15));
            verifyGreaterThan(testCase, numel(boxEdge), 0);
            endpoints = estimation.transport.transportEndpoints( ...
                belief.Angles, belief.Mean, full(diag(belief.Covariance)), deltaP, deltaPsi);
            pairs = estimation.transport.classifyPairs(endpoints, belief.HasReturn, params);
            beamSpacing = mean(diff(belief.Angles));

            % Independently recompute each jump pair's eq. 8 corner bearing
            % and (rho_f - rho_n)^2/4, and take the largest one that lands
            % within half a beam of each boxEdge beam, exactly as
            % estimation.transport.propagateCovariance does, but from the
            % stage functions directly rather than via its output.
            % Measured: pairs 35/46 (the box/wall corners) give qe = 0.2555
            % and 0.2554.
            jumpPairIndices = find(pairs.IsJumpPair);
            for n = 1:numel(boxEdge)
                i = boxEdge(n);
                theta = prediction.Angles(i);
                bestQe = 0;
                for p = 1:numel(jumpPairIndices)
                    j = jumpPairIndices(p);
                    if pairs.ReturnA(j) && pairs.ReturnB(j)
                        isANear = belief.Mean(j) <= belief.Mean(j + 1);
                    else
                        isANear = pairs.ReturnA(j);
                    end
                    if isANear
                        thetaNear = belief.Angles(j); muNear = belief.Mean(j);
                        nIndex = j; fIndex = j + 1;
                    else
                        thetaNear = belief.Angles(j + 1); muNear = belief.Mean(j + 1);
                        nIndex = j + 1; fIndex = j;
                    end
                    thetaStar = (thetaNear + belief.Angles(j) + belief.Angles(j + 1) - thetaNear) / 2;
                    rotation = [cos(-deltaPsi), -sin(-deltaPsi); sin(-deltaPsi), cos(-deltaPsi)];
                    corner = (muNear * [cos(thetaStar), sin(thetaStar)] - deltaP.') * rotation.';
                    cornerBearing = atan2(corner(2), corner(1));
                    if abs(mod(theta - cornerBearing + pi, 2 * pi) - pi) >= beamSpacing / 2
                        continue;
                    end
                    rhoFar = endpoints.Rho(fIndex);
                    if ~belief.HasReturn(fIndex)
                        rhoFar = belief.MaxRange;
                    end
                    bestQe = max(bestQe, (rhoFar - endpoints.Rho(nIndex)) ^ 2 / 4);
                end
                verifyGreaterThan(testCase, bestQe, 0);
                verifyEqual(testCase, Qdiag(i) - qBase, bestQe, 'RelTol', 1e-9);
            end

            % Far from any corner (deep front wall), qe must be exactly zero.
            deepFrontWall = abs(prediction.Angles) < deg2rad(5) & prediction.HasReturn ...
                & ismember(prediction.Label, ["blend", "overlap", "edge"]);
            verifyEqual(testCase, Qdiag(deepFrontWall), qBase * ones(sum(deepFrontWall), 1), ...
                'AbsTol', 1e-12);
        end

        function hypothesisVariancesMatchEquationsAndHaveNoCrossCovariance(testCase)
            % Eq. 11's disocclusion variance, sec. 4.7's newly-explored
            % variance, and Q1's no-return floor, each on a sideways slide
            % in boxInFrontOfWall; plus the hypothesis rows/columns of
            % Sigma_bar having zero off-diagonal entries even with a
            % correlated, nonzero input covariance (eq. 11: "the element
            % begins with no cross-covariance").
            params = estimation.defaultTransportParams();
            [belief, scenario] = boxInFrontOfWallBelief();
            numBeams = numel(belief.Mean);
            % A nonzero, spatially-correlated input covariance, so the
            % cross-covariance check is not vacuous.
            belief.Covariance = 0.01 ^ 2 * eye(numBeams) ...
                + 0.0003 * (diag(ones(numBeams - 1, 1), 1) + diag(ones(numBeams - 1, 1), -1));
            pose = scenario.Observer.InitialPose + [0, 0.06, 0];
            prediction = estimation.transportBoundary(belief, pose, params);
            [covariance, ~, Q] = estimation.transport.propagateCovariance(belief, prediction, params);
            Qdiag = full(diag(Q));

            [deltaP, deltaPsi] = estimation.transport.relativeMotion(belief.ObserverPose, pose);
            qBase = params.ProcessNoiseFloor + params.TranslationNoise * (deltaP.' * deltaP) ...
                + params.RotationNoise * deltaPsi ^ 2;
            endpoints = estimation.transport.transportEndpoints( ...
                belief.Angles, belief.Mean, full(diag(belief.Covariance)), deltaP, deltaPsi);

            isDisoccluded = find(prediction.Label == "disoccluded");
            verifyGreaterThan(testCase, numel(isDisoccluded), 0);
            expectedDisoccluded = (prediction.FarRange(isDisoccluded) ...
                - prediction.ShadowRange(isDisoccluded)) .^ 2 / 12 + params.ProcessNoiseFloor;
            verifyEqual(testCase, Qdiag(isDisoccluded), expectedDisoccluded, 'RelTol', 1e-9);

            isNew = find(prediction.Label == "new");
            sigma0 = params.NewBeamSigmaFraction * belief.MaxRange;
            for n = 1:numel(isNew)
                i = isNew(n);
                k = prediction.FromIndex(i, 1);
                delta = abs(mod(prediction.Angles(i) - endpoints.Phi(k) + pi, 2 * pi) - pi);
                expectedNew = sigma0 ^ 2 + (params.NewBeamSlope * prediction.Mean(i) * delta) ^ 2;
                verifyEqual(testCase, Qdiag(i), expectedNew, 'RelTol', 1e-9);
            end

            isNoReturn = ~prediction.HasReturn & prediction.Label ~= "disoccluded" ...
                & prediction.Label ~= "new";
            verifyGreaterThan(testCase, sum(isNoReturn), 0);
            verifyEqual(testCase, Qdiag(isNoReturn), qBase * ones(sum(isNoReturn), 1), 'AbsTol', 1e-12);

            isHypothesis = ismember(prediction.Label, ["disoccluded", "new"]) | isNoReturn;
            hypothesisRows = full(covariance(isHypothesis, :));
            hypothesisIndices = find(isHypothesis);
            for n = 1:numel(hypothesisIndices)
                hypothesisRows(n, hypothesisIndices(n)) = 0;
            end
            verifyEqual(testCase, hypothesisRows, zeros(size(hypothesisRows)), 'AbsTol', 1e-12);
        end

        function zeroMotionAddsOnlyTheProcessNoiseFloorOnSurfaceBeams(testCase)
            % At zero motion, dp = dpsi = 0, so qBase is just the
            % interpolation floor q0; restricted to "blend" beams, which
            % openRoom has no shortage of and which never sit next to a
            % jump pair's corner (unlike the "edge" beams at its no-return
            % wedge borders, decision D2), so qe,i is exactly zero. With
            % F = I on surface beams (zeroMotionGivesIdentityRowsOnSurfaceBeams),
            % Sigma_bar = Sigma + q0 on those rows/columns.
            params = estimation.defaultTransportParams();
            scenario = scenarios.openRoom();
            pose = scenario.Observer.InitialPose;
            scan = sensing.raycastScan(pose, scenario.Observer.Fov, ...
                scenario.Obstacles, scenario.Sensor, 0);
            estimator = estimation.makePassThroughEstimator();
            state = estimator.Initialize(struct());
            [~, belief] = estimator.Correct(state, scan);
            numBeams = numel(belief.Mean);
            belief.Covariance = 0.01 ^ 2 * eye(numBeams) ...
                + 0.0003 * (diag(ones(numBeams - 1, 1), 1) + diag(ones(numBeams - 1, 1), -1));

            prediction = estimation.transportBoundary(belief, pose, params);
            covariance = estimation.transport.propagateCovariance(belief, prediction, params);
            isSurface = prediction.Label == "blend";
            verifyGreaterThan(testCase, sum(isSurface), 50);
            expected = belief.Covariance + params.ProcessNoiseFloor * eye(numBeams);
            verifyEqual(testCase, full(covariance(isSurface, isSurface)), ...
                full(expected(isSurface, isSurface)), 'AbsTol', 1e-9);
        end

        function longRunWithoutCorrectionStaysSymmetricAndPositiveSemidefinite(testCase)
            % Feeding Sigma_bar back as the next step's prior, with no
            % correction, for about 20 small successive motions: Sigma_bar
            % must stay exactly symmetric (after symmetrisation) and PSD
            % (plan sec. 4.9). Measured: the mean beam variance rises from
            % the initial 1e-4 m^2 (uncorrelated 0.01 m std) to about
            % 0.008-0.02 m^2 as it mixes in the larger hypothesis variances,
            % then stays in that range; min/max eigenvalue ratio about 1e-4,
            % comfortably above -1e-12.
            params = estimation.defaultTransportParams();
            scenario = scenarios.boxInFrontOfWall();
            pose = scenario.Observer.InitialPose;
            scan = sensing.raycastScan(pose, scenario.Observer.Fov, ...
                scenario.Obstacles, scenario.Sensor, 0);
            numBeams = scenario.Sensor.NumRays;

            belief = struct();
            belief.Angles = scan.Angles;
            belief.Mean = scan.Ranges;
            belief.HasReturn = scan.HasReturn;
            belief.Covariance = 0.01 ^ 2 * eye(numBeams);
            belief.ObserverPose = pose;
            belief.MaxRange = scan.MaxRange;
            belief.OpeningAngle = scan.OpeningAngle;

            currentPose = pose;
            for step = 1:20
                offsetStep = [0.02 * cos(0.3 * step), 0.02 * sin(0.3 * step), deg2rad(1)];
                newPose = currentPose + offsetStep;
                prediction = estimation.transportBoundary(belief, newPose, params);
                covariance = estimation.transport.propagateCovariance(belief, prediction, params);

                verifyEqual(testCase, covariance, covariance.', 'AbsTol', 0);
                eigenvalues = eig(full(covariance));
                verifyGreaterThanOrEqual(testCase, min(eigenvalues), ...
                    -1e-12 * max(eigenvalues));

                belief.Mean = prediction.Mean;
                belief.HasReturn = prediction.HasReturn;
                belief.Covariance = covariance;
                belief.ObserverPose = newPose;
                currentPose = newPose;
            end
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

            % Also exercise the disoccluded-beam branch (s_i/r_far,i markers,
            % eq. 11), as scripts/checkTransport.m's sideways-slide case does.
            [boxBelief, boxScenario] = boxInFrontOfWallBelief();
            boxPose = boxScenario.Observer.InitialPose + [0, 0.06, 0];
            boxTruth = sensing.raycastScan(boxPose, boxScenario.Observer.Fov, ...
                boxScenario.Obstacles, boxScenario.Sensor, 0);
            boxPrediction = estimation.transportBoundary( ...
                boxBelief, boxPose, estimation.defaultTransportParams());
            verifyGreaterThan(testCase, sum(boxPrediction.Label == "disoccluded"), 0);
            boxHandles = viz.plotTransportCheck(boxPrediction, boxTruth, 'Visible', false);
            testCase.addTeardown(@() close(boxHandles.Figure));
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

function belief = isolatedPostBelief()
%ISOLATEDPOSTBELIEF A single-ray return (a thin post) between open space.
%
% 11 beams, 0.1 rad apart; beam 6 (theta = 0) returns at range 1 m, every
% other beam has no return at the 5 m cap. Pairs (5,6) and (6,7) are the
% only jump pairs (mixed return / no-return, decision D2); every other
% pair is an open (no-return) pair.

belief = struct();
belief.Angles = linspace(-0.5, 0.5, 11).';
belief.Mean = 5 * ones(11, 1);
belief.Mean(6) = 1;
belief.HasReturn = false(11, 1);
belief.HasReturn(6) = true;
belief.Covariance = sparse(11, 11);
belief.ObserverPose = [0, 0, 0];
belief.MaxRange = 5;
belief.OpeningAngle = 1.2;
end

function [belief, scenario] = boxInFrontOfWallBelief()
%BOXINFRONTOFWALLBELIEF Noiseless belief at the initial pose of S2.

scenario = scenarios.boxInFrontOfWall();
pose = scenario.Observer.InitialPose;
scan = sensing.raycastScan(pose, scenario.Observer.Fov, ...
    scenario.Obstacles, scenario.Sensor, 0);
estimator = estimation.makePassThroughEstimator();
state = estimator.Initialize(struct());
[~, belief] = estimator.Correct(state, scan);
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

function maxErr = finiteDifferenceError(belief, poseB, params, epsilon)
%FINITEDIFFERENCEERROR Max |dMean_i/dmu_j - F_ij| over unaffected surface beams.
%
% Compares every column j (perturbing belief.Mean(j) by +-epsilon) against
% estimation.transport.propagateCovariance's F, scoring only beams i whose
% Label, FromIndex and HasReturn are the same at mu_j, mu_j + epsilon and
% mu_j - epsilon, restricted to surface labels (blend/overlap/edge) and
% comfortably inside (0, MaxRange) so clamping never contaminates the
% comparison.

numBeams = numel(belief.Mean);
prediction = estimation.transportBoundary(belief, poseB, params);
[~, F] = estimation.transport.propagateCovariance(belief, prediction, params);
isSurfaceLabel = ismember(prediction.Label, ["blend", "overlap", "edge"]);
margin = 1e-9 * belief.MaxRange;

maxErr = 0;
for j = 1:numBeams
    perturbedUp = belief;
    perturbedUp.Mean(j) = belief.Mean(j) + epsilon;
    perturbedDown = belief;
    perturbedDown.Mean(j) = belief.Mean(j) - epsilon;
    predictionUp = estimation.transportBoundary(perturbedUp, poseB, params);
    predictionDown = estimation.transportBoundary(perturbedDown, poseB, params);
    finiteDifferenceColumn = (predictionUp.Mean - predictionDown.Mean) / (2 * epsilon);

    scored = isSurfaceLabel ...
        & predictionUp.Label == prediction.Label & predictionDown.Label == prediction.Label ...
        & prediction.HasReturn & predictionUp.HasReturn & predictionDown.HasReturn ...
        & prediction.Mean > margin & prediction.Mean < belief.MaxRange - margin ...
        & predictionUp.Mean > margin & predictionUp.Mean < belief.MaxRange - margin ...
        & predictionDown.Mean > margin & predictionDown.Mean < belief.MaxRange - margin;
    for i = find(scored).'
        if isequaln(predictionUp.FromIndex(i, :), prediction.FromIndex(i, :)) ...
                && isequaln(predictionDown.FromIndex(i, :), prediction.FromIndex(i, :))
            maxErr = max(maxErr, abs(finiteDifferenceColumn(i) - full(F(i, j))));
        end
    end
end
end
