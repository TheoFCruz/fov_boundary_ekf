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
            % is the flat line x = 1.8, so along a beam at relative angle
            % theta the true range to it is 1.8 / cos(theta). Beam 44
            % (3.75 deg) disoccludes behind the box when slid 0.06 m
            % sideways; its r_far must match that flat-wall formula.
            [belief, scenario] = boxInFrontOfWallBelief();
            pose = scenario.Observer.InitialPose + [0, 0.06, 0];
            prediction = estimation.transportBoundary( ...
                belief, pose, estimation.defaultTransportParams());

            verifyEqual(testCase, prediction.Label(44), "disoccluded");
            expectedFarRange = 1.8 / cos(prediction.Angles(44));
            verifyEqual(testCase, prediction.FarRange(44), expectedFarRange, 'AbsTol', 1e-3);
        end

        function boxInFrontOfWallGapNeverBridgedExceptDisoccluded(testCase)
            % Decision D1: a far-surface pair cannot bracket a beam inside
            % a jump gap, so no beam's predicted range should fall strictly
            % between the box's own range (~0.80-0.82 m across its angular
            % width) and the wall's (~1.8-2.0 m) unless it is explicitly
            % "disoccluded" (the free-space hypothesis, eq. 11).
            [belief, scenario] = boxInFrontOfWallBelief();
            params = estimation.defaultTransportParams();
            offsets = [0, 0.03, 0; 0, 0.06, 0; 0, -0.03, 0; 0, -0.06, 0; ...
                0.03, 0, 0; 0.06, 0, 0];
            for row = 1:size(offsets, 1)
                pose = scenario.Observer.InitialPose + offsets(row, :);
                prediction = estimation.transportBoundary(belief, pose, params);
                inGap = prediction.Mean > 0.85 & prediction.Mean < 1.75;
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
