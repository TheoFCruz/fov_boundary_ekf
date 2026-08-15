classdef TestModels < matlab.unittest.TestCase
    %TESTMODELS Tests for the initial FOV and obstacle models.

    methods (Test)
        function storesFovSettings(testCase)
            spec = fov.FovSpec(12, pi/2);

            verifyEqual(testCase, spec.MaxRange, 12);
            verifyEqual(testCase, spec.OpeningAngle, pi/2);
        end

        function createsObserverFromDirectSettings(testCase)
            observer = fov.Observer([1, 2], pi/4, 12, pi/2);

            verifyEqual(testCase, observer.Position, [1, 2]);
            verifyEqual(testCase, observer.Heading, pi/4);
            verifyEqual(testCase, observer.MaxRange, 12);
            verifyEqual(testCase, observer.OpeningAngle, pi/2);
        end

        function createsObserverFromFovSpec(testCase)
            spec = fov.FovSpec(8, pi/3);
            observer = fov.Observer( ...
                'Position', [3, -1], 'Heading', -pi/6, 'Fov', spec);

            verifyEqual(testCase, observer.Position, [3, -1]);
            verifyEqual(testCase, observer.Heading, -pi/6);
            verifyEqual(testCase, observer.Fov, spec);
        end

        function acceptsAndNormalizesClosedPolygon(testCase)
            obstacle = fov.polygonObstacle('box', ...
                [0, 0; 2, 0; 2, 1; 0, 1; 0, 0]);

            verifyEqual(testCase, obstacle.Name, "box");
            verifySize(testCase, obstacle.Vertices, [4, 2]);
            verifyEqual(testCase, obstacle.Vertices(1, :), [0, 0]);
            verifyEqual(testCase, obstacle.Vertices(end, :), [0, 1]);
        end

        function rejectsNonConvexPolygon(testCase)
            vertices = [0, 0; 2, 0; 1, 0.5; 2, 1; 0, 1];

            verifyError(testCase, @() fov.polygonObstacle(vertices), ...
                'fov:polygonObstacle:NonConvexPolygon');
        end

        function rejectsInvalidFov(testCase)
            verifyError(testCase, @() fov.FovSpec(0, pi/2), ...
                'fov:FovSpec:InvalidMaxRange');
            verifyError(testCase, @() fov.FovSpec(10, 0), ...
                'fov:FovSpec:InvalidOpeningAngle');
        end
    end
end
