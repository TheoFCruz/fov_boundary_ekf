function scenario = clutteredField()
%CLUTTEREDFIELD Create a scenario with several convex obstacles.

scenario.Name = "Cluttered field";
scenario.Observer = fov.Observer([0, 0], 0, 12, pi/2);

nearBox = fov.polygonObstacle('near box', ...
    [3, -1.1; 3, 1.1; 3.5, 1.1; 3.5, -1.1]);
upperBox = fov.polygonObstacle('upper box', ...
    [5, 2; 6, 2; 6, 3; 5, 3]);
lowerBox = fov.polygonObstacle('lower box', ...
    [6, -3; 7.5, -3; 7.5, -2; 6, -2]);
farBox = fov.polygonObstacle('far box', ...
    [8, 0.5; 9, 0.5; 9, 1.5; 8, 1.5]);

scenario.Obstacles = [nearBox, upperBox, lowerBox, farBox];
scenario.NumRays = 241;
end
