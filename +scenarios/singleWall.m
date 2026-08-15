function scenario = singleWall()
%SINGLEWALL Create a scenario with one wall crossing the central FOV.

scenario.Name = "Single wall";
scenario.Observer = fov.Observer([0, 0], 0, 10, pi/2);
scenario.Obstacles = fov.polygonObstacle('wall', ...
    [4, -2; 4, 2; 4.3, 2; 4.3, -2]);
scenario.NumRays = 181;
end
