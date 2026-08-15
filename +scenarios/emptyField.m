function scenario = emptyField()
%EMPTYFIELD Create an obstacle-free visualization scenario.

scenario.Name = "Empty field";
scenario.Observer = fov.Observer([0, 0], 0, 10, pi/2);
scenario.Obstacles = struct('Name', {}, 'Vertices', {});
scenario.NumRays = 181;
end
