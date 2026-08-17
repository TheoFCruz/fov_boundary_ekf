% Launch the interactive signed-distance heading explorer.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(projectRoot, 'startup.m'));

scenario = scenarios.singleWall();
viz.interactiveScenario(scenario);
