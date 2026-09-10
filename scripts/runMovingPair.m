%RUNMOVINGPAIR Run, summarize, and replay the checkpoint-one example.

run('startup.m');
scenario = scenarios.movingPair();
result = simulation.runScenario(scenario);
viz.plotSimulationSummary(result);
viz.animateSimulation(result);
