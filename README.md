# Bayesian-Approach-Grid-Localization
Grid Localization is a variant of discrete Bayes Localization. In this method, the map
is converted to a Grid. At each time step, the algorithm finds out the probabilities of
the robot presence at every grid cell. The grid cells with maximum probabilities at each
step, characterize the robot’s trajectory.
Grid Localization runs in two iterative steps. Movement and Observation.
After each movement, we compute if that certain movement can move the robot
between grid cells. For each observation, you should find the most probable cells which
the given observation may have occurred in them.
