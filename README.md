Set up a conda environment using the req.txt file

The code demonstrates sim-to-real on the Kinova Gen 3 arm, for a arbritrary policy trained in robosuite. It currently mimics the simulated robot. Once, real observations are incorporated, this can be adapted to transferring a robosuite policy to a real robot (actual sim-to-real).

Future work: Incorporating computer vision for real observations, using MoveIt Motion Planning instead of the Kinova Kortex API so this works for other robots
