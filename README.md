TurboEngine is a custom game engine developed in UWP/WinRT C++ and targets the Windows10 and XboxOne platforms. 
My main contribution is to this project is gameplay systems and the AI.
The main gameplay is that players can create levels (similar to Halo's forge) and then play as a Tank fighting against either friends over the network, or ai tanks. 

This repo contains a few snippets of code:
- ai-system.cpp
	- the tick() function which processes the data-driven behavior trees for each agent
- CanSeeTank.cpp
	- a decorator node that acts as a tank's main sight/perception for the world
- MoveTo.cpp
	- a behavior node that simply steers a tank towards a point respective of its orientation
-EnvironmentQuery.cpp
	- A highlight of the EnvironmentQuerySystem I implemented for this project. This file is bigger than the others, it features
		- EnvironmentQuery base class used to construct tests 
		- OtherTankDotTest class which is an example of an implemented test
		- QuerySet::score which implements a optimized procedure of score a list of tests against a list of points