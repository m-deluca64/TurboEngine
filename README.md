TurboEngine is a custom game engine developed in UWP/WinRT C++ and targets the Windows10 and XboxOne platforms. 
My main contribution is to this project is gameplay systems and the AI.
The main gameplay is that players can create levels (similar to Halo's forge) and then play as a Tank fighting against either friends over the network, or ai tanks. 

This repo contains a few snippets of code:
- ai-system.cpp
	- the tick() function which processes the data-driven behavior trees for each agent

- QuerySet.cpp
    - my implementation of an Environment Query System (or tactical point analysis) 
	- this file mainly features a score() function which is optimized to perform cheap tests iteratively

- QueryTest.cpp
	- this holds the class definition & an example implementation for Tests used to create full queries in my EQS. 

- CanSeeTank.cpp
	- a decorator node that acts as a tank's main sight/perception for the world
	
- MoveTo.cpp
	- a behavior node that simply steers a tank towards a point respective of its orientation
