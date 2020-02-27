TurboEngine is a custom game engine our team Turboclock is working on. Its developed in UWP/WinRT C++ and targets the Windows10 and XboxOne platforms. The main gameplay is that players can create levels (similar to Halo's forge) and then play as a Tank fighting against either friends over the network, or ai tanks. My main contribution is to this project is gameplay systems and the AI.

This repo represents code snippits I've contributed towards the project, specifically some of the Tank combat behaviors. 
This project is currently under development. As a result, this snippet is from a previous milestone our team reached. Expect this repository to update as the project develops. 

**AISystem.cpp**
- contains snippets from the original file showing off the specific behaviors for a Combat Behavior Tree.
- Contains a simple AStar algorithm that uses navigation data generated from Recast/Detour to find and generate a path. Currently under development
- Behavior Tree Nodes include:
	- Move To - the steering behavior
	- MoveToPath - the parent behavior to pop goals off a path
	- FindWalkablePoint  - given a target location, finds the nearest point on the navigation mesh
	- Find Random Walkable Point - finds a random point on the navigation mesh that the agent can reach
	- FindPath - the behavior that makes the pathing request and sets its result to the agent's blackboard
	- CanSeeTank - a decorator that checks if another tank is in the agent's sight range
	- FacePlayer - behavior to slerp the agen't rotation to face a player/target
	- FindPointAroundTarget - behavior to find a walkable point around a targeted agent
	- ObstacleAvoidance - steering behavior to avoid obstacles not represented through the navigation mesh.
