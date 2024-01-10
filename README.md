# CSCI 5611 [Project 1](https://github.umn.edu/malin146/Project1)

By: Jacob Malin

This version of the project uses motion planning to simulate crowds of people in two scenes, an apartment and a generic unnamed university campus. The agents arbitrarily choose a location to move to, and upon reaching their destination, choose a new goal.

<img src="https://media.github.umn.edu/user/19560/files/dc4c0d92-11db-472c-808c-3d5df22660f0" width="600"/>

<img src="https://media.github.umn.edu/user/19560/files/84a06dd2-62bb-4650-896e-d89f234d6bc9" width="600"/>

## Attempted Components

- Single Agent Navigation
- Improved Agent & Scene Rendering
- Orientation Smoothing
- Multiple Agents Planning
- Challenge: Crowd Simulation

### Single Agent Navigation

Agents move without overlapping obstacles. To better illustrate the agent's navigation, the bounding circle and path were enabled for one agent.

https://github.com/JacobMalin/campus-navigation/assets/34765120/6fee8c32-592e-4b59-92dc-cab922478945

### Improved Agent & Scene Rendering

Agents were rendered using images of people from a bird's eye perspective. The agents use circular bounding geometry. Obstacles were rendered as complex polygons.

https://github.com/JacobMalin/campus-navigation/assets/34765120/ec60732e-1562-4559-9f3d-ce138dfea732

### Orientation Smoothing

Agents were given an orientation with forward as the direction that the 'eyes' of the models are facing. To show how the rotation corresponds with the agent's path, the agent in the bedroom's path is shown (the agent in the blue shirt in the bottom left).

https://github.com/JacobMalin/campus-navigation/assets/34765120/9a4566df-1915-47e5-b405-b5ad0a24a626

### Multiple Agents Planning

Multiple agents are rendered in each scene. Each agent moves towards their own goal that is randomly chosen, and upon reaching that goal will choose a new one.

https://github.com/JacobMalin/campus-navigation/assets/34765120/41d2ea57-b7a1-443b-8d25-d20b676ca48b

### Challenge: Crowd Simulation

Agents avoid collision by using TTC forces supplemented with a repelling force to ensure the agents do not get stuck ontop of eachother. The first video below has both bounding circles and paths enabled to show off collision avoidance. The following two videos show off the two scenes, the apartment and the campus, respectively.

https://github.com/JacobMalin/campus-navigation/assets/34765120/3c2e82c7-be29-4cd0-a5df-4e5497cd3861

https://github.com/JacobMalin/campus-navigation/assets/34765120/3bd18409-c9af-449c-8cb0-32227b82f4d1

https://github.com/JacobMalin/campus-navigation/assets/34765120/0e31a5ae-ed3f-44a2-8d6b-db38077cae71

## Difficulties

One of my main difficulties was biting off more than I could chew. I found myself needing to detect if points were within complex polygons (I wanted to see how hard it would be to not use simple bounding geometry.), and if two line segments were intersecting. I had thought those would be simpler problems then they turned out to be but as that was not the case, my final code on those portions is an amalgamation of online help, cited below, and staring way too hard at a geogebra animation.

Another difficulty was how hard it was to balance all of the forces. At any given moment, if the TTC forces were too high, agents would get thrown into the obstacles, or if the acceleration was too low, they would slide right past the goals. I have it balanced in the final product so that there is minimal obstacle sliding, but it is not completely gone.

## Tools/Libraries used

- For A* in PRM, Java PriorityQueues were used.
- For detecting if points were in shapes (obstacles) the code was adapted from a [processing.org forum](https://discourse.processing.org/t/checking-for-a-point-within-a-2d-shape-v3-5/26874/15) to account for padding.
- For determining if two line segments were intersecting, code was borrowed from [stackoverflow](https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect).
