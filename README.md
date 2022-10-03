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

https://media.github.umn.edu/user/19560/files/9c65c095-6ab4-41e3-ab5b-55a639e204bc

### Improved Agent & Scene Rendering

Agents were rendered using images of people from a bird's eye perspective. The agents use circular bounding geometry. Obstacles were rendered as complex polygons.

https://media.github.umn.edu/user/19560/files/4cb7adf0-ae70-4a07-a233-76a924f392ac

### Orientation Smoothing

Agents were given an orientation with forward as the direction that the 'eyes' of the models are facing. To show how the rotation corresponds with the agent's path, the agent in the bedroom's path is shown (the agent in the blue shirt in the bottom left).

https://media.github.umn.edu/user/19560/files/d8218a2a-6315-49e4-8504-892bf3665dc2

### Multiple Agents Planning

Multiple agents are rendered in each scene. Each agent moves towards their own goal that is randomly chosen, and upon reaching that goal will choose a new one.

https://media.github.umn.edu/user/19560/files/b43803cd-e894-4513-b278-7831ae716c39

### Challenge: Crowd Simulation

Agents avoid collision by using TTC forces supplemented with a repelling force to ensure the agents do not get stuck ontop of eachother. The first video below has both bounding circles and paths enabled to show off collision avoidance. The following two videos show off the two scenes, the apartment and the campus, respectively.

https://media.github.umn.edu/user/19560/files/0b635dac-06a0-4ccb-8e4c-5b0a4de853e5

https://media.github.umn.edu/user/19560/files/f103fdab-a217-4d00-b9e1-ff02e4362bdb

https://media.github.umn.edu/user/19560/files/8b4772ec-2140-47a3-9ef2-2889ecd24d7d

## Tools/Libraries used

- For A* in PRM, Java PriorityQueues were used.
- For detecting if points were in shapes (obstacles) the code was adapted from a [processing.org forum](https://discourse.processing.org/t/checking-for-a-point-within-a-2d-shape-v3-5/26874/15) to account for padding.
- For determining if two line segments were intersecting, code was borrowed from [stackoverflow](https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect).

## Difficulties
