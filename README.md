# planning_playground
This is a playground I created to learn and play with motion and task planning algorithms. My current effort is to focus on creation of a full motion planning pipeline with evaluation. That is the core reason that there are not many motion models, search algorithms, or smoothers. Once the full pipeline is implemented, I will swap to implementing more "interesting" algorithms like Bi-directional RRT*, BIT, D* lite, lattice planners, and trajectory optimization.

You might ask, why choose Python for implementing path planning. Python would not be the first choice for production systems, but this is not intended to be used in production. Also, I mainly work in C++, so I wanted to use Python just for some variety. There is also the added benefit of making eventual optimization and ML tasks quite a bit easier.

## Current map support
Currently only 2d maps are supported. The map can be set up to do collision checking in either discrete or continuous forms.

## Current motion models
- simple holonomic model
- kinematic unicycle
- kinematic bicycle

## Current search methods
- A* implementation
    - implicitly supports hybrid A* search with motion model selection
- RRT implementation
- RRT* implementation


## Current smoothers
- simple spline smoother
- bezier curve smoother
    - current work in progress

## Current development efforts
- path evaluations
    - integration with ROS2 and gazebo to simulate following the path
    - plot tracking error
    - plot "effort" of the robot
- more work on smoothers
