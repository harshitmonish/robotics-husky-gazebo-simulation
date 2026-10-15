# Husky Gazebo Simulation Setup

This project contains a ROS 1 package scaffold for starting a Husky Gazebo playpen simulation. The launch file includes the Husky Gazebo playpen and robot spawn launch files, while `map.txt` contains a grid-like map artifact.

## Contents

- `package.xml` and `CMakeLists.txt`: catkin package metadata/build configuration.
- `launch/lab5.launch`: includes `husky_gazebo` playpen and Husky spawn launches.
- `map.txt`: map data.
- `Assignment-5.pdf`: original coursework handout.

The package archive and extracted package do not contain node implementation files: `src/`, `scripts/`, and `msgs/` are empty. As a result, the assignment behavior cannot be inferred or run from the available code alone.

## Requirements and launch

The launch file requires ROS 1, Gazebo, and the `husky_gazebo` package with its `playpen.launch` and `spawn_husky.launch` files. To open the simulation after placing this repository, with `package.xml` at its root, in a catkin workspace:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch lab5 lab5.launch
```

This starts the simulation only; no assignment node is launched by this file. Add the missing implementation and any runtime dependencies before describing this as a complete project.

## Academic use

Check course rules and the handout's redistribution terms before making the coursework files public.
