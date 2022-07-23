# EBS_AStar

This package was supposed to be a catkin-ised version of the planner based on [EBS A*](https://doi.org/10.1371/journal.pone.0263841) planner implemented in [EBAStar](https://github.com/wanghw1003/EBAStar).

However, as the original code seemed to contain typographical errors for variable names as well as missing header files, "best guess" was used to get the code to eventually compile, but the code did not seem to perform bidirectional search.

As it stands now, this package performs like a zany regular A* star planner.

Nevertheless, this is made public in hopes that this can be fixed in the near future...

# Installation

```
# First, clone repos and deps to 'catkin_ws/src', then install
cd catkin_ws
rosdep update && rosdep install --from-paths src --ignore-src -r -y
catkin build
```

# Launch Simulation

```
# launch without gazebo
roslaunch ebs_astar_demo ebs_astar_sim_demo.launch show_gazebo:=false
```

From RViz, provide a nav goal and you are done!
