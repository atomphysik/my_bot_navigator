# Navigator Package for autonomous driving 

## Obstacle Avoidance 

Obstacle avoidance was developed for the situation in which you cannot
(or do not) use the path planning algorithm for driving.  
This algorithm uses the following two topics to determine whether we need
to make adjustment for avoiding or not.

- _/cmd\_vel_ for current velocity
- _/local\_costmap/costmap\_raw_ for calculating costmap gradient.

So before running, you should activate the slam_toolbox and navigation2.
If you are using my_bot package, you can simply activate them in the terminal
with the following commands.

```bash

ros2 launch my_bot launch_slam.launch .py
ros2 launch nav2_bringup navigation_launch.py

```


### How to run
At first, you should build and source the setup.bash file of the workspace 
which contains this package.
```bash
cd ~/${your_workspace_name}
colcon build --symlink-install
source ./install/setup.bash
```
Then you can simply run this obstacle avoidance node with this command:
```bash
ros2 run my_bot_navigator costmap_obstacle_avoidance
```

But you will casually face to an error message that the node couldn't find the transformation of odom or base_link. It is just a problem of ROS2 system,
and it only happnens when starting up. So if you get this error message, just
rerun this node until it starts to find these transformation succesfully.
