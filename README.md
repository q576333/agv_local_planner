# agv local planner - A Novel Online Time-Optimal Kinodynamic Motion Planning for ROS local planner plugin Using NURBS
In the ROS community, most local planner required [Costmap2D](http://wiki.ros.org/costmap_2d) information to decide velocity commands in each sampling time. In other words, mobile robot is limited by lidar sensor to preception obstacles. Moreover, They are based on optimization method easily fall into local optimal. This will result in robot deliberately deviates from the predefined path and leads the robot oscillation in the narrow space or near goal location.

Therefore, We proposed online time-optimal remaining length interpolation method that based on [move_base_flex](http://wiki.ros.org/move_base_flex) developed the NURBS local planner plugin to solved mentioned problem. The main features are listing below:
* We used numerical calculations can real time compute velocity commands on 100Hz and the velocity profile is smoother than traditional method.
* We adopted stop before obstacle strategy to avoid dynamic obstacle (require nearest obstacle depth information).
* This plugin is map independent of motion planning. It means we can toward research used only vision-based sensing system(Visual SLAM and obstacle detection) to navigation.
* Regarding non-holonomic and kinodynamic constraints.

# Installation
TODO 

# Usage
TODO

