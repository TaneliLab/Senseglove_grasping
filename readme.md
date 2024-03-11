# Grasp Project with Senseglove

## Demo
[![Watch the video](https://img.youtube.com/vi/mCI7lBZvhL8/hqdefault.jpg)](https://www.youtube.com/watch?v=mCI7lBZvhL8))

#### ROS version is noetic

### How to launch our package
#### First, you must build packages in a workspace with senseglove and shadow hand.
#### Second, move soft_ball model to under the ".gazebo" in your home path.
#### Thrid, you must change gazebo world_name in gazebo_hand.launch under the sr_hand package.
```
<arg name="world_name" value="$(find grasp_project)/worlds/our_project.world"/>
```

#### Then, launch our project
```
roslaunch grasp_project our_project.launch
```
##### Other terminal

```
rosrun grasp_project bridge_node
```

#### You should do calibration to match your hand with software.

1. Open and spread your hand&nbsp;&nbsp;&nbsp;-->&nbsp;&nbsp;Type 1&nbsp;&nbsp;&nbsp;// do more than 3 times.
2. Make a fist&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;-->&nbsp;&nbsp;Type 2&nbsp;&nbsp;&nbsp;// do more than 3 times.
3. Type anything&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;-->&nbsp;&nbsp;Calibration finished.
