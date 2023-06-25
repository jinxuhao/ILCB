# ILCB
## **Instantaneous Local Control Barrier Function for Collision Avoidance**

### **Scientific thesis for the procurance of the degree B.Sc. from the Department of Electrical and Computer Engineering at the Technical University of Munich.**

### **Supervised by Univ.-Prof. Dr.-Ing./Univ. Tokio habil. Martin Buss PD Dr.-Ing. Cong Li, PD Dr.-Ing. habil. Marion LeiboldChair of Automatic Control Engineering**
### **Submitted by B.Sc. Xuhao Jin**
## Citation
安装package 后运行程序跑不同案例

case 1 :
	clf+ static 
		roslaunch nexus_4wd_mecanum_gazebo  nexus_4wd_mecanum_world1.launch

		python3 pd_CBF_static.py


case 2:
	clf + dynamic

		roslaunch nexus_4wd_mecanum_gazebo  nexus_4wd_mecanum_world.launch

		python3 pd_CBF_dynamic1.py
		python3 pd_CBF_static_dynamic_case.py

case3：
	clf + small_demo(100,150)

		roslaunch nexus_4wd_mecanum_gazebo  nexus_4wd_mecanum_world.launch

		python3 pd_CBF_static_small_demo.py 



case4：
	comparison

		roslaunch nexus_4wd_mecanum_gazebo nexus_4wd_mecanum_duibi.launch

		export TURTLEBOT3_MODEL=waffle_pi
		source ~/.bashrc


		roslaunch ros_autonomous_slam turtlebot3_world.launch
	
		roslaunch ros_autonomous_slam autonomous_explorer.launch

		roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

		roslaunch ros_autonomous_slam turtlebot3_navigation.launch
	
		python3 pd_CBF_static_duibi1.py



## General Setup
FASTER has been tested with 
* Ubuntu 16.04/ROS Kinetic  
* Ubuntu 18.04/ROS Melodic 

Other ROS versions may require some minor changes, feel free to [create an issue](https://github.com/mit-acl/faster/issues) if you have any problems. The Gurobi versions tested are Gurobi 8.1, Gurobi 9.0, and Gurobi 9.1.

Install the [Gurobi Optimizer](https://www.gurobi.com/products/gurobi-optimizer/). You can test your installation typing `gurobi.sh` in the terminal. Have a look at [this section](#issues-when-installing-gurobi) if you have any issues.

Install the following dependencies:
```
sudo apt-get install ros-"${ROS_DISTRO}"-gazebo-ros-pkgs ros-"${ROS_DISTRO}"-mavros-msgs ros-"${ROS_DISTRO}"-tf2-sensor-msgs
```
```
python -m pip install pyquaternion
```

Create a workspace, and clone this repo and its dependencies:
```
mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/faster.git
wstool init
wstool merge ./faster/faster/install/faster.rosinstall

```

In the following, remember (once the workspace is compiled) to add this to your `~/.bashrc`:
```
source PATH_TO_YOUR_WS/devel/setup.bash
``` 

### Instructions to use FASTER with an aerial robot:

Compile the code:

```
wstool update -j8
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```

And finally open 5 terminals and execute these commands:
```
roslaunch acl_sim start_world.launch
roslaunch acl_sim perfect_tracker_and_sim.launch
roslaunch global_mapper_ros global_mapper_node.launch
roslaunch faster faster_interface.launch
roslaunch faster faster.launch
```
The blue grid shown in Rviz is the unknown space and the orange one is the occupied-known space. Now you can click `Start` in the GUI, and then, in RVIZ, press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the drone. 

> **_NOTE (TODO):_**  Right now the radius of the drone plotted in Gazebo (which comes from the `scale` field of `quadrotor_base_urdf.xacro`) does not correspond with the radius specified in `faster.yaml`. 


### Instructions to use FASTER with a ground robot:

> **_IMPORTANT NOTE:_**  There are some important differences on the performance of the ground robot when using the Gazebo version that comes with ROS Kinetic and the one that comes with ROS Melodic. To achieve a good tracking error (like the one shown [here](https://github.com/mit-acl/faster/blob/master/faster/imgs/gr_sim.gif)), you may have to tune the [gains of the controller](https://github.com/mit-acl/faster/blob/master/faster/scripts/goal_odom_to_cmd_vel_state.py) depending on the specific verion of ROS/Gazebo that you are using. 

Install the following dependencies:
```
sudo apt-get install ros-"${ROS_DISTRO}"-control-toolbox ros-"${ROS_DISTRO}"-ros-control ros-"${ROS_DISTRO}"-robot-localization ros-"${ROS_DISTRO}"-lms1xx ros-"${ROS_DISTRO}"-interactive-marker-twist-server ros-"${ROS_DISTRO}"-hector-gazebo-plugins ros-"${ROS_DISTRO}"-move-base ros-"${ROS_DISTRO}"-ros-control ros-"${ROS_DISTRO}"-ros-controllers ros-"${ROS_DISTRO}"-pointgrey-camera-description ros-"${ROS_DISTRO}"-hardware-interface ros-"${ROS_DISTRO}"-message-to-tf ros-"${ROS_DISTRO}"-gazebo-ros-control
```
Then download the ground_robot-specific packages and compile the repo:

```
wstool merge ./faster/faster/install/faster_ground_robot.rosinstall
wstool update -j8
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```


Then, in [`faster.yaml`](https://github.com/mit-acl/faster/tree/master/faster/param), change these parameters:
```
drone_radius: 0.5  #[m]

z_max: 0.5         #[m] 
z_ground: -0.2

v_max: 1.4         #[m/s]  
a_max: 1.4         #[m/s2] 
j_max: 5.0         #[m/s3]

is_ground_robot: true  
```

And finally open 4 terminals and execute these commands
```
roslaunch faster ground_robot.launch
roslaunch global_mapper_ros global_mapper_node.launch quad:=JA01
roslaunch faster faster_interface.launch quad:=JA01 is_ground_robot:=true
roslaunch faster faster.launch quad:=JA01
```

Now you can click `Start` in the GUI, and then, in RVIZ, press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the ground robot. 

