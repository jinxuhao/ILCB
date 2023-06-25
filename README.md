# ILCB
## **Instantaneous Local Control Barrier Function for Collision Avoidance**

### **Scientific thesis for the procurance of the degree B.Sc. from the Department of Electrical and Computer Engineering at the Technical University of Munich.**

### **Supervised by Univ.-Prof. Dr.-Ing./Univ. Tokio habil. Martin Buss PD Dr.-Ing. Cong Li, PD Dr.-Ing. habil. Marion LeiboldChair of Automatic Control Engineering**
### **Submitted by B.Sc. Xuhao Jin**


## Results Showcase



## User Manual

Case 1 :
	clf+ static 
 
		roslaunch nexus_4wd_mecanum_gazebo  nexus_4wd_mecanum_world1.launch

		python3 pd_CBF_static.py


Case 2:
	clf + dynamic

		roslaunch nexus_4wd_mecanum_gazebo  nexus_4wd_mecanum_world.launch

		python3 pd_CBF_dynamic1.py
		python3 pd_CBF_static_dynamic_case.py

Case 3：
	clf + small_demo(100,150)

		roslaunch nexus_4wd_mecanum_gazebo  nexus_4wd_mecanum_world.launch

		python3 pd_CBF_static_small_demo.py 



Case 4：
	comparison

		roslaunch nexus_4wd_mecanum_gazebo nexus_4wd_mecanum_duibi.launch

		export TURTLEBOT3_MODEL=waffle_pi
		source ~/.bashrc

		roslaunch ros_autonomous_slam turtlebot3_world.launch
	
		roslaunch ros_autonomous_slam autonomous_explorer.launch

		roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

		roslaunch ros_autonomous_slam turtlebot3_navigation.launch
	
		python3 pd_CBF_static_duibi1.py

## Citation

## General Setup
Results has been tested with 
* Ubuntu 18.04/ROS Melodic 

Install dependencies for reference；

	master/requirement.text

Now you can click `Start` in the GUI, and then, in RVIZ, press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the ground robot. 

