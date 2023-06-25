# ILCB
## **Instantaneous Local Control Barrier Function for Collision Avoidance**

### **Scientific thesis for the procurance of the degree B.Sc. from the Department of Electrical and Computer Engineering at the Technical University of Munich.**

### **Supervised by Univ.-Prof. Dr.-Ing./Univ. Tokio habil. Martin Buss PD Dr.-Ing. Cong Li, PD Dr.-Ing. habil. Marion LeiboldChair of Automatic Control Engineering**
### **Submitted by B.Sc. Xuhao Jin**


## Results Showcase
Case 1:	IL-CBF in One Outdoor Scenario
cbf_

https://github.com/jinxuhao/ILCB/assets/93445630/9214cc05-b572-4272-b500-6844a5371b0e

forest

[![IL-CBF for Dynamic Obstacle](./faster/imgs/uav_sim.gif)](https://www.youtube.com/watch?v=fkkkgomkX10 "IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments")     



Case 2:	Comparative Simulation in One Indoor Scenario



Case 3:	IL-CBF for Static small Obstacle
small_demo1
		[![IL-CBF for Static small Obstacle](./faster/imgs/uav_sim.gif)](https://www.youtube.com/watch?v=fkkkgomkX10 "IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments")    

  
Case 4:	IL-CBF for Dynamic Obstacle
dynamic_new
	[![IL-CBF for Dynamic Obstacle](./faster/imgs/uav_sim.gif)](https://www.youtube.com/watch?v=fkkkgomkX10 "IROS 2019: FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments")    


## User Manual

Case 1:	IL-CBF in One Outdoor Scenario
 
		roslaunch nexus_4wd_mecanum_gazebo  nexus_4wd_mecanum_world1.launch

		python3 pd_CBF_static.py


Case 2:	Comparative Simulation in One Indoor Scenario

		roslaunch nexus_4wd_mecanum_gazebo nexus_4wd_mecanum_duibi.launch

		export TURTLEBOT3_MODEL=waffle_pi
		source ~/.bashrc

		roslaunch ros_autonomous_slam turtlebot3_world.launch
	
		roslaunch ros_autonomous_slam autonomous_explorer.launch

		roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

		roslaunch ros_autonomous_slam turtlebot3_navigation.launch
	
		python3 pd_CBF_static_duibi1.py

Case 3:	IL-CBF for Static small Obstacle

		roslaunch nexus_4wd_mecanum_gazebo  nexus_4wd_mecanum_world.launch

		python3 pd_CBF_static_small_demo.py 
Case 4:	IL-CBF for Dynamic Obstacle

		roslaunch nexus_4wd_mecanum_gazebo  nexus_4wd_mecanum_world.launch

		python3 pd_CBF_dynamic1.py
		python3 pd_CBF_static_dynamic_case.py






## Citation

## General Setup
Results has been tested with 
* Ubuntu 18.04/ROS Melodic 

Install dependencies for reference；

	master/requirements.txt

