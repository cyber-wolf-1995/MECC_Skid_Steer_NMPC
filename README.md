# MECC_Skid_Steer_NMPC
Description: This work contains a workspace that comprises of a ROS-Gazebo environment with Clearpath Husky robot traversing on uneven terrain. The robot tracks a given path autonomously using a differential-drive extended kinematic NMPC controller.

Instructions:
1. Install MATLAB
2. Download and install CASADi toolbox https://web.casadi.org/ 
3. Install ROS and Gazebo
4. Download and build package.
5. Run: $ roslaunch mecc_rl_pp husky_gazebo_world.launch
6. $ roslaunch husky_viz view_robot.launch
7. Enable navigation topic in Gazebo
8. $ roslaunch mecc_rl_pp amcl_demo.launch
9. Set the frame in Gazebo to map frame
10. In MATLAB, open mpc_ros.m set IP address of host machine.
11. Run mpc_ros.m
