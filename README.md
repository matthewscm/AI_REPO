# RS2
Repository for RS2 work

# Navigate to workspace root
cd ~/ros2_ws
# Run the System
1.	Connect the robot to the laptop through ethernet 
2.	Ensure e-stop is in proximity and the area is clear of people and obstructions
3.	Power on the robot and ensure all joints are unlocked 
4.	If not already, return robot to home position 
5.	Connect RGBD Camera to the laptop 
6.	Open mission_control_sim.launch.py located in the “opencv_camera_cpp” package
7.	Ensure 'robot_ip': '192.168.0.194' matches the ip of the UR3e robot you are intending to connect to.
8.	Navigate to your ROS Workspace
9.	Run source /opt/ros/humble/setup.bash
10.	Run source install/setup.bash
11.	Run colcon build --symlink-install

# Terminal 1: 
13. Run this launch file ros2 launch opencv_camera_cpp mission_control_sim.launch.py 
# Terminal 2
14. ros2 launch ur_onrobot_control start_robot.launch.py   ur_type:=ur3e   robot_ip:=192.168.0.191   use_fake_hardware:=false launch_rviz:=false
 a.	Ensure that is the correct robot IP 
# Terminal 3: 
15. ros2 control set_controller_state finger_width_trajectory_controller inactive
 ros2 control set_controller_state finger_width_controller inactive
 ros2 control set_controller_state finger_width_controller active
 ros2 control list_controllers | grep finger
16.	ros2 run tf2_ros static_transform_publisher   --x 0.0 --y 0.022 --z 0.144   --roll 0 --pitch -1.5708 --yaw 0   --frame-id tool0 --child-frame-id camera_link
# Terminal 4: 
17. ros2 run ur3e_motion move_to_pose --ros-args --params-file ~/RS2_Repo/RS2_12_3_06/kinematics_onrobot.yaml
  a.	Ensure correct file path
# On GUI 
18.	Press Start to move the robot into first position 
 a.	Wait for completion 
19.	Press Locate Bottle to activate bottle detection and location node 
20.	Press Move to 2nd View to move robot to side position 
21.	Press Detect Bottle to activate bottle detection and categorising node 
22.	Press Complete Mission conduct final actions 
 a.	Pick up bottle 
 b.	Drop bottle off 
