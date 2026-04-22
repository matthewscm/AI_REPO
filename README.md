# RS2
Repository for RS2 work

# OpenCV Camera Package 
RUN  

source install/setup.bash 
ros2 run opencv_camera_cpp camera_node 

REAL SENSE CAMERA

source install/setup.bash 
ros2 launch realsense2_camera rs_launch.py 

 
TEST PUBLISHER 

source install/setup.bash 
ros2 run opencv_camera_cpp bottle_publisher 





Workflow for movement node

-------------------------------------------------------
Terminal 1
-------------------------------------------------------

With On Gripper

cd ~/RS2_Workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.1.102 \
  use_fake_hardware:=true
  
-------------------------------------------------------
Terminal 2
-------------------------------------------------------
  
  source /opt/ros/humble/setup.bash
source ~/RS2_Workspace/install/setup.bash

ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2
  
-------------------------------------------------------
Terminal 3
-------------------------------------------------------
  
  source /opt/ros/humble/setup.bash
source ~/RS2_Workspace/install/setup.bash
colcon build --packages-select ur3e_motion
ros2 run ur3e_motion move_to_pose \
  --ros-args \
  --params-file ~/RS2_Workspace/kinematics_onrobot.yaml
  
