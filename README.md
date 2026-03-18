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
