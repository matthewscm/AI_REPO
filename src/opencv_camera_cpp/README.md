# Purpose
 This subsystem orchestrates the main pipeline for the robot bottle detection and collections. It is the main integrator of all other subsystems and is responsible for the user interface. The launch file sequences the startup of a RealSense Camera, Perception and Mapping and Advanced Capabilities. They are staggered with timer delays to ensure clean initialisation and act as the mission control.
 Once running, the camera node is responsible for visualising the perception and mapping node as it subscribes to the raw RealSense image stream alongside bottle classification and centroid topics, overlays visual feedback onto each frame, and republishes the annotated image. Together with the static environment modelling node, the classifier, and the mission control GUI, the subsystem enables all subsystems to integrate.
 The subsystem also includes the development of custom fingers to be attached to the gripper to allow for seamless bottle pick up.  

# How to run or test the subsystem independently

## Camera Feed Visualisation:
This test reads sample inputs of bottle identification and location of the identified bottle. The data changes the XY location and the colour of the bottle so it should be clearly visualised. 
 ### BUILD
  a.	cd ~/ros2_ws
  b.	colcon build --symlink-install
  c.	source install/setup.bash
 ### REAL SENSE
  a.	source install/setup.bash
  b.	ros2 launch realsense2_camera rs_launch.py
 ### RUN 
  a.	source install/setup.bash
  b.	ros2 run opencv_camera_cpp camera_node
 ### TEST PUBLISHER
  a.	source install/setup.bash
  b.	ros2 run opencv_camera_cpp bottle_test_publisher

## Mission Control Gui: 
The node will display the gui with certain buttons. The topic echo should represent the output of the button pressed, 0=Home, 1=Start, 2=Stop, 3=Resume etc. 
 ### BUILD
  a.	cd ~/ros2_ws
  b.	colcon build --symlink-install
  c.	source install/setup.bash
 ### MISSION CONTROL GUI
  a.	ros2 run opencv_camera_cpp mission_control_gui
 ### TEST
  a.	ros2 topic echo /system_command

 

 
