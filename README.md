demo_ros
========
This package is used as a ROS demonstration. To reproduce the actual package, you can follow these steps :

1) Install ROS (http://wiki.ros.org/ROS/Installation)

2) Install OpenNi drivers with the command : sudo apt-get install ros-hydro-openni-camera ros-hydro-openni-launch (http://wiki.ros.org/openni_kinect)

3) Follow ROS tutorials (http://wiki.ros.org/ROS/Tutorials) to create a ROS workspace. Clone this repository in your package directory. Build the project with the command catkin_make at the root of the workspace.

4) This project uses my own calibration files for one particular kinect. You may modify the kinect.launch to use none or replace them by your own. See http://wiki.ros.org/openni_launch/Tutorials to calibrate your kinect.

5) The plane segmentation code of this package is based on the tutorial available at http://wiki.ros.org/pcl/Tutorials, but modified to work with ROS Hydro. Instead of only outputting the plane coefficients like in the example, the planes are extracted in real time in a similar way to http://www.pointclouds.org/documentation/tutorials/extract_indices.php .You can see the results in RVIZ from the topic "extracted_planes" or directly from the pcl viewer created by the program, where the extracted planes are shown in red.

6) To run the example (in different terminals):  

    roslaunch kinect.launch
    rosrun demo_ros plane_segmentation
