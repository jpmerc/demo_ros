demo_ros
========
This package is used as a ROS demonstration. To reproduce the actual package, you can follow these steps :

1) Install ROS (http://wiki.ros.org/ROS/Installation)

2) Follow the tutorials (http://wiki.ros.org/ROS/Tutorials) to create a ROS workspace and ROS packages

3) To use the Kinect, select the desired drivers from the options given at http://wiki.ros.org/kinect . In this case, I used OpenNi drivers, since the version of freenect available on ROS for the moment is 0.3.2 and the fix for the new kinect model (1473) is version 0.4.0. For more information on how to use OpenNI, refer to http://wiki.ros.org/openni_launch. 

4) If you need to calibrate the Kinect, refer to http://wiki.ros.org/openni_launch/Tutorials

5) The plane segmentation code is based on the tutorial available at http://wiki.ros.org/pcl/Tutorials, but modified to work with ROS Hydro. Instead of only outputting the plane coefficients like in the example, the planes are extracted in real time in a similar way to http://www.pointclouds.org/documentation/tutorials/extract_indices.php .You can see the results in RVIZ from the topic "extracted_planes" or directly from the pcl viewer created by the program. The extracted planes are shown in red.

6) To run the example :  rosrun demo_ros plane_segmentation
