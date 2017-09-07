testing_gmapping
===============

Author: Eric Cristofalo

Affiliation: Stanford University

Date Created: 2017/09/05; Date Last Modified: 2017/09/06

Tested on: ROS Kinetic

# Instructions
* git clone this package in your catkin_ws
* catkin build the workspace
* run: $roscore
* run: $roslaunch testing_gmapping testing_gmapping.launch

# Notes
* For context, this data comes from a Microsoft Kinect's depth map output that I sampled into a "laser scan." Although not as ideal a real laser scanner, this is good enough for the gmapping package. 
* The map topic will show up as "undefined" in RViz initially. Give it a minute to populate since the scans are very slow compared to the odometry messages (there are only five scans total). 
* The overall map doesn't actually look that good and the final estimate robot pose is wrong. This is probably because this was one of the first rosbags I made and I didn't sample the kinect enough. It's just a reference to set up gmapping!

# Requirements
* gmapping

# Launch Files
* testing_gmapping.launch
  * Runs rosbag for gmapping data, sets appropriate tf frames, and launches gmapping itself. See this file's comments for details. 

# Data Files
* testing_gmapping.bag
	* Rosbag containing a scan, pose, and odometry topic. 
* reference_tf_frames.pdf
	* Example tf diagram output using: $rosrun tf view_frames. Your gmapping setup should mimic this one. 
* reference_gmapping_output.png
	* Example screen shot of final gmapping map in RViz. Your gmapping result should look basically like this one.

# RViz Files
* testing_gmapping.rviz
	* Just an RViz environment I configured for this tutorial. 

# Source Files
* tf_rebroadcaster.cpp
	* Re-broadcasts tf frames from raw data. I need this to replicate the tf's published in a read experiment.
    * display_data_flag: integer that indicates if data should be output in terminal
    * input_data_type_index: index that determines subscribed topic. 0=geometry_msg/PoseStamped, 1=nav_msgs/Odometry
    * source_frame: string of parent tf frame
    * destination_framesource_frame: string of child tf frame

