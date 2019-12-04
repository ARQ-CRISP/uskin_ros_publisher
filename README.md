# uskin_ros_publisher
ROS package for publishing data being retrieved by uSkin sensor. (https://www.xelarobotics.com/product_XR1946?lang=en)  
Real-time data visualization is also supported on RViz using the Grid Map ROS package. (https://github.com/ANYbotics/grid_map)

## Dependencies
This repository is dependent on ARQ-CRISP/ros_can_drivers files and they are included here as a gitsubmodule. After cloning this repo, run:  
`git submodule init`

And to get lastest updates on ros_can_drivers simply run:  
`git submodule update --init --recursive`

### Notes:
- **ARQ-CRISP/ros_can_drivers repository has its own dependencies which need to be installed. Refer to https://github.com/ARQ-CRISP/uskin_can_drivers**


- If you make changes to uskin_can_drivers submodule, **do not forget** to push your commits:  
`cd <submodule_dir>`  
`git add <something>`  
`git commit`  
`git push`  
`cd ..` # Back to main repository  
`git add <something>`  
`git commit`  
`git push`  
or simply:  
`git push --recurse-submodules=on-demand` # From main repository
  **You must still commit your changes on the submodule beforehand.**
 
## Real-Time Data Visualization

- Installing Grid Map ROS package:    
`sudo apt-get install ros-$ROS_DISTRO-grid-map`

- Launching the necessary software:  
`roslaunch uskin_ros_publisher grid_map_visualization.launch`

## Data storage

- Current version allows you to store the sensor data into a .csv file which is named after the timestamp when it is opened, and stored in *csv_files* folder. uskin_can_drivers logs are being stored under *log_files* folder.

- You are able to record a ROS bag of observation samples being published to topic */uskin_xyz_values* by running:  
`rosbag record /uskin_xyz_values`  
And play it back by running:  
`rosbag play example.bag` 

