/*
 * Copyright: (C) 2019 CRISP, Advanced Robotics at Queen Mary,
 *                Queen Mary University of London, London, UK
 * Author: Rodrigo Neves Zenha <r.neveszenha@qmul.ac.uk>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 *
 */
/**
 * \file grid_map_plotter.cpp
 *
 * \author Rodrigo Neves Zenha
 * \copyright  Released under the terms of the GNU GPL v3.0.
 */

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "uskin_ros_publisher/uskinFrame.h"
#include <cmath>

using namespace grid_map;

ros::Publisher publisher;

void ploterCallback(const uskin_ros_publisher::uskinFrame &msg)
{
  // Create grid map.
  map.at("elevation", *it) = -1 * msg.frame[(index(1)) * 4 + (index(0))].point.z;
  map.at("normal_x", *it) = msg.frame[(index(1)) * 4 + (index(0))].point.y * 10 / vector_lenght;
  map.at("normal_y", *it) = -1 * msg.frame[(index(1)) * 4 + (index(0))].point.x * 10 / vector_lenght;
  map.at("normal_z", *it) = msg.frame[(index(1)) * 4 + (index(0))].point.z * 10 / vector_lenght;

  ROS_INFO("Printing node %s at position x:%i, y:%i, with value %f", msg.frame[(index(1)) * 4 + (index(0))].header.frame_id.c_str(), index(0), index(1), msg.frame[(index(1)) * 4 + (index(0))].point.z);
}
// Publish grid map.
map.setTimestamp(time.toNSec());
grid_map_msgs::GridMap message;
GridMapRosConverter::toMessage(map, message);
publisher.publish(message);
}

int main(int argc, char **argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_ploter_node");
  ros::NodeHandle nh("~");

  publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  ros::Subscriber sub = nh.subscribe("/uskin_xyz_values_normalized", 1000, ploterCallback);

  ros::spin();
  // Wait for next cycle.

  return 0;
}
