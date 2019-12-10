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
<<<<<<< HEAD

//static unsigned long int **frame_min_reads;
//static int frame_min_reads_size;

ros::Publisher publisher;

void ploterCallback(const uskin_ros_publisher::uskinFrame &msg)
{
  // Create grid map.
=======

ros::Publisher publisher;

void chatterCallback(const uskin_ros_publisher::uskinFrame &msg)
{  // Create grid map.
>>>>>>> parent of bc9a43e... Temporary (unstable) version of the publisher for both raw and normalized data publishing
  GridMap map({"elevation", "normal_x", "normal_y", "normal_z"});
  map.setFrameId("map");
  map.setGeometry(Length(120, 180), 30);
  //ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
  //         map.getLength().x(), map.getLength().y(),
  //         map.getSize()(0), map.getSize()(1));

  // Add data to grid map.
  ros::Time time = ros::Time::now();
  for (GridMapIterator it(map); !it.isPastEnd(); ++it)
  {
    Position position;
    map.getPosition(*it, position);
    Index index;
    map.getIndex(position, index);
    float vector_lenght = sqrt(pow(msg.frame[(index(1)) * 4 + (index(0))].point.x, 2) + pow(msg.frame[(index(1)) * 4 + (index(0))].point.y, 2) + pow(msg.frame[(index(1)) * 4 + (index(0))].point.z, 2));
<<<<<<< HEAD
    map.at("elevation", *it) = -1 * msg.frame[(index(1)) * 4 + (index(0))].point.z;
    map.at("normal_x", *it) = msg.frame[(index(1)) * 4 + (index(0))].point.y * 10 / vector_lenght;
    map.at("normal_y", *it) = -1 * msg.frame[(index(1)) * 4 + (index(0))].point.x * 10 / vector_lenght;
    map.at("normal_z", *it) = msg.frame[(index(1)) * 4 + (index(0))].point.z * 10 / vector_lenght;

=======
    map.at("elevation", *it) = - 1* msg.frame[(index(1)) * 4 + (index(0))].point.z;
    map.at("normal_x", *it) = msg.frame[(index(1)) * 4 + (index(0))].point.y * 10/ vector_lenght;
    map.at("normal_y", *it) = -1 * msg.frame[(index(1)) * 4 + (index(0))].point.x *10 / vector_lenght;
    map.at("normal_z", *it) = msg.frame[(index(1)) * 4 + (index(0))].point.z * 10/ vector_lenght;
>>>>>>> parent of bc9a43e... Temporary (unstable) version of the publisher for both raw and normalized data publishing
    ROS_INFO("Printing node %s at position x:%i, y:%i, with value %f", msg.frame[(index(1)) * 4 + (index(0))].header.frame_id.c_str(), index(0), index(1), msg.frame[(index(1)) * 4 + (index(0))].point.z);
  }
  // Publish grid map.
  map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(map, message);
  publisher.publish(message);
<<<<<<< HEAD
=======
  // ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
>>>>>>> parent of bc9a43e... Temporary (unstable) version of the publisher for both raw and normalized data publishing
}

int main(int argc, char **argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_ploter_node");
  ros::NodeHandle nh("~");
<<<<<<< HEAD

  publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  ros::Subscriber sub = nh.subscribe("/uskin_xyz_values_normalized", 1000, ploterCallback);
=======
  
  publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // ros::Subscriber sub = nh.subscribe("uskin_xyz_values", 1000, boost::bind(chatterCallback,_1,&publisher));
  ros::Subscriber sub = nh.subscribe("/uskin_xyz_values", 1000, chatterCallback);
>>>>>>> parent of bc9a43e... Temporary (unstable) version of the publisher for both raw and normalized data publishing

  ros::spin();
  // Wait for next cycle.

  return 0;
}
