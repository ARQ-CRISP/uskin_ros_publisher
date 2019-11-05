#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "uskin_ros_publisher/uskinFrame.h"
#include <cmath>

using namespace grid_map;

ros::Publisher publisher;

void chatterCallback(const uskin_ros_publisher::uskinFrame &msg)
{
  // Create grid map.
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
    map.at("elevation", *it) = msg.frame[(index(1)) * 4 + (index(0))].point.z;
    map.at("normal_x", *it) = msg.frame[(index(1)) * 4 + (index(0))].point.y * 10/ vector_lenght;
    map.at("normal_y", *it) = -1 * msg.frame[(index(1)) * 4 + (index(0))].point.x *10 / vector_lenght;
    map.at("normal_z", *it) = msg.frame[(index(1)) * 4 + (index(0))].point.z * 10/ vector_lenght;
    ROS_INFO("Printing node %s at position x:%i, y:%i, with value %f", msg.frame[(index(1)) * 4 + (index(0))].header.frame_id.c_str(), index(0), index(1), msg.frame[(index(1)) * 4 + (index(0))].point.z);
  }

  // Publish grid map.
  map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(map, message);
  publisher.publish(message);
  // ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}

int main(int argc, char **argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_ploter_node");
  ros::NodeHandle nh("~");

  publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // ros::Subscriber sub = nh.subscribe("uskin_xyz_values", 1000, boost::bind(chatterCallback,_1,&publisher));
  ros::Subscriber sub = nh.subscribe("/uskin_xyz_values", 1000, chatterCallback);

  ros::spin();
  // Wait for next cycle.

  return 0;
}
