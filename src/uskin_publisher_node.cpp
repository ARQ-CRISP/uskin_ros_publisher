#include "ros/ros.h"
#include "uskin_ros_publisher/uskinFrame.h"
#include "uskin_can_drivers/uskinCanDriver.h"


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "uskin_publisher_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Publisher uskin_xyz_publisher = n.advertise<uskin_ros_publisher::uskinFrame>("uskin_x_values", 1000);

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok() && !ros::isShuttingDown())
  {

    ros::master::check();
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    uskin_ros_publisher::uskinFrame uskin_frame_reading_msg;
    geometry_msgs::PointStamped uskin_node_reading_msg;

    
    node.header.frame_id = "100";
    node.point.x = 10;
    node.point.y = 20;
    node.point.z = 1000;

    frame_x.header.seq = count;
    frame_x.header.stamp = ros::Time::now();
    frame_x.header.frame_id = "1";
    frame_x.frame.push_back(node);
    
    //ROS_INFO("%s", msg.data.c_str());

    frame_x_values_publisher.publish(frame_x);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}