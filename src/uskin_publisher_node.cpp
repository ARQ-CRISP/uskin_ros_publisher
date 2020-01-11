/*
 * Copyright: (C) 2019 CRISP, Advanced Robotics at Queen Mary,
 *                Queen Mary University of London, London, UK
 * Author: Rodrigo Neves Zenha <r.neveszenha@qmul.ac.uk>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 *
 */
/**
 * \file uskin_publisher_node.cpp
 *
 * \author Rodrigo Neves Zenha
 * \copyright  Released under the terms of the GNU GPL v3.0.
 */

#include "uskin_ros_publisher/uskin_publisher_node.h"

std::string log_file_path;
std::string csv_normalized_data_file_path;
std::string csv_data_file_path;

void constructPointStamped(geometry_msgs::PointStamped *uskin_node_reading_msg, _uskin_node_time_unit_reading *current_node_reading)
{
  std::stringstream node_id_str;
  node_id_str << std::hex << current_node_reading->node_id;

  uskin_node_reading_msg->header.frame_id = node_id_str.str();
  uskin_node_reading_msg->point.x = current_node_reading->x_value;
  uskin_node_reading_msg->point.y = current_node_reading->y_value;
  uskin_node_reading_msg->point.z = current_node_reading->z_value;
  return;
}

void constructUskinFrame(uskin_ros_publisher::uskinFrame *uskin_frame_reading_msg, int sequence)
{
  uskin_frame_reading_msg->header.seq = sequence;
  uskin_frame_reading_msg->header.stamp = ros::Time::now();
  return;
}

void constructMessage(uskin_ros_publisher::uskinFrame *uskin_frame_reading_msg, UskinSensor *uskin, int sequence)
{
  // constructPointStamped for each node reading
  int number_of_nodes = uskin->GetUskinFrameSize();
  for (int i = 0; i < number_of_nodes; i++)
  {
    geometry_msgs::PointStamped uskin_node_reading_msg;

    _uskin_node_time_unit_reading *current_node_reading;

    current_node_reading = uskin->GetNodeData_xyzValues(i);

    ROS_INFO("%s", current_node_reading->to_str().c_str());

    constructPointStamped(&uskin_node_reading_msg, current_node_reading);
    uskin_frame_reading_msg->frame.push_back(uskin_node_reading_msg);

    ROS_INFO_STREAM("Values for element: " << std::to_string(i) << " " << current_node_reading->x_value << " " << current_node_reading->y_value << " " << current_node_reading->z_value);
  }
  // constructUskinFrame for frame reading (containing all node readings)
  constructUskinFrame(uskin_frame_reading_msg, sequence);

  return;
}

void retrieveLogPathMode(ros::NodeHandle n)
{
  ROS_INFO("  >> retrieveLogPathMode()");

  if (!n.hasParam("/uskin_publisher/log_file_path"))
  {
    ROS_ERROR("    It was not possible to retrieve Log Path from parameter server!! Shutting down...");

    ros::shutdown();
  }
  else
  {
    n.getParam("/uskin_publisher/log_file_path", log_file_path);

    ROS_INFO("      Found Log Path: %s", log_file_path.c_str());
  }

  ROS_INFO("  << retrieveLogPathMode()");

  return;
}

void retrieveDataCSVPathMode(ros::NodeHandle n)
{
  ROS_INFO("  >> retrieveDataCSVPathMode()");

  if (!n.hasParam("/uskin_publisher/csv_data_file_path"))
  {
    ROS_ERROR("    It was not possible to retrieve CSV path from parameter server!! Shutting down...");

    ros::shutdown();
  }
  else
  {
    n.getParam("/uskin_publisher/csv_data_file_path", csv_data_file_path);

    ROS_INFO("      Found CSV path: %s", csv_data_file_path.c_str());
  }

  ROS_INFO("  << retrieveDataCSVPathMode()");

  return;
}

void retrieveNormalizedDataCSVPathMode(ros::NodeHandle n)
{
  ROS_INFO("  >> retrieveNormalizedDataCSVPathMode()");

  if (!n.hasParam("/uskin_publisher/csv_normalized_data_file_path"))
  {
    ROS_ERROR("    It was not possible to retrieve CSV Path from parameter server!! Shutting down...");

    ros::shutdown();
  }
  else
  {
    n.getParam("/uskin_publisher/csv_normalized_data_file_path", csv_normalized_data_file_path);

    ROS_INFO("      Found CSV Path: %s", csv_normalized_data_file_path.c_str());
  }

  ROS_INFO("  << retrieveNormalizedDataCSVPathMode()");

  return;
}

int main(int argc, char **argv)
{

  int count = 0; // Message sequence counter
  int number_of_nodes;
  //int number_of_subscribers = 0;

  ros::init(argc, argv, "uskin_publisher_node");

  ros::NodeHandle n;

  retrieveLogPathMode(n);
  retrieveNormalizedDataCSVPathMode(n);
  retrieveDataCSVPathMode(n);

  ros::Publisher uskin_xyz_publisher = n.advertise<uskin_ros_publisher::uskinFrame>("/uskin_xyz_values", 1000);
  ros::Publisher normalized_uskin_xyz_publisher = n.advertise<uskin_ros_publisher::uskinFrame>("/uskin_xyz_values_normalized", 1000);

  ros::Rate loop_rate(1);

  UskinSensor *uskin = new UskinSensor(log_file_path); // Will connect to "can0" network  and device "0x201" by default

  // Later could be used to filter incoming messages
  /*  struct can_filter rfilter[1];

  rfilter[0].can_id = 0x135;
  rfilter[0].can_mask = CAN_SFF_MASK; */
  //rfilter[1].can_id   = 0x101;
  //rfilter[1].can_mask = CAN_SFF_MASK;

  ROS_INFO("Let's start the sensor");

  // Check if there are any subscribers to uskin_xyz_publisher
  // while (number_of_subscribers == 0)
  // {
  //   number_of_subscribers = uskin_xyz_publisher.getNumSubscribers();
  // }

  // Starting the sensor
  if (!uskin->StartSensor())
  {
    ROS_ERROR("Problems initiating the sensor!");
    delete uskin;

    return (-1);
  }


  // Calibrate the sensor
  uskin->CalibrateSensor();

  // uskin->SaveData("../csv_files/uskin_data_");
  uskin->SaveData(csv_data_file_path);
  uskin->SaveNormalizedData(csv_normalized_data_file_path);

  ROS_INFO("Let's retrieve some data");

  while (ros::ok() && !ros::isShuttingDown())
  {

    ros::master::check();
    // Message objects to be published
    uskin_ros_publisher::uskinFrame uskin_frame_reading_msg, normalized_uskin_frame_reading_msg;
    // Retrieve new data
    uskin->RetrieveFrameData();

    ROS_INFO("New uskin Frame data sucessfuly retrieved\n");

    // Set message objects with new readings and publish it
    constructMessage(&uskin_frame_reading_msg, uskin, count);
    uskin_xyz_publisher.publish(uskin_frame_reading_msg);

    // Normalize and publish data
    uskin->NormalizeData();

    // Set message objects with normalized readings and publish it
    constructMessage(&normalized_uskin_frame_reading_msg, uskin, count);
    normalized_uskin_xyz_publisher.publish(normalized_uskin_frame_reading_msg);

    ros::spinOnce();

    count++;
  }

  uskin->StopSensor();
  delete uskin;

  return 0;
}