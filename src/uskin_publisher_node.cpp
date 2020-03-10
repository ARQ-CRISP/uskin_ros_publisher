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
#include "uskin_ros_publisher/KeyboardCommand.h"

std::string log_file_path;
std::string csv_normalized_data_file_path;
std::string csv_data_file_path;
bool is_publishing = false;
bool is_first_data_received = false;
UskinSensor *uskin;
_uskin_node_time_unit_reading *instant_frame_data;
std::ofstream csv_file;

void do_stuff()
{

  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();

  int count = 0; // Message sequence counter

  ros::Publisher uskin_xyz_publisher = n->advertise<uskin_ros_publisher::uskinFrame>("/uskin_xyz_values", 1000);
  ros::Publisher normalized_uskin_xyz_publisher = n->advertise<uskin_ros_publisher::uskinFrame>("/uskin_xyz_values_normalized", 1000);

  // ros::Publisher pub_b = node->advertise<std_msgs::Empty>("topic_b", 10);
  while (!is_first_data_received)
  {
  }
  ros::Rate rate(180);
  while (ros::ok())
  {
    uskin_ros_publisher::uskinFrame uskin_frame_reading_msg, normalized_uskin_frame_reading_msg;
    is_publishing = true;
    // ROS_ERROR("Timer!!!!!");// Set message objects with new readings and publish it

    // Set message objects with normalized readings and publish it
    constructMessages(&uskin_frame_reading_msg, &normalized_uskin_frame_reading_msg, uskin, count, constructPointStamped, constructPointStampedNormalized);
    uskin_xyz_publisher.publish(uskin_frame_reading_msg);
    normalized_uskin_xyz_publisher.publish(normalized_uskin_frame_reading_msg);
    ;
    // pub_b.publish(msg);
    is_publishing = false;
    count++;
    rate.sleep();
  }
}

// bool process_keyboard_command(uskin_ros_publisher::KeyboardCommand::Request &req, uskin_ros_publisher::KeyboardCommand::Response &res)
// {
//   std::stringstream ss;

//   switch (req.command)
//   {
//   case START_RECORD:

//     ss << "OK " << req.command;
//     res.message = ss.str();

//     ROS_ERROR("sending back response: %s", res.message.c_str());
//     break;
//     // case END_RECORD:
//     //   /* code */
//     //   break;
//     // case SLIP:
//     //   /* code */
//     //   break;
//     // case NO_SLIP:
//     //   /* code */
//     //   break;
//     // case CALIBRATE:
//     //   /* code */
//     //   break;

//   default:
//     break;
//   }

//   return true;
// }

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

void constructPointStampedNormalized(geometry_msgs::PointStamped *uskin_node_reading_msg, _uskin_node_time_unit_reading *current_node_reading)
{
  std::stringstream node_id_str;
  node_id_str << std::hex << current_node_reading->node_id;

  uskin_node_reading_msg->header.frame_id = node_id_str.str();
  uskin_node_reading_msg->point.x = current_node_reading->x_value_normalized;
  uskin_node_reading_msg->point.y = current_node_reading->y_value_normalized;
  uskin_node_reading_msg->point.z = current_node_reading->z_value_normalized;
  return;
}

void constructUskinFrame(uskin_ros_publisher::uskinFrame *uskin_frame_reading_msg, int sequence)
{
  uskin_frame_reading_msg->header.seq = sequence;
  uskin_frame_reading_msg->header.stamp = ros::Time::now();
  return;
}

void constructMessage(uskin_ros_publisher::uskinFrame *uskin_frame_reading_msg, UskinSensor *uskin, int sequence, void (*contructMessage)(geometry_msgs::PointStamped *, _uskin_node_time_unit_reading *))
{
  // constructPointStamped for each node reading
  int number_of_nodes = uskin->GetUskinFrameSize();

  for (int i = 0; i < number_of_nodes; i++)
  {
    geometry_msgs::PointStamped uskin_node_reading_msg;

    _uskin_node_time_unit_reading *current_node_reading;

    current_node_reading = uskin->GetNodeData_xyzValues(i);

    // ROS_INFO("%s", current_node_reading->to_str().c_str());

    contructMessage(&uskin_node_reading_msg, current_node_reading);
    uskin_frame_reading_msg->frame.push_back(uskin_node_reading_msg);

    ROS_INFO_STREAM("Values for element: " << std::to_string(i) << " " << uskin_node_reading_msg.point.x << " " << uskin_node_reading_msg.point.y << " " << uskin_node_reading_msg.point.z);
  }
  // constructUskinFrame for frame reading (containing all node readings)
  constructUskinFrame(uskin_frame_reading_msg, sequence);

  return;
}

void constructMessages(uskin_ros_publisher::uskinFrame *uskin_frame_reading_msg, uskin_ros_publisher::uskinFrame *uskin_frame_reading_normalized_msg, UskinSensor *uskin, int sequence, void (*contructMessage)(geometry_msgs::PointStamped *, _uskin_node_time_unit_reading *), void (*contructMessage2)(geometry_msgs::PointStamped *, _uskin_node_time_unit_reading *))
{
  // constructPointStamped for each node reading
  int number_of_nodes = uskin->GetUskinFrameSize();

  for (int i = 0; i < number_of_nodes; i++)
  {

    geometry_msgs::PointStamped uskin_node_reading_msg;
    geometry_msgs::PointStamped uskin_node_reading_normalized_msg;

    contructMessage(&uskin_node_reading_msg, &instant_frame_data[i]);
    uskin_frame_reading_msg->frame.push_back(uskin_node_reading_msg);

    contructMessage2(&uskin_node_reading_normalized_msg, &instant_frame_data[i]);
    uskin_frame_reading_normalized_msg->frame.push_back(uskin_node_reading_normalized_msg);

    ROS_INFO_STREAM("Values for element: " << std::to_string(i) << " " << uskin_node_reading_msg.point.x << " " << uskin_node_reading_msg.point.y << " " << uskin_node_reading_msg.point.z);
  }
  // constructUskinFrame for frame reading (containing all node readings)
  constructUskinFrame(uskin_frame_reading_msg, sequence);
  constructUskinFrame(uskin_frame_reading_normalized_msg, sequence);

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

  //int number_of_subscribers = 0;

  ros::init(argc, argv, "uskin_publisher_node");

  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();

  retrieveLogPathMode(*n);
  retrieveNormalizedDataCSVPathMode(*n);
  retrieveDataCSVPathMode(*n);

  // ros::ServiceServer service = n->advertiseService("keyboard_command", process_keyboard_command);

  // ros::Rate loop_rate(1);

  // UskinSensor *uskin = boost::make_shared<UskinSensor>;
  uskin = new UskinSensor(log_file_path); // Will connect to "can0" network  and device "0x201" by default

  ROS_INFO("Let's start the sensor");

  // Starting the sensor
  if (!uskin->StartSensor())
  {
    ROS_ERROR("Problems initiating the sensor!");
    delete uskin;

    return (-1);
  }

  // Calibrate the sensor
  uskin->CalibrateSensor();

  time_t timer;
  struct tm *timeinfo;
  char csv_name[20];
  time(&timer);
  timeinfo = localtime(&timer);

  strftime(csv_name, 20, "%F-%H-%M-%S", timeinfo);

  // Open file with provided filename and timestamp
  csv_file.open(log_file_path + "_" + std::string(csv_name) + ".calibration.csv");

  csv_file << "CanID,";

  csv_file << "X Min Value, X Max Value, Y Min Values, Y Max Values, Z Min Values, Z Max Values";

  csv_file << std::endl;

  frame_min_reads = uskin->getCalibrationValues();

  for (int i = 0; i < uskin->GetUskinFrameSize(); i++)
  {
    int canID = uskin->convertIndextoCanID(i);
    csv_file
        << canID << "," << frame_min_reads[i][0] << ","
        << "45000"
        << "," << frame_min_reads[i][1] << ","
        << "25000"
        << "," << frame_min_reads[i][2] << ","
        << "25000";
    csv_file << std::endl;
  }

  csv_file.close();

  // uskin->SaveData(csv_data_file_path);
  // uskin->SaveNormalizedData(csv_normalized_data_file_path);

  ROS_INFO("Let's retrieve some data");

  // spawn another thread
  std::thread worker(do_stuff);

  while (ros::ok() && !ros::isShuttingDown())
  {
    uskin_time_unit_reading instant_reading;
    ros::master::check();

    // Message objects to be published
    // Retrieve new data
    while (is_publishing)
    {
    }
    uskin->RetrieveFrameData();
    uskin->NormalizeData();

    instant_frame_data = uskin->GetFrameData();

    is_first_data_received = true;

    ROS_INFO("New uskin Frame data sucessfuly retrieved\n");

    // ros::spinOnce();
  }

  uskin->StopSensor();
  delete uskin;

  return 0;
}