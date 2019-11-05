#include "ros/ros.h"
#include "uskin_ros_publisher/uskinFrame.h"
#include "uskin_can_drivers/include/uskinCanDriver.h"

int main(int argc, char **argv)
{

  UskinSensor *uskin = new UskinSensor; // Will connect to "can0" device by default
  time_t timer;
  struct tm *timeinfo;
  char csv_name[20];
  int count = 0;
  int number_of_nodes;

  ros::init(argc, argv, "uskin_publisher_node");

  ros::NodeHandle n;

  ros::Publisher uskin_xyz_publisher = n.advertise<uskin_ros_publisher::uskinFrame>("/uskin_xyz_values", 1000);

  ros::Rate loop_rate(1);

  // Used if we wish to filter incoming messages
  /*  struct can_filter rfilter[1];

  rfilter[0].can_id = 0x135;
  rfilter[0].can_mask = CAN_SFF_MASK; */
  //rfilter[1].can_id   = 0x101;
  //rfilter[1].can_mask = CAN_SFF_MASK;

  time(&timer);
  timeinfo = localtime(&timer);
  strftime(csv_name, 20, "%F_%T", timeinfo);


  ROS_INFO("Let's start the sensor");

  if (!uskin->StartSensor())
  {
    ROS_ERROR("Problems initiating the sensor!");
    delete uskin;

    return (-1);
  }

  uskin->CalibrateSensor(); // Calibrates sensor and data is stored normalized

  uskin->SaveData("uskin_data_" + std::string(csv_name) + ".csv"); //Only called once, will save output to a csv file

  ROS_INFO("Let's retrieve some data");

  while (ros::ok() && !ros::isShuttingDown())
  {

    ros::master::check();
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    uskin_ros_publisher::uskinFrame uskin_frame_reading_msg;
    geometry_msgs::PointStamped uskin_node_reading_msg;

    /* if (count == 0)
    { */
    uskin->RetrieveFrameData(); // Get data. If SavaData was called, data is stored in file

    //ROS_ERROR("New uskin Frame data sucessfuly retrieved\n");
    number_of_nodes = uskin->GetUskinFrameSize();
    for (int i = 0; i < number_of_nodes; i++)
    {
      _uskin_node_time_unit_reading *current_node_reading;

      current_node_reading = uskin->GetNodeData_xyzValues(i);

      ROS_INFO("%s", current_node_reading->to_str().c_str());

      std::stringstream node_id_str;
      node_id_str << std::hex << current_node_reading->node_id;

      uskin_node_reading_msg.header.frame_id = node_id_str.str();
      uskin_node_reading_msg.point.x = current_node_reading->x_value;
      uskin_node_reading_msg.point.y = current_node_reading->y_value;
      uskin_node_reading_msg.point.z = current_node_reading->z_value;

      uskin_frame_reading_msg.header.seq = count;
      uskin_frame_reading_msg.header.stamp = ros::Time::now();
      uskin_frame_reading_msg.frame.push_back(uskin_node_reading_msg);
    }
    uskin_xyz_publisher.publish(uskin_frame_reading_msg);
    // }

    ros::spinOnce();

    count++;
  }

  uskin->StopSensor();
  delete uskin;

  return 0;
}