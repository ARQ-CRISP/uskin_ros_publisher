#include "uskin_publisher/uskin_publisher_node.h"

#include <string>
#include <stdio>

publisher_command command;

void input_read(std::string input)
{
  std::cout << "Please, enter your command: ";
  std::getline(std::cin, input);
  return;
}

bool get_command(std::string input, int *command)
{
  if (input.compare("start") == 0)
  {
    command == START_PUBLISH;
    return true;
  }
  else if (input.compare("stop") == 0)
  {
    command == STOP_PUBLISH;
    return true;
  }
  else if (input.compare("help") == 0)
  {
    std::cout << "Type 'start' to start data transmission, or 'stop' to stop it.";
    return true;
  }

  return false;
}

void user_input()
{

  std::string input;

  while (ros::ok())
  {

    input.clear();
    input_read(input);

    if (!get_command(input, &command))
      std::cout << "Invalid command. Type 'help' to check possible commands.";
  }
}

void sensor_input()
{
  //ros::Publisher uskin_publish = n.advertise<std_msgs::String>("chatter", 1000);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  UskinCanDriver uskin;
  ros::init(argc, argv, "talker");

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();

  ros::Rate loop_rate(10);

  uskin.open_connection();

  uskin.request_data();

  // spawn another thread
  boost::thread thread_read_stdin(user_input), thread_read_sensor(sensor_input);

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
