#ifndef USKINPUBLISHERNODE_H
#define USKINPUBLISHERNODE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/thread/thread.hpp>
#include "uskin_publisher/uskinCanDriver.h"

#include <sstream>

#define int publisher_command;

#define START_PUBLISH 100
#define STOP_PUBLISH 101

void input_read(std::string input);

void user_input(int *publish_rate);

#endif
