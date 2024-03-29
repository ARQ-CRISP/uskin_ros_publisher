/*
 * Copyright: (C) 2019 CRISP, Advanced Robotics at Queen Mary,
 *                Queen Mary University of London, London, UK
 * Author: Rodrigo Neves Zenha <r.neveszenha@qmul.ac.uk>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 *
 */
/**
 * \file uskin_publisher_node.h
 *
 * \author Rodrigo Neves Zenha
 * \copyright  Released under the terms of the GNU GPL v3.0.
 */

#ifndef USKINPUBLISHERNODE_H
#define USKINPUBLISHERNODE_H

#include "ros/ros.h"
#include "uskin_ros_publisher/uskinFrame.h"
#include "uskin_can_drivers/include/uskinCanDriver.h"
#include <thread>

#define CALIBRATE_SENSOR 4

void timerCallback(const ros::TimerEvent&);

void constructPointStamped(geometry_msgs::PointStamped *uskin_node_reading_msg, _uskin_node_time_unit_reading *current_node_reading);

void constructPointStampedNormalized(geometry_msgs::PointStamped *uskin_node_reading_msg, _uskin_node_time_unit_reading *current_node_reading);

void constructUskinFrame(uskin_ros_publisher::uskinFrame *uskin_frame_reading_msg, int sequence);

void constructMessage(uskin_ros_publisher::uskinFrame *uskin_frame_reading_msg, UskinSensor *uskin, int sequence, void (*contructMessage)(geometry_msgs::PointStamped *, _uskin_node_time_unit_reading *));

void constructMessages(uskin_ros_publisher::uskinFrame *uskin_frame_reading_msg, uskin_ros_publisher::uskinFrame *uskin_frame_reading_normalized_msg, UskinSensor * uskin, int sequence, void (*contructMessage)(geometry_msgs::PointStamped *, _uskin_node_time_unit_reading *), void (*contructMessage2)(geometry_msgs::PointStamped *, _uskin_node_time_unit_reading *));

#endif