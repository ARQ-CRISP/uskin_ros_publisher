#!/usr/bin/env python
import rosbag
import rospy
import numpy as np
from uskin_ros_publisher.msg import uskinFrame


class TactileRecordPublisher:

    def __init__(self, bag_file_name, first_timestamp):

        self.current_index = 0
        self.initial_tactile_index = 0
        self.velocity = 1
        self.timestamps = []
        self.raw_timestamps = []
        self.messages = {"/uskin_xyz_values": [],
                         "/uskin_xyz_values_normalized": []}

        # Used to adjust initial tactile frame, if necessary
        self.aux_index_a = -1
        self.aux_index_b = -1

        print('=================================================================================')
        print('Loading Tactile data')
        print('=================================================================================')
        # try:
        bag = rosbag.Bag(bag_file_name)

        for topic, msg, t in bag.read_messages(topics=['/uskin_xyz_values']):
            self.messages["/uskin_xyz_values"].append(msg)

        for topic, msg, t in bag.read_messages(topics=['/uskin_xyz_values_normalized']):
            self.messages["/uskin_xyz_values_normalized"].append(msg)

            current_t = float(str(msg.header.stamp.secs)+"."+str(msg.header.stamp.nsecs).zfill(9))
            # Timestamps will taken from normalized data. In the future this might have to change
            self.raw_timestamps.append(current_t)

        bag.close()

        # Necessary to check if raw and normalized data have same number of messages
        if (self.checkRawNormalizedDataSync() is False):
            del self
            return

        print("Tactile timestamps range from {} to {}".format(
            self.raw_timestamps[0], self.raw_timestamps[-1]))
        print("Searching for tactile initial timestamp (at {})...".format(
            first_timestamp))
        print("Number of tactile frames is {}".format(len(self.raw_timestamps)))
        for t in self.raw_timestamps:
            # current_t = float(str(t.secs)+"."+str(t.nsecs))
            # print("Checking timestamp {}".format(current_t))
            if first_timestamp < t:
                print("Found initial timestamp {} for tactile data at: {}".format(
                    first_timestamp, t))
                self.initial_tactile_index = self.current_index
                self.resetTimestamps(t)
                # self.timestamps = np.array(self.raw_timestamps)
                # self.timestamps = (self.timestamps-first_timestamp)*1000
                break
            else:
                self.current_index += 1
        
        self.getTactileDataRaw()
        self.getTactileDataNormalized()
        self.getTactileChangingEvents()

        self.pub = rospy.Publisher(
            '/uskin_xyz_values_normalized', uskinFrame, queue_size=10)

        # except:
        #     print("Unexpected error:")
        #     del self

    def publishData(self):
        # print(len(self.messages["/uskin_xyz_values_normalized"]))
        # print(self.current_index)
        self.pub.publish(
            self.messages["/uskin_xyz_values_normalized"][self.current_index])

    def multiplyVelocity(self, factor):
        self.velocity *= factor
        self.velocity = int(self.velocity)

        if self.velocity < 1:
            self.velocity = 1

    # def divideVelocity(self):
    #     self.velocity /= 2

    def updateNextIndex(self):
        if self.current_index + self.velocity < len(self.messages["/uskin_xyz_values_normalized"]):
            self.current_index += self.velocity

    def updatePreviousIndex(self):
        if self.current_index - self.velocity >= 0:
            self.current_index -= self.velocity

    def checkRawNormalizedDataSync(self):
        print('Tactile raw data array has size: {}'.format(len(self.messages["/uskin_xyz_values"])))
        print('Tactile normalized data array has size: {}'.format(len(self.messages["/uskin_xyz_values_normalized"])))

        if (len(self.messages["/uskin_xyz_values"]) == len(self.messages["/uskin_xyz_values_normalized"])):
            print('Tactile data already have the same dimensions!')
        else:
            print('It\'s necessary to perform some syncronyzation')
            # If elemets are not in sync at the beginning, remove first elements
            while (abs(self.messages["/uskin_xyz_values"][0].header.stamp.nsecs- self.messages["/uskin_xyz_values_normalized"][0].header.stamp.nsecs)>200):
                raw_time = self.messages["/uskin_xyz_values"][0]
                norm_time = self.messages["/uskin_xyz_values_normalized"][0]
                print(raw_time.header.stamp.nsecs - norm_time.header.stamp.nsecs)
                if (raw_time.header.stamp.nsecs - norm_time.header.stamp.nsecs) > 0:
                    del self.messages["/uskin_xyz_values_normalized"][0]
                    del self.raw_timestamps[0]
                elif (raw_time.header.stamp.nsecs - norm_time.header.stamp.nsecs) < -200:
                    del self.messages["/uskin_xyz_values"][0]
                else:
                    break

            # If one array is bigger than the other, remove latter elements
            while (len(self.messages["/uskin_xyz_values"])>len(self.messages["/uskin_xyz_values_normalized"])):
                del self.messages["/uskin_xyz_values"][-1]
            while (len(self.messages["/uskin_xyz_values"])<len(self.messages["/uskin_xyz_values_normalized"])):
                del self.messages["/uskin_xyz_values_normalized"][-1]
                del self.raw_timestamps[-1]
            
        # Checking if data is in sync
        if (len(self.messages["/uskin_xyz_values"]) != len(self.messages["/uskin_xyz_values_normalized"])):
            print('Problems with data syncronization!')
        elif ( -200 > (self.messages["/uskin_xyz_values"][0].header.stamp.nsecs - self.messages["/uskin_xyz_values_normalized"][0].header.stamp.nsecs) < 200):
                print('Problems with data syncronization!')
                return False

        print('Tactile raw data array has size: {}'.format(len(self.messages["/uskin_xyz_values"])))
        print('Tactile normalized data array has size: {}'.format(len(self.messages["/uskin_xyz_values_normalized"])))
        print('Last message time difference is: {}'.format(self.messages["/uskin_xyz_values"][-1].header.stamp.nsecs - self.messages["/uskin_xyz_values_normalized"][-1].header.stamp.nsecs))
    
        return True

    def adjustInitialFrame(self):
        if self.aux_index_a is -1:
            print('Tactile adjustment: First boundary set!')
            self.aux_index_a = self.current_index

        else:
            print('Tactile adjustment: Second Boundary set!')
            self.aux_index_b = self.current_index
            amount_to_shift = self.aux_index_b - self.aux_index_a
            print('Tactile adjustment: Shifting {} indexs'.format(amount_to_shift))
            self.initial_tactile_index += amount_to_shift
            self.resetTimestamps(
                self.raw_timestamps[self.initial_tactile_index])
            # self.timestamps = np.array(self.raw_timestamps)
            # self.timestamps = (self.timestamps-self.timestamps[self.initial_tactile_index])*1000

            self.aux_index_a = -1
            self.aux_index_b = -1

    # Ground timestamps to initial_timestamo
    def resetTimestamps(self, ground_value):
        self.timestamps = np.array(self.raw_timestamps)
        self.timestamps = (self.timestamps-ground_value)*1000

    def getTactileDataNormalized(self):
        readings_no = len(self.timestamps)
        self.np_tactile_readings_normalized = np.empty((18, 3, readings_no), dtype=float)
        i = 0
        for tactile_frame in self.messages["/uskin_xyz_values_normalized"]:
            j = 0
            for tactile_node in tactile_frame.frame:
                # Remove data from damaged part of the sensor
                if tactile_node.header.frame_id not in ["130", "131", "132", "133", "134", "135"]:
                  self.np_tactile_readings_normalized[j,0, i] = tactile_node.point.x
                  self.np_tactile_readings_normalized[j,1, i] = tactile_node.point.y
                  self.np_tactile_readings_normalized[j,2, i] = tactile_node.point.z
                  j += 1
            i += 1
        return

    def getTactileDataRaw(self):
        readings_no = len(self.timestamps)
        self.np_tactile_readings_raw = np.empty((18, 3, readings_no), dtype=float)
        i = 0
        for tactile_frame in self.messages["/uskin_xyz_values"]:
            j = 0
            for tactile_node in tactile_frame.frame:
                # Remove data from damaged part of the sensor
                if tactile_node.header.frame_id not in ["130", "131", "132", "133", "134", "135"]:
                  self.np_tactile_readings_raw[j,0, i] = tactile_node.point.x
                  self.np_tactile_readings_raw[j,1, i] = tactile_node.point.y
                  self.np_tactile_readings_raw[j,2, i] = tactile_node.point.z
                  j += 1
            i += 1
        return

    def getTactileChangingEvents(self):
        readings_no = len(self.timestamps)
        self.np_tactile_changes_label = np.zeros(readings_no, dtype=int)
        self.tactile_readings_diff = []

        # Get tactile image difference. For now we are using tactile normalized values
        for i in range(1,readings_no): 
            self.tactile_readings_diff.append(self.np_tactile_readings_normalized[:,:,i] - self.np_tactile_readings_normalized[:,:,i-1])
            if np.std(self.tactile_readings_diff[-1]) > 0.1:
                self.np_tactile_changes_label[i] = 1

