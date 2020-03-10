#!/usr/bin/env python
import sys
import os

import rospy

import numpy as np
import h5py
import json

from pynput import keyboard
import datetime
import fnmatch
import glob  # Search for files' paths

from uskin_ros_publisher import ManualLabelsRecord
from uskin_ros_publisher import TactileRecordPublisher
from uskin_ros_publisher import VideoRecordPublisher
from uskin_ros_publisher import ExperimentStatesRecord



vision_rate = 0
tactile_rate = 180
navigation_modes = ["navigate_tactile_frames", "navigate_video_frames"]
mode = False # Chooses navigation mode
config_file_path = ""
config_dict = {}
has_last_frame_been_updated = False
has_data_been_saved = False

# Finds closest existing tactile timestamp to the one provided 
def getEndExperimentTactileIndex(timestamp_end):

    if tactile.timestamps[-1] >= timestamp_end:
        res = next(x for x, val in enumerate(
            tactile.timestamps) if val >= timestamp_end)

        return res
    else:
        return None

# Finds closest existing vision timestamp to the one provided 
def getEndExperimentVisionIndex(timestamp_end):
    # This might be an issue if the video was stopped before time
    if vision.timestamps[-1] >= timestamp_end:
        res = next(x for x, val in enumerate(
            vision.timestamps) if val >= timestamp_end)

        return res
    else:
        return None


def getCompleteFilePaths():

    global config_dict
    for file_type in ['tactile_data_path', 'vision_data_path', 'manual_labels_path', 'h5py_file_path', 'states_file_path']:
        config_dict[file_type] = config_dict[file_type].replace("${object}", config_dict['experiment']['object'])
        config_dict[file_type] = config_dict[file_type].replace("${method}", config_dict['experiment']['method'])


    config_dict['tactile_data_path'] = glob.glob(config_dict['tactile_data_path'].replace("${object}", config_dict['experiment']['object'])+config_dict['experiment']['object']+str(
        config_dict['experiment']['pose_num'])+'/'+'rosbag_Grasp_'+str(config_dict['experiment']['experiment_num'])+'_' + "*.bag")[0]
    rospy.loginfo("Opening Tactile data file: {}".format(
        config_dict['tactile_data_path']))

    # config_dict['vision_data_path'] = glob.glob(config_dict['vision_data_path']+config_dict['experiment']['object']+str(
        # config_dict['experiment']['pose_num'])+'/'+config_dict['vision_data_name'])[0]    
    config_dict['vision_data_path'] = glob.glob(config_dict['vision_data_path']+config_dict['experiment']['object']+str(
        config_dict['experiment']['pose_num'])+'/'+'MVI_' + "*.MP4")[config_dict['experiment']['experiment_num']-1]
    rospy.loginfo("Opening Video File: {}".format(config_dict['vision_data_path']))

    config_dict['manual_labels_path'] = glob.glob(config_dict['manual_labels_path']+config_dict['experiment']['object']+str(
        config_dict['experiment']['pose_num'])+'/'+'Grasp_'+str(config_dict['experiment']['experiment_num'])+'_' + "*.xlsx")[0]
    rospy.loginfo("Opening Manual Labels File: {}".format(
        config_dict['manual_labels_path']))

    config_dict['states_file_path'] = glob.glob(config_dict['states_file_path']+config_dict['experiment']['object']+'_'+str(
        config_dict['experiment']['pose_num'])+'/'+'experiment_' + "*.yaml")[config_dict['experiment']['experiment_num']-1]
    rospy.loginfo("Opening Experiment States File: {}".format(
        config_dict['states_file_path']))

    config_dict['h5py_file_path'] = config_dict['h5py_file_path'] + \
        config_dict['h5py_file_name']
    rospy.loginfo("Opening Manual Labels File: {}".format(
        config_dict['h5py_file_path']))
    return


def readConfigFile():
    global config_dict
    global config_file_path
    rospy.loginfo('=================================================================================')
    rospy.loginfo('Loading Necessary Config Files and Data files')
    rospy.loginfo('=================================================================================')
    
    if rospy.has_param('/produce_manual_labels/json_file_name'):
        config_file_path = rospy.get_param("/produce_manual_labels/json_file_name")

        with open(config_file_path, 'r') as f:
            config_dict = json.load(f)

        # Necessary since most files include an unknown timestamp
        getCompleteFilePaths()

        rospy.loginfo('Sucessfully read from config JSON file')
        return True
    else:
        rospy.loginfo('Unable to find config JSON file')
        return False


def saveH5pyFile(file_path, method_name, object_name, pose_num, exp_num, tactile, video):
    rospy.loginfo('Will attempt to save data into file located at {}'.format(file_path))

    # Get initial and end index's for tactile and vision
    tactile_start_index = tactile.initial_tactile_index
    tactile_end_index = getEndExperimentTactileIndex(
        manual_labels.getTimestampEndExperiment())
    vision_start_index = vision.initial_motion_index
    vision_end_index = getEndExperimentVisionIndex(
        manual_labels.getTimestampEndExperiment())

    # Checks whether all tactile and visual data has been retreieved already
    if tactile_end_index is None or vision_end_index is None:
        rospy.loginfo('Error: You have to navigate torwards the end of the video before being able to save the file!')
        return False

    # HDF5 file opening
    hf = h5py.File(file_path, 'a')

    # Check if experiment has already been recorded in file
    exp_str = method_name + '/' + object_name + \
        '/Pose' + str(pose_num) + '/Exp' + str(exp_num)
    if exp_str in hf:
        rospy.loginfo('Error: Experiment already exists in file, please remove respective subgroups first!')
        hf.close()
        return False

    # Saving Video data
    rospy.loginfo('Saving {} frames of video (start index: {}, end index: {}).'.format(
        vision_end_index-vision_start_index, vision_start_index, vision_end_index))

    try:
        group_vision = hf.create_group(exp_str+'/Video')

        group_vision.create_dataset(
            'video_timestamps', data=vision.timestamps[vision_start_index:vision_end_index])
        group_vision.create_dataset(
            'video_frames', data=vision.video_frames_resized[vision_start_index:vision_end_index])

    except IndexError:
        # In theory we never reach this state (safety net)
        rospy.loginfo('Error: You have to navigate torwards the end of the video before being able to saveb the file!')
        hf.close()
        return False

    # Saving Tactile data
    rospy.loginfo('Saving {} samples of tactile data (start index: {}, end index: {}).'.format(
        tactile_end_index-tactile_start_index, tactile_start_index, tactile_end_index))

    group_tactile = hf.create_group(exp_str+'/Tactile')

    group_tactile.create_dataset(
        'tactile_timestamps', data=tactile.timestamps[tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_data_raw', data=tactile.np_tactile_readings_raw[tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_data_normalized', data=tactile.np_tactile_readings_normalized[tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_changes_label', data=tactile.np_tactile_changes_label[tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_slips_label', data=np_tactile_slips_label[tactile_start_index:tactile_end_index])

    # print(len(tactile.timestamps))
    # print(len(tactile.messages["/uskin_xyz_values"]))
    # print(len(tactile.messages["/uskin_xyz_values_normalized"]))
    # print(tactile.np_tactile_changes_label.size)
    # print(np_tactile_slips_label.size)

    # Experiment data
    rospy.loginfo('Saving other experimental data (experiment steps).')

    group_experiment = hf.create_group(exp_str+'/Experiment/experiment_states')

    for k, v in exp_states.states_timestamps.items():
        group_experiment.create_dataset(k, data=np.array(v, dtype=np.float64))

    hf.close()
    return True

def on_press(key):
    global mode
    global has_last_frame_been_updated
    global has_data_been_saved
    try:

        if key == keyboard.Key.space:
            if np_tactile_slips_label[tactile.current_index] == 0:
                rospy.loginfo("New 'Slip' Label")
                np_tactile_slips_label[tactile.current_index] = 1
            elif np_tactile_slips_label[tactile.current_index] == 1:
                rospy.loginfo("New 'Other Event' Label")
                np_tactile_slips_label[tactile.current_index] = 2
            else:
                rospy.loginfo("Removing Label")
                np_tactile_slips_label[tactile.current_index] = 0

            return False

        # elif hasattr(key, 'char') and key.char == "n":
        #     # do something
        #     # global tactile_data

        #     rospy.loginfo("Jumping to next clue at x")

        #     return False
        
        elif hasattr(key, 'char') and key.char == "h":
            
            rospy.loginfo("==========================KeyBoard Instructions==========================")
            rospy.loginfo("Press '->' key to move torwars next tactile/visual data sample.")
            rospy.loginfo("Press '<-' key to move torwars previous tactile/visual data sample.")
            rospy.loginfo("Press 'm' key to switch navigation mode (tactile or vision samples)")
            rospy.loginfo("Press 'space' bar key to mark data sample as \"slip\", \"no slip\" or \"other event\.")
            rospy.loginfo("Press 'a' key to adjust visual/tactile data synchronization.")
            rospy.loginfo("Press 'f' key to adjust the last tactile/vision data sample.")
            rospy.loginfo("Press 's' key to to save data to HDF5 file.")
            rospy.loginfo("Press 'e' key to to exit.")
            rospy.loginfo("=========================================================================")

            return False

        elif hasattr(key, 'char') and key.char == "s":

            confirm_key = raw_input(
                "Are you sure you want to save your data/labels? Existent data will be overwrtitten!\n")
            rospy.loginfo(confirm_key)
            if confirm_key == 'y':
                rospy.loginfo("Saving all relevant data to HDF5 file")

                if saveH5pyFile(config_dict['h5py_file_path'],
                             config_dict['experiment']['method'],
                             config_dict['experiment']['object'],
                             config_dict['experiment']['pose_num'],
                             config_dict['experiment']['experiment_num'],
                             tactile,
                             vision):
                    rospy.loginfo('Data has been saved!')
                    has_data_been_saved = True
                    return False

                
        
            rospy.loginfo('Data has NOT been saved!')

            return False

        elif hasattr(key, 'char') and key.char == "a":

            rospy.loginfo("Adjusting video/tactile sync")
            tactile.adjustInitialFrame()
            # Will run every moment 'a' is pressed. could be optimized to run only after 'a' is pressed the second time
            manual_labels.resetTimestamps(
                tactile.raw_timestamps[tactile.initial_tactile_index])
            return False

        elif hasattr(key, 'char') and key.char == "m":

            mode = not mode
            rospy.loginfo("Changing navigation mode to {}".format(
                navigation_modes[mode]))
            return False

        elif hasattr(key, 'char') and key.char == "f":

            rospy.loginfo("Defining new end timestamp")
            # End experiment timestamp is being kept in manual_labels. Could be inproved in the future
            manual_labels.setTimestampEndExperiment(
                tactile.timestamps[tactile.current_index])
            has_last_frame_been_updated = True
            return False

        elif key == keyboard.Key.right:

            rospy.loginfo("Jumping to next time instance")

            # Navigate tactile_frames
            if not mode:
                tactile.updateNextIndex()
                while (tactile.timestamps[tactile.current_index] >= vision.timestamps[vision.current_index+1]):
                    _, frame = vision.getNextFrame()
                    if frame is None:
                        break
            else:  # Navigate video frames
                vision.getNextFrame()
                while tactile.timestamps[tactile.current_index] < vision.timestamps[vision.current_index]:
                    tactile.updateNextIndex()

            return False

        elif key == keyboard.Key.left:

            rospy.loginfo("Jumping to previus time instance")
            # Navigate tactile_frames
            if not mode:
                tactile.updatePreviousIndex()
                while (tactile.timestamps[tactile.current_index] < vision.timestamps[vision.current_index]):
                    vision.getPreviousFrame()
            else:  # Navigate video frames
                vision.getPreviousFrame()
                while tactile.timestamps[tactile.current_index-1] >= vision.timestamps[vision.current_index]:
                    tactile.updatePreviousIndex()
            return False

    except AttributeError as ex:
        rospy.loginfo(ex)


def on_release(key):
    if hasattr(key, 'char') and key.char == "e":
        # Stop listener
        rospy.signal_shutdown("Closing the node - 'e' key has been pressed")

        return False


def wait_for_user_input():
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    listener.join()  # wait till listener will stop
    # other stuff


if __name__ == '__main__':

    rospy.init_node('produce_manual_labels', anonymous=True)
    rate = rospy.Rate(50)  # 10hz

    # Read JSON config file to retrieve all relevant paths
    if not readConfigFile():
        # rospy.signal_shutdown("Closing the node - valid JSON file was not found")
        sys.exit()

    # Load all relevant data files
    vision = VideoRecordPublisher(config_dict['vision_data_path'])
    manual_labels = ManualLabelsRecord(config_dict['manual_labels_path'])
    tactile = TactileRecordPublisher(config_dict['tactile_data_path'], manual_labels.getTimestampStartExperiment())
    exp_states = ExperimentStatesRecord(config_dict['states_file_path'])

    # Ground manual label timestamps with initial timestamp data
    manual_labels.resetTimestamps(tactile.raw_timestamps[tactile.initial_tactile_index])

    # Ground experiment tasks timestamps with timestamp associated with end of "Move_1" task
    exp_states.resetTimestamps()

    # In general it should show frame where robot motion stops for the first time
    vision.getVideoInitialFrame()

    # Print general information
    rospy.loginfo('=================================================================================')
    vision_rate = vision.fps
    rospy.loginfo("Vison sample frequency is: {}Hz".format(vision_rate))
    rospy.loginfo("Tactile sample frequency is: {}Hz".format(tactile_rate))

    rospy.loginfo('Begin video timestamp is: {}'.format(
        vision.initial_motion_timestamp))
    rospy.loginfo('Begin tactile timestamp is: {}'.format(
        tactile.raw_timestamps[tactile.current_index]))

    # numpy array with Slips labels (0 - Nothing, 1 - Slip, 2 - Other event)
    np_tactile_slips_label = np.zeros(
        tactile.np_tactile_changes_label.size, dtype=int)

    # Print current information
    while not rospy.is_shutdown():
        vision.showFrame()
        tactile.publishData()
        rospy.loginfo('=================================================================================')
        rospy.loginfo('Current vision timestamp: {}'.format(
            vision.timestamps[vision.current_index]))
        rospy.loginfo('Current tactile timestamp: {}'.format(
            tactile.timestamps[tactile.current_index]))

        rospy.loginfo('Current robot state is: {}'.format(exp_states.getCurrentTask(tactile.timestamps[tactile.current_index])))

        rospy.loginfo('Tactile CHANGES labels: ( {}  {}  {}  {}  [ {} ]  {}  {}  {}  {})'.format(
            *tactile.np_tactile_changes_label[tactile.current_index-4:tactile.current_index+5]))
        rospy.loginfo('Tactile SLIPS   labels: ( {}  {}  {}  {}  [ {} ]  {}  {}  {}  {})'.format(
            *np_tactile_slips_label[tactile.current_index-4:tactile.current_index+5]))

        rospy.loginfo('--------------Future Events-----------------')
        rospy.loginfo('Next tactile slips happening at:')
        rospy.loginfo(list(event for event in manual_labels.getTimestampsSlips()
                   if event > tactile.timestamps[tactile.current_index]))
        rospy.loginfo('Last tactile/frame frame timestamp is: {}'.format(
            manual_labels.getTimestampEndExperiment()))
        rospy.loginfo('Has last tactile/video frame been updated?: {}'.format(has_last_frame_been_updated))
        rospy.loginfo('Has data been saved?: {}'.format(has_data_been_saved))
        #print('current start_index is: {}, current end_index is: {}'.format(tactile.initial_tactile_index, getEndExperimentTactileIndex(manual_labels.getTimestampEndExperiment())))
        rospy.loginfo('=================================================================================')
        wait_for_user_input()
        rate.sleep()

    del vision
    del manual_labels
    del tactile
    del exp_states
