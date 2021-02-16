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
# from uskin_ros_publisher import VideoRecordPublisher
# from uskin_ros_publisher import ExperimentStatesRecord


vision_rate = 0
tactile_rate = 180

navigation_modes = ["navigate_tactile_frames", "navigate_video_frames"]
mode = False  # Chooses navigation mode

config_file_path = ""
config_dict = {}

has_last_vision_frame_been_updated = False
has_last_tactile_frame_been_updated = False
has_data_been_saved = False

fine_tuned_shifted_elements = 0

# Used for group of samples labeling
group_label_index1 = None
group_label_index2 = None

# Finds closest existing tactile timestamp to the one provided


def getEndExperimentTactileIndex(timestamp_end):

    if timestamp_end is None:
        return None
    if tactile.timestamps[-1] >= timestamp_end:
        res = next(x for x, val in enumerate(
            tactile.timestamps) if val >= timestamp_end)

        # Returns index where tactile timestamp is closest to timestamp_end
        return res+1
    else:
        return None

# Finds closest existing vision timestamp to the one provided


def getEndExperimentVisionIndex(timestamp_end):
    if timestamp_end is None:
        return None
    # This might be an issue if the video was stopped before time
    if vision.timestamps[-1] >= timestamp_end:
        res = next(x for x, val in enumerate(
            vision.timestamps) if val >= timestamp_end)

        # Returns index where vision timestamp is closest to timestamp_end
        return res+1
    elif vision.has_reach_end:
        return len(vision.timestamps)
    else:
        return None

# Read experimental information from json congig file and get necessary file paths


def getCompleteFilePaths():

    global config_dict
    for file_type in ['tactile_data_path', 'manual_labels_path', 'h5py_file_path', 'h5py_tactile_file_name']:
        config_dict[file_type] = config_dict[file_type].replace(
            "${object}", config_dict['experiment']['object'])
        config_dict[file_type] = config_dict[file_type].replace(
            "${method}", config_dict['experiment']['method'])

    config_dict['tactile_data_path'] = glob.glob(config_dict['tactile_data_path']+config_dict['experiment']['object']+str(
        config_dict['experiment']['pose_num'])+'/'+'rosbag_Grasp_'+str(config_dict['experiment']['experiment_num'])+'_' + "*.bag")[0]
    rospy.loginfo("Opening Tactile data file: {}".format(
        config_dict['tactile_data_path']))

    
    config_dict['manual_labels_path'] = glob.glob(config_dict['manual_labels_path']+config_dict['experiment']['object']+str(
        config_dict['experiment']['pose_num'])+'/'+'Grasp_'+str(config_dict['experiment']['experiment_num'])+'_' + "*.xlsx")[0]
    rospy.loginfo("Opening Manual Labels File: {}".format(
        config_dict['manual_labels_path']))

  
    # config_dict['h5py_file_path'] = config_dict['h5py_file_path'] + \
    #     config_dict['h5py_file_name']
    # rospy.loginfo("Opening Manual Labels File: {}".format(
    #     config_dict['h5py_file_path']))
    # Experiment Key identifier

    config_dict['experiment_key'] = config_dict['experiment']['method'] + '/' + config_dict['experiment']['object'] + \
        '/Pose' + str(config_dict['experiment']['pose_num']) + \
        '/Exp' + str(config_dict['experiment']['experiment_num'])
    return


def readConfigFile():
    global config_dict
    global config_file_path
    rospy.loginfo(
        '=================================================================================')
    rospy.loginfo('Loading Necessary Config Files and Data files')
    rospy.loginfo(
        '=================================================================================')

    if rospy.has_param('/produce_manual_labels/json_file_name'):
        print('Here!')

        config_file_path = rospy.get_param(
            "/produce_manual_labels/json_file_name")

        with open(config_file_path, 'r') as f:
            config_dict = json.load(f)

        # Necessary since most files include an unknown timestamp
        getCompleteFilePaths()

        rospy.loginfo('Sucessfully read from config JSON file')
        return True
    else:
        rospy.loginfo('Unable to find config JSON file')
        return False

# Check if experiment has already been recorded in hdf5 file.


def groupExistsHDF5(experiment_key, hf):
    if experiment_key in hf:
        return True
    return False


def confirmOverwriteHDF5Data(experiment_key, hf):
    confirm_key = raw_input(
        "Experiment already exists in file. Are you sure you wish to overwrite your data?\n")
    rospy.loginfo(confirm_key)
    if confirm_key != 'y':
        hf.close()
        return False
    # Delete all existing data in HDF5 file for this experiment
    else:
        del hf[experiment_key]

    return True


def overwriteHDF5Data(experiment_key, hf):
    del hf[experiment_key]

    return


def saveH5pyFile(experiment_key, file_path, tactile):
    rospy.loginfo(
        'Will attempt to save data into file located at {}'.format(file_path))

    # Get initial and end index's for tactile and vision
    tactile_start_index = tactile.initial_tactile_index
    tactile_end_index = getEndExperimentTactileIndex(
        tactile.getTimestampEndExperiment())
    # vision_start_index = vision.initial_motion_index
    # vision_end_index = getEndExperimentVisionIndex(
        # vision.getTimestampEndExperiment())

    # Checks whether all tactile and visual data has been retreieved already
    if tactile_end_index is None:
        rospy.loginfo(
            'Error: You have to navigate torwards the end of the experiment before being able to save the file!')
        return False

    # # Saving Video data
    # hf = h5py.File(file_path+config_dict['h5py_vision_file_name'], 'a')

    # # Check if experiment has already been recorded in hdf5 file.
    # if groupExistsHDF5(experiment_key, hf):
    #     # Confirm that user wants to overwrite data
    #     if not confirmOverwriteHDF5Data(experiment_key, hf):
    #         return False

    # rospy.loginfo('Saving {} frames of video (start index: {}, end index: {}).'.format(
    #     vision_end_index-vision_start_index, vision_start_index, vision_end_index))

    # try:
    #     group_vision = hf.create_group(experiment_key)
    #     # group_vision = hf.create_group(experiment_key+'/Video')

    #     group_vision.create_dataset(
    #         'video_timestamps', data=vision.timestamps[vision_start_index:vision_end_index])
    #     group_vision.create_dataset(
    #         'video_frames', data=vision.video_frames_resized[vision_start_index:vision_end_index], compression="gzip", compression_opts=4)
    #     # group_vision.create_dataset(
    #     #     'video_frames', data=vision.video_frames_resized[vision_start_index:vision_end_index])

    # except IndexError:
    #     # In theory we never reach this state (safety net)
    #     rospy.loginfo(
    #         'Error: You have to navigate torwards the end of the video before being able to saveb the file!')
    #     hf.close()
    #     return False

    # hf.close()

    # Saving Tactile data
    hf = h5py.File(file_path+config_dict['h5py_tactile_file_name'], 'a')

    # Check if experiment has already been recorded in hdf5 file.
    if groupExistsHDF5(experiment_key, hf):
        # Overwrite data (user has already confirmed for vision)
        overwriteHDF5Data(experiment_key, hf)

    rospy.loginfo('Saving {} samples of tactile data (start index: {}, end index: {}).'.format(
        tactile_end_index-tactile_start_index, tactile_start_index, tactile_end_index))

    group_tactile = hf.create_group(experiment_key)
    # group_tactile = hf.create_group(experiment_key+'/Tactile')

    group_tactile.create_dataset(
        'tactile_timestamps', data=tactile.timestamps[tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_data_raw', data=tactile.np_tactile_readings_raw[:, :, tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_data_normalized', data=tactile.np_tactile_readings_normalized[:, :, tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_changes_label', data=tactile.np_tactile_changes_label[tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_slips_label', data=np_tactile_slips_label[tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'fine_tuned_shifted_elements', data=np.array(fine_tuned_shifted_elements, dtype=np.int32))

    hf.close()

    # # Experimental data
    # hf = h5py.File(file_path+config_dict['h5py_experiment_file_name'], 'a')

    # # Check if experiment has already been recorded in hdf5 file.
    # if groupExistsHDF5(experiment_key, hf):
    #     # Overwrite data (user has already confirmed for vision)
    #     overwriteHDF5Data(experiment_key, hf)

    # rospy.loginfo('Saving other experimental data (experiment steps).')

    # group_experiment = hf.create_group(experiment_key+'/experiment_states')
    # # group_experiment = hf.create_group(experiment_key+'/Experiment/experiment_states')

    # # Save timestamps for each experiment state
    # for k, v in exp_states.states_timestamps.items():
    #     group_experiment.create_dataset(k, data=np.array(v, dtype=np.float64))

    # hf.close()
    return True


def checkPreviouslyStoredTactileData():
    global fine_tuned_shifted_elements
    previous_label_data = None
    try:
        hf = h5py.File(config_dict['h5py_file_path'] +
                       config_dict['h5py_tactile_file_name'], 'r')

        # If there exists a previously saved label data
        if groupExistsHDF5(config_dict['experiment_key'], hf):
            fine_tuned_shifted_elements = hf[config_dict['experiment_key'] +
                                             '/fine_tuned_shifted_elements'].value
            previous_label_data = np.array(
                hf[config_dict['experiment_key']+'/tactile_slips_label'])

            rospy.loginfo('Successfuly read experiment from HDF5 file!')

        else:
            rospy.loginfo('Experiment does not yet exist in HDF5 file!')
            
        hf.close()

        return previous_label_data
    except:
        rospy.logerr('Could not open HDF5 file (probabily doesn\'t exist).')

    return None


def generateTactileSlipLabelArray(previous_label_data):
    np_tactile_slips_label = np.zeros(
        tactile.np_tactile_changes_label.size, dtype=int)

    # If there exists a previously saved label data
    if previous_label_data is not None:
        rospy.loginfo('Loading Slips label array from saved data!!')
        np_tactile_slips_label[tactile.initial_tactile_index:tactile.initial_tactile_index +
                               previous_label_data.size] = previous_label_data

    return np_tactile_slips_label

def labelSamplesGroup(label):
    global group_label_index1
    global group_label_index2

    if group_label_index1 is None:
        group_label_index1 = tactile.current_index
        return None

    elif group_label_index2 is None:
        group_label_index2 = tactile.current_index
        number_of_samples = group_label_index2 - group_label_index1 + 1
        
        for i in range(group_label_index1, group_label_index2+1):
            np_tactile_slips_label[i] = label
        
        group_label_index1 = None
        group_label_index2 = None

        return number_of_samples

    return None

def on_press(key):
    global mode
    # global has_last_vision_frame_been_updated
    global has_last_tactile_frame_been_updated
    global has_data_been_saved
    global np_tactile_slips_label

    try:

        if key == keyboard.Key.space:
            if np_tactile_slips_label[tactile.current_index] == 0:
                rospy.loginfo("New 'Other Event' Label")
                np_tactile_slips_label[tactile.current_index] = 6
            elif np_tactile_slips_label[tactile.current_index] == 6:
                rospy.loginfo("New 'Slip + Arm Move' Label")
                np_tactile_slips_label[tactile.current_index] = 5
            elif np_tactile_slips_label[tactile.current_index] == 5:
                rospy.loginfo("New 'Arm Move' Label")
                np_tactile_slips_label[tactile.current_index] = 4
            elif np_tactile_slips_label[tactile.current_index] == 4:
                rospy.loginfo("New 'Grasp' Label")
                np_tactile_slips_label[tactile.current_index] = 3
            elif np_tactile_slips_label[tactile.current_index] == 3:
                rospy.loginfo("New 'Release' Label")
                np_tactile_slips_label[tactile.current_index] = 2
            elif np_tactile_slips_label[tactile.current_index] == 2:
                rospy.loginfo("New 'Slip' Label")
                np_tactile_slips_label[tactile.current_index] = 1
            else:
                rospy.loginfo("Removing Label")
                np_tactile_slips_label[tactile.current_index] = 0

            return False

        elif hasattr(key, 'char') and key.char == "h":

            rospy.loginfo(
                "==========================KeyBoard Instructions==========================")
            rospy.loginfo(
                "Press '->' key to move torwars next tactile/visual data sample.")
            rospy.loginfo(
                "Press '<-' key to move torwars previous tactile/visual data sample.")
            rospy.loginfo(
                "Press 'm' key to switch navigation mode (tactile or vision samples)")
            rospy.loginfo(
                "Press 'space' bar key to mark data sample as \"slip\":1, \"release\":2, \"grasp\":3, \"arm move\":4, \"slip + arm move\":5 or \"other event\":6.")
            rospy.loginfo(
                "Press 'a' key to adjust visual/tactile data synchronization.")
            rospy.loginfo(
                "Press 'f' key to adjust the last tactile/vision data sample.")
            rospy.loginfo("Press 's' key to to save data to HDF5 file.")
            rospy.loginfo("Press 'e' key to to exit.")
            rospy.loginfo(
                "=========================================================================")

            return False

        elif hasattr(key, 'char') and key.char == "s":

            confirm_key = raw_input(
                "Do you want to save your data/labels? Existent data will be overwrtitten!\n")
            rospy.loginfo(confirm_key)
            if confirm_key == 'y':
                rospy.loginfo("Saving all relevant data to HDF5 file")

                if saveH5pyFile(config_dict['experiment_key'],
                                config_dict['h5py_file_path'],
                                tactile):
                    rospy.loginfo('Data has been saved!')
                    has_data_been_saved = True
                    return False

            rospy.loginfo('Data has NOT been saved!')

            return False

        elif hasattr(key, 'char') and key.char == "a":
            global fine_tuned_shifted_elements
            rospy.loginfo("Adjusting video/tactile sync")

            shifted_elements = tactile.adjustInitialFrame()

            # It is also necessary to shift label elements
            if shifted_elements is not None:
                fine_tuned_shifted_elements += shifted_elements
                rospy.loginfo('Rolling slip labels')
                np_tactile_slips_label = np.roll(
                    np_tactile_slips_label, shifted_elements)
            return False

        # elif hasattr(key, 'char') and key.char == "m":

        #     mode = not mode
        #     rospy.loginfo("Changing navigation mode to {}".format(
        #         navigation_modes[mode]))
        #     return False

        elif hasattr(key, 'char') and key.char == "f":

            rospy.loginfo("Defining new end timestamp")
            # End experiment timestamp is being kept in manual_labels. Could be inproved in the future
            # if mode:
            #     vision.setTimestampEndExperiment(
            #         vision.timestamps[vision.current_index])
            #     has_last_vision_frame_been_updated = True

            # else:
            tactile.setTimestampEndExperiment(
                tactile.timestamps[tactile.current_index])
            has_last_tactile_frame_been_updated = True

            return False

        elif hasattr(key, 'char') and key.char == "0":
            rospy.loginfo("Group Label: 0")
            number_of_samples_labled = labelSamplesGroup(0)
            if(number_of_samples_labled is None):
                rospy.loginfo("Move to next index and press '0' again")
            else:
                rospy.loginfo("Successfuly labeled group of {} samples with '0'".format(number_of_samples_labled))

            return False

        elif hasattr(key, 'char') and key.char == "1":
            rospy.loginfo("Group Label: 1")
            number_of_samples_labled = labelSamplesGroup(1)
            if(not number_of_samples_labled):
                rospy.loginfo("Move to next index and press '1' again")
            else:
                rospy.loginfo("Successfuly labeled group of {} samples with '1'".format(number_of_samples_labled))

            return False
        
        elif hasattr(key, 'char') and key.char == "2":
            rospy.loginfo("Group Label: 2")
            number_of_samples_labled = labelSamplesGroup(2)
            if(not number_of_samples_labled):
                rospy.loginfo("Move to next index and press '2' again")
            else:
                rospy.loginfo("Successfuly labeled group of {} samples with '2'".format(number_of_samples_labled))

            return False
        
        elif hasattr(key, 'char') and key.char == "3":
            rospy.loginfo("Group Label: 3")
            number_of_samples_labled = labelSamplesGroup(3)
            if(not number_of_samples_labled):
                rospy.loginfo("Move to next index and press '3' again")
            else:
                rospy.loginfo("Successfuly labeled group of {} samples with '3'".format(number_of_samples_labled))

            return False
        
        elif hasattr(key, 'char') and key.char == "4":
            rospy.loginfo("Group Label: 4")
            number_of_samples_labled = labelSamplesGroup(4)
            if(not number_of_samples_labled):
                rospy.loginfo("Move to next index and press '4' again")
            else:
                rospy.loginfo("Successfuly labeled group of {} samples with '4'".format(number_of_samples_labled))

            return False

        elif hasattr(key, 'char') and key.char == "5":
            rospy.loginfo("Group Label: 5")
            number_of_samples_labled = labelSamplesGroup(5)
            if(not number_of_samples_labled):
                rospy.loginfo("Move to next index and press '5' again")
            else:
                rospy.loginfo("Successfuly labeled group of {} samples with '5'".format(number_of_samples_labled))

            return False

        elif hasattr(key, 'char') and key.char == "6":
            rospy.loginfo("Group Label: 6")
            number_of_samples_labled = labelSamplesGroup(6)
            if(not number_of_samples_labled):
                rospy.loginfo("Move to next index and press '6' again")
            else:
                rospy.loginfo("Successfuly labeled group of {} samples with '6'".format(number_of_samples_labled))

            return False

        elif key == keyboard.Key.right:

            rospy.loginfo("Jumping to next time instance")

            # Navigate tactile_frames
            if not mode:
                tactile.updateNextIndex()
                # while (tactile.timestamps[tactile.current_index] >= vision.timestamps[vision.current_index+1]):
                #     _, frame = vision.getNextFrame()
                #     if frame is None:
                #         break
            # else:  # Navigate video frames
            #     vision.getNextFrame()
            #     while tactile.timestamps[tactile.current_index] < vision.timestamps[vision.current_index]:
            #         tactile.updateNextIndex()

            return False

        elif key == keyboard.Key.left:

            rospy.loginfo("Jumping to previus time instance")
            # Navigate tactile_frames
            if not mode:
                tactile.updatePreviousIndex()
                # while (tactile.timestamps[tactile.current_index] < vision.timestamps[vision.current_index]):
                #     vision.getPreviousFrame()
            # else:  # Navigate video frames
            #     vision.getPreviousFrame()
            #     while tactile.timestamps[tactile.current_index-1] >= vision.timestamps[vision.current_index]:
                    # tactile.updatePreviousIndex()
            return False

        elif key == keyboard.Key.up:

            rospy.loginfo("Velocity increasead by 2x")

            # Navigate tactile_frames
            if not mode:
                tactile.multiplyVelocity(2)
                # while (tactile.timestamps[tactile.current_index] >= vision.timestamps[vision.current_index+1]):
                #     _, frame = vision.getNextFrame()
                #     if frame is None:
                #         break
            # else:  # Navigate video frames
            #     vision.getNextFrame()
            #     while tactile.timestamps[tactile.current_index] < vision.timestamps[vision.current_index]:
            #         tactile.updateNextIndex()
            return False

        elif key == keyboard.Key.down:

            rospy.loginfo("Velocity halved (0.5x)")

            # Navigate tactile_frames
            if not mode:
                tactile.multiplyVelocity(0.5)
                # while (tactile.timestamps[tactile.current_index] >= vision.timestamps[vision.current_index+1]):
                #     _, frame = vision.getNextFrame()
                #     if frame is None:
                #         break
            # else:  # Navigate video frames
            #     vision.getNextFrame()
            #     while tactile.timestamps[tactile.current_index] < vision.timestamps[vision.current_index]:
            #         tactile.updateNextIndex()

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

    previous_label_data = checkPreviouslyStoredTactileData()

    # Load all relevant data files
    # vision = VideoRecordPublisher(config_dict['vision_data_path'])
    manual_labels = ManualLabelsRecord(config_dict['manual_labels_path'])
    tactile = TactileRecordPublisher(
        config_dict['tactile_data_path'], manual_labels.getTimestampStartExperiment(), fine_tuned_shifted_elements)
    # exp_states = ExperimentStatesRecord(config_dict['states_file_path'])

    # Ground manual label timestamps with initial timestamp data
    manual_labels.resetTimestamps(
        tactile.raw_timestamps[tactile.initial_tactile_index])

    # Ground experiment tasks timestamps with timestamp associated with end of "Move_1" task
    # exp_states.resetTimestamps()

    # In general it should show frame where robot motion stops for the first time
    # vision.getVideoInitialFrame()

    # Print general information
    rospy.loginfo(
        '=================================================================================')
    # vision_rate = vision.fps
    # rospy.loginfo("Vison sample frequency is: {}Hz".format(vision_rate))
    rospy.loginfo("Tactile sample frequency is: {}Hz".format(tactile_rate))

    # rospy.loginfo('Begin video timestamp is: {}'.format(
        # vision.initial_motion_timestamp))
    rospy.loginfo('Begin tactile timestamp is: {}'.format(
        tactile.raw_timestamps[tactile.current_index]))

    # numpy array with Slips labels (0 - Nothing, 1 - Slip, 2 - Other event)
    np_tactile_slips_label = generateTactileSlipLabelArray(previous_label_data)

    # Print current information
    while not rospy.is_shutdown():
        # vision.showFrame()
        tactile.publishData()
        rospy.loginfo(
            '=================================================================================')
        # rospy.loginfo('Current vision timestamp: {}'.format(
            # vision.timestamps[vision.current_index]))
        rospy.loginfo('Current tactile timestamp: {}'.format(
            tactile.timestamps[tactile.current_index]))

        # rospy.loginfo('Current robot state is: {}'.format(
            # exp_states.getCurrentTask(tactile.timestamps[tactile.current_index])))

        try:
            rospy.loginfo('Tactile CHANGES labels: ( {}  {}  {}  {}  [ {} ]  {}  {}  {}  {})'.format(
                *tactile.np_tactile_changes_label[tactile.current_index-4:tactile.current_index+5]))
            rospy.loginfo('Tactile SLIPS   labels: ( {}  {}  {}  {}  [ {} ]  {}  {}  {}  {})'.format(
                *np_tactile_slips_label[tactile.current_index-4:tactile.current_index+5]))
        except IndexError:
            rospy.loginfo('Tactile CHANGES labels: ( {}  {}  {}  {}  [ {} ])'.format(
                *tactile.np_tactile_changes_label[tactile.current_index-4:]))
            rospy.loginfo('Tactile SLIPS   labels: ( {}  {}  {}  {}  [ {} ])'.format(
                *np_tactile_slips_label[tactile.current_index-4:]))

        rospy.loginfo('--------------Future Events-----------------')
        rospy.loginfo('Next tactile slips happening at:')
        rospy.loginfo(list(event for event in manual_labels.getTimestampsSlips()
                           if event > tactile.timestamps[tactile.current_index]))
        # rospy.loginfo('Last vision frame timestamp is: {}'.format(
            # vision.getTimestampEndExperiment()))
        rospy.loginfo('Last tactile frame timestamp is: {}'.format(
            tactile.getTimestampEndExperiment()))
        # rospy.loginfo(
            # 'Has last vision frame been updated?: {}'.format(has_last_vision_frame_been_updated))
        rospy.loginfo(
            'Has last tactile frame been updated?: {}'.format(has_last_tactile_frame_been_updated))
        rospy.loginfo('Has data been saved?: {}'.format(has_data_been_saved))
        #print('current start_index is: {}, current end_index is: {}'.format(tactile.initial_tactile_index, getEndExperimentTactileIndex(manual_labels.getTimestampEndExperiment())))
        rospy.loginfo(
            '=================================================================================')
        wait_for_user_input()
        rate.sleep()

    # del vision
    del manual_labels
    del tactile
    # del exp_states
