#!/usr/bin/env python

from collections import OrderedDict
import pandas as pd
import yaml

# states_scenario_1 = ['Start_Move_1, End_Move_1', Start_Pregrasp]
class ExperimentStatesRecord:

    def __init__(self, file_path):
        print('=================================================================================')
        print('Loading Experiment States data')
        print('=================================================================================')

        self.data_has_been_reset = False
        self.states_timestamps = OrderedDict()

        try:   
            with open(file_path) as file:
                temporary_timestamps = yaml.load(file)

                for state, timestamp in temporary_timestamps.items():
                    print(state, ":", timestamp)
            
            if(len(temporary_timestamps) not in [14, 22]):
                print('Error: Unexpected number of states. Something is wrong with the file')
                del self
            elif (len(temporary_timestamps) is 14):
                self.states_timestamps.update({'Start_Arm_Approach': temporary_timestamps['Start Move State_0']})
                self.states_timestamps.update({'End_Arm_Approach': temporary_timestamps['End Move State_1']})
                self.states_timestamps.update({'Start_Gripper_Pregrasp': temporary_timestamps['Start Grasp State_2']})
                self.states_timestamps.update({'End_Gripper_Pregrasp': temporary_timestamps['End Grasp State_3']})
                self.states_timestamps.update({'Start_Arm_Reach_Object': temporary_timestamps['Start Move State_4']})
                self.states_timestamps.update({'End_Arm_Reach_Object': temporary_timestamps['End Move State_5']})
                self.states_timestamps.update({'Start_Gripper_Grasp': temporary_timestamps['Start Grasp State_6']})
                self.states_timestamps.update({'End_Gripper_Grasp': temporary_timestamps['End Grasp State_7']})
                self.states_timestamps.update({'Start_Arm_Lift': temporary_timestamps['Start Move State_8']})
                self.states_timestamps.update({'End_Arm_Lift': temporary_timestamps['End Move State_9']})
                self.states_timestamps.update({'Start_Arm_Move_Away': temporary_timestamps['Start Move State_10']})
                self.states_timestamps.update({'End_Arm_Move_Away': temporary_timestamps['End Move State_11']})
                self.states_timestamps.update({'Start_Gripper_Relax': temporary_timestamps['Start Grasp State_12']})
                self.states_timestamps.update({'End_Gripper_Relax': temporary_timestamps['End Grasp State_13']})

            else:
                self.states_timestamps.update({'Start_Arm_Approach': temporary_timestamps['Start Move State_0']})
                self.states_timestamps.update({'End_Arm_Approach': temporary_timestamps['End Move State_1']})
                self.states_timestamps.update({'Start_Gripper_Pregrasp': temporary_timestamps['Start Grasp State_2']})
                self.states_timestamps.update({'End_Gripper_Pregrasp': temporary_timestamps['End Grasp State_3']})
                self.states_timestamps.update({'Start_Arm_Reach_Object': temporary_timestamps['Start Move State_4']})
                self.states_timestamps.update({'End_Arm_Reach_Object': temporary_timestamps['End Move State_5']})
                self.states_timestamps.update({'Start_Gripper_Grasp': temporary_timestamps['Start Grasp State_6']})
                self.states_timestamps.update({'End_Gripper_Grasp': temporary_timestamps['End Grasp State_7']})
                self.states_timestamps.update({'Start_Arm_Lift': temporary_timestamps['Start Move State_8']})
                self.states_timestamps.update({'End_Arm_Lift': temporary_timestamps['End Move State_9']})
                self.states_timestamps.update({'Start_Arm_Trajectory': temporary_timestamps['Start Trajectory State_10']})
                self.states_timestamps.update({'End_Arm_Trajectory': temporary_timestamps['End Trajectory State_11']})
                self.states_timestamps.update({'Start_Arm_Reach_Table': temporary_timestamps['Start Move State_12']})
                self.states_timestamps.update({'End_Arm_Reach_Table': temporary_timestamps['End Move State_13']})
                self.states_timestamps.update({'Start_Gripper_Release': temporary_timestamps['Start Grasp State_14']})
                self.states_timestamps.update({'End_Gripper_Release': temporary_timestamps['End Grasp State_15']})
                self.states_timestamps.update({'Start_Arm_Raise': temporary_timestamps['Start Move State_16']})
                self.states_timestamps.update({'End_Arm_Raise': temporary_timestamps['End Move State_17']})
                self.states_timestamps.update({'Start_Arm_Move_Away': temporary_timestamps['Start Move State_18']})
                self.states_timestamps.update({'End_Arm_Move_Away': temporary_timestamps['End Move State_19']})
                self.states_timestamps.update({'Start_Gripper_Relax': temporary_timestamps['Start Grasp State_20']})
                self.states_timestamps.update({'End_Gripper_Relax': temporary_timestamps['End Grasp State_21']})
                
        except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
            print ('Error: An error has ocurred while handling the file!')
            del self 

    def resetTimestamps(self, ground_timestamp=False):
        # By default, ground values according to End_Arm_Approach timestamp
        # It is not possible, for now, to run this method more than once
        if not self.data_has_been_reset:
            if ground_timestamp:
                for key, value in self.states_timestamps.items():
                    value = str(value)
                    self.states_timestamps[key] = float(value[0:10]+"."+value[10:].zfill(9))
                    self.states_timestamps[key] = (self.states_timestamps[key] - ground_timestamp)*1000
            else:
                ground_timestamp = str(self.states_timestamps['End_Arm_Approach'])
                ground_timestamp = float(ground_timestamp[0:10]+"."+ground_timestamp[10:].zfill(9))
                print('Grounding values with value {}'.format(ground_timestamp))
                for key, value in self.states_timestamps.items():
                    value = str(value)
                    self.states_timestamps[key] = float(value[0:10]+"."+value[10:].zfill(9))
                    self.states_timestamps[key] = (self.states_timestamps[key] - ground_timestamp)*1000
            self.data_has_been_reset = True
        else:
            print('Error: Timestamps for experiment tasks label have already been reset')

    def getCurrentTask(self, timestamp):
        if not self.data_has_been_reset:
            print('Run resetTimestamps() method first!')
            return
        # There are a few (and small) time windows when the robot is paused and thous no task in being executed
        for start, end in zip(list(self.states_timestamps.items())[::2], list(self.states_timestamps.items())[1::2]):
            # if self.states_timestamps['Start_Move_1'] <= timestamp <= 
            if start[1] <= timestamp <= end[1]:
                # Using either start or end variable would be ok 
                return start[0].split('_', 1)[1]
        
        return 'Pause'
    
    # def setTimestampEndExperiment(self, timestamp):
    #     self.timestamps[-1] = timestamp

        

