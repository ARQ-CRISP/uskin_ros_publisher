#!/usr/bin/env python
import time
import numpy as np
import cv2


def rotateImage(frame):
    # frame_array = np.array(frame)
    # frame_array  = np.transpose(frame_array, (1,0,2))
    # cv2.flip(frame_array)
    frame_array = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame_array


def resize(img):
    scale_percent = 40  # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    return resized


def distMap(frame1, frame2):
    """outputs pythagorean distance between two frames"""
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    frame1_32 = np.float32(gray1)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    frame2_32 = np.float32(gray2)
    diff32 = frame1_32 - frame2_32
    # norm32 = np.sqrt(diff32[:,:,0]**2 + diff32[:,:,1]**2 + diff32[:,:,2]**2)/np.sqrt(255**2 + 255**2 + 255**2)
    # dist = np.uint8(norm32*255)
    return diff32


class VideoRecordPublisher:

    # self.start_tasks = ('Start Move1', 'Start Pregrasp', 'Start Move2', 'Start Grasp', 'Start Move3',
    #                    'Start Move4', 'Start Trajectory', 'Start Move5', 'Start Grasp', 'Start Move6')

    # self.end_tasks = ('End Move1', 'End Pregrasp', 'End Move2', 'End Grasp', 'End Move3',
    #                  'End Move4', 'End Trajectory', 'End Move5', 'End Grasp', 'End Move6')

    def __init__(self, video_path):

        # capture video stream from camera source. 0 refers to first camera, 1 referes to 2nd and so on.
        self.cap = cv2.VideoCapture(video_path)
        
        # thereshold that defines movement
        self.sdThresh = 4

        # # font 
        # self.font = cv2.FONT_HERSHEY_SIMPLEX

        # Get video's fps value
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        # print('FPS value:{}'.format(self.fps))

        # store all video's frames timestamps
        self.timestamps = [self.cap.get(cv2.CAP_PROP_POS_MSEC)]

        # store index's of "beginning" and "ending" of motion 
        self.initial_motion_index = 0
        self.initial_motion_timestamp = 0

        # flags if movement is corrently being detected
        self.move_detect = False
        
        # flags if video end has been reached
        self.has_reach_end = False


        # stores video_frames
        self.video_frames_resized = []

        self.current_index = -1

        self.end_experiment_timestamp = None

        cv2.namedWindow('frame')
        cv2.startWindowThread()

    def __del__(self):
        cv2.destroyAllWindows()
        self.cap.release()
        

    def readFrame(self):
        frame_exists, frame = self.cap.read()
        
        if frame_exists:
            self.video_frames_resized.append(resize(frame))
            
            # After initial frame has been found, start appending updated timestamps
            if self.initial_motion_index != 0:
                self.timestamps.append(self.cap.get(cv2.CAP_PROP_POS_MSEC)-self.initial_motion_timestamp)
            else:
                self.timestamps.append(self.cap.get(cv2.CAP_PROP_POS_MSEC))

        return frame_exists, frame
        
    def getVideoInitialFrame(self):
        print('=================================================================================')
        print('Searching for Video initial Frame')
        print('=================================================================================')
        frame_exists, frame1 = self.getNextFrame()

        frame_exists, frame2 = self.getNextFrame()


        total_frames_number = self.cap.get(cv2.CAP_PROP_FRAME_COUNT)
        print('Number of video frames is {}'.format(total_frames_number))

        # for i in range(500):
        while True:
            
            if self.current_index % int(total_frames_number/10) == 0:
                print ("Uploading video at {}%: ".format(self.current_index/total_frames_number * 100))


            frame_exists, frame3 = self.getNextFrame()
            # print('Frame exists is {}'.format(frame_exists))

            if not frame_exists:
                break
            
            # rows, cols, _ = np.shape(frame3)
            dist = distMap(frame1, frame3)
            frame1 = frame2
            frame2 = frame3

            # apply Gaussian smoothing
            # mod = cv2.GaussianBlur(dist, (9,9), 0)

            # apply thresholding
            _, thresh = cv2.threshold(dist, 100, 255, 0)

            # calculate st dev test
            _, stDev = cv2.meanStdDev(dist)

            if stDev > self.sdThresh:
                if self.move_detect is False:
                    # print("Motion detected!!")
                    self.move_detect = True

            # Check if robot has alreay stoped
            elif self.move_detect and round(stDev[0][0], 0) == 2:
                # print("Motion Stoped!!")
                self.move_detect = False
                
                self.showFrame()
                resp = raw_input("Is this the initial frame? ")
                if resp == "y":
                    self.initial_motion_index = self.current_index
                    self.initial_motion_timestamp = self.timestamps[self.current_index]
                    print("Previous video timestamp: {}".format(self.timestamps[self.current_index]))
                    # self.timestamps.append(self.cap.get(cv2.CAP_PROP_POS_MSEC)-self.initial_motion_timestamp)
                    self.timestamps = [x - self.initial_motion_timestamp for x in self.timestamps]
                    print("New video timestamp: {}".format(self.timestamps[self.current_index])) # Should print 0.0
                    break

    def getNextFrame(self):

        self.current_index += 1
        
        # Necessary to check if we have already retrieved this frame
        if self.current_index >= len(self.video_frames_resized):
            # A new frame needs to be retrieved
            frame_exists, frame = self.readFrame()
            if not frame_exists:
                self.current_index -= 1
                print('End of the video Reached!!!')
                self.has_reach_end = True
            return frame_exists, frame

        return None, None


    def getPreviousFrame(self):
        if self.current_index != 0:
            self.current_index -= 1

    def showFrame(self, index=None):
        
        if index is None:
            frame_to_show = rotateImage(self.video_frames_resized[self.current_index])
            cv2.imshow('frame', frame_to_show)
        else:
            frame_to_show = rotateImage(self.video_frames_resized[index])
            cv2.imshow('frame', frame_to_show)

        cv2.waitKey(1)
        # cv2.destroyAllWindows()

    def setTimestampEndExperiment(self, timestamp):
        self.end_experiment_timestamp = timestamp

    def getTimestampEndExperiment(self):
        return self.end_experiment_timestamp
