#!/usr/bin/env python
import numpy as np
import cv2

sdThresh = 4
font = cv2.FONT_HERSHEY_SIMPLEX

start_tasks = ('Start Move1', 'Start Pregrasp', 'Start Move2', 'Start Grasp', 'Start Move3',
               'Start Move4', 'Start Trajectory', 'Start Move5', 'Start Grasp', 'Start Move6')

end_tasks = ('End Move1', 'End Pregrasp', 'End Move2', 'End Grasp', 'End Move3',
             'End Move4', 'End Trajectory', 'End Move5', 'End Grasp', 'End Move6')


def rotateImage(frame):
    # frame_array = np.array(frame)
    # frame_array  = np.transpose(frame_array, (1,0,2))
    # cv2.flip(frame_array)
    frame_array = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame_array


def resize(img):
    scale_percent = 60  # percent of original size
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

# capture video stream from camera source. 0 refers to first camera, 1 referes to 2nd and so on.
cap = cv2.VideoCapture(
    '/home/rodrigo/Documents/github/uskin_publisher/src/uskin_ros_publisher/files/debug/MVI_2123.MP4')
fps = cap.get(cv2.CAP_PROP_FPS)

timestamps = [cap.get(cv2.CAP_PROP_POS_MSEC)]
lable = []

frame_exists, frame1 = cap.read()
frame_exists, frame2 = cap.read()

robot_move_detect = False
robot_state = 0

robot_state_timestamps = dict()

while(True):
    frame_exists, frame3 = cap.read()
    if not frame_exists:
        break

    test = rotateImage(frame3)
    test = resize(test)
    # resize image
    cv2.imshow('frame', test)

    # frame3=rotateImage(frame3, 90)
    # frame3 = cv2.flip(frame3, flipCode=-0.5)
    # frame3 = cv2.rotateImage(frame3, 90)
    if not frame_exists:
        break
    rows, cols, _ = np.shape(frame3)
    # cv2.imshow('dist', frame3)
    dist = distMap(frame1, frame3)
    frame1 = frame2
    frame2 = frame3
    # apply Gaussian smoothing
    # mod = cv2.GaussianBlur(dist, (9,9), 0)
    # apply thresholding
    _, thresh = cv2.threshold(dist, 100, 255, 0)
    # calculate st dev test
    _, stDev = cv2.meanStdDev(dist)
    # cv2.imshow('dist', dist)
    # cv2.putText(frame2, "Standard Deviation - {}".format(round(stDev[0][0],0)), (70, 70), font, 1, (255, 0, 255), 1, cv2.LINE_AA)

    if stDev > sdThresh:
        if robot_move_detect is False:
            print("Motion detected!!")
            robot_move_detect = True

            if robot_state in range(1,10) :
                robot_state_timestamps[start_tasks[robot_state]] = cap.get(
                    cv2.CAP_PROP_POS_MSEC)

            robot_state += 1

    # Check if robot has alreay stoped
    if robot_move_detect and round(stDev[0][0], 0) == 2:
        print("Motion Stoped!!")
        robot_move_detect = False
        # We can only detect the end of first state
        if robot_state in range(0, 10):
            robot_state_timestamps[end_tasks[robot_state-1]] = cap.get(
                cv2.CAP_PROP_POS_MSEC)
            
    # cv2.imshow('frame', frame2)
    
    if cv2.waitKey(1) & 0xFF == 27:
        break

    timestamps.append(cap.get(cv2.CAP_PROP_POS_MSEC))

cap.release()
cv2.destroyAllWindows()

for i, ts in enumerate(timestamps):
    print('Frame: {}, timestamp: {}'.format(i, ts))

for key, value in robot_state_timestamps.items():
    print('Key: {}, Value: {}'.format(key, value))

print('FPS value: {}'.format(fps))
