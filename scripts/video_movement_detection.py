# import cv2
# import numpy as np
 
# # Create a VideoCapture object and read from input file
# # If the input is the camera, pass 0 instead of the video file name
# cap = cv2.VideoCapture('/home/rodrigo/Documents/github/uskin_publisher/src/uskin_ros_publisher/scripts/MVI_2077.MP4')
 
# # Check if camera opened successfully
# if (cap.isOpened()== False): 
#   print("Error opening video stream or file")
 
# # Read until video is completed
# while(cap.isOpened()):
#   # Capture frame-by-frame
#   ret, frame = cap.read()
#   if ret == True:
 
#     # Display the resulting frame
#     cv2.imshow('Frame',frame)
 
#     # Press Q on keyboard to  exit
#     if cv2.waitKey(25) & 0xFF == ord('q'):
#       break
 
#   # Break the loop
#   else: 
#     break
 
# # When everything done, release the video capture object
# cap.release()
 
# # Closes all the frames
# cv2.destroyAllWindows()

import numpy as np
import cv2

sdThresh = 4
font = cv2.FONT_HERSHEY_SIMPLEX
#TODO: Face Detection 1

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

cv2.namedWindow('frame')
cv2.namedWindow('dist')

#capture video stream from camera source. 0 refers to first camera, 1 referes to 2nd and so on.
# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture('/home/rodrigo/Documents/github/uskin_publisher/src/uskin_ros_publisher/scripts/MVI_2123.MP4')
fps = cap.get(cv2.CAP_PROP_FPS)

timestamps = [cap.get(cv2.CAP_PROP_POS_MSEC)]
calc_timestamps = [0.0]
lable = []


frame_exists, frame1 = cap.read()
frame_exists, frame2 = cap.read()

robot_move_detect = False
robot_state = 0

robot_state_timestamps = dict()

while(frame_exists and True):
    frame_exists, frame3 = cap.read()
    if not frame_exists:
        break
    rows, cols, _ = np.shape(frame3)
    cv2.imshow('dist', frame3)
    dist = distMap(frame1, frame3)
    frame1 = frame2
    frame2 = frame3
    # apply Gaussian smoothing
    # mod = cv2.GaussianBlur(dist, (9,9), 0)
    # apply thresholding
    _, thresh = cv2.threshold(dist, 100, 255, 0)
    # calculate st dev test
    _, stDev = cv2.meanStdDev(dist)
    cv2.imshow('dist', dist)
    cv2.putText(frame2, "Standard Deviation - {}".format(round(stDev[0][0],0)), (70, 70), font, 1, (255, 0, 255), 1, cv2.LINE_AA)
    
    # if stDev > sdThresh:
    #     if robot_move_detect is False:
    #         print("Motion detected!!");
    #         robot_move_detect = True

    #         if robot_state == 1:
    #             robot_state_timestamps['Start Move1'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #         elif robot_state == 2:
    #             robot_state_timestamps['Start Grasp'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #         elif robot_state == 3:
    #             robot_state_timestamps['Start Move2'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #         elif robot_state == 5:
    #             robot_state_timestamps['Start Trajectory'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #         elif robot_state == 6:
    #             robot_state_timestamps['Start Move3'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #         elif robot_state == 7:
    #             robot_state_timestamps['Start Release'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #         elif robot_state == 8:
    #             robot_state_timestamps['Start Move4'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #         elif robot_state == 9:
    #             robot_state_timestamps['Start Move5'] = cap.get(cv2.CAP_PROP_POS_MSEC)

    #         robot_state += 1


    # # Check if robot has alreay stoped
    # if robot_move_detect and round(stDev[0][0],0) == 2:
    #     print("Motion Stoped!!")
    #     robot_move_detect = False
    #     # We can only detect the end of first state
    #     if robot_state == 1:
    #         robot_state_timestamps['End Move 1'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #     elif robot_state == 2:
    #             robot_state_timestamps['End Move1'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #     elif robot_state == 3:
    #             robot_state_timestamps['Start Grasp'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #     elif robot_state == 5:
    #             robot_state_timestamps['Start Move2'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #     elif robot_state == 6:
    #             robot_state_timestamps['Start Trajectory'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #     elif robot_state == 7:
    #             robot_state_timestamps['Start Move3'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #     elif robot_state == 8:
    #             robot_state_timestamps['Start Release'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #     elif robot_state == 9:
    #             robot_state_timestamps['Start Move4'] = cap.get(cv2.CAP_PROP_POS_MSEC)
    #     elif robot_state == 10:
    #             robot_state_timestamps['Start Move5'] = cap.get(cv2.CAP_PROP_POS_MSEC)

    cv2.imshow('frame', frame2)
    if cv2.waitKey(1) & 0xFF == 27:
        break
    timestamps.append(cap.get(cv2.CAP_PROP_POS_MSEC))
    # calc_timestamps.append(calc_timestamps[-1] + 1000/fps)

cap.release()
cv2.destroyAllWindows()

for i, ts in enumerate(timestamps):
    print('Frame: {}, timestamp: {}'.format(i, ts))

for key, value in robot_state_timestamps.items():
    print('Key: {}, Value: {}'.format(key, value))

print('FPS value: {}'.format(fps))

