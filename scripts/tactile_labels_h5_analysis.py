import cv2
import h5py 
import numpy as np

def print_attrs(name): 
    print (name) 
    # for key, val in obj.attrs.items(): 
    #     print ("{}: {}".format(key, val)) 
 
f_vision = h5py.File('/media/rodrigo/WD_BLACK/Grasp_Benchmarking_Experiments/Experiments/GGCNN2/labels/slipDataset_vision.h5','r') 
f_tactile = h5py.File('/media/rodrigo/WD_BLACK/Grasp_Benchmarking_Experiments/Experiments/GGCNN2/labels/slipDataset_tactile.h5','r') 
f_other = h5py.File('/media/rodrigo/WD_BLACK/Grasp_Benchmarking_Experiments/Experiments/GGCNN2/labels/slipDataset_other.h5','r') 
# f = h5py.File('/home/rodrigo/Documents/github/uskin_publisher/src/uskin_ros_publisher/files/h5py/slipDataset.h5','r') 
f_vision.visit(print_attrs)

video_frames = f.get('GGCNN2/adversarial/Pose1/Exp1/video_frames')
video_timestamps = f.get('GGCNN2/adversarial/Pose1/Exp1/video_timestamps')

i = 0

# frames len is number of frames
while i < len(video_frames):
    cv2.imshow('frame', video_frames[i])
    print('video timestamp {}'.format(video_timestamps[i]))
    x = raw_input()
    print('Frame {}'.format(i))
    i += 1
    if cv2.waitKey(1) & 0xFF == 27:
        break

f_vision.close()
f_tactile.close()
f_other.close()