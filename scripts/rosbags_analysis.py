# Script to analyze tactile data sampling frequency
import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import rospkg


if __name__ == '__main__':

    if len(sys.argv) < 2:
            print ("Provide valid rosbag path to open")
            exit()
    
    rospack = rospkg.RosPack()
    path = rospack.get_path('uskin_ros_publisher')
    bag = rosbag.Bag(path+'/files/ros_bags/' + sys.argv[1])

    messages = {"/uskin_xyz_values": [], "/uskin_xyz_values_normalized": [] } 
    messages["/uskin_xyz_values"]= [ msg for topic, msg, t in bag.read_messages(topics=['/uskin_xyz_values'])] 
    messages["/uskin_xyz_values_normalized"]= [ msg for topic, msg, t in bag.read_messages(topics=['/uskin_xyz_values_normalized'])]       
    bag.close()

    times = [message.header.stamp.secs+message.header.stamp.nsecs/(10**9) for message in messages["/uskin_xyz_values_normalized"]]

    diff_times = []

    for i in range(len(times)-1): 
        diff_times.append(float(times[i+1])-float(times[i]))

    aux_np = np.array(diff_times)

    len(np.where(aux_np >= 0.001)[0])

    mean = np.mean(aux_np)
    print(mean)
    stand = np.std(aux_np)
    print(stand)

    _ = plt.hist(aux_np, bins='auto')

    plt.show()