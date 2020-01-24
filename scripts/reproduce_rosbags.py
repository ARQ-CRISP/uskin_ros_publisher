#!/usr/bin/env python
import rosbag
import rospy
from pynput import keyboard
from uskin_ros_publisher.msg import uskinFrame
import sys

def on_press(key):

    try:

        if key == keyboard.Key.right:
            # do something
            rospy.loginfo("right key arrow was pressed")
            global current_index
            current_index += 1
            return False
            
        elif key == keyboard.Key.left:
            rospy.loginfo("left key arrow was pressed")
            global current_index
            current_index -= 1
            return False

    except AttributeError as ex:
        print(ex)
    

def on_release(key):
    if  hasattr(key, 'char') and key.char == "e":
        # Stop listener
        rospy.signal_shutdown("Closing the node - 'e' key has been pressed")
        return False

def wait_for_user_input():
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        listener.join() # wait till listener will stop
    # other stuff

if __name__ == '__main__':
    
    if len(sys.argv) < 2:
        print ("Provide valid rosbag path to open")
        exit()

    current_index = 0

    try:
        
        rospy.init_node('reproduce_rosbags', anonymous=True)
        rate = rospy.Rate(180) # 10hz
        
        bag = rosbag.Bag(sys.argv[1])
        messages = {"/uskin_xyz_values": [], "/uskin_xyz_values_normalized": [] }
        messages["/uskin_xyz_values"]= [ msg for topic, msg, t in bag.read_messages(topics=['/uskin_xyz_values'])]
        messages["/uskin_xyz_values_normalized"]= [ msg for topic, msg, t in bag.read_messages(topics=['/uskin_xyz_values_normalized'])]      
        bag.close()

        pub = rospy.Publisher('/uskin_xyz_values_normalized', uskinFrame, queue_size=10)

        while not rospy.is_shutdown():
            pub.publish(messages["/uskin_xyz_values_normalized"][current_index])
            wait_for_user_input()

    
    except rospy.ROSInterruptException:
        pass
    except:
        print ("An error has occurred")

