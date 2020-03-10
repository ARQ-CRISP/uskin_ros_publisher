#!/usr/bin/env python
import rospy
from pynput import keyboard
import sys
from uskin_ros_publisher import TactileRecordPublisher

tactile_data = 0

def on_press(key):

    try:
        
        if key == keyboard.Key.up:
            # do something
            # global tactile_data
            tactile_data.multiplyVelocity(2)
            rospy.loginfo("Velocity at x{}".format(tactile_data.velocity))

            return False

        if key == keyboard.Key.down:
            # do something
            # global tactile_data
            tactile_data.multiplyVelocity(0.5)
            rospy.loginfo("Velocity at x{}".format(tactile_data.velocity))

            return False

        if key == keyboard.Key.right:
            # do something
            rospy.loginfo(">> x{}".format(tactile_data.velocity))
            # global tactile_data
            tactile_data.updateNextIndex()
            return False
            
        elif key == keyboard.Key.left:
            rospy.loginfo("<< x{}".format(tactile_data.velocity))
            # global tactile_data
            tactile_data.updatePreviousIndex()
            return False

        # elif key == keyboard.Key.space:
        #     rospy.loginfo(">> x50")
        #     global current_index
        #     current_index += 50
        #     return False

        # elif key == keyboard.Key.backspace:
        #     rospy.loginfo("<< x50")
        #     global current_index
        #     current_index += 50
        #     return False

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


    rospy.init_node('reproduce_rosbags', anonymous=True)
    rate = rospy.Rate(180) # 10hz
    
    try:
        tactile_data = TactileRecordPublisher(sys.argv[1])

        while not rospy.is_shutdown():
            tactile_data.publishData()
            wait_for_user_input()


    except rospy.ROSInterruptException:
        pass
    # except:
    #     print ("An error has occurred")

