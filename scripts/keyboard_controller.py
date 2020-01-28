#!/usr/bin/env python
import rospy
from pynput import keyboard
from std_msgs.msg import String
from uskin_ros_publisher.srv import *
from uskin_ros_publisher.msg import uskinFrame
# import rosbag
import xlsxwriter
import rospkg
from threading import Thread
import datetime



record_data_flag = False

# Start from the first cell below the headers.
row = 1
col = 0

# def callback_values(data):
#     if record_data_flag:
#         global lock1
#         lock1.acquire()
#         rospy.loginfo("callback_values")
#         bag.write('/uskin_xyz_values', data)
#         lock1.release()

# def callback_normalized_values(data):
#      if record_data_flag:
#         global lock2
#         lock2.acquire()
#         rospy.loginfo("callback_normalized_values")
#         bag.write('/uskin_xyz_values_normalized', data)
#         lock2.release()
        
# def listener():

#     rospy.Subscriber("/uskin_xyz_values", uskinFrame, callback_values)
#     rospy.Subscriber("/uskin_xyz_values_normalized", uskinFrame, callback_normalized_values)

#     # # spin() simply keeps python from exiting until this node is stopped
#     # rospy.spin()

def terminateCallback():
    global record_data_flag

    print "shutdown time!"
    if record_data_flag:
        record_data_flag = False

        now = rospy.get_rostime()

        worksheet.write(row, col, "Moving Stop")
        worksheet.write(row, col+1, str(now.secs)+"."+str(now.nsecs))
        worksheet.write_formula(row, col+2, '=($B%d/86400)+DATE(1970,1,1)' % (row+1), time_format)

        workbook.close()

def keyboard_command_client(command):
    rospy.wait_for_service('keyboard_command')
    try:
        keyboard_command = rospy.ServiceProxy('keyboard_command', KeyboardCommand)
        resp1 = keyboard_command(command)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def on_press(key):
    global row
    global col
    global record_data_flag
    global worksheet
    global time_format

    try:

        if key == keyboard.Key.space:
            # do something
            rospy.loginfo("space key was pressed")
            global slip_state
            
            if not record_data_flag:
                now = rospy.get_rostime()

                worksheet.write(row, col, "Moving Start")
                worksheet.write(row, col+1, str(now.secs)+"."+str(now.nsecs))
                worksheet.write_formula(row, col+2, '=($B%d/86400)+DATE(1970,1,1)' % (row+1), time_format)

                row += 1
                record_data_flag = True

                rospy.loginfo("Moving START")            
            
            elif record_data_flag:
                now = rospy.get_rostime()
                worksheet.write(row, col, "SLIP Start")
                worksheet.write(row, col+1, str(now.secs)+"."+str(now.nsecs))
                worksheet.write_formula(row, col+2, '=($B%d/86400)+DATE(1970,1,1)' % (row+1), time_format)

                row += 1
                #slip_state = not slip_state
                # res = keyboard_command_client(SLIP)
                # rospy.loginfo(res)
                rospy.loginfo("SLIP START")

            # elif record_data_flag and slip_state:
            #     now = rospy.get_rostime()
            #     worksheet.write(row, col, "SLIP Stop")
            #     worksheet.write(row, col+1, str(now.secs)+"."+str(now.nsecs))
            #     worksheet.write_formula(row, col+2, '=($B%d/86400)+DATE(1970,1,1)' % (row+1), time_format)

            #     row += 1
            #     slip_state = not slip_state
            #     # res = keyboard_command_client(NO_SLIP)
            #     # rospy.loginfo(res)
            #     rospy.loginfo("SLIP STOP")

            return False

    except AttributeError as ex:
        print(ex)
    

def on_release(key):
    if  hasattr(key, 'char') and key.char == "e":
        # Stop listener
        rospy.signal_shutdown("Closing the node - 'e' key has been pressed")
        return False

def wait_for_user_input():
    while not rospy.is_shutdown():
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        listener.join() # wait till listener will stop
    # other stuff

if __name__ == '__main__':
    try:
        #slip_state = False
        # pub = rospy.Publisher('user_input', String, queue_size=10)
        rospy.init_node('keyboard_controller', anonymous=True)
        rate = rospy.Rate(100) # 100hz

        rospy.on_shutdown(terminateCallback)

        if rospy.has_param('/keyboard_controller/csv_file_name'):
            csv_file_name = rospy.get_param("/keyboard_controller/csv_file_name")
            print(csv_file_name)

            # Create a workbook and add a worksheet.
            # rospack = rospkg.RosPack()
            # rospack.get_path('rospy_tutorials')
            # workbook = xlsxwriter.Workbook(rospack.get_path('uskin_ros_publisher') + '/scripts/Grasp_test.xlsx')
            workbook = xlsxwriter.Workbook(csv_file_name + "_" + (datetime.datetime.now() - datetime.timedelta(milliseconds=18)).strftime("%Y-%m-%d-%H-%M-%S") + ".xlsx")
            worksheet = workbook.add_worksheet()

            # Add a bold format to use to highlight cells.
            bold = workbook.add_format({'bold': True})

            # Add a number format for cells with money.
            time_format = workbook.add_format({'num_format': 'dd/mm/yyyy hh:mm:ss.000'})

            # Write some data headers.
            worksheet.write('A1', 'Event', bold)
            worksheet.write('B1', 'Timestamp (Epochs)', bold)
            worksheet.write('C1', 'Timestamp (Date)', bold)

            t = Thread(target=wait_for_user_input)
            t.start()

            while not rospy.is_shutdown():
                rate.sleep()
        else:
            rospy.logerr("Must provide csv_file_name")
            rospy.signal_shutdown("Closing the node - 'e' key has been pressed")

    except rospy.ROSInterruptException:
        pass