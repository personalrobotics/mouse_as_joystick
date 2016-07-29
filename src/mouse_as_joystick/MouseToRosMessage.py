#!/usr/bin/env python

#reads specified joystick values, and publishes them
import struct
import threading
import rospy
from sensor_msgs.msg import Joy
import numpy as np

import argparse

from LinuxMouseHelpers import *



joystick_data_topic = "/ada/joy"

#make sure to run the following to be able to read:
# sudo chmod a+r /dev/input/mouse1
#to disable, first run the following:
# xinput
#find the device id of the mouse you want to disable. Then
# xinput set-prop id "Device Enabled" 0
#default_mouse_filename = "/dev/input/mouse1"
#default_mouse_id = 12



class MouseData(object):
    max_joystick_dx = 1.
    max_joystick_dy = 1.
    def __init__(self, bLeft=0, bRight=0, bMiddle=0, x=0, y=0):
        self.bLeft = bLeft
        self.bRight = bRight
        self.bMiddle = bMiddle
        self.x = x
        self.y = y

    def toJoyMsg(self):
        return Joy(axes = np.array([float(self.x)/MouseData.max_joystick_dx, float(self.y)/MouseData.max_joystick_dy]), buttons=np.array([self.bLeft, self.bRight, self.bMiddle]))

    def UpdateMaxValues(self):
        if self.x > MouseData.max_joystick_dx:
            MouseData.max_joystick_dx = self.x
        elif -self.x > MouseData.max_joystick_dx:
            MouseData.max_joystick_dx = -self.x

        if self.y > MouseData.max_joystick_dy:
            MouseData.max_joystick_dy = self.y
        elif -self.y > MouseData.max_joystick_dy:
            MouseData.max_joystick_dy = -self.y

    def __str__(self):
        return "L: " + str(self.bLeft) + " M: " + str(self.bMiddle) + " R: " + str(self.bRight) + " x: " + str(self.x) + " y: " + str(self.y) + " max x: " + str(self.max_joystick_dx) + " max y: " + str(self.max_joystick_dy)
        #print ("L:%d, M: %d, R: %d, x: %d, y: %d\n" % (bLeft,bMiddle,bRight, x, y) );


class MouseHandler(object):
    def __init__(self, mouse_filename):
       #print mouse_filename
       self.mouse_file = open(mouse_filename, "rb", )
       self.mouse_data_lock = threading.Lock()
       self.mouse_data = None
       self.mouse_data_last = MouseData()
       self.num_times_nodata_limit = 3
       self.num_times_nodata = 0

    #get the mouse data from where the thread puts it
    #if nothing is there, and we have exceeded the limit of not receiving data, send zeros
    #otherwise, send the last received mouse command
    def getMouseData(self):
        mouse_data = None
        with self.mouse_data_lock:
            mouse_data = self.mouse_data
            self.mouse_data = None
        if mouse_data is None:
            self.num_times_nodata += 1
            if self.num_times_nodata > self.num_times_nodata_limit:
                return MouseData()
            else:
                return self.mouse_data_last
        else:
            self.num_times_nodata = 0
            self.mouse_data_last = mouse_data

            #update max values we have seen
            mouse_data.UpdateMaxValues()
        return mouse_data


    def getMouseData_StartLoop(self):
        self.thread = threading.Thread(target=self.getMouseLoop)
        self.thread.daemon = True
        self.thread.start()


    def getMouseData_Event(self):
        buf = self.mouse_file.read(3);
        button = ord( buf[0] );
#        bLeft = button & 0x1;
#        bMiddle =  button & 0x4;
#        bRight =  button & 0x2;
        bLeft = int((button & 0x1) > 0)
        bMiddle = int(( button & 0x2 ) > 0)
        bRight = int(( button & 0x4 ) > 0)
        x,y = struct.unpack( "bb", buf[1:] );
        return MouseData(bLeft, bMiddle, bRight, x, y)

    def getMouseLoop(self):
        while not rospy.is_shutdown():
            mouse_data_next = self.getMouseData_Event()
            with self.mouse_data_lock:
                self.mouse_data = mouse_data_next    

        self.closeMouseHandler()


    def closeMouseHandler(self):
        self.mouse_file.close();



def MouseDataPublisher(mouse_handler):
    pub = rospy.Publisher(joystick_data_topic, Joy, queue_size=10)
    rospy.init_node('mouse_to_joystick', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        mouse_data = mouse_handler.getMouseData()
        print mouse_data
        mouse_data_rosmsg = mouse_data.toJoyMsg()
        mouse_data_rosmsg.header.stamp = rospy.Time.now()
        pub.publish(mouse_data_rosmsg)
        #pub.publish(mouse_data_ros_msg)
        rate.sleep()

    



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Wrapper to read from mouse and publish ros messages like a joystick")
    parser.add_argument('-id', '--mouse-id', help='mouse id. Run xinput in a shell, and specify the id number of the mouse you want to connect to', type=str)
    parser.add_argument('-num', '--mouse-num', help='mouse number given by X in /dev/input/mouseX', type=str)

    args = parser.parse_args()

    #load mouse id if specified, otherwise guess based on highest id number
    if args.mouse_id:
        mouse_id = int(args.mouse_id)
    else:
        mouse_id = get_mouse_process_id()

    disable_mouse_by_id(mouse_id)

    #find mouse filename to read from
    if args.mouse_num:
        mouse_filename = '/dev/input/mouse'+ (args.mouse_num)
    else:
        mouse_filename = get_mouse_filename_from_id(mouse_id)
        
    make_mouse_readable(mouse_filename)

    #start loop to read data
    mouse_handler = MouseHandler(mouse_filename)
    mouse_handler.getMouseData_StartLoop()

    #create publisher loop
    try:
        MouseDataPublisher(mouse_handler)
    except rospy.ROSInterruptException:
        mouse_handler.close()


