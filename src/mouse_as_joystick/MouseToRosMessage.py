#!/usr/bin/env python

#reads specified joystick values, and publishes them
import struct
import threading
import rospy
from sensor_msgs.msg import Joy
import numpy as np

max_joystick_dx = 20.
max_joystick_dy = 20.

joystick_data_topic = "/ada/joy"

#make sure to run the following to be able to read:
# sudo chmod a+r /dev/input/mouse1
#to disable, first run the following:
# xinput
#find the device id of the mouse you want to disable. Then
# xinput set-prop id "Device Enabled" 0
mouse_filename = "/dev/input/mouse1"

class MouseData(object):
    def __init__(self, bLeft=0, bRight=0, bMiddle=0, x=0, y=0):
        self.bLeft = bLeft
        self.bRight = bRight
        self.bMiddle = bMiddle
        self.x = x
        self.y = y

    def toJoyMsg(self):
        return Joy(axes = np.array([float(self.x)/max_joystick_dx, float(self.y)/max_joystick_dy]), buttons=np.array([self.bLeft, self.bRight, self.bMiddle]))



    def __str__(self):
        return "L: " + str(self.bLeft) + " M: " + str(self.bMiddle) + " R: " + str(self.bRight) + " x: " + str(self.x) + " y: " + str(self.y)
        #print ("L:%d, M: %d, R: %d, x: %d, y: %d\n" % (bLeft,bMiddle,bRight, x, y) );


class MouseHandler(object):
    def __init__(self):
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
        mouse_data_rosmsg = mouse_data.toJoyMsg()
        mouse_data_rosmsg.header.stamp = rospy.Time.now()
        pub.publish(mouse_data_rosmsg)
        #pub.publish(mouse_data_ros_msg)
        rate.sleep()

    



if __name__ == "__main__":
    mouse_handler = MouseHandler()
    mouse_handler.getMouseData_StartLoop()

    try:
        MouseDataPublisher(mouse_handler)
    except rospy.ROSInterruptException:
        mouse_handler.close()





# copied code from ros publisher 
#import rospy
#from std_msgs.msg import String
#
#def talker():
#    pub = rospy.Publisher('chatter', String, queue_size=10)
#    rospy.init_node('talker', anonymous=True)
#    rate = rospy.Rate(10) # 10hz
#    while not rospy.is_shutdown():
#        hello_str = "hello world %s" % rospy.get_time()
#        rospy.loginfo(hello_str)
#        pub.publish(hello_str)
#        rate.sleep()
#
#if __name__ == '__main__':
#    try:
#        talker()
#    except rospy.ROSInterruptException:
#        pass
