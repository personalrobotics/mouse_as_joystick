#!/usr/bin/env python

#reads specified joystick values, and publishes them

import struct

max_joystick_dx = 20.
max_joystick_dy = 20.


#make sure to run the following to be able to read:
# sudo chmod a+r /dev/input/mouse1
#to disable, first run the following:
# xinput
#find the device id of the mouse you want to disable. Then
# xinput set-prop id "Device Enabled" 0
mouse_filename = "/dev/input/mouse1"

class MouseHandler(object):
    def __init__(self):
       self.mouse_file = open(mouse_filename, "rb", )
   

def getMouseEvent(mouse_file):
    buf = mouse_file.read(3);
    button = ord( buf[0] );
    bLeft = button & 0x1;
    bMiddle = ( button & 0x4 ) > 0;
    bRight = ( button & 0x2 ) > 0;
    x,y = struct.unpack( "bb", buf[1:] );
    print ("L:%d, M: %d, R: %d, x: %d, y: %d\n" % (bLeft,bMiddle,bRight, x, y) );


if __name__ == "__main__":
    print 'asdf'
    mouse_handler = MouseHandler()

    while( 1 ):
        getMouseEvent(mouse_handler.mouse_file);
    mouse_handler.mouse_file.close();



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
