#!/usr/bin/env python3
import rospy
from my_package.msg import Corner   #publish motion increment 
from std_msgs.msg import Int32      #subscribe drone corner
    
class cornerMoniter:
    def __init__(self):
        self.corner = Int32()
        
    def stateCb(self, msg):
        self.corner = msg

if __name__ == "__main__":

    rospy.init_node("controller_node")
    
    rate = rospy.Rate(20.0)

    cornerMt = cornerMoniter()
    
    # Subscribe to drone's corner
    state_sub = rospy.Subscriber("/square_corner", Int32, callback = cornerMt.stateCb)

    # Publish incremental motion to complete a square path
    motion_pub = rospy.Publisher('/square_motion', Corner, queue_size=10)
    motion_msg = Corner()
  

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown()):
        if (cornerMt.corner.data==1):
            motion_msg.corner_x=4
            motion_msg.corner_y=0
            motion_msg.corner_z=0
            motion_pub.publish(motion_msg)
        elif (cornerMt.corner.data==2):
            motion_msg.corner_x=0
            motion_msg.corner_y=-4
            motion_msg.corner_z=0
            motion_pub.publish(motion_msg)
        elif (cornerMt.corner.data==3):
            motion_msg.corner_x=-4
            motion_msg.corner_y=0
            motion_msg.corner_z=0
            motion_pub.publish(motion_msg)
        elif (cornerMt.corner.data==4):
            motion_msg.corner_x=0
            motion_msg.corner_y=4
            motion_msg.corner_z=0
            motion_pub.publish(motion_msg)
        rate.sleep()
     