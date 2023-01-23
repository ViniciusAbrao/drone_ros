#!/usr/bin/env python3

# ROS python API
import rospy

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class Modes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except (rospy.ServiceException, e):
            print ("Service arming call failed: %s",e)

    def auto_set_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            # setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.set_mode.request.custom_mode)
            setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="AUTO.MISSION")
        except (rospy.ServiceException, e):
            print ("Service takeoff call failed: %s",e)

    def wpPull(self,wps):
        rospy.wait_for_service('mavros/mission/pull')
        try:
            wpSetCurrent = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
            wpSetCurrent.call(wps)
            wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull,persistent=True)
            print (wpPullService().wp_received)

            print ("Waypoint Pulled")
        except (rospy.ServiceException, e):
            print ("Service Puling call failed: %s",e)

class stateMoniter:
    def __init__(self):
        self.state = State()
        self.cur_wp = WaypointReached()
        
    def stateCb(self, msg):
        self.state = msg
        
    def wpCb(self, msg):
        self.cur_wp = msg


def main():
    rospy.init_node('controller_node')
    rate = rospy.Rate(20.0)

    stateMt = stateMoniter()
    md = Modes()

    md.wpPull(0)
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)
    
    rospy.Subscriber("/mavros/mission/reached",WaypointReached, stateMt.wpCb)

    # Arming the drone
    while not stateMt.state.armed:
        md.setArm()
        rate.sleep()
    # Switching the state to auto mode
    while not stateMt.state.mode=="AUTO.MISSION":
        md.auto_set_mode()
        rate.sleep()
        print ("AUTO.MISSION")
        
    while not rospy.is_shutdown():
        print (stateMt.cur_wp.wp_seq)
        if stateMt.cur_wp.wp_seq==2:
            print (stateMt.cur_wp.wp_seq)
            rate.sleep()
        rate.sleep()

    # rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
