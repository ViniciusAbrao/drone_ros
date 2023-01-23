#!/usr/bin/env python3

# ROS python API
import rospy

# import messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from my_package.msg import Corner
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from std_msgs.msg import Int32

class Commander:
    # initialization method
    def __init__(self):
        # Drone state
        self.current_state = State()
        self.local_pos = Point(0.0, 0.0, 0.0)
        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(20)
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.setpose = PoseStamped()
              
     ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        #print(self.local_pos)

    ## Drone State callback
    def state_cb(self, msg):
        self.current_state = msg
        #print("atualizou",self.state.armed)
        
    def force_land(self):
        self.setpose.pose.position.x = self.local_pos.x 
        self.setpose.pose.position.y = self.local_pos.y
        self.setpose.pose.position.z = self.local_pos.z - 3
        # Send a few setpoints
        for i in range(100):   
            self.local_pos_pub.publish(self.setpose)
            self.rate.sleep()       
        self.set_mode_client(0,"AUTO.LAND")
        print ("Disarm command")  

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

    def auto_set_mode(self,mode_str):
        rospy.wait_for_service('mavros/set_mode')
        try:
            setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode=mode_str)
        except (rospy.ServiceException, e):
            print ("Service takeoff call failed: %s",e)

    def wpPull(self,wps):
        rospy.wait_for_service('mavros/mission/pull')
        try:
            wpSetCurrent = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
            wpSetCurrent.call(wps)
            wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull,persistent=True)
            wp_reic = wpPullService().wp_received
        except (rospy.ServiceException, e):
            print ("Service Puling call failed: %s",e)

class stateMoniter:
    def __init__(self):
        self.state = State()
        self.cur_wp = WaypointReached()
        self.motion = Corner()
        
    def stateCb(self, msg):
        self.state = msg
        
    def wpCb(self, msg):
        self.cur_wp = msg

    def controllerCb(self, msg):
        self.motion = msg

def main():
    rospy.init_node('commander_node')
    rate = rospy.Rate(20.0)

    stateMt = stateMoniter()
    md = Modes()
    cmm = Commander()

    rospy.on_shutdown(cmm.force_land)

    md.wpPull(0)

    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)
    rospy.Subscriber("/mavros/mission/reached",WaypointReached, stateMt.wpCb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cmm.posCb)

    rospy.Subscriber('/square_motion', Corner, stateMt.controllerCb)

    corner_pub = rospy.Publisher('/square_corner', Int32, queue_size=1)
    corner_msg = Int32()

    # Send a few setpoints before starting
    cmm.setpose.pose.position.x = cmm.local_pos.x 
    cmm.setpose.pose.position.y = cmm.local_pos.y 
    cmm.setpose.pose.position.z = cmm.local_pos.z 
    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        cmm.local_pos_pub.publish(cmm.setpose)
        cmm.rate.sleep()
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'


    # Arming the drone
    while not stateMt.state.armed:
        md.setArm()
        rate.sleep()

    # Switching to Mission
    while not stateMt.state.mode=="AUTO.MISSION":
        md.auto_set_mode("AUTO.MISSION")
        rate.sleep()
        #print ("AUTO.MISSION")

    # Start Mission 
    transition=0   
    while not rospy.is_shutdown():

        #print (stateMt.cur_wp.wp_seq)
        if stateMt.cur_wp.wp_seq==2 and transition==0:
             
            transition=1
            cmm.setpose.pose.position.x = cmm.local_pos.x
            cmm.setpose.pose.position.y = cmm.local_pos.y
            cmm.setpose.pose.position.z = cmm.local_pos.z
            corner_msg.data = 1 
            for i in range(50):   
                if(rospy.is_shutdown()):
                    break
                cmm.local_pos_pub.publish(cmm.setpose)
                corner_pub.publish(corner_msg)
                cmm.rate.sleep() 
                #print("sp:",cmm.setpose.pose)

            # Switching to OFFBOARD
            while not stateMt.state.mode=="OFFBOARD":
                if(cmm.current_state.mode != "OFFBOARD"):
                    if(cmm.set_mode_client.call(offb_set_mode).mode_sent == True):
                        rospy.loginfo("OFFBOARD enabled")
                cmm.local_pos_pub.publish(cmm.setpose)
                cmm.rate.sleep()
                #print ("changing to offboard")
            
            for corners in [1, 2, 3, 4]:
                print("Current Corner:",corners)
                cmm.setpose.pose.position.x = cmm.local_pos.x + stateMt.motion.corner_x
                cmm.setpose.pose.position.y = cmm.local_pos.y + stateMt.motion.corner_y
                cmm.setpose.pose.position.z = cmm.local_pos.z + stateMt.motion.corner_z
                corner_msg.data = corners + 1               
                # Send a few setpoints
                for i in range(100):   
                    if(rospy.is_shutdown()):
                        break
                    cmm.local_pos_pub.publish(cmm.setpose)
                    corner_pub.publish(corner_msg)
                    cmm.rate.sleep()
                
        
        elif transition == 1:

            transition=2
            # Switching to Mission
            while not stateMt.state.mode=="AUTO.MISSION":
                md.auto_set_mode("AUTO.MISSION")
                rate.sleep()
                #print ("AUTO.MISSION")

            md.wpPull(8)
            #print (stateMt.cur_wp.wp_seq)
            rate.sleep()
 
        else:
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
