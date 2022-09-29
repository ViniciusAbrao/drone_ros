#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class Controller:
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
        
        # Setpoint velocity publisher    
        self.setvelocity_pub=rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped,queue_size=10)
        self.setv=TwistStamped()
        self.setv.twist.linear.x = 0.0
        self.setv.twist.linear.y = 0.0
        self.setv.twist.linear.z = -0.3
        self.setv.twist.angular.x = 0.0
        self.setv.twist.angular.y = 0.0
        self.setv.twist.angular.z = 0.0
        
     ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        #print(self.local_pos.z)

    ## Drone State callback
    def state_cb(self, msg):
        self.current_state = msg
        #print("atualizou",self.state.armed)
        
    def pousar(self):
        self.setpose.pose.position.x = self.local_pos.x 
        self.setpose.pose.position.y = self.local_pos.y
        self.setpose.pose.position.z = self.local_pos.z - 3
        # Send a few setpoints
        for i in range(100):   
            self.local_pos_pub.publish(self.setpose)
            self.rate.sleep()       
        self.set_mode_client(0,"AUTO.LAND")
        print ("Comando para desarmar")  
    
if __name__ == "__main__":

    rospy.init_node("offb_node_py")
    
    # controller object
    cnt = Controller()

    rospy.on_shutdown(cnt.pousar)
    
    # Subscribe to drone's state
    state_sub = rospy.Subscriber("mavros/state", State, callback = cnt.state_cb)
    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not cnt.current_state.connected):
        cnt.rate.sleep()
     
    # Wait for cnt.posCb 
    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        cnt.rate.sleep()

    cnt.setpose.pose.position.x = cnt.local_pos.x
    cnt.setpose.pose.position.y = cnt.local_pos.y
    cnt.setpose.pose.position.z = cnt.local_pos.z + 2
    
    print("Altitude: ", cnt.setpose.pose.position.z)

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        cnt.local_pos_pub.publish(cnt.setpose)
        cnt.rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    time_takeoff = rospy.Time.now()
    
    while(not rospy.is_shutdown() and (rospy.Time.now() - time_takeoff) < rospy.Duration(25.0)):
    
        if(cnt.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(cnt.set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
            
        else:
            if(not cnt.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()

        cnt.local_pos_pub.publish(cnt.setpose)
        cnt.rate.sleep()
        
    # Move to new X    
    print("New X: ", cnt.setpose.pose.position.x)
    cnt.setpose.pose.position.x = cnt.local_pos.x + 2
    cnt.setpose.pose.position.y = cnt.local_pos.y
    cnt.setpose.pose.position.z = cnt.local_pos.z 
    # Send a few setpoints
    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        cnt.local_pos_pub.publish(cnt.setpose)
        cnt.rate.sleep()
               
    # Move to new X    
    print("New X: ", cnt.setpose.pose.position.x)
    cnt.setpose.pose.position.x = cnt.local_pos.x 
    cnt.setpose.pose.position.y = cnt.local_pos.y
    cnt.setpose.pose.position.z = cnt.local_pos.z - 3
    # Send a few setpoints
    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        cnt.local_pos_pub.publish(cnt.setpose)
        cnt.rate.sleep()
        
    cnt.set_mode_client(0,"AUTO.LAND")
    print ("Comando para desarmar")    

