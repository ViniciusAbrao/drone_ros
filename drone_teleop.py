#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, Point
import sys, select, os
import math
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
  
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
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.setpose = PoseStamped()
              
    ## local position callback
    def posCb(self, msg):     
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        
    ## Drone State callback
    def state_cb(self, msg):
        self.current_state = msg
        #print("atualizou",self.state.armed)
        
    def pousar(self):
        # Setpoint velocity publisher    
        self.setvelocity_pub=rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped,queue_size=10)
        self.setv=TwistStamped()
        self.setv.twist.linear.x = 0.0
        self.setv.twist.linear.y = 0.0
        self.setv.twist.linear.z = -0.5
        self.setv.twist.angular.x = 0.0
        self.setv.twist.angular.y = 0.0
        self.setv.twist.angular.z = 0.0
        # Send a few setpoints
        for i in range(200):   
            self.setvelocity_pub.publish(self.setv)
            self.rate.sleep()       
        self.set_mode_client(0,"AUTO.LAND")
        print ("Disarm")  
        
class Monitor:
    # initialization method
    def __init__(self):
    
        self.orientation = 0
        
     ## local position callback
    def posCb(self, msg):
        
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        
        #t0 = +2.0 * (w * x + y * z)
        #t1 = +1.0 - 2.0 * (x * x + y * y)
        #roll_x = math.atan2(t0, t1)
     
        #t2 = +2.0 * (w * y - z * x)
        #t2 = +1.0 if t2 > +1.0 else t2
        #t2 = -1.0 if t2 < -1.0 else t2
        #pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        self.orientation = yaw_z   


LIN_VEL_STEP_SIZE = 0.5
ANG_VEL_STEP_SIZE = 0.2
VERT_VEL_STEP_SIZE = 0.2
LAT_VEL_STEP_SIZE = 0.5

msg = """
Control Your Drone!
---------------------------
Moving around:

w/x : increase/decrease vertical velocity 
a/d : increase/decrease angular velocity 
i/k : increase/decrease frontal velocity 
l/j : increase/decrease lateral velocity 
space key, x : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""
       
def getKey():
    if os.name == 'nt':
      if sys.version_info[0] >= 3:
        return msvcrt.getch().decode()
      else:
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('drone_teleop')
    
    # controller object
    cnt = Controller()

    rospy.on_shutdown(cnt.pousar)
    
    # Subscribe to drone's state
    state_sub = rospy.Subscriber("/mavros/state", State, callback = cnt.state_cb)
    
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
  
    # monitor object
    mnt = Monitor()
    # Subscribe to drone's local orientation
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, mnt.posCb)
     # Subscribe to drone's local position
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cnt.posCb)

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    target_vertical_vel = 0.0
    target_lateral_vel = 0.0
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)    

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not cnt.current_state.connected):
        cnt.rate.sleep()
     
    # Wait for mnt.posCb 
    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        cnt.rate.sleep()
 
    cnt.setpose.pose.position.x = cnt.local_pos.x
    cnt.setpose.pose.position.y = cnt.local_pos.y
    cnt.setpose.pose.position.z = cnt.local_pos.z
    
    twist_stamped = TwistStamped()
    twist_stamped.twist.linear.x = 0.0; twist_stamped.twist.linear.y = 0.0; twist_stamped.twist.linear.z = 0.0
    twist_stamped.twist.angular.x = 0.0; twist_stamped.twist.angular.y = 0.0; twist_stamped.twist.angular.z = 0.0
    
    print("Takeoff: ", cnt.setpose.pose.position.z)

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

    # Move back to change mode
    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        pub.publish(twist_stamped)
        cnt.rate.sleep()
    
    try:
    
        print(msg)
        while(1):
            key = getKey()
            if key == 'i' :
                target_linear_vel = target_linear_vel + LIN_VEL_STEP_SIZE
                status = status + 1
                print("frontal_vel: ", target_linear_vel)
            elif key == 'k' :
                target_linear_vel = target_linear_vel - LIN_VEL_STEP_SIZE
                status = status + 1
                print("frontal_vel: ", target_linear_vel)
            elif key == 'a' :
                target_angular_vel = target_angular_vel + ANG_VEL_STEP_SIZE
                status = status + 1
                print("angular_vel: ", target_angular_vel)
            elif key == 'd' :
                target_angular_vel = target_angular_vel - ANG_VEL_STEP_SIZE
                status = status + 1
                print("angular_vel: ", target_angular_vel)
            elif key == 'w' :
                target_vertical_vel = target_vertical_vel + VERT_VEL_STEP_SIZE
                status = status + 1
                print("vertical_vel: ", target_vertical_vel)
            elif key == 's' :
                target_vertical_vel = target_vertical_vel - VERT_VEL_STEP_SIZE
                status = status + 1
                print("vertical_vel: ", target_vertical_vel)
            elif key == 'j' :
                target_lateral_vel = target_lateral_vel + LAT_VEL_STEP_SIZE
                status = status + 1
                print("lateral_vel: ", target_lateral_vel)
            elif key == 'l' :
                target_lateral_vel = target_lateral_vel - LAT_VEL_STEP_SIZE
                status = status + 1
                print("lateral_vel: ", target_lateral_vel)
            elif key == ' ' or key == 'x' :
                target_linear_vel   = 0.0
                target_angular_vel  = 0.0
                target_vertical_vel = 0.0
                target_lateral_vel = 0.0
                print("stop")
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            x0 = target_linear_vel
            y0 = target_lateral_vel
            z0 = target_vertical_vel
            twist_stamped.twist.linear.x = x0*math.cos(mnt.orientation)-y0*math.sin(mnt.orientation)
            twist_stamped.twist.linear.y = x0*math.sin(mnt.orientation)+y0*math.cos(mnt.orientation)
            twist_stamped.twist.linear.z = z0

            twist_stamped.twist.angular.x = 0.0; twist_stamped.twist.angular.y = 0.0; twist_stamped.twist.angular.z = target_angular_vel

            pub.publish(twist_stamped)

    except:
        print(e)

    #finally:
    #    cnt.pousar()

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
