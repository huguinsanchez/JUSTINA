#!/usr/bin/env python
###THIS NODE HAS SIMILAR FUNCTIONALITY TO OMNI_BASE_NODE, BUT IT USES THE SPEED CONTROL
###ALREADY IMPLEMENTED IN THE ROBOCLAW BOARD. THE SIMULATION OPTION HAS BEEN SUPRESSED
###FOR SIMULATION YOU SHOULD USE THE OMNI_BASE_SIMUL NODE. 
import serial, time, sys, math
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from hardware_tools import roboclaw
import tf

base_diameter = 0.48;
rc_address_frontal = 0x80;
rc_address_lateral  = 0x80; 
rc_frontal = roboclaw.Roboclaw("/dev/ttyACM0", 115200); #Roboclaw controling motors for frontal movement (left and right)
rc_lateral = roboclaw.Roboclaw("/dev/ttyACM1", 115200); #Roboclaw controling motors for lateral movement (front and rear)
rc_acceleration = 1000000;

def print_help():
    print "MOBILE BASE BY MARCOSOFT. Options:"
    print "\t --port \t Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
    print " - Mobile base can be moved by publishing either mobile_base/cmd_vel or"
    print " - mobile_base/speeds. Speeds must be values in [-1, 1] where a value of 1 "
    print " - represents the maximum speed that each motor can generate."
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."

def check_speed_ranges(s_left, s_right, s_front, s_rear): #speeds: left, right, front and rear
    max_value_frontal = max(abs(s_left), abs(s_right));
    max_value_lateral = max(abs(s_front), abs(s_rear));

    if max_value_lateral > 1.0:
        print "MobileBase.->Warning! front and rear speeds should not be greater than 1.0. Normalized speeds used instead"
        s_left  /= max_value_lateral;
        s_right /= max_value_lateral;
        s_front /= max_value_lateral;
        s_rear  /= max_value_lateral;
        max_value_frontal/= max_value_lateral;
    if max_value_frontal > 2.0:
        s_left  = s_left  / max_value_frontal * 2.0;
        s_right = s_right / max_value_frontal * 2.0;
        s_front = s_front / max_value_frontal * 2.0;
        s_rear  = s_rear  / max_value_frontal * 2.0;
    return (s_left, s_right, s_front, s_rear);

def send_speeds(s_left, s_right, s_front, s_rear):
    #Speeds are assumed to be floats in [-1,1] for each tire. The values in [0,1] need to be transformed to [-QPPR,QPPR]
    #where QPPR is the maximum motor speed. This constant is the number of encoders ticks per turn times
    #the motor angular speed. It can also be obtained with IonMotion Studio
    #A float value of -1, indicates the maximum speed backwards
    #Similar for +1
    (s_left, s_right, s_front, s_rear) = check_speed_ranges(s_left, s_right, s_front, s_rear);
    s_left  =  int(s_left  * 32767 * 16.0/35.0);
    s_right =  int(s_right * 32767 * 16.0/35.0);
    s_front = -int(s_front * 32767);
    s_rear  = -int(s_rear  * 32767);
    rc_frontal.DutyM1M2(rc_address_frontal, s_left, s_right);
    rc_lateral.DutyM1M2(rc_address_lateral, s_front, s_rear);
    #s_left  =  int(s_left  * QPPS_LEFT  * 16.0/35.0);             
    #s_right =  int(s_right * QPPS_RIGHT * 16.0/35.0);             
    #s_front = -int(s_front * QPPS_FRONT);                         
    #s_rear  = -int(s_rear  * QPPS_REAR);
    #rc_frontal.SpeedAccelM1M2(rc_address_frontal, rc_acceleration, s_left, s_right);
    #rc_lateral.SpeedAccelM1M2(rc_address_lateral, rc_acceleration, s_front, s_rear);
    global new_data;
    new_data = True;

def callback_stop(msg):
    rc_frontal.ForwardM1(rc_address_frontal, 0);
    rc_frontal.ForwardM2(rc_address_frontal, 0);
    rc_lateral.ForwardM1(rc_address_lateral, 0);
    rc_lateral.ForwardM2(rc_address_lateral, 0);
    global new_data;
    new_data = True;

def callback_speeds(msg):
    speed_left  = msg.data[0];
    speed_right = msg.data[1];
    speed_front = (speed_right - speed_left)/2.0;
    speed_rear  = (speed_left - speed_right)/2.0;
    send_speeds(speed_left, speed_right, speed_front, speed_rear);
        
def callback_cmd_vel(msg):
    speed_left  = msg.linear.x - msg.angular.z * base_diameter/2.0
    speed_right = msg.linear.x + msg.angular.z * base_diameter/2.0
    speed_front = msg.linear.y + msg.angular.z * base_diameter/2.0
    speed_rear  = msg.linear.y - msg.angular.z * base_diameter/2.0
    send_speeds(speed_left, speed_right, speed_front, speed_rear);
 
def calculate_odometry(pos_x, pos_y, pos_theta, enc_left, enc_right, enc_front, enc_rear):
    TICKS_PER_METER_LATERAL = 336857.5; #Ticks per meter for the slow motors (front and rear)
    TICKS_PER_METER_FRONTAL = 158891.2; #Ticks per meter for the fast motors (left and right)
    enc_left  /= TICKS_PER_METER_FRONTAL;
    enc_right /= TICKS_PER_METER_FRONTAL;
    enc_front /= TICKS_PER_METER_LATERAL;
    enc_rear  /= TICKS_PER_METER_LATERAL;

    delta_theta = (enc_right - enc_left + enc_front - enc_rear)/base_diameter/2.0
    if math.fabs(delta_theta) >= 0.00001:
        rg_x = (enc_left + enc_right)/(2*delta_theta)
        rg_y = (enc_rear + enc_front)/(2*delta_theta)
        delta_x = rg_x*math.sin(delta_theta)     + rg_y*(1-math.cos(delta_theta))
        delta_y = rg_x*(1-math.cos(delta_theta)) + rg_y*math.sin(delta_theta)
    else:
        delta_x = (enc_left + enc_right)/2
        delta_y = (enc_rear + enc_front)/2
    pos_x += delta_x * math.cos(pos_theta) - delta_y * math.sin(pos_theta)
    pos_y += delta_x * math.sin(pos_theta) + delta_y * math.cos(pos_theta)
    pos_theta += delta_theta
    return (pos_x, pos_y, pos_theta);


def main(port_name_frontal, port_name_lateral):
    print "MobileBase.-> INITIALIZING THE AMAZING MOBILE BASE NODE BY MARCOSOFT..."

    #ROS CONNECTION
    rospy.init_node("mobile_base");
    pubBattery = rospy.Publisher("mobile_base/base_battery", Float32, queue_size = 1);
    subStop    = rospy.Subscriber("robot_state/stop", Empty, callback_stop, queue_size=1);
    subSpeeds  = rospy.Subscriber("/hardware/mobile_base/speeds",  Float32MultiArray, callback_speeds, queue_size=1);
    subCmdVel  = rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, callback_cmd_vel, queue_size=1);
    br   = tf.TransformBroadcaster()
    rate = rospy.Rate(20);

    #ROBOCLAW CONNECTION
    rc_frontal.comport = port_name_frontal;
    rc_lateral.comport = port_name_lateral;
    if rc_frontal.Open() != 1:
        print "MobileBase.-> Cannot open Roboclaw for left and right motors on " + rc_frontal.comport;
        return;
    if rc_lateral.Open() != 1:
        print "MobileBase.-> Cannot open Roboclaw for front and rear motors on " + rc_lateral.comport;
        return;
    print "MobileBase.-> Roboclaw frontal open on port " + rc_frontal.comport;
    print "MobileBase.-> Roboclaw lateral open on port " + rc_lateral.comport;
    rc_frontal.ResetEncoders(rc_address_frontal);
    rc_lateral.ResetEncoders(rc_address_lateral);

    #ROBOCLAW CONFIGURATION CONSTANTS
    pos_PID_left  = rc_frontal.ReadM1PositionPID(rc_address_frontal);
    pos_PID_right = rc_frontal.ReadM2PositionPID(rc_address_frontal);
    pos_PID_front = rc_lateral.ReadM1PositionPID(rc_address_lateral);
    pos_PID_rear  = rc_lateral.ReadM2PositionPID(rc_address_lateral);
    vel_PID_left  = rc_frontal.ReadM1VelocityPID(rc_address_frontal);
    vel_PID_right = rc_frontal.ReadM2VelocityPID(rc_address_frontal);
    vel_PID_front = rc_lateral.ReadM1VelocityPID(rc_address_lateral);
    vel_PID_rear  = rc_lateral.ReadM2VelocityPID(rc_address_lateral);
    print "MobileBase.->Position PID constants:  Success   P   I   D   MaxI  Deadzone  MinPos  MaxPos";
    print "MobileBase.->Left Motor:  " + str(pos_PID_left );
    print "MobileBase.->Right Motor: " + str(pos_PID_right);
    print "MobileBase.->Front Motor: " + str(pos_PID_front);
    print "MobileBase.->Rear Motor:  " + str(pos_PID_rear );
    print "MobileBase.->Velocity PID constants:  Success  P   I   D   QPPS"; #QPPS = speed in ticks/s when motor is at full speed
    print "MobileBase.->Left Motor:  " + str(vel_PID_left );
    print "MobileBase.->Right Motor: " + str(vel_PID_right);
    print "MobileBase.->Front Motor: " + str(vel_PID_front);
    print "MobileBase.->Left Motor:  " + str(vel_PID_rear );
    global QPPS_LEFT 
    global QPPS_RIGHT
    global QPPS_FRONT
    global QPPS_REAR 
    QPPS_LEFT  = vel_PID_left[4];
    QPPS_RIGHT = vel_PID_right[4];
    QPPS_FRONT = vel_PID_front[4];
    QPPS_REAR  = vel_PID_rear[4];
    if QPPS_LEFT != QPPS_RIGHT or QPPS_REAR != QPPS_FRONT:
        print "MobileBase.->WARNING: Motors have different max speeds!! Stall condition may occur."
    if rc_frontal.ReadPWMMode(rc_address_frontal)[1] == 1:
        print "MobileBase.->PWM Mode for left and right motors: Sign magnitude"
    else:
        print "MobileBase.->PWM Mode for left and right motors: Locked antiphase"
    if rc_lateral.ReadPWMMode(rc_address_lateral)[1] == 1:
        print "MobileBase.->PWM Mode for front and rear motors: Sign magnitude"
    else:
        print "MobileBase.->PWM Mode for front and rear motors: Locked antiphase"
    
    global new_data;
    new_data = False;
    no_new_data_counter = 5;  #If no new speed data arrive after 5 cycles, motors are stopped.
    robot_x = 0;
    robot_y = 0;
    robot_t = 0;
    encoder_left  = 0;
    encoder_right = 0;
    encoder_front = 0;
    encoder_rear  = 0;
    encoder_last_left  = 0;
    encoder_last_right = 0;
    encoder_last_front = 0;
    encoder_last_rear  = 0;
        
    while not rospy.is_shutdown():
        if not new_data:
            no_new_data_counter -= 1;
            if no_new_data_counter == 0:
                rc_frontal.ForwardM1(rc_address_frontal, 0);
                rc_frontal.ForwardM2(rc_address_frontal, 0);
                rc_lateral.ForwardM1(rc_address_lateral, 0);
                rc_lateral.ForwardM2(rc_address_lateral, 0);
            if no_new_data_counter < -1:
                no_new_data_counter = -1;
        else:
            new_data = False;
            no_new_data_counter = 5;
        #Getting encoders for odometry calculation
        encoder_left  =  rc_frontal.ReadEncM1(rc_address_frontal)[1];
        encoder_right =  rc_frontal.ReadEncM2(rc_address_frontal)[1];
        encoder_front = -rc_lateral.ReadEncM1(rc_address_lateral)[1];
        encoder_rear  = -rc_lateral.ReadEncM2(rc_address_lateral)[1];
        delta_left  = encoder_left  - encoder_last_left;
        delta_right = encoder_right - encoder_last_right;
        delta_front = encoder_front - encoder_last_front;
        delta_rear  = encoder_rear  - encoder_last_rear;
        encoder_last_left  = encoder_left 
        encoder_last_right = encoder_right
        encoder_last_front = encoder_front
        encoder_last_rear  = encoder_rear
        if abs(delta_left) < 24000 and abs(delta_right) < 24000 and abs(delta_front) < 48000 and abs(delta_rear) < 48000:
            (robot_x,robot_y,robot_t)=calculate_odometry(robot_x, robot_y, robot_t, delta_left, delta_right, delta_front, delta_rear);
        else:
            print "MobileBase.->Invalid encoder readings. OMFG!!!!!!!"
            print "Encoders delta: " + str(delta_left) + "\t" + str(delta_right) + "\t" + str(delta_front) + "\t" + str(delta_rear);

        quaternion = tf.transformations.quaternion_from_euler(0, 0, robot_t);
        br.sendTransform((robot_x, robot_y, 0), quaternion, rospy.Time.now(), "base_link", "odom");
        pubBattery.publish(Float32(rc_frontal.ReadMainBatteryVoltage(rc_address_frontal)[1]));
        rate.sleep();

    print "MobileBase.->Stopping motors..."
    rc_frontal.ForwardM1(rc_address_frontal, 0);
    rc_frontal.ForwardM2(rc_address_frontal, 0);
    rc_lateral.ForwardM1(rc_address_lateral, 0);
    rc_lateral.ForwardM2(rc_address_lateral, 0);
        
if __name__ == '__main__':
    if "--help" in sys.argv or "-h" in sys.argv:
        print_help();
        sys.exit();
    port_frontal = "/dev/ttyACM0";
    port_lateral = "/dev/ttyACM1";    
    if "--port1" in sys.argv:
        port_frontal = sys.argv[sys.argv.index("--port1") + 1];
    if "--port2" in sys.argv:
        port_lateral = sys.argv[sys.argv.index("--port2") + 1];
    main(port_frontal, port_lateral);
