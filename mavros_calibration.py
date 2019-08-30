import rospy
import argparse
import time
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import State
from pymavlink.dialects.v20 import common as mavlink
from mavlink_calibration import sensor_pack

send_command_long = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

system_status = mavlink.MAV_STATE_STANDBY

def state_callback(data):
    global system_status
    system_status = data.system_status


if __name__ == "__main__":
    # Init node
    rospy.init_node('mavros_calibration')
    rospy.loginfo('Init mavros calibration node')
    # Add parser
    parser = argparse.ArgumentParser(description='Send calibration command to px4 device. Options are: gyro or level. Default id gyro.')
    parser.add_argument('--sensor', default='gyro', type=str,
                        help="Sensor to calibrate. Available are gyro or level. Default is gyro.")   
    args = parser.parse_args()
    # Check --sensor parameter
    if tuple(sensor_pack(args.sensor))==(0,0,0,0,0,0,0):
        rospy.loginfo("Calibration is stopped. Please check --sensor option value (now is {})".format(args.sensor))
        rospy.signal_shutdown("Off")
        quit()
    # Subscribe to heartbeat topic
    rospy.Subscriber('mavros/state', State, state_callback)
    # Make calibration message
    calibration_message = sensor_pack(args.sensor)
    # Send mavlink calibration command 
    send_command_long(False, mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, *calibration_message)
    rospy.loginfo('Send {} calibration message'.format(args.sensor))
    # Wait until system status to uninit (during calibration on px4)
    while system_status != mavlink.MAV_STATE_UNINIT:
        rospy.sleep(0.1)
    rospy.loginfo("Start {} calibration. Please, don't move the drone!".format(args.sensor))
    # Wait until the end of the calibration
    while system_status != mavlink.MAV_STATE_STANDBY:
        rospy.sleep(0.1)
    rospy.loginfo("Calibration is finished!")
    
