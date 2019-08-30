import rospy
import argparse
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import State
from pymavlink.dialects.v20 import common as mavlink

send_command_long = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

if __name__ == "__main__":
    # Init node
    rospy.init_node('mavros_reboot')
    rospy.loginfo('Init mavros reboot node')
    # Send mavlink reboot command 
    send_command_long(False, mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
    rospy.loginfo('Send reboot message')