from pymavlink import mavutil
import time
import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('hostname', nargs='?', default='192.168.11.1',
                        help="Filename of route csv table")
    parser.add_argument('--tries', default=10, type=int,
                        help="Number of tries, default is 3")

    args = parser.parse_args()

    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection('tcp:{}:5760'.format(args.hostname), input=False)

    # Wait for the first heartbeat 
    # This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print("Received heartbeat from {}: system {} component {}".format(args.hostname, the_connection.target_system, the_connection.target_component))

    try:
        heartbeat = the_connection.messages['HEARTBEAT']
        #print('First MAV_STATE is {}'.format(heartbeat.system_status))
    except:
        print("No heartbeat message received")

    while heartbeat.system_status != 0:
        for i in range(args.tries):
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                        0, 0, 0, 0, 0, 0, 0) 
            time.sleep(0.001)   
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                1, 0, 0, 0, 0, 0, 0)
        time.sleep(0.001) 
        the_connection.wait_heartbeat()
        try:
            heartbeat = the_connection.messages['HEARTBEAT']
            #print('MAV_STATE is {}'.format(heartbeat.system_status))
        except:
            print("No heartbeat message received")

    print("Start gyro calibration. Please, don't move the drone!")
                               
    while heartbeat.system_status != 3:
        the_connection.wait_heartbeat()
        try:
            heartbeat = the_connection.messages['HEARTBEAT']
            #print('MAV_STATE is {}'.format(heartbeat.system_status))
        except:
            print("No heartbeat message received")
    
    print("Gyro calibration is finished!")