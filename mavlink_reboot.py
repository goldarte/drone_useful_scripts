from pymavlink import mavutil
import argparse
import time
from mavlink_calibration import set_connection, wait_heartbeat, check_ack

def reboot(connection, delay=0.01, wait_ack=True, timeout=2.):
    print('Send reboot message')
    ack = False
    start_time = time.time()
    confirmation = 0
    while not ack:
        connection.reboot_autopilot()
        if wait_ack:
            print('Send MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, attempt {}'.format(confirmation))
            ack = check_ack(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, connection) or not wait_ack
            confirmation = (confirmation + 1) % 255
            if time.time() - start_time > timeout:
                print('Send calibration start is timed out, try to reconnect')
                connection.reconnect()
                start_time = time.time()
        else: 
            ack = True

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Send calibration command to px4 device. Options are: gyro or level.')
    parser.add_argument('hostname', nargs='?', default='192.168.11.1',
                        help="Address for connection")
    parser.add_argument('--port', default=5760, type=int,
                        help="Port, default is 5760")
    parser.add_argument('--type', default='tcp', type=str,
                        help="Type of connection, default is tcp")

    # Assemble connection string
    args = parser.parse_args()
    connection_string = "{}:{}:{}".format(args.type,args.hostname,args.port)

    # Start a connection and wait first heartbeat
    connection, heartbeat = set_connection(connection_string)
    print("Got heartbeat from {}: system {} component {}".format(args.hostname, connection.target_system, connection.target_system))

    reboot(connection)