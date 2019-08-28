from pymavlink import mavutil
import argparse


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('hostname', nargs='?', default='192.168.11.1',
                        help="Filename of route csv table")
    args = parser.parse_args()

    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection('tcp:{}:5760'.format(args.hostname), input=False)

    # Wait for the first heartbeat 
    #   This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print("Heartbeat from {}: system {} component {}".format(args.hostname, the_connection.target_system, the_connection.target_system))

    the_connection.reboot_autopilot()