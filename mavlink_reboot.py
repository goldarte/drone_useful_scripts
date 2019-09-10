from pymavlink_wrapper import *
import argparse

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

    send_msg_reboot(connection)

    connection.close()