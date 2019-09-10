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
    parser.add_argument('--sensor', default='gyro', type=str,
                        help="Sensor to send_msg_calibration. Available are gyro or level. Default is gyro.")


    # Assemble connection string
    args = parser.parse_args()
    connection_string = "{}:{}:{}".format(args.type,args.hostname,args.port)

    # Initiate connection and receive a heartbeat
    connection, heartbeat = set_connection(connection_string)

    # Check --sensor parameter
    if tuple(calibration_msg(args.sensor))==(0,0,0,0,0,0,0):
        send_msg_calibration('stop',connection)
        print("Calibration is stopped. Please check --sensor option value (now is {})".format(args.sensor))
        quit()

    calibrate(args.sensor,connection)

    # Close the connection
    connection.close()
