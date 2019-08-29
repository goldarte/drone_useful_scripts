from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import time
import argparse
import inspect

def sensor_pack(sensor):
    mavlink_message = [0,0,0,0,0,0,0]
    index, value = {
        'gyro': (0,1),
        'level':(4,2)
    }.get(sensor, (0,0))
    mavlink_message[index]=value
    return mavlink_message

def check_ack(command, connection, blocking=True, timeout=0.1):
    # Check ACK
    # Wait for ACK command
    try:
        ack_msg = connection.recv_match(type='COMMAND_ACK', blocking=blocking, timeout=timeout)
    except Exception, msg:
        print("Wrong connection {}".format(str(msg)))
        return False
    if ack_msg:
        ack_msg = ack_msg.to_dict()
        print("Received ack for command {}".format(ack_msg['command']))
        return ack_msg['command']==command
    print ("No ack...")
    return False  

def calibrate(sensor, connection, delay=0.01, wait_ack=True, timeout=2.):
    print('Send calibrating {} message'.format(sensor))
    ack = False
    start_time = time.time()
    confirmation = 0
    while not ack:
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, confirmation,
                                            *sensor_pack(sensor))
        if wait_ack:
            print('Send MAV_CMD_PREFLIGHT_CALIBRATION {}, attempt {}'.format(sensor, confirmation))
            ack = check_ack(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, connection) or not wait_ack
            confirmation = (confirmation + 1) % 255
            if time.time() - start_time > timeout:
                print('Send calibration start is timed out, try to reconnect')
                connection.reconnect()
                start_time = time.time()
        else: 
            ack = True

def set_connection(address, delay=0.01):

    print('Connect to {}'.format(address))

    heartbeat = None

    connection = mavutil.mavlink_connection(address, input=False, autoreconnect=True, retries=6)
    # Start a connection listening to a UDP port
    while heartbeat is None:
        # Wait for the first heartbeat 
        # This sets the system and component ID of remote system for the link
        try:
            print(connection.wait_heartbeat(timeout=2.))
            heartbeat = connection.messages['HEARTBEAT']
            #print("Received heartbeat from {}: system {} component {}".format(args.hostname, connection.target_system, connection.target_component))
        except:
            print("No heartbeat message received, trying to reconnect")
            connection.reconnect()

    print('Connection is set')

    return connection, heartbeat

def wait_heartbeat(connection, timeout_to_reconnect=2.):
    heartbeat = None
    while heartbeat is None:
        try:
            connection.wait_heartbeat(timeout=timeout_to_reconnect)
            heartbeat = connection.messages['HEARTBEAT']
            #print('MAV_STATE is {}'.format(heartbeat.system_status))
        except:
            print("No heartbeat message received, trying to reconnect")
            connection.reconnect()
    return heartbeat

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('hostname', nargs='?', default='192.168.11.1',
                        help="Address for connection")
    parser.add_argument('--port', default=5760, type=int,
                        help="Port, default is 5760")
    parser.add_argument('--type', default='tcp', type=str,
                        help="Type of connection, default is tcp")
    parser.add_argument('--tries', default=10, type=int,
                        help="Number of tries, default is 3")
    parser.add_argument('--sensor', default='gyro', type=str,
                        help="Sensor to calibrate. Available are gyro or level.")


    # Assemble connection string
    args = parser.parse_args()
    connection_string = "{}:{}:{}".format(args.type,args.hostname,args.port)

    # Initiate connection and receive a heartbeat
    connection, heartbeat = set_connection(connection_string)

    # Send stop calibration message
    calibrate('stop', connection)

    # Check --sensor parameter
    if tuple(sensor_pack(args.sensor))==(0,0,0,0,0,0,0):
        print("Calibration is stopped. Please check --sensor option value (now is {})".format(args.sensor))
        quit()

    # Calibrate sensor
    calibrate(args.sensor, connection)

    # Wait until the system changes state to calibration
    while heartbeat.system_status != 0:   
        heartbeat = wait_heartbeat(connection)

    print("Start {} calibration. Please, don't move the drone!".format(args.sensor))

    # Wait until the end of the calibration                                
    while heartbeat.system_status != 3:
        print("Calibrating...")
        heartbeat = wait_heartbeat(connection)    
    print("Calibration of {} is finished!".format(args.sensor))

    # Close the connection
    connection.close()
