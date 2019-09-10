from pymavlink import mavutil
import time

def set_connection(address):
    print('Connect to {}'.format(address))
    heartbeat = None
    connection = mavutil.mavlink_connection(address, input=False, autoreconnect=True, retries=5)
    # Start a connection listening to a UDP port
    heartbeat = wait_heartbeat(connection)
    print('Connection is set')
    return connection, heartbeat

def send_msg(message_id, connection, message_parameters=[0,0,0,0,0,0,0], wait_ack=True, timeout=2.):
    ack = False
    start_time = time.time()
    confirmation = 0
    while not ack:
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                            message_id, confirmation, *message_parameters)
        if wait_ack:
            print('Attempt {}: send message {} with {},{},{},{},{},{},{} params'.format(confirmation, message_id, *message_parameters))
            ack = check_ack(message_id, connection) or not wait_ack
            confirmation = (confirmation + 1) % 255
            if time.time() - start_time > timeout:
                print('Send calibration start is timed out, try to reconnect')
                connection.reconnect()
                start_time = time.time()
        else: 
            return

def send_msg_calibration(sensor, connection, wait_ack=True):
    print('Send calibrating {} message'.format(sensor))
    send_msg(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, connection, calibration_msg(sensor), wait_ack)

def send_msg_reboot(connection, wait_ack=True):
    print('Send autopilot reboot message')
    send_msg(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, connection, [1,0,0,0,0,0,0], wait_ack)

def wait_heartbeat(connection, timeout_to_reconnect=2.):
    heartbeat = None
    while heartbeat is None:
        try:
            connection.wait_heartbeat(timeout=timeout_to_reconnect)
            heartbeat = connection.messages['HEARTBEAT']
        except:
            print("No heartbeat message received, trying to reconnect")
            connection.reconnect()
    return heartbeat

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
        return ack_msg['command']==command and ack_msg['result']==0
    print ("No ack...")
    return False

def calibration_msg(sensor):
    mavlink_message = [0,0,0,0,0,0,0]
    index, value = {
        'gyro': (0,1),
        'level':(4,2)
    }.get(sensor, (0,0))
    mavlink_message[index]=value
    return mavlink_message

def calibrate(sensor, connection, timeout=30.):
    start_time = time.time()
    send_msg_calibration('stop',connection)
    # Send calibrating sensor msg
    send_msg_calibration(sensor, connection)
    # Wait until the system changes state to calibration
    heartbeat = wait_heartbeat(connection)
    while heartbeat.system_status != 0:   
        heartbeat = wait_heartbeat(connection)
        if time.time() - start_time > timeout:
            print("{} calibration is timed out. Please, check sensor status in QGC.")
            send_msg_calibration('stop', connection)
            return False
    print("Start {} calibration. Please, don't move the drone!".format(sensor))
    start_time = time.time()
    # Wait until the end of the calibration                                
    while heartbeat.system_status != 3:
        print("Calibrating... system status is {}".format(heartbeat.system_status))
        heartbeat = wait_heartbeat(connection) 
        if time.time() - start_time > timeout:
            print("{} calibration is timed out. Please, check sensor status in QGC.")
            send_msg_calibration('stop', connection)
            return False   
    print("Calibration of {} is finished in {} s!".format(sensor, time.time()-start_time))
    return True
