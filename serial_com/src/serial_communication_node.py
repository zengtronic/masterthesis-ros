#!/usr/bin/env python3
import sys
import time
from datetime import datetime
import traceback
import serial.tools.list_ports
import json
import yaml
import os
import rospy

import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
from project_messages.msg import Telemetry, Balancing, Command, DriveControl, ParameterUpdate, ParameterFeedback

settings = []

throttlefactor = 1
steeringfactor = 1

buffer = []

LENGTH_FULL_TELEMETRY = 48
LENGTH_SMALL_PACKET = 24

zerocommand = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
commandblock = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
commandtosend = False


def int2bytes(integer):
    byte1 = (integer >> 8) & 0xFF
    byte2 = (integer % 256) & 0xFF
    return [byte1, byte2]


def bytes2int(bytes):
    int_p1 = (bytes[0] * 256)
    int_p2 = (bytes[1])
    return int_p1 + int_p2


def debug(text):
    date = datetime.now().strftime("%H:%M:%S.%f")
    print(date + ": " + text)


def load_settings(file):
    global settings
    # Open settings
    if os.path.isfile(file):
        settings_file = open(file).read()
        settings = json.loads(settings_file)
        print("Loaded settings.")
    else:
        print("No settings found. Bye!")
        exit(0)


def command_callback(msg):
    global commandtosend, commandblock
    print(f"Received Command: {msg.commandname}")
    commandblock = zerocommand
    commandblock[0] = msg.command
    commandblock[1:17] = msg.data
    commandtosend = True


# Callback for the driving messages
def drivecontrol_callback(msg):
    global commandtosend, commandblock
    print("received drive message")
    # Calculate the right value
    throttle = round(msg.throttle * throttlefactor * 1000)
    steering = round(msg.steering * steeringfactor * 1000)
    # Prepare for command block
    throttle_abs = abs(throttle)
    throttle_sign = True if throttle < 0 else False
    steering_abs = abs(steering)
    steering_sign = True if steering < 0 else False
    var1 = int2bytes(throttle_abs)
    var2 = int2bytes(steering_abs)
    # Set command block
    commandblock = zerocommand
    commandblock[0] = 128
    commandblock[1] = throttle_sign
    commandblock[2] = var1[0]
    commandblock[3] = var1[1]
    commandblock[4] = steering_sign
    commandblock[5] = var2[0]
    commandblock[6] = var2[1]
    # Data ready to send flag
    commandtosend = True


# Callback for parameter updates
def parameterupdate_callback(msg):
    global commandtosend, commandblock
    print("received parameter update for " + str(msg.name))
    if msg.setting == 1:
        # update pid constants
        balancing_kp = round(msg.float1 * 1000)
        balancing_ki = round(msg.float2 * 1000)
        balancing_kd = round(msg.float3 * 1000)
        balancing_mirror = msg.bool1
        speed_kp = round(msg.float4 * 1000)
        speed_ki = round(msg.float5 * 1000)
        speed_kd = round(msg.float6 * 1000)
        speed_mirror = msg.bool2

        var1 = int2bytes(balancing_kp)
        var2 = int2bytes(balancing_ki)
        var3 = int2bytes(balancing_kd)
        var4 = int2bytes(speed_kp)
        var5 = int2bytes(speed_ki)
        var6 = int2bytes(speed_kd)
        # Set command block
        commandblock = zerocommand
        commandblock[0] = 1
        commandblock[1] = var1[0]
        commandblock[2] = var1[1]
        commandblock[3] = var2[0]
        commandblock[4] = var2[1]
        commandblock[5] = var3[0]
        commandblock[6] = var3[1]
        commandblock[7] = balancing_mirror
        commandblock[8] = var4[0]
        commandblock[9] = var4[1]
        commandblock[10] = var5[0]
        commandblock[11] = var5[1]
        commandblock[12] = var6[0]
        commandblock[13] = var6[1]
        commandblock[14] = speed_mirror
        # Data ready to send flag
        commandtosend = True

    elif msg.setting == 2:
        # update angle offset
        offset = round(msg.float1 * 1000)
        offset_abs = abs(offset)
        offset_sign = True if offset < 0 else False
        var1 = int2bytes(offset_abs)
        commandblock = zerocommand
        commandblock[0] = 6
        commandblock[1] = offset_sign
        commandblock[2] = var1[0]
        commandblock[3] = var1[1]
        commandblock[4] = msg.bool1
        # Data ready to send flag
        commandtosend = True

    elif msg.setting == 3:
        # request parameter feedback
        commandblock = zerocommand
        commandblock[0] = 7
        # Data ready to send flag
        commandtosend = True
        
    elif msg.setting == 4:
        # update drive mode
        print(msg)
        var1 = int2bytes(msg.int1)
        commandblock = zerocommand
        commandblock[0] = 96
        commandblock[1] = var1[0]
        commandblock[2] = var1[1]
        # Data ready to send flag
        commandtosend = True
    elif msg.setting == 5:
        # update linear parameters
        step = round(msg.float1 * 1000)
        back_mult = round(msg.float2 * 1000)
        var1 = int2bytes(step)
        var2 = int2bytes(back_mult)
        
        commandblock[0] = 2
        commandblock[1] = var1[0]
        commandblock[2] = var1[1]
        commandblock[3] = var2[0]
        commandblock[4] = var2[1]
        # Data ready to send flag
        commandtosend = True


def main():
    global settings, commandtosend
    """
    main function
    """
    print("Starting serial communication node")
    rospy.init_node('ser_com', anonymous=True, disable_signals=True)

    settingsfile = rospy.get_param("~settings_file")
    load_settings(settingsfile)
    ser = None
    try:
        ser = serial.Serial(
            port="/dev/ttyAMA1",  # settings['port'],  # port.name,
            baudrate=115200,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
    except:
        print(f"ERROR: Could not open port {settings['port']}. Please look in the settings file if port is correct.")
        print(traceback.format_exc())
        sys.exit(1)

    command_subscriber = rospy.Subscriber('/robot/command', Command, command_callback, queue_size=None)
    drive_subscriber = rospy.Subscriber('/robot/drive', DriveControl, drivecontrol_callback, queue_size=None)
    parameter_subscriber = rospy.Subscriber('/robot/parameters', ParameterUpdate, parameterupdate_callback,
                                            queue_size=None)
    balancing_publisher = rospy.Publisher('/robot/telemetry', Telemetry, queue_size=2)
    parameterfeedback_publisher = rospy.Publisher('/robot/parameters/feedback', ParameterFeedback, queue_size=2)

    while (1):
        if commandtosend:
            print(f"New command to send to the robot: {commandblock}")
            ser.write(bytes(commandblock))
            commandtosend = False
        if ser.in_waiting:
            char = ser.read(1)
            buffer.append(char[0])
            if len(buffer) > LENGTH_FULL_TELEMETRY:
                buffer.pop(0)

            # for x in buffer:
            #    print(x, end=' ')
            # print(len(buffer))
            if len(buffer) >= LENGTH_SMALL_PACKET:
                if buffer[-1] == 64 and buffer[-LENGTH_SMALL_PACKET] == 63:
                    x = -LENGTH_SMALL_PACKET
                    if buffer[x + 1] == 2:
                        debug("Got small telemetry package")
                        msg = Telemetry()
                        msg.small = True
                        msg.yaw = int.from_bytes([buffer[x + 2], buffer[x + 3]], "little", signed=True) / 100
                        msg.pitch = int.from_bytes([buffer[x + 4], buffer[x + 5]], "little", signed=True) / 100
                        msg.roll = int.from_bytes([buffer[x + 6], buffer[x + 7]], "little", signed=True) / 100
                        msg.motor_left = int.from_bytes([buffer[x + 8], buffer[x + 9]], "little", signed=True) / 100
                        msg.motor_right = int.from_bytes([buffer[x + 10], buffer[x + 11]], "little", signed=True) / 100
                        msg.speed_actual = int.from_bytes([buffer[x + 12], buffer[x + 13]], "little", signed=True) / 100
                        msg.angle_actual = int.from_bytes([buffer[x + 14], buffer[x + 15]], "little", signed=True) / 100
                        msg.throttle = int.from_bytes([buffer[x + 16], buffer[x + 17]], "little", signed=True) / 100
                        msg.steering = int.from_bytes([buffer[x + 18], buffer[x + 19]], "little", signed=True) / 100
                        msg.fallen = bool(buffer[x + 20])
                        balancing_publisher.publish(msg)
                        buffer.clear()
                    elif buffer[-LENGTH_SMALL_PACKET + 1] == 3:
                        debug("Got parameter package")
                        msg = ParameterFeedback()
                        msg.balancing_kp = int.from_bytes([buffer[x + 2], buffer[x + 3]], "little", signed=True) / 1000
                        msg.balancing_ki = int.from_bytes([buffer[x + 4], buffer[x + 5]], "little", signed=True) / 1000
                        msg.balancing_kd = int.from_bytes([buffer[x + 6], buffer[x + 7]], "little", signed=True) / 1000
                        msg.speed_kp = int.from_bytes([buffer[x + 8], buffer[x + 9]], "little", signed=True) / 1000
                        msg.speed_ki = int.from_bytes([buffer[x + 10], buffer[x + 11]], "little", signed=True) / 1000
                        msg.speed_kd = int.from_bytes([buffer[x + 12], buffer[x + 13]], "little", signed=True) / 1000
                        msg.angle_offset = int.from_bytes([buffer[x + 14], buffer[x + 15]], "little", signed=True) / 1000
                        msg.balancing_mirror = bool(buffer[x+17])
                        msg.speed_mirror = bool(buffer[x+16])
                        msg.linear_step = int.from_bytes([buffer[x + 18], buffer[x + 19]], "little", signed=True) / 1000
                        msg.linear_back_multiplicator = int.from_bytes([buffer[x + 20], buffer[x + 21]], "little", signed=True) / 1000
                        msg.auto_offset_active = bool(buffer[x+22])
                        
                        parameterfeedback_publisher.publish(msg)
                        buffer.clear()

                if len(buffer) == LENGTH_FULL_TELEMETRY:
                    x = -LENGTH_FULL_TELEMETRY
                    if buffer[-1] == 64 and buffer[0] == 63:
                        debug("Got full telemetry package")
                        msg = Telemetry()
                        msg.small = False
                        msg.yaw = int.from_bytes([buffer[x + 2], buffer[x + 3]], "little", signed=True) / 100
                        msg.pitch = int.from_bytes([buffer[x + 4], buffer[x + 5]], "little", signed=True) / 100
                        msg.roll = int.from_bytes([buffer[x + 6], buffer[x + 7]], "little", signed=True) / 100

                        msg.gyro_x = int.from_bytes([buffer[x + 8], buffer[x + 9]], "little", signed=True) / 100
                        msg.gyro_y = int.from_bytes([buffer[x + 10], buffer[x + 11]], "little", signed=True) / 100
                        msg.gyro_z = int.from_bytes([buffer[x + 12], buffer[x + 13]], "little", signed=True) / 100
                        msg.acc_x = int.from_bytes([buffer[x + 14], buffer[x + 15]], "little", signed=True) / 100
                        msg.acc_y = int.from_bytes([buffer[x + 16], buffer[x + 17]], "little", signed=True) / 100
                        msg.acc_z = int.from_bytes([buffer[x + 18], buffer[x + 19]], "little", signed=True) / 100

                        msg.balancing_in = int.from_bytes([buffer[x + 20], buffer[x + 21]], "little", signed=True) / 100
                        msg.balancing_out = int.from_bytes([buffer[x + 22], buffer[x + 23]], "little", signed=True) / 100
                        msg.balancing_set = int.from_bytes([buffer[x + 24], buffer[x + 25]], "little", signed=True) / 100
                        msg.speed_in = int.from_bytes([buffer[x + 26], buffer[x + 27]], "little", signed=True) / 100
                        msg.speed_out = int.from_bytes([buffer[x + 28], buffer[x + 29]], "little", signed=True) / 100
                        msg.speed_set = int.from_bytes([buffer[x + 30], buffer[x + 31]], "little", signed=True) / 100

                        msg.angle_actual = int.from_bytes([buffer[x + 32], buffer[x + 33]], "little", signed=True) / 100
                        msg.speed_actual = int.from_bytes([buffer[x + 34], buffer[x + 35]], "little", signed=True) / 100

                        msg.throttle = int.from_bytes([buffer[x + 36], buffer[x + 37]], "little", signed=True) / 100
                        msg.steering = int.from_bytes([buffer[x + 38], buffer[x + 39]], "little", signed=True) / 100

                        msg.motor_left = int.from_bytes([buffer[x + 40], buffer[x + 41]], "little", signed=True) / 100
                        msg.motor_right = int.from_bytes([buffer[x + 42], buffer[x + 43]], "little", signed=True) / 100
                        msg.fallen = bool(buffer[x + 44])
                        balancing_publisher.publish(msg)
                        buffer.clear()

        else:
            time.sleep(0.01)


#################
# execution     #
#################
if __name__ == '__main__':
    main()
