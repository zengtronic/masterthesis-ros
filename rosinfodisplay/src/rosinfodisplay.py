#!/usr/bin/env python3
import sys
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106, ssd1306
from PIL import Image, ImageDraw, ImageFont
import time
from datetime import datetime
import traceback
import json
import yaml
import os
import rospy

import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
from project_messages.msg import Telemetry, Balancing, Command, DriveControl, ParameterUpdate, ParameterFeedback

telemetry = None


def debug(text):
    date = datetime.now().strftime("%H:%M:%S.%f")
    print(date + ": " + text)

def telemetry_callback(msg):
    global telemetry
    telemetry = msg


def main():
    """
    main function
    """
    print("Starting rosinfodisplay node")
    rospy.init_node('rosinfodisplay', anonymous=True, disable_signals=True)
    
    serial = i2c(port=3, address=0x3C)
    device = ssd1306(serial)
    oled_font = ImageFont.truetype('/home/ubuntu/startup/FreeSans.ttf', 14)

    command_subscriber = rospy.Subscriber('/robot/telemetry', Telemetry, telemetry_callback, queue_size=None)
    while 1:
        if telemetry is not None:
            with canvas(device) as draw:      
                  draw.rectangle((0,0,127, 15), outline = "white", fill = "white")
                  draw.text((20, 0),"TELEMETRY", font = oled_font, fill = "black")
                  draw.text((0, 15),"TILT: " + str(round(telemetry.pitch,2)) + "°", font = oled_font, fill = "white")
        else:
            with canvas(device) as draw:      
                  draw.rectangle((0,0,127, 15), outline = "white", fill = "white")
                  draw.text((20, 0),"TELEMETRY", font = oled_font, fill = "black")
                  draw.text((0, 15),"Device offline", font = oled_font, fill = "white")
        time.sleep(0.016)


#################
# execution     #
#################
if __name__ == '__main__':
    main()
