#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim

import os
import select
import sys
import rclpy
from rclpy.node import Node
import websockets
import asyncio
import json


from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)



class webSockTeleop(Node):
    
    def __init__(self):
        super().__init__('websock_teleop')
        self.qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', self.qos)
        
        self.status = 0
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        self.SignalStatus = 0
        
        start_server = websockets.serve(self.accept, "localhost", 9998);
        asyncio.get_event_loop().run_until_complete(start_server);
        asyncio.get_event_loop().run_forever();

    
    async def accept(self, websocket, path):
        while True:
            stringifiedJsonData = await websocket.recv();
            data = json.loads(stringifiedJsonData)['input']
            print("receive : " + data);
            await websocket.send("echo : " + data);
            if data == 'FORWARD':
                self.target_linear_velocity =\
                    check_linear_limit_velocity(self.target_linear_velocity + LIN_VEL_STEP_SIZE)
                print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif data == 'BACKWARD':
                self.target_linear_velocity =\
                    check_linear_limit_velocity(self.target_linear_velocity - LIN_VEL_STEP_SIZE)
                print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif data == 'LEFT':
                self.target_angular_velocity =\
                    check_angular_limit_velocity(self.target_angular_velocity + ANG_VEL_STEP_SIZE)
                print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif data == 'RIGHT':
                self.target_angular_velocity =\
                    check_angular_limit_velocity(self.target_angular_velocity - ANG_VEL_STEP_SIZE)
                print_vels(self.target_linear_velocity, self.target_angular_velocity)
            elif data == ' ' or data == 'STOP':
                self.target_linear_velocity = 0.0
                self.control_linear_velocity = 0.0
                self.target_angular_velocity = 0.0
                self.control_angular_velocity = 0.0
                print_vels(self.target_linear_velocity, self.target_angular_velocity)
            else:
                print('hi')


            twist = Twist()
            self.control_linear_velocity = make_simple_profile(
                self.control_linear_velocity,
                self.target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))
            twist.linear.x = self.control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.
            self.control_angular_velocity = make_simple_profile(
                self.control_angular_velocity,
                self.target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.control_angular_velocity
            self.pub.publish(twist)

    
    

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    websock_teleop = webSockTeleop()
    rclpy.spin(websock_teleop)

    try:
        while(1):
            key = get_key(settings)
            if (key == '\x03'):
                break

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
