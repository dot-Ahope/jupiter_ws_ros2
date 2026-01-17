#!/usr/bin/env python3
# encoding: utf-8
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your SLAM-Bot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
    'I': (1, 0),
    'O': (1, -1),
    'J': (0, 1),
    'L': (0, -1),
    'U': (1, 1),
    'M': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

class JupiterKeyboard(Node):
    def __init__(self):
        super().__init__('jupiter_keyboard')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0
        self.th = 0
        self.status = 0
        self.count = 0
        self.acc = 0.1
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)

def main(args=None):
    if args is None:
        args = sys.argv
        
    rclpy.init(args=args)
    
    node = JupiterKeyboard()
    
    try:
        print(msg)
        print(node.vels(node.speed, node.turn))
        
        while True:
            key = node.getKey()
            
            if key in moveBindings.keys():
                node.x = moveBindings[key][0]
                node.th = moveBindings[key][1]
                node.count = 0
            elif key in speedBindings.keys():
                node.speed = node.speed * speedBindings[key][0]
                node.turn = node.turn * speedBindings[key][1]
                node.count = 0
                print(node.vels(node.speed, node.turn))
                if node.status == 14:
                    print(msg)
                node.status = (node.status + 1) % 15
            elif key == ' ' or key == 'k':
                node.x = 0
                node.th = 0
                node.control_speed = 0
                node.control_turn = 0
            else:
                node.count = node.count + 1
                if node.count > 4:
                    node.x = 0
                    node.th = 0
                if key == '\x03':
                    break
                    
            # Create and publish Twist message
            twist = Twist()
            
            target_speed = node.speed * node.x
            target_turn = node.turn * node.th
            
            if target_speed > node.control_speed:
                node.control_speed = min(target_speed,
                                       node.control_speed + 0.02)
            elif target_speed < node.control_speed:
                node.control_speed = max(target_speed,
                                       node.control_speed - 0.02)
            else:
                node.control_speed = target_speed
            
            if target_turn > node.control_turn:
                node.control_turn = min(target_turn,
                                      node.control_turn + 0.1)
            elif target_turn < node.control_turn:
                node.control_turn = max(target_turn,
                                      node.control_turn - 0.1)
            else:
                node.control_turn = target_turn
                
            twist.linear.x = node.control_speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = node.control_turn
            
            node.publisher.publish(twist)
            
    except Exception as e:
        print(e)
        
    finally:
        # Stop the robot
        twist = Twist()
        node.publisher.publish(twist)
        
        # Reset terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

