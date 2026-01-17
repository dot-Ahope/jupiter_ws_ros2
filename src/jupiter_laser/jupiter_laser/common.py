#!/usr/bin/env python3
# coding:utf-8
import rclpy
from rclpy.node import Node
from jupiter_msgs.srv import Buzzer
from geometry_msgs.msg import Twist
from jupiter_msgs.msg import JoyState


class ROSCtrl(Node):
    def __init__(self, node_name='ros_ctrl'):
        super().__init__(node_name)
        self.Joy_active = False
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 3)
        self.sub_JoyState = self.create_subscription(
            JoyState,
            '/JoyState',
            self.JoyStateCallback,
            3
        )
        self.Buzzer_client = self.create_client(Buzzer, "/Buzzer")

    def JoyStateCallback(self, msg):
        if not isinstance(msg, JoyState): return
        self.Joy_active = msg.state

    async def Buzzer_srv(self, value):
        '''
        Buzzer control
        :param value:
        [0: close,
        1: Keep ringing,
        >=10: automatically closes after xx milliseconds (value is a multiple of 10)]
        '''
        if not self.Buzzer_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Buzzer service not available')
            return False
            
        request = Buzzer.Request()
        request.buzzer = value
        try:
            response = await self.Buzzer_client.call_async(request)
            return response.result
        except Exception as e:
            self.get_logger().error(f"Buzzer error: {str(e)}")
        return False

    def cancel(self):
        # Create a coroutine to call Buzzer service
        rclpy.get_global_executor().create_task(self.Buzzer_srv(0))
        
        # Stop robot
        self.pub_vel.publish(Twist())
        
        # Cleanup
        self.destroy_node()
        self.get_logger().info("Shutting down this node.")


class SinglePID:
    def __init__(self, P=0.1, I=0.0, D=0.1):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("init_pid: ", P, I, D)
        self.pid_reset()

    def Set_pid(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("set_pid: ", P, I, D)
        self.pid_reset()

    def pid_compute(self, target, current):
        self.error = target - current
        self.intergral += self.error
        self.derivative = self.error - self.prevError
        result = self.Kp * self.error + self.Ki * self.intergral + self.Kd * self.derivative
        self.prevError = self.error
        return result

    def pid_reset(self):
        self.error = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0
