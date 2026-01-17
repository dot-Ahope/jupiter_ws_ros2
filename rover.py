#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_cli_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # =======================================================
        # 2단계 속도 모드 설정
        # =======================================================
        # corrected (mode) speeds: user-facing/corrected values (0.1, 0.4)
        self.corrected_profiles = {
            1: (0.1, 0.2),  # corrected/mode values (linear, angular)
            2: (0.4, 0.4),
        }

        # effective (publish) speeds to use when publishing and timing (actual robot speed)
        # (linear, angular)
        self.effective_profiles = {
            1: (0.17, 0.17),
            2: (0.5, 0.5),
        }
        # =======================================================

        # nominal vs effective runtime fields
        self.linear_speed_nominal = 0.0
        self.angular_speed_nominal = 0.0
        # effective speeds used for timing/publishing
        self.linear_speed = 0.0
        self.angular_speed = 0.0  # rad/s

        # If True, profiles' angular values are provided in degrees/sec and will be converted to rad/s
        self.angular_in_degrees = False

        # calibration multiplier for angular speed (adjust when robot turns faster/slower)
        self.angular_scale = 1

        # current speed level placeholder
        self.speed_level = None

        # set default speed level
        self.set_speed_level(1)

        self.get_logger().info('Robot Controller (2-speed mode) is ready.')

    def set_speed_level(self, level):
        if level in self.corrected_profiles:
            self.speed_level = level
            # corrected/mode values (user-facing)
            self.linear_speed_nominal, self.angular_speed_nominal = self.corrected_profiles[level]
            # effective (publish) values used for motion
            if level in self.effective_profiles:
                eff_linear, eff_angular = self.effective_profiles[level]
                self.linear_speed = eff_linear
                # convert angular if profiles are in degrees
                if self.angular_in_degrees:
                    eff_ang_rad = math.radians(eff_angular)
                else:
                    eff_ang_rad = eff_angular
                # apply global angular scale calibration
                self.angular_speed = eff_ang_rad * self.angular_scale
            else:
                # fallback to nominal if no effective profile provided
                self.linear_speed = self.linear_speed_nominal
                # nominal angular may also be in degrees if flag set
                base_ang = math.radians(self.angular_speed_nominal) if self.angular_in_degrees else self.angular_speed_nominal
                self.angular_speed = base_ang * self.angular_scale

            # log shows both representations for clarity
            ang_nominal_display = f"{self.angular_speed_nominal} deg/s" if self.angular_in_degrees else f"{self.angular_speed_nominal} rad/s"
            ang_eff_display = f"{math.degrees(self.angular_speed):.4f} deg/s / {self.angular_speed:.4f} rad/s"
            self.get_logger().info(
                f"Speed level set to {level}. Corrected(mode) Linear: {self.linear_speed_nominal} m/s, "
                f"Corrected(mode) Angular: {ang_nominal_display} | "
                f"Effective Linear: {self.linear_speed} m/s, Effective Angular: {ang_eff_display}"
            )
        else:
            self.get_logger().warn(f"Invalid speed level: {level}. Please use {list(self.corrected_profiles.keys())}.")

    def set_angular_scale(self, scale):
        try:
            val = float(scale)
        except Exception:
            self.get_logger().error(f"Invalid angular scale value: {scale}")
            return
        self.angular_scale = val
        # re-apply current speed level so angular_speed is recalculated
        if self.speed_level is not None:
            self.set_speed_level(self.speed_level)
        self.get_logger().info(f"Angular scale set to {self.angular_scale}")

    def move_robot(self, distance_meters):
        twist_msg = Twist()
        # publish nominal (corrected) speed, but compute duration using effective speed
        if distance_meters > 0:
            twist_msg.linear.x = self.linear_speed_nominal
        else:
            twist_msg.linear.x = -self.linear_speed_nominal

        # effective linear_speed is used for timing
        if self.linear_speed == 0:
            self.get_logger().error("Cannot move, linear speed is zero.")
            return

        duration = abs(distance_meters) / self.linear_speed / 2
        self.get_logger().info(f"Moving {'forward' if distance_meters > 0 else 'backward'} by {abs(distance_meters)}m for {duration:.2f} seconds.")

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(twist_msg)
            time.sleep(0.05)
        self.stop_robot()

    def rotate_robot(self, angle_degrees):
        twist_msg = Twist()
        angle_radians = math.radians(angle_degrees)

        # publish using the effective angular speed (rad/s) so duration and published speed match
        if angle_radians > 0:
            twist_msg.angular.z = self.angular_speed
        else:
            twist_msg.angular.z = -self.angular_speed

        if self.angular_speed == 0:
            self.get_logger().error("Cannot rotate, angular speed is zero.")
            return

        duration = abs(angle_radians) / self.angular_speed / 6.5
        self.get_logger().info(f"Rotating by {angle_degrees} degrees for {duration:.2f} seconds.")

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(twist_msg)
            time.sleep(0.05)
        self.stop_robot()

    def stop_robot(self):
        twist_msg = Twist()
        self.publisher_.publish(twist_msg)
        self.get_logger().info('Action complete. Robot stopped.')

def print_usage():
    print("Usage: python your_script_name.py [command value] ...")
    print("\nAvailable Commands:")
    print("  move <distance_meters>   : Move the robot forward/backward.")
    print("  rot <angle_degrees>      : Rotate the robot counter-clockwise/clockwise.")
    print("  vel <level>              : Set the speed level (1:slow, 2:medium).") # 사용법 안내 수정
    print("\nExample:")
    print("  python your_script_name.py vel 1 move 0.5")


def main(args=None):
    if len(sys.argv) < 2:
        print_usage()
        return

    rclpy.init(args=args)
    node = RobotController()
    time.sleep(1) 

    try:
        command_args = sys.argv[1:]
        i = 0
        while i < len(command_args):
            command = command_args[i]
            
            if i + 1 >= len(command_args):
                print(f"Error: Missing value for command '{command}'")
                break
            
            value_str = command_args[i+1]
            
            if command == "move":
                node.move_robot(float(value_str))
            elif command == "rot":
                node.rotate_robot(float(value_str))
            elif command == "vel":
                node.set_speed_level(int(value_str))
            else:
                print(f"Error: Unknown command '{command}'")
                print_usage()
                break
            
            i += 2
            if i < len(command_args):
                time.sleep(1)

    except (ValueError, IndexError):
        print("Error: Invalid command or value.")
        print_usage()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()