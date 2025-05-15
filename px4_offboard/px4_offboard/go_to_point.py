#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleAttitude, VehicleCommand, VehicleOdometry
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool

from math import pi

class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.offboard_velocity_sub = self.create_subscription(Twist, '/offboard_velocity_cmd', self.offboard_velocity_callback, qos_profile)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile)
        self.arm_sub = self.create_subscription(Bool, '/arm_message', self.arm_message_callback, qos_profile)
        self.goto_sub = self.create_subscription(Bool, '/go_to_point', self.goto_callback, qos_profile)

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.arm_timer_ = self.create_timer(0.1, self.arm_timer_callback)
        self.timer = self.create_timer(0.02, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.yaw = 0.0
        self.trueYaw = 0.0
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        self.current_state = "IDLE"
        self.last_state = self.current_state

        self.odom_position = np.zeros(3)
        self.odom_orientation_q = np.array([0.0, 0.0, 0.0, 1.0])
        self.initial_position = None
        self.go_to_point = False
        self.goal_body = np.zeros(3)
        self.Kp = 0.5

    def arm_message_callback(self, msg):
        self.arm_message = msg.data

    def goto_callback(self, msg):
        if msg.data and self.initial_position is not None:
            goal_enu = np.array([5.0, 5.0, 5.0])
            R_enu_to_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
            goal_ned = R_enu_to_ned @ goal_enu
            self.goal_body = goal_ned - self.initial_position
            self.go_to_point = True
            self.get_logger().info(f"Objetivo ENU: {goal_enu} → NED: {goal_ned} → relativo: {self.goal_body}")

    def odometry_callback(self, msg):
        current_ned = np.array([msg.position[0], msg.position[1], msg.position[2]])
        if self.initial_position is None:
            self.initial_position = current_ned.copy()
            self.get_logger().info(f"Desfase inicial registrado: {self.initial_position}")
        self.odom_position = current_ned
        self.odom_orientation_q = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])

    def arm_timer_callback(self):
        match self.current_state:
            case "IDLE":
                if self.flightCheck and self.arm_message:
                    self.current_state = "ARMING"
            case "ARMING":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10:
                    self.current_state = "TAKEOFF"
                self.arm()
            case "TAKEOFF":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                    self.current_state = "LOITER"
                self.arm()
                self.take_off()
            case "LOITER":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.current_state = "OFFBOARD"
                self.arm()
            case "OFFBOARD":
                if not self.flightCheck or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                    self.current_state = "IDLE"
                self.state_offboard()
        if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm_message = False
        if self.last_state != self.current_state:
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)
        self.myCnt += 1

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    def attitude_callback(self, msg):
        orientation_q = msg.q
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]),
                                  1.0 - 2.0*(orientation_q[0]**2 + orientation_q[1]**2)))

    def offboard_velocity_callback(self, msg):
        self.velocity.x = -msg.linear.y
        self.velocity.y = msg.linear.x
        self.velocity.z = -msg.linear.z
        self.yaw = msg.angular.z

    def cmdloop_callback(self):
        if self.offboardMode:
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            self.publisher_offboard_mode.publish(offboard_msg)

            if self.go_to_point and self.initial_position is not None:
                relative_pos = self.odom_position - self.initial_position
                error = self.goal_body - relative_pos
                velocity_body = self.Kp * error
                self.get_logger().info(f"Pos actual relativa: {relative_pos}, Error: {error}, Vel: {velocity_body}")
            else:
                velocity_body = np.array([self.velocity.x, self.velocity.y, self.velocity.z])

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.velocity[0] = velocity_body[0]
            trajectory_msg.velocity[1] = velocity_body[1]
            trajectory_msg.velocity[2] = velocity_body[2]
            trajectory_msg.position[:] = [float('nan')] * 3
            trajectory_msg.acceleration[:] = [float('nan')] * 3
            trajectory_msg.yaw = float('nan')
            trajectory_msg.yawspeed = self.yaw
            self.publisher_trajectory.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()