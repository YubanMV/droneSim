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
        super().__init__('go_to_circle_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscripciones
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile)
        self.arm_sub = self.create_subscription(Bool, '/arm_message', self.arm_message_callback, qos_profile)
        self.circle_sub = self.create_subscription(Bool, '/go_to_point', self.circle_callback, qos_profile)

        # Publicadores
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Temporizadores
        self.arm_timer_ = self.create_timer(0.1, self.arm_timer_callback)
        self.timer = self.create_timer(0.02, self.cmdloop_callback)

        # Estados del sistema
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

        # Estado de odometría y control
        self.odom_position = np.zeros(3)
        self.odom_orientation_q = np.array([0.0, 0.0, 0.0, 1.0])
        self.initial_pose_ned = None
        self.circle_active = False
        self.start_time = None
        self.Kp = 0.5

    def arm_message_callback(self, msg):
        self.arm_message = msg.data

    def circle_callback(self, msg):
        if msg.data and self.initial_pose_ned is not None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.circle_active = True
            self.get_logger().info("Comenzando trayectoria circular en ENU → convertida a NED")

    def odometry_callback(self, msg):
        current_ned = np.array([msg.position[0], msg.position[1], msg.position[2]])
        if self.initial_pose_ned is None:
            self.initial_pose_ned = current_ned.copy()
            self.get_logger().info(f"Pose inicial NED registrada: {self.initial_pose_ned}")
        self.odom_position = current_ned
        self.odom_orientation_q = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])

    def attitude_callback(self, msg):
        q = msg.q
        self.trueYaw = -(np.arctan2(2.0 * (q[3]*q[0] + q[1]*q[2]), 1.0 - 2.0 * (q[0]**2 + q[1]**2)))

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

    def cmdloop_callback(self):
        if not self.offboardMode:
            return

        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        velocity_body = np.zeros(3)

        if self.circle_active and self.start_time is not None:
            t = self.get_clock().now().nanoseconds / 1e9 - self.start_time

            # Traza el punto en ENU
            r = 2.0
            omega = 0.5  # rad/s
            x = r * np.cos(omega * t)
            y = r * np.sin(omega * t)
            z = 5.0
            goal_enu = np.array([x, y, z])

            # ENU → NED
            R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
            goal_ned = R @ goal_enu

            # Calcular error en marco NED
            current_rel = self.odom_position - self.initial_pose_ned
            error = goal_ned - current_rel
            velocity_body = self.Kp * error

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory_msg.velocity[0] = velocity_body[0]
        trajectory_msg.velocity[1] = velocity_body[1]
        trajectory_msg.velocity[2] = velocity_body[2]
        trajectory_msg.position[:] = [float('nan')] * 3
        trajectory_msg.acceleration[:] = [float('nan')] * 3
        trajectory_msg.yaw = float('nan')
        trajectory_msg.yawspeed = 0.0
        self.publisher_trajectory.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()