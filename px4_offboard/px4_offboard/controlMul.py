#!/usr/bin/env python3

import sys, termios, tty
import rclpy
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy
)
from std_msgs.msg     import Bool
from geometry_msgs.msg import Twist

NUM_DRONES = 4

msg = """
Tele-op multí­plex:
  W/S/A/D  … vertical (z) y yaw
  Flechas  … pitch y roll
  ESPACIO  … ARM/DISARM → /arm_message
  H        … GO_TO_POINT → /go_to_point
"""

moveBindings = {
    'w': (0, 0, 1, 0),
    's': (0, 0, -1,0),
    'a': (0, 0, 0, -1),
    'd': (0, 0, 0, 1),
    '\x1b[A':(0,1,0,0),
    '\x1b[B':(0,-1,0,0),
    '\x1b[C':(-1,0,0,0),
    '\x1b[D':(1,0,0,0),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    k = sys.stdin.read(1)
    if k=='\x1b': k += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return k

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')

    # QoS
    qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    vel_pub = node.create_publisher(Twist, '/offboard_velocity_cmd', qos)
    arm_pub = node.create_publisher(Bool, '/arm_message', qos)
    go_pub  = node.create_publisher(Bool, '/go_to_point', qos)

    arm_toggle = False
    speed, turn = 0.5, 0.2
    x=y=z=yaw=0.0

    print(msg)
    try:
        while True:
            key = getKey(settings)

            if key.lower() == 'h':
                m = Bool(); m.data = True
                go_pub.publish(m)
                print("→ GO_TO_POINT enviado")
                continue

            if key in moveBindings:
                dx, dy, dz, dth = moveBindings[key]
                x   += dx*speed
                y   += dy*speed
                z   += dz*speed
                yaw += dth*turn
            elif key == ' ':
                arm_toggle = not arm_toggle
                m = Bool(); m.data = arm_toggle
                arm_pub.publish(m)
                print(f"Arm toggle → {arm_toggle}")
            elif key == '\x03':
                break

            t = Twist()
            t.linear.x  = x
            t.linear.y  = y
            t.linear.z  = z
            t.angular.z = yaw
            vel_pub.publish(t)
            print(f"V: X={x:.2f} Y={y:.2f} Z={z:.2f} Yaw={yaw:.2f}")

    finally:
        vel_pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
