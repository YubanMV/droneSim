#!/usr/bin/env python3
import sys
import rclpy
import geometry_msgs.msg
import std_msgs.msg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
Tele-op:
  W/S/A/D  … como antes
  Flechas  … como antes
  ESPACIO  … arm / disarm
  H        … activar ir a (5,5,-5) en marco del dron
"""

moveBindings = {
    'w': (0, 0, 1, 0),
    's': (0, 0, -1, 0),
    'a': (0, 0, 0, -1),
    'd': (0, 0, 0, 1),
    '\x1b[A': (0, 1, 0, 0),
    '\x1b[B': (0, -1, 0, 0),
    '\x1b[C': (-1, 0, 0, 0),
    '\x1b[D': (1, 0, 0, 0),
}


def getKey(settings):
    if sys.platform == 'win32':
        return msvcrt.getwch()
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return None if sys.platform == 'win32' else termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old):
    if sys.platform != 'win32':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def main():
    settings = saveTerminalSettings()
    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')

    qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos)
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos)
    go_to_point_pub = node.create_publisher(std_msgs.msg.Bool, '/go_to_point', qos)

    arm_toggle = False
    speed, turn = 0.5, 0.2
    x_val = y_val = z_val = yaw_val = 0.0

    try:
        print(msg)
        while True:
            key = getKey(settings)

            if key.lower() == 'h':
                go_msg = std_msgs.msg.Bool()
                go_msg.data = True
                go_to_point_pub.publish(go_msg)
                print("→ Enviado: go_to_point = True")
                continue

            if key in moveBindings:
                x, y, z, th = moveBindings[key]
            else:
                x = y = z = th = 0.0
                if key == '\x03':
                    break

            if key == ' ':
                arm_toggle = not arm_toggle
                arm_msg = std_msgs.msg.Bool()
                arm_msg.data = arm_toggle
                arm_pub.publish(arm_msg)
                print(f"Arm toggle → {arm_toggle}")

            twist = geometry_msgs.msg.Twist()
            x_val += x * speed
            y_val += y * speed
            z_val += z * speed
            yaw_val += th * turn

            twist.linear.x = x_val
            twist.linear.y = y_val
            twist.linear.z = z_val
            twist.angular.z = yaw_val
            pub.publish(twist)
            print(f"X:{x_val:.2f} Y:{y_val:.2f} Z:{z_val:.2f} Yaw:{yaw_val:.2f}")

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        pub.publish(twist)
        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
