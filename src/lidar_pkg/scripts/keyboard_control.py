#!/usr/bin/env python3
"""
键盘控制小车建图 (WASD + X)

按键映射:
    W / w : 前进      (0x11)
    S / s : 后退      (0x12)
    X / x : 停止      (0x13)
    A / a : 前左转    (0x14)
    D / d : 前右转    (0x15)
    Q / q : 退出程序
    空格  : 停止

使用方法:
    ros2 run lidar_pkg keyboard_control
或:
    python3 keyboard_control.py
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


# cmd_byte 映射 (与 base_driver.cpp 一致)
CMD_FORWARD      = 0x11  # 前进
CMD_BACKWARD     = 0x12  # 后退
CMD_STOP         = 0x13  # 停止
CMD_FORWARD_LEFT = 0x14  # 前左转
CMD_FORWARD_RIGHT= 0x15  # 前右转


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.pub = self.create_publisher(UInt8, '/cmd_byte', 10)
        self.current_cmd = CMD_STOP

        self.get_logger().info('=' * 50)
        self.get_logger().info('键盘控制已启动')
        self.get_logger().info('按键映射:')
        self.get_logger().info('  W : 前进')
        self.get_logger().info('  S : 后退')
        self.get_logger().info('  A : 前左转')
        self.get_logger().info('  D : 前右转')
        self.get_logger().info('  X / 空格 : 停止')
        self.get_logger().info('  Q : 退出')
        self.get_logger().info('=' * 50)

        # 停止信号定时器 (10Hz 持续发送, 防止 base_driver 超时停止)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_send_time = self.get_clock().now()

    def timer_callback(self):
        """持续发送当前指令, 防止 base_driver 0.5s 超时"""
        msg = UInt8()
        msg.data = self.current_cmd
        self.pub.publish(msg)

    def set_cmd(self, cmd):
        if cmd != self.current_cmd:
            self.current_cmd = cmd
            names = {
                CMD_FORWARD: '前进',
                CMD_BACKWARD: '后退',
                CMD_STOP: '停止',
                CMD_FORWARD_LEFT: '前左转',
                CMD_FORWARD_RIGHT: '前右转',
            }
            self.get_logger().info(f'指令: 0x{cmd:02X} ({names.get(cmd, "未知")})')

    def publish_stop(self):
        """退出时发送停止指令"""
        msg = UInt8()
        msg.data = CMD_STOP
        for _ in range(5):  # 多发几次确保收到
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)


def get_key(settings, timeout=0.1):
    """非阻塞读取单键"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = KeyboardControl()

    try:
        while rclpy.ok():
            key = get_key(settings)
            if not key:
                # 没按键时也要 spin 一下, 让定时器发指令
                rclpy.spin_once(node, timeout_sec=0.05)
                continue

            key_lower = key.lower()

            if key_lower == 'w':
                node.set_cmd(CMD_FORWARD)
            elif key_lower == 's':
                node.set_cmd(CMD_BACKWARD)
            elif key_lower == 'a':
                node.set_cmd(CMD_FORWARD_LEFT)
            elif key_lower == 'd':
                node.set_cmd(CMD_FORWARD_RIGHT)
            elif key_lower in ('x', ' '):
                node.set_cmd(CMD_STOP)
            elif key_lower == 'q':
                node.get_logger().info('退出键盘控制')
                break
            else:
                # 其他键不处理, 但要 spin
                pass

            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发送停止指令
        node.publish_stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
