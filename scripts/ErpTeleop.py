#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import tty
import termios
import select  # 비동기 입력 처리를 위한 모듈
from erp42_mini_msg.msg import ErpMiniCmdMsg
import os

msg = """
Control Your erp42-mini
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease velocity
a/d : increase/decrease steering angle 

s : force stop

CTRL-C to quit

"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher = self.create_publisher(ErpMiniCmdMsg, '/erp42_ctrl_cmd', 10)
        self.velocity = 0
        self.steering = 1570
        self.brake = 1500
        self.MAX_Velocity = 800
        self.MAX_R_STEER = 1900
        self.MAX_L_STEER = 1220
        self.pubmsg = ErpMiniCmdMsg()

        self.timer = self.create_timer(0.05, self.timer_callback)

        
    def getkey(self):
        fd = sys.stdin.fileno()
        original_attributes = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # 0.1초 동안 입력 기다림
            if rlist:
                ch = sys.stdin.read(1)
                return ch
            else:
                return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, original_attributes)

    def timer_callback(self):
        key = self.getkey()
        if key == 'w':
            self.velocity += 30
        elif key == 's':
            self.velocity = 0
            self.steering = 1570
        elif key == 'a':
            self.steering -= 30
        elif key == 'd':
            self.steering += 30
        elif key == 'x':
            self.velocity -= 30
        elif key == '\x03':  # Ctrl+C
            rclpy.shutdown()
            return

        if self.velocity >= 0:
            gear = 0
            pub_vel = self.velocity
        else:
            gear = 1
            pub_vel = abs(self.velocity)

        if self.velocity >= self.MAX_Velocity:
            self.velocity = self.MAX_Velocity

        if self.steering >= self.MAX_R_STEER:
            self.steering = self.MAX_R_STEER
        
        if self.steering <= self.MAX_L_STEER:
            self.steering = self.MAX_L_STEER
        
        self.pubmsg.gear = gear
        self.pubmsg.speed = pub_vel
        self.pubmsg.steer = self.steering
        self.pubmsg.brake = self.brake
        self.pubmsg.en_rst = False
        self.publisher.publish(self.pubmsg)

        os.system('clear')
        print(msg)
        print('velocity: {}, steering: {}'.format(self.velocity, self.steering))

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
