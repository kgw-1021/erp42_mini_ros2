#!/usr/bin/env python3
from struct import pack
import rclpy
from rclpy.node import Node
from erp42_mini_msg.msg import ErpMiniStatusMsg, ErpMiniCmdMsg

import serial
import numpy as np

from ByteHandler import ErpMsg2Packet, Packet2ErpMsg

"""
RECV BYTES
┌─────┬─────┬─────┬──────┬───────┬───────┬───────┬───────┬───────┬───────┬───────┬───────┬───────┬───────┬────────┬────────┬────────┬────────┬────────┬───────┬──────┬─────┐ 
│  S  │  T  │  X  │ Gear │ Spd_L │ Spd_H │ Str_L │ Str_H │ Brk_H │ Brk_H │ En_LL │ En_LH │ En_HL │ En_HH │ En_Rst │ GL-R_L │ GL-R_H │ GL-L_L │ GL-L_H │ ETX0  │ ETX1 │ CLC │
├─────┼─────┼─────┼──────┼───────┴───────┼───────┴───────┼───────┴───────┼───────┴───────┴───────┴───────┼────────┼────────┴────────┼────────┴────────┼───────┼──────┼─────┤
│0x53 │0x54 │0x58 │ 0, 1 │    0 ~ 800    │  1250 ~ 1900  │  1200, 1500   │           0 ~ 2^32            │  bool  │     0 ~ 200     │     0 ~ 200     │  0x0D │ 0x0A │     │ 
└─────┴─────┴─────┴──────┴───────────────┴───────────────┴───────────────┴───────────────────────────────┴────────┴─────────────────┴─────────────────┴───────┴──────┴─────┘  

SEND BYTES
┌─────┬─────┬─────┬──────┬───────┬───────┬───────┬───────┬───────┬───────┬────────┬──────┬──────┬─────┐
│  S  │  T  │  X  │ Gear │ Spd_L │ Spd_H │ Str_L │ Str_H │ Brk_H │ Brk_H │ En_Rst │ ETX0 │ ETX1 │ CLC │ 
├─────┼─────┼─────┼──────┼───────┴───────┼───────┴───────┼───────┴───────┼────────┼──────┼──────┼─────┤
│0x53 │0x54 │0x58 │ 0, 1 │    0 ~ 800    │  1250 ~ 1900  │  1200, 1500   │  bool  │ 0x0D │ 0x0A │     │
└─────┴─────┴─────┴──────┴───────────────┴───────────────┴───────────────┴────────┴──────┴──────┴─────┘
"""

START_BITS = "535458"

class ERPHandler(Node):
    def __init__(self):
        super().__init__("erp42_mini_base")
        _port = self.declare_parameter("port", "/dev/ttyUSB0").value
        _baudrate = self.declare_parameter("baudrate", 115200).value
        self.get_logger().info(f"erp_base::Uart Port : {_port}")
        self.get_logger().info(f"erp_base::Baudrate  : {_baudrate}")

        self.serial = serial.Serial(port=_port, baudrate=_baudrate)
        self.get_logger().info(f"Serial {_port} Connected")
        self.packet = ErpMiniCmdMsg()
        self.packet.gear = 0
        self.packet.speed = 0
        self.packet.steer = 1573
        self.packet.brake = 1500
        self.packet.en_rst = True


        self.erp_motion_pub = self.create_publisher(
            ErpMiniStatusMsg, "/erp42_status", 3
        )
        self.erp_cmd_sub = self.create_subscription(
            ErpMiniCmdMsg, "/erp42_ctrl_cmd", self.send_packet, 3
        )

        self.timer = self.create_timer(1.0 / 40.0, self.loop_callback)

    def recv_packet(self):
        packet = self.serial.read(23)
        if not packet.hex().find(START_BITS) == 0:
            end, data = packet.hex().split(START_BITS)
            packet = bytes.fromhex(START_BITS + data + end)
            self.erp_motion_pub.publish(Packet2ErpMsg(packet))
        else:            
            self.erp_motion_pub.publish(Packet2ErpMsg(packet))

    def send_packet(self, _data: ErpMiniCmdMsg):
        self.packet = _data

    def serial_send(self):
        packet = ErpMsg2Packet(self.packet, self.packet.en_rst)
        if self.packet.en_rst:
            self.packet.en_rst = False
        self.serial.write(packet)

    def loop_callback(self):
        self.recv_packet()
        self.serial_send()


def main(args=None):
    rclpy.init(args=args)
    ehandler = ERPHandler()
    rclpy.spin(ehandler)
    ehandler.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
