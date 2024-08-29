#!/usr/bin/env python3

import struct
import numpy as np
from erp42_mini_msg.msg import ErpMiniStatusMsg, ErpMiniCmdMsg  

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

def Packet2ErpMsg(_byte: bytes) -> ErpMiniStatusMsg:
    msg = ErpMiniStatusMsg()

    gear = _byte[4]

    Spd_L = _byte[5]
    Spd_H = _byte[6]

    Spd = (Spd_H << 8) | Spd_L

    Str_L = _byte[7]
    Str_H = _byte[8]

    Str = (Str_H << 8) | Str_L

    Brk_L = _byte[9]
    Brk_H = _byte[10]

    Brk = (Brk_H << 8) | Brk_L

    En_LL = _byte[11]
    En_LH = _byte[12]
    En_HL = _byte[13]
    En_HH = _byte[14]

    En = (En_HH << 24) | (En_HL << 16) | (En_LH << 8) | En_LL

    En_rst = _byte[15]

    GL_R_L = _byte[16]
    GL_R_H = _byte[17]
    GL_L_L = _byte[18]
    GL_L_H = _byte[19]

    GL_R = (GL_R_H << 8) | GL_R_L
    GL_L = (GL_L_H << 8) | GL_L_L

    msg.gear = gear
    msg.speed = Spd
    msg.steer = Str
    msg.brake = Brk
    msg.encoder = En
    msg.garmin = [GL_L, GL_R]

    return msg


def ErpMsg2Packet(_msg: ErpMiniCmdMsg, init_flag: bool) -> bytes:
    data = []

    gear = _msg.gear
    speed = _msg.speed
    steer = _msg.steer
    brake = _msg.brake

    speed_L = speed & 0xff
    speed_H = speed >> 8

    steer_L = steer & 0xFF
    steer_H = steer >> 8

    brake_L = brake & 0xff
    brake_H = brake >> 8
    
    sum_c = gear + speed_L + speed_H + steer_L + steer_H + brake_L + brake_H + 13 + 10 
    clc = np.uint8(~sum_c)

    data.append(0x53)
    data.append(0x54)
    data.append(0x58)
    data.append(gear)
    data.append(speed_L)
    data.append(speed_H)
    data.append(steer_L)
    data.append(steer_H)
    data.append(brake_L)
    data.append(brake_H)
    if init_flag:
        data.append(0x01)
    else:
        data.append(0x00)
    data.append(0x0D)
    data.append(0x0A)
    data.append(clc)

    return data
