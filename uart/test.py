# coding=utf-8
import serial
import time
import struct
import os

data0 = 'AAAF5003000300AF'  # 一键起飞/降落


def float_to_hex(data):
    return (struct.pack('<f', data)).hex()


def cmd_data_hex(flydata):
    dataHead = "AAAF501D01"
    strTrim = "0000000000000000"
    strMode = "01000000"
    cmd_text = dataHead + float_to_hex(flydata[0]) + float_to_hex(flydata[1]) \
               + float_to_hex(flydata[2]) + float_to_hex(flydata[3]) + strTrim + strMode
    h_byte = bytes.fromhex(cmd_text)
    checksum = 0
    for a_byte in h_byte:
        checksum += a_byte
    H = hex(checksum % 256)
    cmd_text = cmd_text + H[-2] + H[-1]
    if H[-2] == 'x':  # 0xF -> 0x0F
        cmd_text = cmd_text + '0' + H[-1]
    return cmd_text


def send_com(ser_port, s_str):
    h_str = bytes.fromhex(s_str)
    ser_port.write(h_str)
    ser_port.flush()


def init_com():
    xser = serial.Serial('/dev/ttyUSB0', 500000, timeout=0.5)
    time.sleep(0.001)
    return xser


def doSend(sec, fd):
    i = 0
    hex_data = cmd_data_hex(fd)
    while i < sec * 300:
        send_com(yser, hex_data)
        time.sleep(0.001)
        i += 1


def takeOff():
    send_com(yser, data0)
    pass


def xdoSend(moveStat):
    if moveStat == 2:
        doSend(0.5, [0, 7, 0, 50])
        pass
    if moveStat == 3:
        doSend(1, [-4, 0, 0, 50])
    if moveStat == 4:
        doSend(1, [3, 0, 0, 50])
        pass

yser = None

if os.path.exists('/dev/ttyUSB0'):
    try:
        yser = init_com()
        takeOff()
        time.sleep(3)
        takeOff()
        # doSend(4, [5, 0, 0, 63])
    except:
        print('[Error] 设备繁忙')
        pass
else:
    print('[Error] 设备不存在')
