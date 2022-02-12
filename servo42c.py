from pickletools import int4
import serial
import math

# interface the servo42c
alpha_motor = 0xE0
gamma_motor = 0xE1

uart = serial.Serial()

def uint16_to_hl(val16):
    h = int(val16 / 256)
    l = int(val16 - h * 256)
    return [h, l]

def hl_to_uint16(h, l):
    val16 = int(h) * 256 + int(l)
    return val16

def get_direction(angle):
    if angle < 0:
        return -1
    return 1

# return uint16 0 ... 0xFFFF
def cmd_read_encoder(addr):
    return [addr, 0x30]

def cmd_read_pulses(addr):
    return [addr, 0x33]

# return int32 0 ... 65535 for full circle
def cmd_read_angle(addr):
    return [addr, 0x36]

# run the motor @ constant speed -128 ... 127 +/- for direction
def cmd_run_speed(addr, speed):
    if(speed < 0):
        speed = 0x80 - speed
    return [addr, 0xF6, speed]

# stop the motor
def cmd_stop_motor(addr):
    return [addr, 0xF7]

# run the motor to an angle (in pulses) @ constant speed -128 ... 127 +/- for direction
def cmd_run_to_angle(addr, speed, pulses):
    [p_h, p_l] = uint16_to_hl(pulses)
    if(speed < 0):
        speed = 0x80 - speed
    return [addr, 0xFD, speed, p_h, p_l]

def print_cmd(pre, cmd):
    print("{:s}".format(pre), end='')
    for c in cmd:
        print(" {:02X}".format(c), end='')
    print()

def send_cmd(cmd, rx_len):
    uart.flushInput()
    #print_cmd(' out:', cmd)
    uart.write(cmd)
    resp = uart.read(rx_len)
    #print_cmd(' in: ', list(resp))
    return resp

def open(com = 'COM3'):
    uart.port = com
    uart.baudrate = 115200
    uart.timeout = 0.1
    uart.open()
    uart.flushOutput()
    uart.flushInput()

def close():
    uart.close()

def stop_motors():
    alpha_cmd = cmd_stop_motor(alpha_motor)
    gamma_cmd = cmd_stop_motor(gamma_motor)
    alpha_resp = send_cmd(alpha_cmd, 2)
    gamma_resp = send_cmd(gamma_cmd, 2)
    return [alpha_resp, gamma_resp]

def move_pulses(alpha_pulses, alpha_speed, gamma_pulses, gamma_speed):
    print("move_pulses({:d}, {:d})".format(alpha_pulses, gamma_pulses))
    alpha_dir = get_direction(alpha_pulses)
    alpha_pulses = abs(alpha_pulses)
    alpha_speed = alpha_dir * alpha_speed
    alpha_cmd = cmd_run_to_angle(alpha_motor, alpha_speed, alpha_pulses)
    gamma_dir = get_direction(gamma_pulses)
    gamma_pulses = abs(gamma_pulses)
    gamma_speed = gamma_dir * gamma_speed
    gamma_cmd = cmd_run_to_angle(gamma_motor, gamma_speed, gamma_pulses)

    alpha_resp = send_cmd(alpha_cmd, 2)
    gamma_resp = send_cmd(gamma_cmd, 2)
    return [alpha_resp, gamma_resp]

def resp_parse_read_angle(resp):
    if len(resp) != 5:
        return 0
    res = int.from_bytes(resp[3:5], byteorder='big', signed = True) 
    return 2 * math.pi * (res / 65336)

def read_current_angle():
    alpha_cmd = cmd_read_angle(alpha_motor)
    gamma_cmd = cmd_read_angle(gamma_motor)
    alpha_resp = send_cmd(alpha_cmd, 5)
    gamma_resp = send_cmd(gamma_cmd, 5)
    alpha_resp_rad = resp_parse_read_angle(alpha_resp)
    gamma_resp_rad = resp_parse_read_angle(gamma_resp)
    return [alpha_resp_rad, gamma_resp_rad]

def resp_parse_read_pulses(resp):
    if len(resp) != 5:
        return 0
    return int.from_bytes(resp[3:5], byteorder='big', signed = True) 

def read_pulses():
    alpha_cmd = cmd_read_pulses(alpha_motor)
    gamma_cmd = cmd_read_pulses(gamma_motor)
    alpha_resp = send_cmd(alpha_cmd, 5)
    gamma_resp = send_cmd(gamma_cmd, 5)
    alpha_resp_pulses = resp_parse_read_pulses(alpha_resp)
    gamma_resp_pulses = resp_parse_read_pulses(gamma_resp)
    return [alpha_resp_pulses, gamma_resp_pulses]
