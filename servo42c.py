import serial
import math

# interface the servo42c
alpha_motor = 0xE0
gamma_motor = 0xE1

alpha = 0   # in rad
gamma = 0   # in rad

uart = serial.Serial()

def uint16_to_hl(val16):
    h = int(val16 / 256)
    l = int(val16 - h * 256)
    return [h, l]

# calc pulses from angle
def angle_to_pulses(angle, deg_step = math.radians(1.8), msteps = 16):
    return int(angle / deg_step * msteps)

def get_direction(angle):
    if angle < 0:
        return -1
    return 1

# return uint16 0 ... 0xFFFF
def cmd_read_encoder(addr):
    return [addr, 0x30]

# return int32 0 ... 65535 for full circle
def cmd_read_angle(addr):
    return [addr, 0x36]

# run the motor @ constant speed -128 ... 127 +/- for direction
def cmd_run_speed(addr, speed):
    return [addr, 0xF6, speed]

# stop the motor
def cmd_stop(addr):
    return [addr, 0xF7]

# run the motor to an angle (in pulses) @ constant speed -128 ... 127 +/- for direction
def cmd_run_to_angle(addr, speed, pulses):
    [p_h, p_l] = uint16_to_hl(pulses)
    return [addr, 0xF6, speed, p_h, p_l]

def print_cmd(pre, cmd):
    print("{:s}".format(pre), end='')
    for c in cmd:
        print(" {:02X}".format(c), end='')
    print()

def send_cmd(cmd, rx_len):
    uart.flushInput()
    print_cmd(' out:', cmd)
    uart.write(cmd)
    resp = uart.read(rx_len)
    print_cmd(' in: ', resp)
    return resp

def open(com = 'COM3'):
    uart.port = com
    uart.baudrate = 56700
    uart.timeout = 0.1
    uart.open()
    uart.flushOutput()
    uart.flushInput()

def close():
    uart.close()

def zero_pos(alpha_new, gamma_new):
    alpha = alpha_new
    gamma = gamma_new

def goto_angle(alpha_new, gamma_new):
    print("goto_angle({:f}, {:f})".format(alpha_new, gamma_new))
    #alpha_diff = alpha - alpha_new
    alpha_diff = alpha_new
    alpha_dir = get_direction(alpha_diff)
    alpha_pulses = angle_to_pulses(abs(alpha_diff))
    alpha_speed = alpha_dir * 1
    alpha_cmd = cmd_run_to_angle(alpha_motor, alpha_speed, alpha_pulses)
    #gamma_diff = gamma - gamma_new
    gamma_diff = gamma_new
    gamma_dir = get_direction(gamma_diff)
    gamma_pulses = angle_to_pulses(abs(gamma_diff))
    gamma_speed = gamma_dir * 1
    gamma_cmd = cmd_run_to_angle(alpha_motor, gamma_speed, alpha_pulses)

    alpha_resp = send_cmd(alpha_cmd, 2)
    gamma_resp = send_cmd(gamma_cmd, 2)
