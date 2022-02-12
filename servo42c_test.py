import servo42c
import math
import time

# define stepper motors and gears for alpha and gamma
alpha_gear = 58.5/10   #end gear dia / motor gear dia
gamma_gear = 48.0/10   #end gear dia / motor gear dia
motor_deg_steps = math.radians(1.8)
motor_msteps = 16

# calc pulses from angle
def angle_to_pulses(angle, gear, deg_step, msteps):
    return int(angle / deg_step * msteps * gear)

print("Set arm to zero position before starting\n")
servo42c.zero_pos(math.radians(45), math.radians(45))
servo42c.open('COM3')
print(servo42c.stop_motors())

alpha_pulses = 0
gamma_pulses = 0
alpha_diff = math.radians(90)
gamma_diff = math.radians(90)
alpha_diff_pulses = angle_to_pulses(alpha_diff, alpha_gear, motor_deg_steps, motor_msteps)
gamma_diff_pulses = angle_to_pulses(gamma_diff, gamma_gear, motor_deg_steps, motor_msteps)
servo42c.move_pulses(alpha_diff_pulses, gamma_diff_pulses)
alpha_pulses += alpha_diff_pulses
gamma_pulses += gamma_diff_pulses

for n in range(1,1000):
    time.sleep(0.1)
    """
    [a, g] = servo42c.read_current_angle()
    print("angle")
    print(math.degrees(a))
    print(math.degrees(g))
    """
    [a,g] = servo42c.read_pulses()
    a = -1 * a
    g = -1 * g
    print("step: {:d}".format(n))
    print("goto pulses:  alpha: {:d}, gamma: {:d}". format(alpha_pulses, gamma_pulses))
    print("get_pulses(), alpha: {:d}, gamma: {:d}". format(a, g))
    if alpha_pulses == a and gamma_pulses == g:
        print("reached angle, used {:d} ms".format(n * 100))
        break

alpha_diff = math.radians(-90)
gamma_diff = math.radians(-90)
alpha_diff_pulses = angle_to_pulses(alpha_diff, alpha_gear, motor_deg_steps, motor_msteps)
gamma_diff_pulses = angle_to_pulses(gamma_diff, gamma_gear, motor_deg_steps, motor_msteps)
servo42c.move_pulses(alpha_diff_pulses, gamma_diff_pulses)
alpha_pulses += alpha_diff_pulses
gamma_pulses += gamma_diff_pulses

for n in range(1,1000):
    time.sleep(0.1)
    """
    [a, g] = servo42c.read_current_angle()
    print("angle")
    print(math.degrees(a))
    print(math.degrees(g))
    """
    [a,g] = servo42c.read_pulses()
    a = -1 * a
    g = -1 * g
    print("step: {:d}".format(n))
    print("goto pulses:  alpha: {:d}, gamma: {:d}". format(alpha_pulses, gamma_pulses))
    print("get_pulses(), alpha: {:d}, gamma: {:d}". format(a, g))
    if alpha_pulses == a and gamma_pulses == g:
        print("reached angle, used {:d} ms".format(n * 100))
        break

servo42c.close()