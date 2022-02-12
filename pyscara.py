from svgpathtools import svg2paths, wsvg
import scarakinematics as skin
import servo42c
import math
import time

def str2points(str, scale):
    elems = str.split(' ')
    coordinates = []
    for e in elems:
        if e.find(',') >= 1:
            #this seems as a coordinate
            [x, y] = e.split(',')
            point = [float(x)*scale, float(y)*scale]
            coordinates.append(point)
    return coordinates

def svg2coord(filename, scale):
    print("svg2coord({:s}, scale = {:.4f})".format(filename, scale))
    paths, attributes = svg2paths(filename)
    return str2points(attributes[0]['d'], scale)

# calc pulses from angle
def angle_to_pulses(angle, gear, deg_step, msteps):
    return int(angle / deg_step * msteps * gear)

#working variables
alpha = 0
gamma = 0
alpha_pulses = 0
gamma_pulses = 0
def scara_start_move(alpha_new, gamma_new):
    print("scara_start_move({:.3f}, {:.3f})".format(math.degrees(alpha_new), math.degrees(gamma_new)))
    global alpha
    global gamma
    global alpha_min
    global alpha_max
    global gamma_min
    global gamma_max
    global alpha_pulses
    global gamma_pulses
    if alpha_new > alpha_max:
        print(" error alpha_new too large (max: {:.3f})".format(math.degrees(alpha_max)))
        return False
    if alpha_new < alpha_min:
        print(" error alpha_new too small (max: {:.3f})".format(math.degrees(alpha_min)))
        return False
    if gamma_new > gamma_max:
        print(" error gamma_new too large (max: {:.3f})".format(math.degrees(gamma_max)))
        return False
    if gamma_new < gamma_min:
        print(" error gamma_new too large (max: {:.3f})".format(math.degrees(gamma_min)))
        return False
    # with respect to direction
    alpha_diff = alpha_new - alpha
    gamma_diff = gamma - gamma_new
    print(" alpha: {:.3f}, gamma: {:.3f}". format(math.degrees(alpha), math.degrees(gamma)))
    print(" alpha_diff: {:.3f}, gamma_diff: {:.3f}". format(math.degrees(alpha_diff), math.degrees(gamma_diff)))
    alpha_diff_pulses = angle_to_pulses(alpha_diff, alpha_gear, motor_deg_steps, motor_msteps)
    gamma_diff_pulses = angle_to_pulses(gamma_diff, gamma_gear, motor_deg_steps, motor_msteps)
    print(" alpha_diff_pulses: {:d}, gamma_diff_pulses: {:d}". format(alpha_diff_pulses, gamma_diff_pulses))
    servo42c.move_pulses(alpha_diff_pulses, 100, gamma_diff_pulses, 100)
    alpha_pulses += alpha_diff_pulses
    gamma_pulses += gamma_diff_pulses
    alpha = alpha_new
    gamma = gamma_new
    return True

def scara_move_process(max_retries = 1000, pause_time = 0.1):
    global alpha_pulses
    global gamma_pulses
    for n in range(1,max_retries):
        time.sleep(pause_time)
        [a,g] = servo42c.read_pulses()
        # wrong direction, correct
        a = -1 * a
        g = -1 * g
        #print("step: {:d}".format(n))
        #print("goto pulses:  alpha: {:d}, gamma: {:d}". format(alpha_pulses, gamma_pulses))
        print("read_pulses(), alpha: {:d} ({:d}), gamma: {:d} ({:d})". format(a, alpha_pulses, g, gamma_pulses))
        if abs(alpha_pulses - a) <= 5 and abs(gamma_pulses - g) <= 5:
            # OK, done
            print("reached angle, used {:.3f} s".format(n * pause_time))
            return True
    #error while processing
    return False

# define stepper motors and gears for alpha and gamma
alpha_gear = 58.5/10   #end gear dia / motor gear dia
gamma_gear = 48.0/10   #end gear dia / motor gear dia
motor_steps = 200
motor_msteps = 16
motor_deg_steps = math.radians(360/motor_steps)
alpha_0 = math.radians(0)
alpha_min = math.radians(-90)
alpha_max = math.radians(+90)
gamma_0 = math.radians(180)
gamma_min = math.radians(45)
gamma_max = math.radians(180)

print("define scara arm: ")
print(" L1: {l1:.3f} m, L2: {l2:.3f} m".format(l1 = skin.L1, l2 = skin.L2))
print(" alpha_gear: {:.3f}, gamma_gear: {:.3f}".format(alpha_gear, gamma_gear))
print(" stepper motors for alpha and gamma, nb steps: {:d}, nb micro steps: {:d}".format(motor_steps, motor_msteps))
print("---------------------------------------------------")
print(" start from")
print(" alpha: {:.3f}".format(math.degrees(alpha_0)))
print(" gamma: {:.3f}".format(math.degrees(gamma_0)))
print(" min/max")
print(" alpha min: {:.3f}, max: {:.3f}".format(math.degrees(alpha_min), math.degrees(alpha_max)))
print(" gamma min:  {:.3f}, max: {:.3f}".format(math.degrees(gamma_min), math.degrees(gamma_max)))

#init working angles
alpha = alpha_0
gamma = gamma_0

servo42c.open('COM3')
print(servo42c.stop_motors())
"""
scara_start_move(alpha_max, gamma_min)
scara_move_process()
scara_start_move(alpha_min, gamma_max)
scara_move_process()
scara_start_move(alpha_0, gamma_0)
scara_move_process()

print("done")
exit()
"""
coords = svg2coord('test1.svg', 1/1000)

angles = []
err = False
# add babysteps
for c in coords:
    res, alpha, gamma = skin.inverse_kinematics(c[0], c[1])
    if res == True:
        #res, x, y = skin.foreward_kinematics(alpha, gamma)
        #print("{:f}, {:f}".format(x,y))
        angles.append([alpha, gamma])
    else:
        print("ERROR, point out of reach: ({:f}, {:f})".format(c[0], c[1]))
        err = True

if err == True:
    print("ERROR occured, cannot proceed")
print(angles)

for p in angles:
    if scara_start_move(p[0], p[1]) == True:
        scara_move_process()
#    else: 
#        print("cannot move to angles ({:.3f}, {:.3f})".format(p[0], p[1]))

