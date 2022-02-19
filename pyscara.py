from svgpathtools import svg2paths, wsvg
import scarakinematics as skin
import servo42c
import math
import time
import matplotlib.pyplot as plt

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

def check_coords(coords, x0, y0, sizex, sizey):
    for cx, cy in coords:
        print(cx, cy)
        if cx < x0 or cx > sizex:
            return False
        if cy < y0 or cy > sizey:
            return False
    return True

def add_origin(coords, x0, y0):
    coords2 = []
    for cx, cy in coords:
        coords2.append([cx+x0, cy+y0])
    return coords2

# calc pulses from angle
def angle_to_pulses(angle, gear, deg_step, msteps):
    return int(angle / deg_step * msteps * gear)

#working variables
alpha = 0
gamma = 0
alpha_pulses = 0
gamma_pulses = 0
def move_angles(alpha_new, gamma_new):
    print("move_angles({:.3f}, {:.3f})".format(math.degrees(alpha_new), math.degrees(gamma_new)))
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
    alpha_diff = alpha - alpha_new
    gamma_diff = gamma - gamma_new
    #correct direction
    alpha_diff *= -1
    #gamma_diff *= -1
    print(" alpha: {:.3f}, gamma: {:.3f}". format(math.degrees(alpha), math.degrees(gamma)))
    print(" alpha_diff: {:.3f}, gamma_diff: {:.3f}". format(math.degrees(alpha_diff), math.degrees(gamma_diff)))
    alpha_diff_pulses = angle_to_pulses(alpha_diff, alpha_gear, motor_deg_steps, motor_msteps)
    gamma_diff_pulses = angle_to_pulses(gamma_diff, gamma_gear, motor_deg_steps, motor_msteps)
    print(" alpha_diff_pulses: {:d}, gamma_diff_pulses: {:d}". format(alpha_diff_pulses, gamma_diff_pulses))
    servo42c.move_pulses(alpha_diff_pulses, 50, gamma_diff_pulses, 50)
    alpha_pulses += alpha_diff_pulses
    gamma_pulses += gamma_diff_pulses
    alpha = alpha_new
    gamma = gamma_new
    return True

def move_home():
    return move_angles(alpha_0, gamma_0)

def move_xy(x, y):
    print("move_xy({:.3f}, {:.3f})".format(x, y))
    res, a, g = skin.inverse_kinematics(x, y)
    if res == True:
        return move_angles(a, g)
    return False

def move_process(max_retries = 1000, pause_time = 0.1):
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
        #print("read_pulses(), alpha: {:d} ({:d}), gamma: {:d} ({:d})". format(a, alpha_pulses, g, gamma_pulses))
        if abs(alpha_pulses - a) <= 5 and abs(gamma_pulses - g) <= 5:
            # OK, done
            print(" reached pulses: {:d}, {:d}, used {:.3f} s".format(a, g, n * pause_time))
            print()
            return True
    #error while processing
    return False

def test_visualize(coords, angles, work_area):
    fig = plt.figure()
    ax = fig.add_subplot()
    ax.set_aspect('equal', 'box')
    posx = []
    posy = []
    armx = []
    army = []
    for a in angles:
        [res, xf, yf] = skin.foreward_kinematics(a[0], a[1])
        #print(" x: {x:.6f} m, y: {y:.6f} m".format(x = xf, y = yf))
        if res == False:
            #print("could not calc foreward_kinematics(), skip")
            #print()
            continue
        #print()
        # skip plotting
        #continue
        xa = skin.L1 * math.cos(a[0])
        ya = skin.L1 * math.sin(a[0])
        plt.plot([0, xa, xf], [0, ya, yf], '-+')
        #plt.plot(xa, ya, 'o')
        #plt.plot(xf, yf, 'x')
        armx.append(xa)
        army.append(ya)
        posx.append(xf)
        posy.append(yf)

    cordx = []
    cordy = []
    for cx, cy in coords:
        cordx.append(cx)
        cordy.append(cy)
    wareax = []
    wareay = []
    for wx, wy in work_area:
        wareax.append(wx)
        wareay.append(wy)

    plt.plot(posx, posy, '-x')
    plt.plot(cordx, cordy, '-o', color='purple', linewidth=2)
    plt.plot(wareax, wareay, '-o', color='black', linewidth=3)
    plt.grid(True)
    plt.show()

#origin of drawing
x0 =  0.1
y0 = -0.1
sizex = 0.1
sizey = 0.1
work_area = [[x0, y0], [x0, y0+sizey], [x0+sizex, y0+sizey], [x0+sizex, y0], [x0, y0]]
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

def init():
    #origin of drawing
    global x0
    global y0
    global sizex
    global sizey
    global work_area
    # define stepper motors and gears for alpha and gamma
    global alpha_gear
    global gamma_gear
    global motor_steps
    global motor_msteps
    global motor_deg_steps
    global alpha_0
    global alpha_min
    global alpha_max
    global gamma_0
    global gamma_min
    global gamma_max

    print("define print area")
    print(" origin x: {:.3f} m, y: {:.3f} m, sizex: {:.3f} m, sizey: {:.3f} m".format(x0, y0, sizex, sizey))
    print("---------------------------------------------------")
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
    print(" alpha pulses for {:d} deg: {:d}".format(360, angle_to_pulses(math.radians(360), alpha_gear, motor_deg_steps, motor_msteps)))
    print(" gamma min:  {:.3f}, max: {:.3f}".format(math.degrees(gamma_min), math.degrees(gamma_max)))
    print(" gamma pulses for {:d} deg: {:d}".format(360, angle_to_pulses(math.radians(360), gamma_gear, motor_deg_steps, motor_msteps)))

    #init working angles
    global alpha
    global gamma
    alpha = alpha_0
    gamma = gamma_0

def load_svg(filename):
    coords = svg2coord(filename, 1/1000)
    return add_origin(coords, x0, y0)
    if check_coords(coords, 0, 0, sizex, sizey) == False:
        print("error: SVG out of size")
        exit()
    return add_origin(coords, x0, y0)

# add points in between p0 and p1, p0 and p1 will not be present in return value
def add_points_inbetween(p0, p1, add, min_dist):
    add_p = []
    d = math.dist(p0, p1)
    if d < min_dist:
        return add_p
    # y = a*x + b, a = dy/dx, b = y0-a*x0, x: [0], y: [1]
    a = (p1[1] - p0[1])/(p1[0] - p0[0])
    b = p0[1]-a*p0[0]
    return add_p

# do not proceed here
def add_points(points, add = 4, min_dist = 0.01):
    return points
    points2 = []
    nb_points = len(points)
    if nb_points < 2:
        # error, no or not enough data
        return points
    for n in range(0, nb_points-1):
        add_points_inbetween(points[n], points[n+1], add, min_dist)
        points2.append(points[n])
    return points2

def open_servo(com = 'COM3'):
    servo42c.open(com)
    print(servo42c.stop_motors())

def inverse(coords):
    angles = []
    angles_deg = []
    err = False
    # add babysteps
    for c in coords:
        res, alpha, gamma = skin.inverse_kinematics(c[0], c[1])
        if res == True:
            #res, x, y = skin.foreward_kinematics(alpha, gamma)
            #print("{:f}, {:f}".format(x,y))
            angles.append([alpha, gamma])
            angles_deg.append([math.degrees(alpha), math.degrees(gamma)])
        else:
            print("ERROR, point out of reach: ({:f}, {:f})".format(c[0], c[1]))
            err = True

    if err == True:
        print("ERROR occured, cannot proceed")
    print(angles_deg)
    return angles

"""
test_visualize(coords, angles, work_area)

fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect('equal', 'box')
pulses = []
for a, g in angles:
    ap = angle_to_pulses(a, alpha_gear, motor_deg_steps, motor_msteps)
    gp = angle_to_pulses(g, gamma_gear, motor_deg_steps, motor_msteps)
    pulses.append([ap, gp])
    plt.plot(ap, gp, '-o')#, color='black', linewidth=3)

plt.grid(True)
plt.show()
print(pulses)
"""
"""#exit()
for p in angles:
    if scara_start_move(p[0], p[1]) == True:
        scara_move_process()
#    else: 
#        print("cannot move to angles ({:.3f}, {:.3f})".format(p[0], p[1]))
"""
