from svgpathtools import svg2paths, wsvg
import scarakinematics as skin
import servo42c
import math

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

print("Set arm to zero position before starting\n")
servo42c.zero_pos(math.radians(45), math.radians(45))
servo42c.open('COM3')
servo42c.goto_angle(math.radians(90), math.radians(90))
servo42c.close()

print("define scara arm: ")
print("L1: {l1:.3f} m, L2: {l2:.3f} m".format(l1 = skin.L1, l2 = skin.L2))

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


