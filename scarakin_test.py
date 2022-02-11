import scarakinematics as skin
import math
import matplotlib.pyplot as plt

print("testing scarakinematics")
print("L1: {l1:.3f} m, L2: {l2:.3f} m".format(l1 = skin.L1, l2 = skin.L2))
print("alpha: [{a1:.3f} ... {a2:.3f}], [{a3:.3f} ... {a4:.3f}]".format(a1 = skin.alpha_min, a2 = skin.alpha_max, a3 = math.degrees(skin.alpha_min), a4 = math.degrees(skin.alpha_max)))
print("gamma: [{g1:.3f} ... {g2:.3f}], [{g3:.3f} ... {g4:.3f}]".format(g1 = skin.gamma_min, g2 = skin.gamma_max, g3 = math.degrees(skin.gamma_min), g4 = math.degrees(skin.gamma_max)))

print("\n==== testing foreward kinematics ====")

test_angles = [
    [0,0],
    [-30, 180], 
    [-20, 180], 
    [-10, 180], 
    [0, 180],
    [10, 180],
    [20, 180],
    [30, 180],
    [60, 180],
    [90, 180],
    [120, 180],
    [150, 180],
    [180, 180],
    [0,45],
    [0,90],
    [0,135],
    [0,180],
    [-30,0],
    [-30,45],
    [-30,90],
    [-30,135],
    [-30,180],
    [60,0],
    [60,45],
    [60,90],
    [60,135],
    [60,180],
    [150,0],
    [150,45],
    [150,90],
    [150,135],
    [150,180],
    ]
posx = []
posy = []
armx = []
army = []

for a in test_angles:
    [res, xf, yf] = skin.foreward_kinematics(math.radians(a[0]), math.radians(a[1]))
    print(" x: {x:.6f} m, y: {y:.6f} m".format(x = xf, y = yf))
    if res == False:
        print("could not calc foreward_kinematics(), skip")
        print()
        continue
    print()
    # skip plotting
    continue
    xa = skin.L1 * math.cos(math.radians(a[0]))
    ya = skin.L1 * math.sin(math.radians(a[0]))
    fig = plt.figure()
    ax = fig.add_subplot()
    plt.plot([0, xa, xf], [0, ya, yf], '-+')
    ax.set_aspect('equal', 'box')
    plt.grid(True)
    #plt.plot(xa, ya, 'o')
    #plt.plot(xf, yf, 'x')
    plt.show()
    armx.append(xa)
    army.append(ya)
    posx.append(xf)
    posy.append(yf)


print("\n=== testing inverse kinematics ====")
test_pos = [
    [0.1, 0.1],
    [0.1, -0.1],
    [-0.1, 0.1],
    [-0.1, -0.1],
    [0.1,0.0],
    [0.0,0.1],
    [-0.1,0.0],
    [0.0,-0.1],
    [0.1,0.1],
    [0.1,0.11],
    [0,0.1],
    [0, -0.1],
    [0, -0.15],
    [0.05, 0.15],
    [0.05, 0.05],
    [0.05, 0.1],
    [0.05, -0.05],
    [0.05, -0.1],
    [-0.05, 0.1],
    [-0.05, 0.15],
    [0.5, 0],
    [0, 0],
    [0, 0.01],
    [0, 0.02],
    [0, 0.03],
    [0, 0.04],
    [0, 0.05],
    [0, 0.06],
    [0, 0.07],
    [0, 0.08],
    [0, 0.09],
    [0, 0.1],
    [0, 0.11],
    [0, 0],
    [0.01, 0],
    [0.02, 0],
    [0.03, 0],
    [0.04, 0],
    [0.05, 0],
    [0.06, 0],
    [0.07, 0],
    [0.08, 0],
    [0.09, 0],
    [0.1, 0],
    [0.11, 0],
    ]

for p in test_pos:
    [res, alpha, gamma] = skin.inverse_kinematics(p[0], p[1])
    print(" alpha: {a:.6f}, gamma: {g:.6f}".format(a = math.degrees(alpha), g = math.degrees(gamma)))
    if res == False:
        print("Could not calculate angles, skip")
        continue
    # skip plotting
    continue
    xa = skin.L1 * math.cos(alpha)
    ya = skin.L1 * math.sin(alpha)
    fig = plt.figure()
    ax = fig.add_subplot()
    plt.plot([0], [0], '+')
    plt.plot([0, xa], [0, ya], '-o')
    plt.plot([xa, p[0]], [ya, p[1]], '-x')
    ax.set_aspect('equal', 'box')
    plt.grid(True)    
    plt.show()


