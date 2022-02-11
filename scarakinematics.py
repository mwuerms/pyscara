import math

"""
use SI units
length: m (meters)
angle:  rad (pi = 180°)
"""
# define scara arm
L1 = 100/1000   # in m
L2 = 100/1000   # in m
alpha_min = math.radians( -90)
alpha_max = math.radians( 270)
gamma_min = math.radians(  20)
gamma_max = math.radians( 180)
x_offset = 0
y_offset = 0

# math helpers
L1_2 = math.pow(L1, 2)
L2_2 = math.pow(L1, 2)
L1L2_2 = 2*L1*L2

# ° to rad: math.radians()
# rad to °: math.degrees()
def foreward_kinematics(alpha_in, gamma_in):
    #print("foreward_kinematics({a:.6f}, {g:.6f})".format(a = math.degrees(alpha_in), g = math.degrees(gamma_in)))
    #sanity tests
    if alpha_in < alpha_min:
        return [False, 0, 0]
    if alpha_in > alpha_max:
        return [False, 0, 0]
    if gamma_in < gamma_min:
        return [False, 0, 0]
    if gamma_in > gamma_max:
        return [False, 0, 0]
    
    if gamma_in == 0:
        # basically OK, where ever alpha_in points, the lengt rf_f == 0 -> so [0, 0]
        return [True, 0, 0]

    #scara to polar
    r_foreward_2 = L1_2 + L2_2 - 2*L1*L2*math.cos(gamma_in)
    r_foreward = math.sqrt(r_foreward_2)
    #print(" rf: {r:.6f}, rf_2: {r2:.6f}".format(r=r_foreward, r2=r_foreward_2))
    alpha = math.acos((L1_2 + r_foreward_2 - L2_2) / (2*L1*r_foreward))
    alpha_foreward = alpha_in - alpha
    #print(" alpha: {a:.6f}, alpha_foreward: {a2:.6f}".format(a=math.degrees(alpha), a2 = math.degrees(alpha_foreward)))
    #polar to kartesian
    xf = r_foreward * math.cos(alpha_foreward) + x_offset
    yf = r_foreward * math.sin(alpha_foreward) + y_offset
    return [True, xf, yf]

def inverse_kinematics(x_in, y_in):
    #print("inverse_kinematics({x:.6f}, {y:.6f})".format(x = x_in, y = y_in))
    # kartesian to polar
    r_inverse_2 = math.pow(x_in - x_offset, 2) + math.pow(y_in - y_offset, 2)
    r_inverse = math.sqrt(r_inverse_2)
    if r_inverse > L1+L2:
        #error, out of reach
        return [False, 0, 0]
    alpha_inverse = math.atan2(y_in, x_in)
    # polar to scara
    gamma_out = math.acos((L1_2 + L2_2 - r_inverse_2) / L1L2_2 )
    if gamma_out < gamma_min:
        return [False, 0, 0]
    if gamma_out > gamma_max:
        return [False, 0, 0]
    alpha     = math.acos((L1_2 + r_inverse_2 - L1_2) / (2*L1*r_inverse))
    #print(" alpha: {a:.6f}, alpha_inverse: {ai:.6f}".format(a=math.degrees(alpha), ai=math.degrees(alpha_inverse)))
    alpha_out = (alpha_inverse + alpha)
    if alpha_out < alpha_min:
        return [False, 0, 0]
    if alpha_out > alpha_max:
        return [False, 0, 0]
    return [True, alpha_out, gamma_out]
