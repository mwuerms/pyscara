import pyscara as pys
import matplotlib.pyplot as plt
import time
pys.init()
pys.open_servo()
coords = pys.load_svg('test3.svg')
print(coords)
pys.test_visualize(coords, [], [])
exit()
for x, y in coords:
    if pys.move_xy(x,y) == True:
        pys.move_process()
        time.sleep(0.1)
pys.move_home()
exit()

angles = pys.inverse(coords)

for a in angles:
    if pys.move_angles(a[0], a[1]) == True:
        pys.move_process()
        time.sleep(0.1)

pys.move_home()
