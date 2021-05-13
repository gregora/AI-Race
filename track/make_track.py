import math

straight_long = 1000
straight_short = 200
turning_radius = 150
resolution = 12
points = []

for i in range(4):


    if(i == 0):
        starting_point = (straight_short/2, straight_long/2)
    elif(i == 1):
        starting_point = (-straight_short/2, straight_long/2)
    elif(i == 2):
        starting_point = (-straight_short/2, -straight_long/2)
    elif(i == 3):
        starting_point = (straight_short/2, -straight_long/2)

    angle_add = 90 / (resolution - 1)
    for j in range(resolution):
        x = math.cos((i*90 + j*angle_add)*3.14/180)*turning_radius + starting_point[0]
        y = math.sin((i*90 + j*angle_add)*3.14/180)*turning_radius + starting_point[1]
        points.append((int(x), int(y)));





for p in points:
    print(str(p[0]) + ", " + str(p[1]))
