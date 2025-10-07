import math

PI = 3.14159265359

resolution = 12
r = 7
points = []
for i in range(resolution+1):
    angle = (i/resolution) * (2*PI)
    x = r* math.cos(angle) + 10
    y = r* math.sin(angle) + 10
    points.append((round(x), round(y)))

print(points)
