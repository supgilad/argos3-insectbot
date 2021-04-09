from itertools import product
from random import random

def build_point(x,y):
    return '<box id="'+str(random())+'" size="0.05,0.05,0.05" movable="false"><body position="'+str(x)+','+str(y)+',0" orientation="0,0,0" /></box>'
def point_is_in_circle(x,y,radius):
    return x**2 + y**2 == radius**2

def get_all_points_of_circle(radius):
    for x in [x * 0.001 for x in range(-2000, 2000)]:
        for y in [x * 0.001 for x in range(-2000, 2000)]:
            if point_is_in_circle(x,y,radius):
                yield build_point(x,y)

print("".join(list(get_all_points_of_circle(radius=0.5))))