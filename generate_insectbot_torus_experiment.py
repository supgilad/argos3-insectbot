from random import random
import sys
from math import atan2, degrees

def build_box(i,x,y):
    angle = degrees(atan2(float(y), float(x)))
    return '<box id="circle-box-'+str(i)+'" size="0.03,0.2,0.03" movable="false"><body position="'+str(x)+','+str(y)+',0" orientation="'+str(int(angle))+',0,0" /></box>'

def is_point_in_circle(x,y,radius):
    return x**2 + y**2 == radius**2

def get_all_points_of_circle(radius):
    for x in [i * 0.001 for i in range(-2000, 2000)]:
        for y in [i * 0.001 for i in range(-2000, 2000)]:
            if is_point_in_circle(x,y,radius):
                yield dict(x=x,y=y)

def build_cylinder_xml(radius):
    return '<cylinder id="cylinder" height="0.05" radius="'+str(radius)+'" movable="false" ><body position="0,0,0" orientation="0,0,0" /></cylinder>'

def main():
    radius=float(sys.argv[1])
    all_points_of_circle = get_all_points_of_circle(radius=radius)
    boxes_as_a_circle = (build_box(i,point['x'],point['y']) for i,point in enumerate(all_points_of_circle))
    circle_xml = "".join(list(boxes_as_a_circle))

    cylinder_radius = float(sys.argv[2])
    cilynder_xml = build_cylinder_xml(radius=cylinder_radius)
    print(cilynder_xml)
    with open(sys.argv[3], "rt") as fin:
        with open(sys.argv[4], "wt") as fout:
            for line in fin:
                new_line=str(line).replace('<!--Generated Circle Placeholder-->', circle_xml)
                new_line=new_line.replace('<!--Generated Cilynder Placeholder-->', cilynder_xml)
                fout.write(new_line)
main()