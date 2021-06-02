from random import random
import sys
from math import atan2, degrees
import argparse

def build_box(i,x,y):
    angle = degrees(atan2(float(y), float(x)))
    return '<box id="circle-box-'+str(i)+'" size="0.03,0.2,0.03" movable="false"><body position="'+str(x)+','+str(y)+',0" orientation="'+str(int(angle))+',0,0" /></box>'

def is_point_in_circle(x,y,radius):
    return abs(x**2 + y**2 - radius**2) <0.0001

def get_all_points_of_circle(radius):
    '''
    getting all points of a circle with center at (0,0) with a given radius between [-2,2]
    '''
    for x in [i * 0.001 for i in range(-2000, 0)]:
        for y in [i * 0.001 for i in range(-2000, 0)]:
            if is_point_in_circle(x,y,radius):
                yield dict(x=x,y=y)
                yield dict(x=x,y=-y)
                yield dict(x=-x,y=y)
                yield dict(x=-x,y=-y)
                break

def build_cylinder_xml(radius):
    return '<cylinder id="cylinder" height="0.05" radius="'+str(radius)+'" movable="false" ><body position="0,0,0" orientation="0,0,0" /></cylinder>'

def build_robots_xml(radius,num_robots,controller_id):
    ''' 
        You can distribute entities randomly. Here, we distribute
        10 kilobots in this way:
        - the position is uniformly distributed
        on the ground, in the square whose corners are (-2,-2) and (2,2)
        - the orientations are non-zero only when rotating around Z and chosen
        from a gaussian distribution, whose mean is zero degrees and
        standard deviation is 360 degrees.
        distribute options - uniform, gaussian,constant, grid
    '''
    robot_area=radius/1.5
    return '<distribute>\
    <position method="uniform"\
     min="'+str(-robot_area)+','+str(-robot_area)+',0"\
     max="'+str(robot_area)+','+str(robot_area)+',0" />\
      <orientation method="gaussian" mean="0,0,0" std_dev="360,0,0" />\
      <entity quantity="'+str(num_robots)+'" max_trials="100"><kilobot id="kb-'+str(controller_id)+'-">\
          <controller config="'+str(controller_id)+'" /></kilobot></entity>\
    </distribute>'

def main():
    radius, cylinder_radius, num_robots1,num_robots2 = float(sys.argv[1]),float(sys.argv[2]),int(sys.argv[3]),int(sys.argv[4])
    all_points_of_circle = get_all_points_of_circle(radius=radius)
    boxes_as_points_of_a_circle = (build_box(i,point['x'],point['y']) for i,point in enumerate(all_points_of_circle))
    circle_xml = "".join(list(boxes_as_points_of_a_circle))

    cilynder_xml = build_cylinder_xml(radius=cylinder_radius)
    robots1_xml= build_robots_xml(radius=radius,num_robots=num_robots1,controller_id=1)
    robots2_xml= build_robots_xml(radius=radius,num_robots=num_robots2,controller_id=2)

    with open(sys.argv[5], "rt") as fin:
        with open(sys.argv[6], "wt") as fout:
            for line in fin:
                new_line=str(line).replace('<!--Generated Circle Placeholder-->', circle_xml)
                new_line=new_line.replace('<!--Generated Cilynder Placeholder-->', cilynder_xml)
                new_line=new_line.replace('<!--Generated robots for config 1 Placeholder-->', robots1_xml)
                new_line=new_line.replace('<!--Generated robots for config 2 Placeholder-->', robots2_xml)
                fout.write(new_line)
              
main()