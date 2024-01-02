#! /usr/bin/env python3
import math

def tangent_funtion(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def compute_ik(point: list):
    result = []
    r_1 = 0.0
    r_2 = 1.0
    r_3 = 1.0
    theta_1_plus = math.atan2(point[1], point[0])
    theta_1_minus = tangent_funtion(theta_1_plus + math.pi)
    theta_1 = [theta_1_plus, theta_1_minus]
    for theta1 in theta_1:
        Px = point[0] * math.cos(theta1) + point[1] * math.sin(theta1)
        Py = point[2]
        theta_3_plus = math.acos((Px**2 + Py**2 - r_2**2 - r_3**2)/(2 * r_2 * r_3)) # LIKE G in course
        theta_3_minus = tangent_funtion(-theta_3_plus)
        theta_3 = [theta_3_plus, theta_3_minus]

        for theta3 in theta_3:
            theta2 = tangent_funtion(math.atan2(Py, Px) - math.atan2(r_3*math.sin(theta3), r_2 + r_3 * math.cos(theta3)))  # Formula used from course
            # checking constraints
            status = False
            if -math.pi/4 <= theta2 <= 3*math.pi/4 and -3*math.pi/4 <= theta3 <= 3*math.pi/4:
                status = True

            result.append(([theta1, theta2, theta3], status))



    return result


if __name__=='__main__':
    P3 = [0.5,0.6,0.7]

    print(f'Position P3: {P3}')
    res = compute_ik(P3)
    
    for r in res:
        print(r)
