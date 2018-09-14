# -*- coding: utf-8 -*-

import numpy as np


def get_theta(p_robot, p_target, curr_heading=0.0, deg=False):
    theta_1 = np.arctan2((p_target[1] - p_robot[1]),
                            (p_target[0] - p_robot[0]))
    theta_1 -= curr_heading
    if theta_1 < 0:
        theta_2 = theta_1 + 2.0 * np.pi
    else:
        theta_2 = theta_1 - 2.0 * np.pi
    if deg:
        return np.rad2deg(np.array([theta_1, theta_2]))
    else:
        return np.array([theta_1, theta_2])


def choose_heading(theta_options):
    return theta_options[np.argmin(np.fabs(theta_options))]

#p0 = np.array([0.0, 0.0])
#p1 = np.array([-1.0, 1.0])
#
#heading = np.pi * 0.5
#
#h_0 = get_theta(p0, p1, curr_heading=heading, deg=True)
#h_1 = choose_heading(h_0)
#
#
#print h_0
#print h_1
