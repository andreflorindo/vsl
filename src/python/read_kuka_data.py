# coding=utf-8

import sys
import numpy as np
import math
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D


class JointStates:
    def __init__(self):
        self.a1 = []
        self.a2 = []
        self.a3 = []
        self.a4 = []
        self.a5 = []
        self.a6 = []


class EEStates:
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []
        self.a = []
        self.b = []
        self.c = []


class RobotState:
    def __init__(self):
        self.time = []
        self.something = JointStates()
        self.joint_states = JointStates()
        self.ee_states = EEStates()


def read_path(robot_state_from_file):
    input = np.loadtxt(
        "/home/andreflorindo/workspaces/vsl_motion_planner_ws/bags/kuka_robot_01_11.dat", dtype='f')
    for i in range(0, len(input)):
        robot_state_from_file.time.append(input[i][0])
        robot_state_from_file.something.a1.append(input[i][1])
        robot_state_from_file.something.a2.append(input[i][2])
        robot_state_from_file.something.a3.append(input[i][3])
        robot_state_from_file.something.a4.append(input[i][4])
        robot_state_from_file.something.a5.append(input[i][5])
        robot_state_from_file.something.a6.append(input[i][6])
        robot_state_from_file.joint_states.a1.append(input[i][7]*np.pi/180)
        robot_state_from_file.joint_states.a2.append(input[i][8]*np.pi/180)
        robot_state_from_file.joint_states.a3.append(input[i][9]*np.pi/180)
        robot_state_from_file.joint_states.a4.append(input[i][10]*np.pi/180)
        robot_state_from_file.joint_states.a5.append(input[i][11]*np.pi/180)
        robot_state_from_file.joint_states.a6.append(input[i][12]*np.pi/180)
        robot_state_from_file.ee_states.x.append(input[i][13])
        robot_state_from_file.ee_states.y.append(input[i][14])
        robot_state_from_file.ee_states.z.append(input[i][15])
        robot_state_from_file.ee_states.a.append(input[i][16])
        robot_state_from_file.ee_states.b.append(input[i][17])
        robot_state_from_file.ee_states.c.append(input[i][18])


def clean_path(robot_state_from_file, robot_state):
    j = 0
    for i in range(1, len(robot_state_from_file.time)):
        if robot_state_from_file.something.a1[i] != robot_state_from_file.something.a1[i-1]:
            robot_state.time.append(0.004*j)
            robot_state.something.a1.append(
                robot_state_from_file.something.a1[i])
            robot_state.something.a2.append(
                robot_state_from_file.something.a2[i])
            robot_state.something.a3.append(
                robot_state_from_file.something.a3[i])
            robot_state.something.a4.append(
                robot_state_from_file.something.a4[i])
            robot_state.something.a5.append(
                robot_state_from_file.something.a5[i])
            robot_state.something.a6.append(
                robot_state_from_file.something.a6[i])
            robot_state.joint_states.a1.append(
                robot_state_from_file.joint_states.a1[i])
            robot_state.joint_states.a2.append(
                robot_state_from_file.joint_states.a2[i])
            robot_state.joint_states.a3.append(
                robot_state_from_file.joint_states.a3[i])
            robot_state.joint_states.a4.append(
                robot_state_from_file.joint_states.a4[i])
            robot_state.joint_states.a5.append(
                robot_state_from_file.joint_states.a5[i])
            robot_state.joint_states.a6.append(
                robot_state_from_file.joint_states.a6[i])
            robot_state.ee_states.x.append(
                robot_state_from_file.ee_states.x[i])
            robot_state.ee_states.y.append(
                robot_state_from_file.ee_states.y[i])
            robot_state.ee_states.z.append(
                robot_state_from_file.ee_states.z[i])
            robot_state.ee_states.a.append(
                robot_state_from_file.ee_states.a[i])
            robot_state.ee_states.b.append(
                robot_state_from_file.ee_states.b[i])
            robot_state.ee_states.c.append(
                robot_state_from_file.ee_states.c[i])
            j = j+1
    print(j)


def plot_six(robot_state):
    
    plt.plot(robot_state.time, robot_state.joint_states.a1)
            #  robot_state.time, robot_state.joint_states.a2, '.-',
            #  robot_state.time, robot_state.joint_states.a3, '.-',
            #  robot_state.time, robot_state.joint_states.a4, '.-',
            #  robot_state.time, robot_state.joint_states.a5, '.-',
            #  robot_state.time, robot_state.joint_states.a6, '.-')
    plt.show()


if __name__ == "__main__":

    robot_state_from_file = RobotState()
    robot_state = RobotState()

    read_path(robot_state_from_file)
    clean_path(robot_state_from_file, robot_state)
    plot_six(robot_state_from_file)
