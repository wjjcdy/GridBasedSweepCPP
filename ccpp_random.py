#coding:utf-8
"""
Grid based sweep planner
author: Atsushi Sakai
"""

import math
import os
import sys
from enum import IntEnum

import matplotlib.pyplot as plt
import numpy as np
import random

from a_star import AStarPlanner

sys.path.append(os.path.relpath("./grid_map_lib/"))
try:
    from grid_map_lib import GridMap
except ImportError:
    raise

DT = 0.1  # time tick [s]
V_SPEED = 3.0
YAW_SPEED = 0.5
MAX_ANGLE_CHANGE = 10.0*math.pi/180
MIN_ANGLE_CHANGE = 90.0*math.pi/180
do_animation = True


# find max length edge as sweep direction, 
def find_sweep_direction_and_start_posi(ox, oy):
    # find sweep_direction
    max_dist = 0.0
    vec = [0.0, 0.0]
    sweep_start_pos = [0.0, 0.0]
    for i in range(len(ox) - 1):
        dx = ox[i + 1] - ox[i]
        dy = oy[i + 1] - oy[i]
        d = np.sqrt(dx ** 2 + dy ** 2)

        if d > max_dist:
            max_dist = d
            vec = [dx, dy]
            sweep_start_pos = [ox[i], oy[i]]

    return vec, sweep_start_pos


def search_free_grid_index_at_edge_y(grid_map, from_upper=False):
    yind = None
    xinds = []

    if from_upper:
        xrange = range(grid_map.height)[::-1]
        yrange = range(grid_map.width)[::-1]
    else:
        xrange = range(grid_map.height)
        yrange = range(grid_map.width)

    for iy in xrange:
        for ix in yrange:
            if not grid_map.check_occupied_from_xy_index(ix, iy,occupied_val=0.5):
                yind = iy
                xinds.append(ix)
        if yind:       #在高度这个维度上，找到第一个空闲的点则停止，并记录水平方向上所有的点
            break

    return xinds, yind


def setup_grid_map(ox, oy, ox_in,oy_in,reso, offset_grid=10):
    width = math.ceil((max(ox) - min(ox)) / reso) + offset_grid    # 创建一个栅格地图，可包括整个区域
    height = math.ceil((max(oy) - min(oy)) / reso) + offset_grid
    center_x = np.mean(ox)
    center_y = np.mean(oy)

    grid_map = GridMap(width, height, reso, center_x, center_y)    #创建

    grid_map.set_value_from_polygon(ox, oy, 1.0, inside=False)     #将多边形外均设为1，内部默认为0

    for i in range(len(ox_in)):
        grid_map.set_value_from_polygon(ox_in[i], oy_in[i], 1.0, inside=True)     #将多边形内均设为1，外部默认为0

    grid_map.expand_grid()    # 膨胀一个栅格大小
    return grid_map

def planning_animation(ox, oy,ox_in,oy_in, reso):  # pragma: no cover
    px, py = planning(ox, oy, ox_in,oy_in,reso)

    # animation
    # if do_animation:
    #     for ipx, ipy in zip(px, py):
    #         plt.cla()
    #         plt.plot(ox, oy, "-xb")
    #         plt.plot(px, py, "-r")
    #         # plt.plot(ipx, ipy, "or")
    #         plt.axis("equal")
    #         plt.grid(True)
    #         # plt.pause(0.1)

    # plt.cla()
    # plt.plot(ox, oy, "-xb")
    # plt.plot(px, py, "-xr")
    # plt.axis("equal")
    # plt.grid(True)
    # plt.pause(0.1)

def animation(world_map, sweep_map, reso, X,ax1,ax2,ox, oy,ox_in,oy_in):
    #world_map.plot_grid_map(ax)
    # sweep_map.plot_grid_map(ax1)


    # plt.cla()
    # for i in range(len(ox_in)):
    #     ax2.plot(ox_in[i], oy_in[i], "-xb")
    # ax2.plot(ox, oy, "-xb")
    ax2.plot(X[0, 0], X[1, 0], ".g")
    x_pos = X[0,0] + 1.5 * math.cos(X[2, 0]) 
    y_pos = X[1,0] + 1.5 * math.sin(X[2, 0])    
    ax2.plot(x_pos, y_pos, ".r")
    plt.axis("equal")
    plt.pause(0.05)


def calc_input(v,yawrate):
    u = np.array([[v], [yawrate]])
    return u


# x的状态包括x,y,yaw, v
# 控制量：v,yaw
def motion_model(x, v, yaw_rate):
    u = np.array([[v], [yaw_rate]])
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F @ x + B @ u

    return x

# 根据机器人当前位姿状态获取机器人前方障碍状态
def get_obstacle(x,gmap,reso):
    x_pos = x[0,0] + reso * math.cos(x[2, 0]) 
    y_pos = x[1,0] + reso * math.sin(x[2, 0])                                            # 机器人前方位置
    nxind, nyind = gmap.get_xy_index_from_xy_pos(x_pos, y_pos)                           # 获取地图坐标
    obstacle_flag = gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5)    # 获取障碍状态
    return obstacle_flag

def mapping_creat(x,gmap,reso,val=1.0):
    x_pos = x[0,0] + reso * math.cos(x[2, 0]) 
    y_pos = x[1,0] + reso * math.sin(x[2, 0])                                            # 机器人前方位置
    gmap.set_value_from_xy_pos(x_pos, y_pos, val)                                        # 设置为障碍
    return gmap

def norm_angle(angle):
    s= math.sin(angle)
    c=math.cos(angle)
    angle_norm = math.atan2(s,c)
    return angle_norm

def move_target(x,xind,yind,gmap):
    target_x , target_y = gmap.calc_grid_central_xy_position_from_xy_index(xind, yind)
    temp_x = target_x - x[0,0]
    temp_y = target_y - x[1,0]

    if math.fabs(temp_x)<0.1 and math.fabs(temp_y)<0.1:
        v = 0.0
        yaw_rate = 0.0
        return v,yaw_rate, True

    angle_target = math.atan2(temp_y,temp_x)
    angle_temp = angle_target - x[2,0]
    angle_temp = norm_angle(angle_temp)

    if angle_temp > 0.02:
        v = 0.0
        yaw_rate = YAW_SPEED
    elif angle_temp < -0.02:
        v = 0.0
        yaw_rate = -YAW_SPEED
    else:
        v = V_SPEED
        yaw_rate = 0.0

    return v,yaw_rate,False

def turn_target(x,target_angle):
    finish_flag = False
    angle_temp = target_angle - x[2,0] 
    angle_temp = norm_angle(angle_temp)
    if math.fabs(angle_temp) < 0.1:
        v = 0.0
        yaw_rate = 0.0
        finish_flag = True
    elif angle_temp > 0:
        v = 0.0
        yaw_rate = YAW_SPEED
    else:
        v = 0.0
        yaw_rate = -YAW_SPEED

    return v,yaw_rate,finish_flag
    
def model_cal(xTrue):
    model_x = []
    model_y = []

    x = xTrue[0,0]
    y = xTrue[1,0]
    yaw = xTrue[2,0] 

    x_front = x + 1.5 * math.cos(yaw) 
    y_front = y + 1.5 * math.sin(yaw)

    x_back = x - 0.5 * math.cos(yaw) 
    y_back = y - 0.5 * math.sin(yaw)

    yaw_pi_2 = yaw + math.pi/2

    x_pos_1 = x_front + 1 * math.cos(yaw_pi_2)
    y_pos_1 = y_front + 1 * math.sin(yaw_pi_2)

    model_x.append(x_pos_1)
    model_y.append(y_pos_1)

    x_pos = x_front - 1 * math.cos(yaw_pi_2)
    y_pos = y_front - 1 * math.sin(yaw_pi_2)

    model_x.append(x_pos)
    model_y.append(y_pos)

    x_pos = x_back - 1.5 * math.cos(yaw_pi_2)
    y_pos = y_back - 1.5 * math.sin(yaw_pi_2)

    model_x.append(x_pos)
    model_y.append(y_pos)

    x_pos = x_back + 1.5 * math.cos(yaw_pi_2)
    y_pos = y_back + 1.5 * math.sin(yaw_pi_2)

    model_x.append(x_pos)
    model_y.append(y_pos)

    model_x.append(x_pos_1)
    model_y.append(y_pos_1)


    return model_x, model_y

def search_start_grid(grid_map):
    xinds = []
    y_ind = 0
    xinds, y_ind = search_free_grid_index_at_edge_y(grid_map, from_upper=False)

    return min(xinds), y_ind

    raise ValueError("self.moving direction is invalid ")    # 找到清扫的起点

def main():  # pragma: no cover
    print("start!!")
    head_path_x=[]
    head_path_y=[]

    path_x=[]
    path_y=[]

    # ox_outside = [0.0, 200.0, 200, 0.0, 0.0]
    # oy_outside = [0.0, 0.0,  60,  60.0, 0.0]
    ox_outside = [0.0, 240.0, 240,   150,   150.0, 50, 50, 0.0,  0.0]
    oy_outside = [0.0, 0.0,   45.0,  45.0,  60.0,  60, 45, 45,   0.0]

    for i in range(len(ox_outside)):
        ox_outside[i] = ox_outside[i] / 4.0
        oy_outside[i] = oy_outside[i] / 2.0

    ox_inside = [[90, 90, 70,70,90],[170, 170, 130,130,170]]
    oy_inside = [[18, 28, 28,18,18],[18, 28, 28,18,18] ]
    # ox_inside = []
    # oy_inside = []
    for i in range(len(ox_inside)):
        for j in range(len(ox_inside[0])):
            ox_inside[i][j] = ox_inside[i][j]/4.0
            oy_inside[i][j] = oy_inside[i][j]/2.0
    reso = 1

    real_world_gmap = setup_grid_map(ox_outside, oy_outside, ox_inside,oy_inside, reso)           #此为假设为真实环境地图


    # 创建地图
    # 1.即机器人绕行外边界一周，跟随外边界功能
    # 2.记录行走后的坐标，作为外边界
    ox_recode = [0.0, 240.0, 240,   150,   150.0, 50, 50, 0.0,  0.0]
    oy_recode = [0.0, 0.0,   45.0,  45.0,  60.0,  60, 45, 45,   0.0]

    for i in range(len(ox_recode)):
        ox_recode[i] = ox_recode[i] / 4.0
        oy_recode[i] = oy_recode[i] / 2.0

    ox_recode_in = []
    oy_recode_in = []

    cxind, cyind = search_start_grid(real_world_gmap)                                      # 获取起始位置索引

    x_start, y_start = real_world_gmap.calc_grid_central_xy_position_from_xy_index(cxind, cyind)          # 获取起始位置

    xTrue = np.array([                                                                                    # 初始状态
        [x_start],
        [y_start],
        [0],
        [0]
        ])

    
    v = 0.2             # 初始控制量
    yaw_rate = 0


    fig, ax = plt.subplots(1)
    for i in range(len(ox_inside)):
        ax.plot(ox_inside[i], oy_inside[i], "-xb")
    ax.plot(ox_recode, oy_recode, "-xb")

    TURN_ST = 0
    GO_STRAIGHT = 1

    Curr_state = GO_STRAIGHT

    curr_angle = 0
    target_angle = curr_angle

    T = 0
    while T < 500:
        xTrue = motion_model(xTrue, v, yaw_rate)                     # 实时更新当前位置
        T += DT

        obstacle_flag = get_obstacle(xTrue,real_world_gmap,reso)     # 获取当前运动方向前方障碍状态,此处实际应为传感器获取，这里假设直接读取
        if Curr_state == TURN_ST:
            v,yaw_rate,finish_flag = turn_target(xTrue,target_angle)
            if finish_flag:
                Curr_state = GO_STRAIGHT
        else:
            v = V_SPEED
            yaw_rate = 0
            if obstacle_flag: 
                v = 0
                yaw_rate = 0
                Curr_state = TURN_ST
                curr_angle = xTrue[2,0]
                random_angle = random.uniform(MIN_ANGLE_CHANGE,MAX_ANGLE_CHANGE)
                pos_flag = T % 200
                if pos_flag < 100:
                    target_angle = curr_angle + random_angle
                else:
                    target_angle = curr_angle - random_angle
            

        path_x.append(xTrue[0,0])
        path_y.append(xTrue[1,0])

        robot_model_x, robot_model_y = model_cal(xTrue)

        x_pos = xTrue[0,0] + 1.5 * math.cos(xTrue[2, 0]) 
        y_pos = xTrue[1,0] + 1.5 * math.sin(xTrue[2, 0])
        
        head_path_x.append(x_pos)
        head_path_y.append(y_pos)

        plt.cla()
        for i in range(len(ox_inside)):
            ax.plot(ox_inside[i], oy_inside[i], "-xb")
        ax.plot(ox_recode, oy_recode, "-xb")
        ax.plot(robot_model_x, robot_model_y, "-.g")
        ax.plot(path_x, path_y, ".b")
        ax.plot(head_path_x,head_path_y,".r")
        plt.axis("equal")
        plt.pause(0.000001)    


    print("done!!")
    
    plt.show()

    


if __name__ == '__main__':
	main()
