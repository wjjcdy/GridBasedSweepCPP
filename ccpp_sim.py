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

from a_star import AStarPlanner

sys.path.append(os.path.relpath("./grid_map_lib/"))
try:
    from grid_map_lib import GridMap
except ImportError:
    raise

DT = 0.1  # time tick [s]
V_SPEED = 3.0
YAW_SPEED = 0.3
do_animation = True


class SweepSearcher:
    class SweepDirection(IntEnum):
        UP = 1
        DOWN = -1

    class MovingDirection(IntEnum):
        RIGHT = 1
        LEFT = -1

    def __init__(self, mdirection, sdirection):
        self.moving_direction = mdirection
        self.sweep_direction = sdirection
        self.turing_window = []
        self.update_turning_window()

    def goal_set(self,xinds_goaly,goaly):
        self.xinds_goaly = xinds_goaly
        self.goaly = goaly


    def move_target_grid(self, cxind, cyind, gmap):          # 根据当前点和栅格图状态，查找下一个点
        nxind = self.moving_direction + cxind                # x方向根据扫描方向。 如向右，则 x_next = x_curr+ 1
        nyind = cyind                                        # y坐标暂时不变，用于判断此刻是否为

        # found safe grid
        if not gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5):  # 若为空闲直接返回下个点，未走过的是空闲。第一次探索不可以为已经走过的路
            return nxind, nyind
        else:  # occupided
            ncxind, ncyind = self.find_safe_turning_grid(cxind, cyind, gmap)       # 判断拐弯所有的点，包括当前位置上方三个位置（即8连通的上方3个位置）
            if (ncxind is None) and (ncyind is None):                              # 如果都没有空间，
                # moving backward                                                  # 则应回退一格
                ncxind = -self.moving_direction + cxind
                ncyind = cyind
                if gmap.check_occupied_from_xy_index(ncxind, ncyind):              # 回退一格可以为走过的路，即回退可为行走过的路，
                    # moved backward, but the grid is occupied by obstacle
                    return None, None
            else:                                                                  # 若拐弯存在空闲点, 即需要转向下一行，需要从下一行尽头开始(注意此时位置和下一刻位置有可能不再连续)
                # keep moving until end
                # while not gmap.check_occupied_from_xy_index(ncxind + self.moving_direction, ncyind, occupied_val=0.5):
                 #    ncxind += self.moving_direction
                self.swap_moving_direction()                                       # 左右扫描的方向需要进行切换
            return ncxind, ncyind

    def find_safe_turning_grid(self, cxind, cyind, gmap):

        for (dxind, dyind) in self.turing_window:

            nxind = dxind + cxind
            nyind = dyind + cyind

            # found safe grid
            if not gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5): # 探索必须未经过的位置
                return nxind, nyind

        return None, None

    def is_search_done(self, gmap):
        for ix in self.xinds_goaly:
            if not gmap.check_occupied_from_xy_index(ix, self.goaly, occupied_val=0.5):
                return False

        # all lower grid is occupied
        return True

    def update_turning_window(self):
        self.turing_window = [
            (self.moving_direction, 0.0),
            (self.moving_direction, self.sweep_direction),
            (0, self.sweep_direction),
            (-self.moving_direction, self.sweep_direction),
        ]

    def swap_moving_direction(self):
        self.moving_direction *= -1
        self.update_turning_window()

    def search_start_grid(self, grid_map):
        xinds = []
        y_ind = 0
        if self.sweep_direction == self.SweepDirection.DOWN:
            xinds, y_ind = search_free_grid_index_at_edge_y(grid_map, from_upper=True)
        elif self.sweep_direction == self.SweepDirection.UP:
            xinds, y_ind = search_free_grid_index_at_edge_y(grid_map, from_upper=False)

        if self.moving_direction == self.MovingDirection.RIGHT:
            return min(xinds), y_ind
        elif self.moving_direction == self.MovingDirection.LEFT:
            return max(xinds), y_ind

        raise ValueError("self.moving direction is invalid ")    # 找到清扫的起点

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

# convert coordinate , keep start_pos as origin point and keep max length edge as x 
def convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_posi):
    tx = [ix - sweep_start_posi[0] for ix in ox]
    ty = [iy - sweep_start_posi[1] for iy in oy]

    th = math.atan2(sweep_vec[1], sweep_vec[0])

    c = np.cos(-th)
    s = np.sin(-th)

    rx = [ix * c - iy * s for (ix, iy) in zip(tx, ty)]
    ry = [ix * s + iy * c for (ix, iy) in zip(tx, ty)]

    return rx, ry

# return back coordinate
def convert_global_coordinate(x, y, sweep_vec, sweep_start_posi):
    th = math.atan2(sweep_vec[1], sweep_vec[0])
    c = np.cos(th)
    s = np.sin(th)

    tx = [ix * c - iy * s for (ix, iy) in zip(x, y)]
    ty = [ix * s + iy * c for (ix, iy) in zip(x, y)]

    rx = [ix + sweep_start_posi[0] for ix in tx]
    ry = [iy + sweep_start_posi[1] for iy in ty]

    return rx, ry


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


def find_goal_map(grid_map, sweep_direction):
    xinds_goaly = []
    goaly = 0
    if sweep_direction == SweepSearcher.SweepDirection.UP:        # 根据上下查找方向找到 最后一个点坐标y的坐标
        xinds_goaly, goaly = search_free_grid_index_at_edge_y(grid_map, from_upper=True)
    elif sweep_direction == SweepSearcher.SweepDirection.DOWN:
        xinds_goaly, goaly = search_free_grid_index_at_edge_y(grid_map, from_upper=False)
    return xinds_goaly, goaly


# 计算路线
def sweep_path_search(sweep_searcher, gmap, grid_search_animation=False):
    # search start grid
    cxind, cyind = sweep_searcher.search_start_grid(gmap)     #查找起点，索引
    if not gmap.set_value_from_xy_index(cxind, cyind, 0.5):   #设置起点为0.5
        print("Cannot find start grid")
        return [], []

    x, y = gmap.calc_grid_central_xy_position_from_xy_index(cxind, cyind)  # 获取索引对应的位置
    px, py = [x], [y]                                                      # 清扫路线第一个位置

    if grid_search_animation:
        fig, ax = plt.subplots()

    while True:
        cxind, cyind = sweep_searcher.move_target_grid(cxind, cyind, gmap) # 根据当前点，和栅格图，移动到下一个点

        if sweep_searcher.is_search_done(gmap) or (cxind is None or cyind is None):
            print("Done")
            break

        x, y = gmap.calc_grid_central_xy_position_from_xy_index(           # 计算位置并放入清扫路线向量中
            cxind, cyind)

        px.append(x)
        py.append(y)

        gmap.set_value_from_xy_index(cxind, cyind, 0.5)                    # 设置为0.5

        if grid_search_animation:
            gmap.plot_grid_map(ax=ax)
            plt.pause(1.0)

    return px, py


def planning(ox, oy, ox_in,oy_in,reso,
             moving_direction=SweepSearcher.MovingDirection.RIGHT,
             sweeping_direction=SweepSearcher.SweepDirection.UP,
             ):
    sweep_vec, sweep_start_posi = find_sweep_direction_and_start_posi(ox, oy)    # get max length edge, vector (dx,dy)   and  start pose

    rox, roy = convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_posi)

    fig, ax = plt.subplots()
    gmap = setup_grid_map(rox, roy, ox_in,oy_in, reso)
    gmap.plot_grid_map(ax=ax)
    sweep_searcher = SweepSearcher(moving_direction, sweeping_direction)

    px=[]
    py=[]

    for i in range(4):
        xinds_goaly, goaly = find_goal_map(gmap, sweeping_direction)
        if len(xinds_goaly) == 0:
            break
        sweep_searcher.goal_set(xinds_goaly,goaly)
        px_temp, py_temp = sweep_path_search(sweep_searcher, gmap)
        gmap.plot_grid_map(ax=ax)
        px.append(px_temp)
        py.append(py_temp)

    plt.cla()
    plt.plot(rox, roy, "-xb")
    for i in range(len(ox_in)):
        plt.plot(ox_in[i], oy_in[i], "-xb")
    rx = []
    ry = []
    path_len = 0
    color=["-xr","-xg","-xb","-.","-o"]
    for i in range(len(px)):
        plt.plot(px[i], py[i], color[i])
        rx_temp, ry_temp = convert_global_coordinate(px[i], py[i], sweep_vec, sweep_start_posi)
        rx.append(rx_temp)
        ry.append(ry_temp)
        path_len = path_len + len(rx_temp)
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.1)


    print("Path zero length:", len(rx))
    print("Path length:", path_len)


    return rx, ry


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
    if math.fabs(angle_temp) < 0.02:
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



def main():  # pragma: no cover
    print("start!!")
    head_path_x=[]
    head_path_y=[]

    path_x=[]
    path_y=[]

    # ox_outside = [0.0, 200.0, 200, 0.0, 0.0]
    # oy_outside = [0.0, 0.0,  60,  60.0, 0.0]
    ox_outside = [0.0, 200.0, 200,   150,   150.0, 50, 50, 0.0,  0.0]
    oy_outside = [0.0, 0.0,   45.0,  45.0,  60.0,  60, 45, 45,   0.0]

    for i in range(len(ox_outside)):
        ox_outside[i] = ox_outside[i] / 4.0
        oy_outside[i] = oy_outside[i] / 2.0

    ox_inside = [[140, 140, 70,70,140]]
    oy_inside = [[18, 28, 28,18,18]]
    # ox_inside = []
    # oy_inside = []
    for i in range(len(ox_inside)):
        for j in range(len(ox_inside[0])):
            ox_inside[i][j] = ox_inside[i][j]/4.0
            oy_inside[i][j] = oy_inside[i][j]/2.0
    reso = 3

    real_world_gmap = setup_grid_map(ox_outside, oy_outside, ox_inside,oy_inside, reso)           #此为假设为真实环境地图

    # A star init
    astar_path = AStarPlanner(ox_outside, oy_outside, ox_inside,oy_inside, reso, reso)

    # real_world_gmap.plot_grid_map()
    robot_pos_start = [0,0,0]                                            #此为机器人启动位置和方向

    # 创建地图
    # 1.即机器人绕行外边界一周，跟随外边界功能
    # 2.记录行走后的坐标，作为外边界
    ox_recode = [0.0, 200.0, 200,   150,   150.0, 50, 50, 0.0,  0.0]
    oy_recode = [0.0, 0.0,   45.0,  45.0,  60.0,  60, 45, 45,   0.0]

    for i in range(len(ox_recode)):
        ox_recode[i] = ox_recode[i] / 4.0
        oy_recode[i] = oy_recode[i] / 2.0

    ox_recode_in = []
    oy_recode_in = []
    # 3.构建地图，外边界地图,此地图不变；

    path_gird_map = setup_grid_map(ox_outside, oy_outside, ox_recode_in,oy_recode_in, reso)           #此为假设为真实环境地图

    # 4. 第一次清扫遍历且更新地图
    sweep_searcher = SweepSearcher(SweepSearcher.MovingDirection.RIGHT, SweepSearcher.SweepDirection.UP)  # 创建建清扫类
    cxind, cyind = sweep_searcher.search_start_grid(real_world_gmap)                                      # 获取起始位置索引

    x_start, y_start = real_world_gmap.calc_grid_central_xy_position_from_xy_index(cxind, cyind)          # 获取起始位置

    xTrue = np.array([                                                                                    # 初始状态
        [x_start],
        [y_start],
        [0],
        [0]
        ])

    
    v = 0.2             # 初始控制量
    yaw_rate = 0

    path_gird_map.set_value_from_xy_index(cxind, cyind, 0.5)         # 起点位置设置为0.5 


    HOR_search = 0
    RIGHT_UP_move = 1
    RIGHT_UP_search = 2
    UP_move = 3
    UP_search = 4
    LEFT_UP_move = 5
    LEFT_UP_search =6
    BACK_turn = 7
    BACK_search = 8 
    BACK_move = 9
    FRONT_move = 10
    X_DIRECTION_change = 11
    FINISH = 12


    curr_state = HOR_search

    sweep_x_direction = 1      # 初始清扫方向为从左到右
    sweep_x_direction_back = sweep_x_direction # for back mode 
    nxind = cxind 
    nyind = cyind

    Target_finished_flag = True   # 平移结束标志
    target_x_ind = nxind
    target_y_ind = nyind


    fig, ax = plt.subplots(1)
    ax1 = ax
    ax2 = ax
    for i in range(len(ox_inside)):
        ax2.plot(ox_inside[i], oy_inside[i], "-xb")
    ax2.plot(ox_recode, oy_recode, "-xb")


    while True:
        while curr_state != FINISH:                                      # 规划未结束
            xTrue = motion_model(xTrue, v, yaw_rate)                     # 实时更新当前位置

            obstacle_flag = get_obstacle(xTrue,real_world_gmap,reso)     # 获取当前运动方向前方障碍状态,此处实际应为传感器获取，这里假设直接读取

            if obstacle_flag: 
                path_gird_map = mapping_creat(xTrue,path_gird_map,reso)  # 存在障碍，绘图，即标注为1

            path_gird_map.set_value_from_xy_pos(xTrue[0,0], xTrue[1,0], 0.5)  # 当前所在位置，已经走过的位置，应标注为0.5 

            # 状态机
            cxind, cyind = path_gird_map.get_xy_index_from_xy_pos(xTrue[0,0], xTrue[1,0])  # 获取当前地图坐标
            # 当前方向，向右， 右边状态，若空闲则退出，若非空闲，跳跃至右上角
            if curr_state == HOR_search:
                nxind = cxind + sweep_x_direction
                nyind = cyind
                if (not path_gird_map.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5) and \
                not real_world_gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5)):   # 无障碍，且移动完成
                    curr_state = HOR_search                                                 # 下周期继续水平方向
                    v, yaw_rate,flag = move_target(xTrue,nxind,nyind,path_gird_map)
                    target_x_ind = nxind
                    target_y_ind = nyind
                else:
                    v, yaw_rate,flag = move_target(xTrue,target_x_ind,target_y_ind,path_gird_map)
                    curr_state = HOR_search
                    if flag:
                        curr_state = RIGHT_UP_move   
                        v = 0               # 停止运动
                        yaw_rate = 0           
            # 旋转至右上角方向（若左遍历，则左上角方向）
            elif curr_state == RIGHT_UP_move: 
                if sweep_x_direction>0:
                    v,yaw_rate,finish_flag = turn_target(xTrue,math.pi/4 )
                else:
                    v,yaw_rate,finish_flag = turn_target(xTrue,3* math.pi/4)
                if finish_flag:
                    v = 0.0
                    yaw_rate = 0.0
                    curr_state = RIGHT_UP_search
            # 查看右上角障碍物状态（若左遍历，则左上角方向）
            elif curr_state == RIGHT_UP_search:  #
                nxind = cxind + sweep_x_direction
                nyind = cyind + 1
                if not path_gird_map.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5) and \
                not real_world_gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5):   # 无障碍:   # 无障碍
                    curr_state = FRONT_move                                                 # 下周期继续水平方向
                else:
                    curr_state = UP_move
                v = 0.0
                yaw_rate = 0.0
            # 旋转至上方          
            elif curr_state == UP_move: 
                if sweep_x_direction>0:
                    v,yaw_rate,finish_flag  = turn_target(xTrue,math.pi/2 )
                else:
                    v,yaw_rate,finish_flag  = turn_target(xTrue,math.pi/2)
                if finish_flag:
                    v = 0.0
                    yaw_rate = 0.0
                    curr_state = UP_search
            # 查看上方障碍物状态
            elif curr_state == UP_search:  #
                nxind = cxind 
                nyind = cyind + 1
                if not path_gird_map.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5) and \
                not real_world_gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5):   # 无障碍:   # 无障碍
                    curr_state = FRONT_move                                                 # 下周期继续水平方向
                else:
                    curr_state = LEFT_UP_move
                v = 0.0
                yaw_rate = 0.0

            # 旋转至左上方（若左遍历，则右上角方向）          
            elif curr_state == LEFT_UP_move: 
                if sweep_x_direction >0:
                    v,yaw_rate,finish_flag  = turn_target(xTrue,3*math.pi/4 )
                else:
                    v,yaw_rate,finish_flag  = turn_target(xTrue,math.pi/4)
                if finish_flag:
                    v = 0.0
                    yaw_rate = 0.0
                    curr_state = LEFT_UP_search
            # 查看左上方障碍物状态（若左遍历，则右上角方向）
            elif curr_state == LEFT_UP_search:  #
                nxind = cxind - sweep_x_direction_back
                nyind = cyind + 1
                if not path_gird_map.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5) and \
                not real_world_gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5):   # 无障碍:   # 无障碍
                    curr_state = FRONT_move                                                 # 下周期继续水平方向
                else:
                    curr_state = BACK_turn
                v = 0.0
                yaw_rate = 0.0    
            # 需要回退一格，因此先旋转至后退方向
            elif curr_state == BACK_turn:  #
                if sweep_x_direction>0:
                    v,yaw_rate,finish_flag  = turn_target(xTrue,math.pi)
                else:
                    v,yaw_rate,finish_flag  = turn_target(xTrue,0)
                if finish_flag:
                    v = 0.0
                    yaw_rate = 0.0
                    curr_state = BACK_search
            # 需要回退一格
            elif curr_state == BACK_search:  #
                nxind = cxind - sweep_x_direction
                nyind = cyind
                if not path_gird_map.check_occupied_from_xy_index(nxind, nyind, occupied_val=1.0):   # 已走过的或者空白的均可
                    curr_state = BACK_move                                                 # 下周期继续水平方向
                    nxind = cxind - sweep_x_direction
                    nyind = cyind 
                else: 
                    curr_state = FINISH
                v = 0.0
                yaw_rate = 0.0 

            # 旋转至回退的方向
            elif curr_state == BACK_move:
                # nxind = cxind - sweep_x_direction
                # nyind = cyind 
                v, yaw_rate,flag = move_target(xTrue,nxind,nyind,path_gird_map)
                if math.fabs(v)<0.00001 and math.fabs(yaw_rate) < 0.000001 :
                    nxind = cxind - sweep_x_direction_back
                    nyind = cyind + 1
                    if not path_gird_map.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5):  
                        sweep_x_direction_back = -sweep_x_direction
                        curr_state = LEFT_UP_move                                               # 回退后，重复判断左上角
                    else:
                        curr_state = BACK_search

            # 向前移动一个栅格到目标位置
            elif curr_state == FRONT_move:                                                  # 前行一个栅格距离
                v, yaw_rate, flag = move_target(xTrue,nxind,nyind,path_gird_map)
                if math.fabs(v)<0.00001 and math.fabs(yaw_rate) < 0.000001 :
                    curr_state = X_DIRECTION_change

            # 移动下一行旋转至水平遍历方向
            elif curr_state == X_DIRECTION_change:                                          # 当前行旋转至水平方向
                if sweep_x_direction>0:
                    v,yaw_rate,finish_flag  = turn_target(xTrue,-math.pi)
                else:
                    v,yaw_rate,finish_flag  = turn_target(xTrue,0)
                if finish_flag:
                    v = 0.0
                    yaw_rate = 0.0
                    curr_state = HOR_search
                    sweep_x_direction = -sweep_x_direction                                  # 切换水平遍历方向
                    sweep_x_direction_back = sweep_x_direction

            print("curr: (%d), v:(%f), yaw_rate:(%f), cx:(%d), nx:(%d), cy:(%d) ny:(%d), direction:(%d),back_direction:(%d)" \
                %(curr_state,v,yaw_rate,cxind, nxind,cyind, nyind, sweep_x_direction,sweep_x_direction_back))
            # x,y=path_gird_map.calc_grid_central_xy_position_from_xy_index(nxind, nyind)
            # print("curr_x:(%f), curr_y:(%f),head:(%f),n_x:(%f),n_y:(%f)"%(xTrue[0,0], xTrue[1,0],xTrue[2,0],x,y))

            # animation(path_gird_map, path_gird_map, reso, xTrue,ax1,ax2,ox_recode,oy_recode,ox_inside,oy_inside)
            
            path_x.append(xTrue[0,0])
            path_y.append(xTrue[1,0])

            robot_model_x, robot_model_y = model_cal(xTrue)

            x_pos = xTrue[0,0] + 1.5 * math.cos(xTrue[2, 0]) 
            y_pos = xTrue[1,0] + 1.5 * math.sin(xTrue[2, 0])
            
            head_path_x.append(x_pos)
            head_path_y.append(y_pos)

            plt.cla()
            for i in range(len(ox_inside)):
                ax2.plot(ox_inside[i], oy_inside[i], "-xb")
            ax2.plot(ox_recode, oy_recode, "-xb")
            ax2.plot(robot_model_x, robot_model_y, "-.g")
            ax2.plot(path_x, path_y, ".b")
            ax2.plot(head_path_x,head_path_y,".r")
            plt.axis("equal")
            plt.pause(0.000001)

        xinds_goaly, goaly = find_goal_map(path_gird_map, -1)
        print("free point number: %d" %(len(xinds_goaly)))
        if len(xinds_goaly) == 0:
            break        
        
        xind_start, yind_start = sweep_searcher.search_start_grid(path_gird_map)
        gx, gy = path_gird_map.calc_grid_central_xy_position_from_xy_index(xind_start,yind_start)
        
        x_temp = xTrue[0,0]
        y_temp = xTrue[1,0]

        rx, ry = astar_path.planning(x_temp, y_temp, gx, gy)

        if len(rx) <= 1:
            print("can't reach target pos, len: %d"%(len(rx)))
            break        

        # while True:
        #     v, yaw_rate,flag = move_target(xTrue,nxind,nyind,path_gird_map)
        #     xTrue = motion_model(xTrue, v, yaw_rate)

        xTrue = np.array([                                                                                    # 初始状态
        [rx[-1]],
        [ry[-1]],
        [0],
        [0]
        ])
        path_x = path_x + rx
        path_y = path_y + ry

        curr_state = HOR_search
        sweep_x_direction = 1      # 初始清扫方向为从左到右
        sweep_x_direction_back = sweep_x_direction # for back mode 
        nxind = cxind 
        nyind = cyind
        v = 0
        yaw_rate = 0

    # plt.clf()
    # ax2.plot(ox_inside, oy_inside, "-xb")
    # ax2.plot(ox_recode, oy_recode, "-xb")
    # ax2.plot(path_x, path_y, ".b")
    # ax2.plot(head_path_x,head_path_y,".r")
    # plt.axis("equal")
    # plt.pause(0.1)

    # for x,y,head_x,head_y in zip(path_x,path_y,head_path_x,head_path_y):
    #     ax2.plot(x, y, ".b")
    #     ax2.plot(head_x,head_y,".r")
    #     plt.axis("equal")
    #     plt.pause(0.01)
    # planning_animation(ox_outside, oy_outside, ox_inside, oy_inside, reso)

    # ox = [0.0, 20.0, 50.0, 200.0, 130.0, 40.0, 0.0]
    # oy = [0.0, -80.0, 0.0, 30.0, 60.0, 80.0, 0.0]
    # reso = 5.0
    # planning_animation(ox, oy, reso)
    
    plt.show()

    print("done!!")


if __name__ == '__main__':
	main()
