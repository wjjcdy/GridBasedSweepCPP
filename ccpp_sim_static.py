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

        ncxind_list = []
        ncyind_list = []

        # found safe grid
        if not gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5):  # 若为空闲直接返回下个点，未走过的是空闲。第一次探索不可以为已经走过的路
            ncxind_list.append(nxind)
            ncyind_list.append(nyind)
            return ncxind_list, ncyind_list
        else:  # occupided
            ncxind, ncyind = self.find_safe_turning_grid(cxind, cyind, gmap)       # 判断拐弯所有的点，包括当前位置上方三个位置（即8连通的上方3个位置）
            if (ncxind is None) and (ncyind is None):                              # 如果都没有空间，
                # moving backward                                                  # 则应回退一格
                ncxind = -self.moving_direction + cxind
                ncyind = cyind

                if gmap.check_occupied_from_xy_index(ncxind, ncyind):              # 回退一格可以为走过的路，即回退可为行走过的路，
                    # moved backward, but the grid is occupied by obstacle
                    return ncxind_list, ncyind_list
                else:
                    ncxind_list.append(ncxind)
                    ncyind_list.append(ncyind)
            else:                                                                  # 若拐弯存在空闲点, 即需要转向下一行，需要从下一行尽头开始(注意此时位置和下一刻位置有可能不再连续)
                # keep moving until end
                ncxind_list.append(ncxind)
                ncyind_list.append(ncyind)
                while not gmap.check_occupied_from_xy_index(ncxind + self.moving_direction, ncyind, occupied_val=0.5):
                    ncxind += self.moving_direction
                    ncxind_list.append(ncxind)
                    ncyind_list.append(ncyind)
                self.swap_moving_direction()                                       # 左右扫描的方向需要进行切换
            return ncxind_list, ncyind_list


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
            x_pos, y_pos = grid_map.calc_grid_central_xy_position_from_xy_index(xinds[-1],yind)
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
        cxind_list, cyind_list = sweep_searcher.move_target_grid(cxind, cyind, gmap) # 根据当前点，和栅格图，移动到下一个点

        if sweep_searcher.is_search_done(gmap) or (len(cxind_list)==0):
            print("Done")
            break

        for cx, cy in zip(cxind_list, cyind_list):
            x, y = gmap.calc_grid_central_xy_position_from_xy_index(           # 计算位置并放入清扫路线向量中
                cx, cy)

            px.append(x)
            py.append(y)

        cxind = cxind_list[-1]
        cyind = cyind_list[-1]
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

    astar_path = AStarPlanner(ox, oy, ox_in,oy_in, reso, reso)

    px=[]
    py=[]

    for i in range(10):
        xinds_goaly, goaly = find_goal_map(gmap, sweeping_direction)
        if len(xinds_goaly) == 0:
            break
        sweep_searcher.goal_set(xinds_goaly,goaly)
        px_temp, py_temp = sweep_path_search(sweep_searcher, gmap)
        gmap.plot_grid_map(ax=ax)
        px.append(px_temp)
        py.append(py_temp)

        xinds_goaly, goaly = find_goal_map(gmap, sweeping_direction)
        if len(xinds_goaly) == 0:
            break
        xind_start, yind_start = sweep_searcher.search_start_grid(gmap)
        gx, gy = gmap.calc_grid_central_xy_position_from_xy_index(xind_start,yind_start)
        rx, ry = astar_path.planning(px_temp[-1], py_temp[-1], gx, gy)

        px.append(rx)
        py.append(ry)

    plt.cla()
    plt.plot(rox, roy, "-xb")
    for i in range(len(ox_in)):
        plt.plot(ox_in[i], oy_in[i], "-xb")
    rx = []
    ry = []
    path_len = 0
    color=["-xr","-.","-or"]
    color1=["-x","-.","-o"]
    for i in range(len(px)):
        if i==2 :
            for ipx, ipy in zip(px[i],py[i]):
                plt.plot(ipx, ipy, color[i%3])
                plt.axis("equal")
                plt.grid(True)
                plt.pause(0.0001)
        else:
            plt.plot(px[i], py[i], color1[i%3])
        #map coordinate convert to world 
        rx_temp, ry_temp = convert_global_coordinate(px[i], py[i], sweep_vec, sweep_start_posi)
        rx.append(rx_temp)
        ry.append(ry_temp)
        path_len = path_len + len(rx_temp)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(1)
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


def main():  # pragma: no cover
    print("start!!")

    # ox = [0.0, 20.0, 50.0, 100.0, 130.0, 40.0, 0.0]
    # oy = [0.0, -20.0, 0.0, 30.0, 60.0, 80.0, 0.0]
    # reso = 5.0
    # planning_animation(ox, oy, reso)

    # ox = [0.0, 50.0, 50.0, 0.0, 0.0]
    # oy = [0.0, 0.0, 30.0, 30.0, 0.0]
    # reso = 1.3
    # planning_animation(ox, oy, reso)

    # ox = [0.0, 100.0, 100.0, 50, 50, 75, 75, 100, 100, 0.0, 0.0]
    # oy = [0.0, 0.0,  18.0,   18, 28, 28, 18, 18 , 40,  40.0, 0.0]
    # reso = 3
    # planning_animation(ox, oy, reso)

    # ox = [0.0, 200.0, 200.0, 50, 50, 75, 75, 100, 100, 150, 150, 200 ,200, 0.0, 0.0]
    # oy = [0.0, 0.0,  18.0,   18, 28, 28, 18, 18 , 28,  28,  18,  18, 40,  40.0, 0.0]
    # reso = 3
    # planning_animation(ox, oy, reso)

    ox_outside = [0.0, 200.0, 200, 120, 80, 0.0, 0.0]
    oy_outside = [0.0, 0.0,  60,  90 ,  90,  60.0, 0.0]

    # ox_outside = [0.0, 200.0, 120, 80, 0.0, 0.0]
    # oy_outside = [0.0, 0.0 ,  90 ,  90,  60.0, 0.0]

    # ox_inside = [[50, 50, 90, 75, 50],[100, 150, 130, 100],[160, 170, 130 , 160]]
    # oy_inside = [[18, 48, 48, 28, 18],[18 , 45,  28,  18] ,[20,  30,  10,   20]]

    ox_inside = [[50,  90, 75, 50],[100, 150, 130, 100],[160, 170, 130 , 160]]
    oy_inside = [[18,  48, 28, 18],[18 , 45,  28,  18] ,[20,  30,  10,   20]]
    
    reso = 3
    planning_animation(ox_outside, oy_outside, ox_inside, oy_inside, reso)

    # ox = [0.0, 20.0, 50.0, 200.0, 130.0, 40.0, 0.0]
    # oy = [0.0, -80.0, 0.0, 30.0, 60.0, 80.0, 0.0]
    # reso = 5.0
    # planning_animation(ox, oy, reso)

    plt.show()

    print("done!!")


if __name__ == '__main__':
	main()
