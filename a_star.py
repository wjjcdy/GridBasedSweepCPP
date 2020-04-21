"""
A* grid planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import math

import matplotlib.pyplot as plt
import numpy as np
import os
import sys
sys.path.append(os.path.relpath("./grid_map_lib/"))
try:
    from grid_map_lib import GridMap
except ImportError:
    raise

class AStarPlanner:

    def __init__(self, ox, oy, ox_in,oy_in,reso, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy,ox_in,oy_in)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_x_id, start_y_id = self.obmap.get_xy_index_from_xy_pos(sx, sy)
        goal_x_id, goal_y_id = self.obmap.get_xy_index_from_xy_pos(gx, gy)

        nstart = self.Node(start_x_id,
                           start_y_id, 0.0, -1)
        ngoal = self.Node(goal_x_id,
                          goal_y_id, 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(ngoal, closed_set)

        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final course

        rx_goal,ry_goal = self.obmap.calc_grid_central_xy_position_from_xy_index(ngoal.x, ngoal.y)
        rx,ry = [rx_goal],[ry_goal]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            x,y = self.obmap.calc_grid_central_xy_position_from_xy_index(n.x, n.y)
            rx.append(x)
            ry.append(y)
            pind = n.pind

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, minp):
        """
        calc grid position
        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

    def calc_grid_index(self, node):
        return self.obmap.calc_grid_index_from_xy_index(node.x,node.y)

    def verify_node(self, node):
        if node.x <0 or node.x >= self.xwidth:
            return False
        if node.y <0 or node.y >= self.ywidth:
            return False

        # collision check
        if self.obmap.check_occupied_from_xy_index(node.x, node.y, occupied_val=1.0):
            return False

        return True

    def calc_obstacle_map(self, ox, oy, ox_in, oy_in):

        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))

        # self.xwidth = round((self.maxx - self.minx) / self.reso)
        # self.ywidth = round((self.maxy - self.miny) / self.reso)

        self.xwidth = math.ceil((max(ox) - min(ox)) / self.reso)     # 创建一个栅格地图，可包括整个区域
        self.ywidth = math.ceil((max(oy) - min(oy)) / self.reso)
        center_x = np.mean(ox)
        center_y = np.mean(oy)

        grid_map = GridMap(self.xwidth,  self.ywidth, self.reso, center_x, center_y)    #创建

        grid_map.set_value_from_polygon(ox, oy, 1.0, inside=False)     #将多边形外均设为1，内部默认为0

        for i in range(len(ox_in)):
            grid_map.set_value_from_polygon(ox_in[i], oy_in[i], 1.0, inside=True)     #将多边形内均设为1，外部默认为0

        grid_map.expand_grid()    # 膨胀一个栅格大小

        # obstacle map generation
        self.obmap = grid_map

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


