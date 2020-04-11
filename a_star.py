"""
A* grid planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import math

import matplotlib.pyplot as plt

import random

from itertools import cycle
cycol = cycle('bgrcmk')

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()
        self.found = False

    class Node:
        def __init__(self, x, y, collected, pind, parent):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.pind = pind
            self.parent = parent
            self.collected = collected

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.pind) + "," + str(self.collected)

    def planning(self, gx, gy, list_of_start_pos, list_collectables):
        """
        A star path search
        input:
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        open_set, closed_set, visited_ids = [], [], []
        for x,y in list_of_start_pos:
            n = self.Node(self.calc_xyindex(x, self.minx), self.calc_xyindex(y, self.miny) , 0, -1, None)
            open_set.append(n)
            visited_ids.append(n.pind)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx), self.calc_xyindex(gy, self.miny), 0, -1, None)

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = 0
            current = open_set.pop(c_id)

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.minx), self.calc_grid_position(current.y, self.miny), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set) % 10 == 0:
                    plt.pause(0.001)
            if len(list_collectables) == 0 and (current.x == ngoal.x and current.y == ngoal.y):
                print(current.collected)
                print("Find goal")
                ngoal.pind = current.pind
                break

            # Add it to the closed set
            

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = None
                if(self.motion[i][2]):
                    node = self.Node(current.x + self.motion[i][0], current.y + self.motion[i][1], current.collected + 1, 0, current)
                else:
                    node = self.Node(current.x + self.motion[i][0], current.y + self.motion[i][1], current.collected, 0, current)
                n_id = str(self.calc_grid_index(node))+","+str(node.collected)
                node.pind = n_id

                # If the node is not safe, do nothing

                if (n_id not in visited_ids) and self.verify_node(node, list_collectables, self.motion[i][2]):
                    open_set.append(node)  # discovered a new node
                    visited_ids.append(node.pind)
                    closed_set.append(current)

        rx, ry = self.calc_final_path(ngoal, closed_set)

        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final course
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        print(len(closedset))
        #parent = closedset[-1].parent
        while pind != -1:
            n = closedset.pop(-1)
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            pind = n.pind
           # parent = n.parent

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

    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node, collectables_list, collect):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)
        #print(collect)
        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[node.x][node.y]:
            return False
        # collection
        if collect and not((px, py) in collectables_list):
            return False
        if collect and ((px, py) in collectables_list):
            collectables_list.remove((px, py))
            print(collectables_list)
        #print(True)

        return True

    def calc_obstacle_map(self, ox, oy):

        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        print("minx:", self.minx)
        print("miny:", self.miny)
        print("maxx:", self.maxx)
        print("maxy:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        self.ywidth = round((self.maxy - self.miny) / self.reso)
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)

        # obstacle map generation
        self.obmap = [[False for i in range(self.ywidth)]
                      for i in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, False],
                  [0, 1, False],
                  [-1, 0, False],
                  [0, -1, False],
                  [-1, -1,False],
                  [-1, 1, False],
                  [1, -1, False],
                  [1, 1, False],
                  [0, 0, True]]

        return motion

def three(sx, sy, tlen):
    for i in range(sx, sx+tlen):
        ox.append(i)
        oy.append(50.0)
    # -1-
    for i in range(0, 10):
        ox.append(sx)
        oy.append(sy - i)
    for i in range(sx, sx+tlen):
        ox.append(i)
        oy.append(sy - tlen)
    for i in range(sx, sx+tlen):
        ox.append(10)
        oy.append(40 - i)
    for i in range(0, 11):
        ox.append(i)
        oy.append(30.0)

def main():
    print(__file__ + " start!!")


    # set obstable positions
    ox, oy = [], []
    for i in range(0, 100):
        ox.append(i)
        oy.append(0)
    for i in range(0, 100):
        ox.append(100)
        oy.append(i)
    for i in range(0, 100):
        ox.append(i)
        oy.append(100)
    for i in range(0, 100):
        ox.append(0)
        oy.append(i)
    #3
    #--
    for i in range(5, 15):
        ox.append(i)
        oy.append(50.0)
    # -1-
    for i in range(0, 10):
        ox.append(15)
        oy.append(50 - i)
    for i in range(5, 15):
        ox.append(i)
        oy.append(40.0)
    for i in range(0, 10):
        ox.append(15)
        oy.append(40 - i)
    for i in range(4, 15):
        ox.append(i)
        oy.append(30.0)
    #7
    #--
    for i in range(20, 40):
        ox.append(i)
        oy.append(50.0)
    #/
    for i in range(0, 20):
        ox.append(40 - i)
        oy.append(50 - i)
    #-
    for i in range(40, 50):
        ox.append(i)
        oy.append(40.0)
    #1
    for i in range(0, 20):
        ox.append(60)
        oy.append(50 - i)
    #7
    #--
    for i in range(70, 90):
        ox.append(i)
        oy.append(50.0)
    #/
    for i in range(0, 20):
        ox.append(90 - i)
        oy.append(50 - i)
    sx = 70  # [m]
    sy = 70  # [m]
    list_pos = [(70, 70), (20, 20), (60, 60), (90, 90)]
    #list_pos = []
    list_collectables = [(60, 80), (30, 30)]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        #plt.plot(sx, sy, "oy")
        for pos in (list_pos):
            plt.plot(pos[0], pos[1], c=next(cycol), marker="o")
        for x,y in list_collectables:
            plt.plot(x, y, c=next(cycol), marker="v")
        plt.plot(gx, gy, c="r", marker="$g$")
        plt.grid(True)
        plt.axis("equal")
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(gx, gy, list_pos, list_collectables)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()
#s