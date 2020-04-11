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

goal = (50,50)


# 1) Randomly initialize populations p
# 2) Determine fitness of population
# 3) Untill convergence repeat:
#       a) Select parents from population
#       b) Crossover and generate new population
#       c) Perform mutation on new population
#       d) Calculate fitness for new population

class Chromosome:
    def __init__(self, node, collected = 0, health = 100):
        self.fitness = 0
        self.node = node
        self.collected = collected
        self.health = health
        self.calc_fitness()
        
    def __str__(self):
        return str(self.node) + " -- collected = " + str(self.collected) + " -- health = " + str(self.health) + " -- fitness = " + str(self.fitness)
        
    # def verify(self, node, collectables_list):
    #     px = self.calc_grid_position(node.x, self.minx)
    #     py = self.calc_grid_position(node.y, self.miny)
        
    #     if px < self.minx:
    #         return False
    #     elif py < self.miny:
    #         return False
    #     elif px >= self.maxx:
    #         return False
    #     elif py >= self.maxy:
    #         return False

    #     # collision check
    #     if self.obmap[node.x][node.y]:
    #         return False
        
    #     return True

    def calc_fitness(self):
        # fitness function --> param --> path (position between node and target) , number of collectables, health and safe
        dist = math.sqrt((goal[0]-self.node.x)**2 + (goal[1]-self.node.y)**2)
        wl = 1
        wc = 25
        wh = 1
        ws = 1
        safe = 0 # function to be implemented to cal safe
        self.fitness = (1/(wl*dist + ws*safe)) + wc*self.collected + wh*self.health

class GA:
    def __init__(self,a_star,population_size,elitism,crossover,mutation,num_iterations = 100):
        self.population_size = population_size
        self.num_iterations = num_iterations
        self.elitismNum = elitism 
        self.crossoverNum = crossover 
        self.mutationNum = mutation 
        self.population = []
        self.a_star = a_star
        self.firstRandomPopluation()
        
    
    def firstRandomPopluation(self):
        while(len(self.population) != self.population_size):
            p = [0,0]
            p[0] = random.randint(1, 99)
            p[1] = random.randint(1, 99)
            # print(p)
            if(self.a_star.verify(AStarPlanner.Node(p[0],p[1],0,None,None))):
                chrom = Chromosome(AStarPlanner.Node(p[0],p[1],0,None,None))
                self.population.append(chrom)
            # print(len(self.population))
            
    def sortByFitness(self,population):
        return population.fitness
    
    def elitism(self):
        self.population.sort(key=self.sortByFitness)
        return self.population[0:self.elitismNum]
    
    def crossover(self):
        self.population.sort(key=self.sortByFitness)
        elite = self.population[0]
        crossoverList = []
        history = [] # for better results
        while len(crossoverList) != self.crossoverNum:
            # crossover between the elite and a random chromosome
            alpha = 0.4
            r = random.randint(1,len(self.population) - 1)
            cross = self.population[r]
            if cross not in history:
                history.append(cross)
                x1 = int((elite.node.x*alpha) + (cross.node.x*(1-alpha)))
                y1 = int((elite.node.y*alpha) + (cross.node.y*(1-alpha)))
                newNode1 = AStarPlanner.Node(x1,y1,0,None,None)
                newCollection1 = int((elite.collected * alpha) + (cross.collected * (1-alpha)))
                newHealth1 = int((elite.health * alpha) + (cross.health * (1-alpha)))
                newChromosome1 = Chromosome(newNode1,newCollection1,newHealth1)
                
                x2 = int((elite.node.x*(1-alpha)) + (cross.node.x*alpha))
                y2 = int((elite.node.y*(1-alpha)) + (cross.node.y*alpha))
                newNode2 = AStarPlanner.Node(x2,y2,0,None,None)
                newCollection2 = int((elite.collected * (1-alpha)) + (cross.collected * alpha))
                newHealth2 = int((elite.health * (1-alpha)) + (cross.health * alpha))
                newChromosome2 = Chromosome(newNode2,newCollection2,newHealth2)
                
                if self.a_star.verify(newNode1) and len(crossoverList) < self.crossoverNum:
                    crossoverList.append(newChromosome1)
                    
                if self.a_star.verify(newNode2) and len(crossoverList) < self.crossoverNum:
                    crossoverList.append(newChromosome2)
        
        return crossoverList
    
    def mutation(self):
        worstChrom = self.population[len(self.population) - 1]
        x = random.randint(1,99)
        y = random.randint(1,99)
        while not self.a_star.verify(AStarPlanner.Node(x,y,0,None,None)):
            x = random.randint(1,99)
            y = random.randint(1,99)
        if (worstChrom.health + random.randint(0,100)) > 100:
            worstChrom.health = 100 
        else: 
            worstChrom.health = worstChrom.health + random.randint(0,100)
        return Chromosome(AStarPlanner.Node(x,y,0,None,None),worstChrom.collected,worstChrom.health)
        
    def runGA(self):
        plt.grid(True)
        plt.axis("equal")
        for x in range(self.num_iterations):
            print("inn")
            e = self.elitism()
            c = self.crossover()
            m = self.mutation()
            self.population = []
            self.population += e
            self.population += c
            self.population.append(m)
            print(len(self.population))
            if show_animation:  # pragma: no cover
                for pos in (self.population):
                    plt.plot(pos.node.x, pos.node.y, c=next(cycol), marker="o")
                plt.show()
                
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
            return "(" + str(self.x) + "," + str(self.y) + ")"

    def planning(self, gx, gy, list_of_start_pos, list_collectables):
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
                plt.plot(self.calc_grid_position(current.x, self.minx), self.calc_grid_position(current.y, self.miny), ".c")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set) % 10 == 0:
                    plt.pause(0.001)
            if len(list_collectables) == 0 and (current.x == ngoal.x and current.y == ngoal.y):
                print(current.collected)
            #if current.collected>= 1:
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
    
    # for GA
    def verify(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)
        
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
    print("start!!")

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
    list_pos = [(70, 70), (20, 20), (60, 60)]
    #list_pos = []
    list_collectables = [(60, 80), (30, 30), (10,60)]
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
    # rx, ry = a_star.planning(gx, gy, list_pos, list_collectables)
    
    ga = GA(a_star,20,2,17,1)
    ga.runGA()

    # if show_animation:  # pragma: no cover
    #     plt.plot(rx, ry, "-r")
    #     plt.show()
    



main()