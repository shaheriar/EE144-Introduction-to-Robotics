#!/usr/bin/env python

import math

#--------------------------------------------------------------------
class node: #NODE CLASS TO HELP STORE INFORMATION BETTER
    def __init__(self,coord, f, g, parent):
        self.g = g  #relative cost
        self.f = f  #total cost - f(N) = H(N) + G(N) where H(N) is the value returned by the heuristic_distance function
        self.parent = parent    #the parent of the current node
        self.coord = coord      #the coordinates of the current node

def obs(curr,obstacles):    #FUNCTION TO CHECK IF CURR IS AN OBSTACLE
    for x in obstacles:
        if (curr == x):
            return True
    return False
#--------------------------------------------------------------------

def neighbors(current):
    # define the list of 4 neighbors
    neighbors = [(0,1),(0,-1),(-1,0),(1,0)] #UP,DOWN,LEFT,RIGHT
    return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ]

def heuristic_distance(candidate, goal):
    return math.sqrt((candidate[0]-goal[0])**2 + (candidate[1]-goal[1])**2)    #EUCLIDEAN DISTANCE

def get_path_from_A_star(start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)] 
    open = list()
    closed = dict()
    path = list()
    open.append(node(start, heuristic_distance(start,goal), 0, None))    #APPEND ROOT NODE
    
    while len(open) > 0:
        curr = open.pop(0)  #GET TOP OF THE LIST AND POP FROM LIST
        closed[curr.coord] = curr.coord #ADD TO VISITED LIST

        if curr.coord == goal:  #IF ITS THE GOAL THEN BACK TRACK UNTIL ROOT NODE IS REACHED
            c = curr
            while c is not None:
                path.append(c.coord)
                c = c.parent
            return path[-2::-1] #RETURNING REVERSED LIST ELIMINATING THE START NODE

        for n in neighbors(curr.coord): #FOR ALL THE NEIGHBORS OF CURR, GET THE DISTANCE AND APPEND
            if not (n in closed) and (not obs(n, obstacles)):
                open.append(node(n, heuristic_distance(n, goal) + curr.g, curr.g + 1, curr))

        open.sort(key=lambda x: x.f)  #SORT THE LIST SO LOWEST COST IS FIRST
    return path