#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Author: Italo Barros
Email: ircbarros@pm.me
License: MIT
The Path Planning algorithms for the World Grid Module!
You can run this module using the command line:
$ python3 -m memory_profiler main.py
The program will popups, and you can run your Path Planning tests.
After you closes the PyGame Screen the memory usage will be shown
at the terminal. For more informations about the memory profiler
package go to:
https://pypi.org/project/memory-profiler/
Classes:
    PathPlanning: A class with some Path Planning Algorithms
Comments:
If you are using VS Code, please install the Better Comments Extension:
    # Indicates the Method
    #* Indicates some Important Information
    #! Indicates a deprecated or Warning Information
    #? Indicates possible future changes and questions
    #TODO: Indicates the future changes and optimizations
Need to change the code? Refactor? Help the next developer! Use a 
Style Guide to help others understand of your code. For more informations
about the Google Style Guide used here go to:
https://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_google.html
'''

import sys
import argparse
import pygame
import random
import timeit
import concurrent.futures
import multiprocessing
import memory_profiler
import pandas as pd
import numpy as np
from operator import itemgetter
#? Using the collections module since is the most efficient
#? to implement and manipulate a queue list
from collections import deque
#* To see the sum of the system and user CPU Time of
#* of the current process
from time import process_time
from timeit import default_timer as timer
#* To see the memory usage in each line
import memory_profiler
#* Import the createWorld Module
from world import createBigWorld
from world.settings import settingsBigWorld
# LOAD THE WORLD VARIABLES
#* Horizontal Layout
world = createBigWorld.create
# LOAD THE PYGAME DEFAULTS VARIABLES
default = settingsBigWorld.PygameDefaults()
# LOAD THE COLORS FROM THE SETTINGS
paint = settingsBigWorld.Colors()
# CHECK NaN VALUES
nan_value = float('nan')
# THE SELECTED STARTS
SelectedStarts = set()
# THE CA* HASH TABLE (DICT)
ReservationTable = []
# THE SAVED PATHS
DesiredPaths = []
# CBS CONSTRAINTS
CbsConstraints = []
# THE SWAM HASH TABLE
swarmAreas = {}
# THE NUMBER OF SWARMS
swarm_count = 0
#* Load the PyGame Vector2 lib
vec = pygame.math.Vector2


class PathPlanning:
    """
    The Class Responsible to multiple Path Planning Algorithms and functions,
    responsible to return or execute the Free Space, the Breadth-First Search,
    the Dijkstra's, the A-Star (A*), the Space-Time A* (STA*), the Multi Agent
    Pathfinding (MAPF), or the Boid Flocking algorithm.
    For more informations of each method, see the specified docstring!
    Functions:
        find_free_space(graph, position): returns the free space available in
                                          the grid
        breath_first_search: returns shortest path executed by the algorithm
        dijkstra: returns shortest the path executed by the algorithm
        a_star: returns the shortest path executed by the algorithm
        sta_star: returns the path executed by the algorithm
        mapf: execute the MAPF planning with the sta_star at low level
        mapf_swarm: execute the MAPF planning in conjunction with the
                    boid flocking algorithm
    """
    def __init__(self):

        pass

    def find_all_nodes(self, graph, goal):

        # TRANSFORM THE POSITION TO THE PYGAME VECTOR
        self.goal = goal
        # IMPORT THE DEQUE TO PUT THE NODES
        self.frontier = deque()
        # APPEND THE FRONTIER WITH THE POSITION
        self.frontier.append(self.goal)
        # THE LIST OF VISITED NODES
        self.visited = []
        # THE POSITION WILL BE PUT AT THE VISITED QUEUE (IS WHERE WE ARE)
        self.visited.append(self.goal)
        # START OUR LOOP
        #* As long there's nodes on the frontier do
        while len(self.frontier) > 0:
            # THE CURRENT NODE WE WANT TO LOOK IS THE NEXT NODE
            #* Pop's the next on the queue list
            self.current = self.frontier.popleft()
            # THE NEIGHBOORS OF THE CURRENT TILE
            for next in graph.find_all_neighbors(self.current):
                # IF THE NEXT NODE IS NOT VISITED
                if next not in self.visited:
                    # ADD THE NODE TO THE FRONTIER LIST
                    self.frontier.append(next)
                    # PUT ON THE VISITED NODES
                    self.visited.append(next)
        # PRINT ALL THE VISITED NODES
        global allNodes
        allNodes = len(self.visited)
        print(f'\nThe All Node Cells Available are:\n{self.visited}')
        return self.visited

    def find_free_space(self, graph, goal=world.GOAL):
        """
        Reads free nodes in World Grid using the find_neighbors(node) function,
        and returns a list of free nodes that can be explored starting from the
        position inputed.
        Attributes:
            (graph) (Class)
            (position) (tuple)
        
        Args:
            (graph): A World Grid Class
            (goal): Is the goal position in the World Grid that
                    we want to find the free space as a tuple
        
        Returns:
            A list with the free nodes available in the World Grid
        [Example]

            If using the default values, this call will provide the following vectors:
            $ python3 pathPlanning.py
            [<Vector2(0, 0)>, <Vector2(1, 0)>, <Vector2(0, 1)>, <Vector2(2, 0)>, <Vector2(0, 2)>,
            <Vector2(3, 0)>, <Vector2(0, 3)>, <Vector2(4, 0)>, <Vector2(1, 3)>, <Vector2(0, 4)>,
            <Vector2(5, 0)>, <Vector2(4, 1)>, <Vector2(2, 3)>, <Vector2(0, 5)>, <Vector2(6, 0)>,
            <Vector2(4, 2)>, <Vector2(3, 3)>, <Vector2(0, 6)>, <Vector2(7, 0)>, <Vector2(4, 3)>,
            <Vector2(1, 6)>, <Vector2(0, 7)>, <Vector2(8, 0)>, <Vector2(5, 3)>, <Vector2(4, 4)>,
            <Vector2(2, 6)>, <Vector2(1, 7)>, <Vector2(0, 8)>, <Vector2(9, 0)>, <Vector2(8, 1)>,
            <Vector2(6, 3)>, <Vector2(4, 5)>, <Vector2(3, 6)>, <Vector2(2, 7)>, <Vector2(1, 8)>,
            <Vector2(9, 1)>, <Vector2(8, 2)>, <Vector2(7, 3)>, <Vector2(4, 6)>, <Vector2(3, 7)>,
            <Vector2(2, 8)>, <Vector2(9, 2)>, <Vector2(8, 3)>, <Vector2(5, 6)>, <Vector2(4, 7)>,
            <Vector2(3, 8)>, <Vector2(9, 3)>, <Vector2(8, 4)>, <Vector2(6, 6)>, <Vector2(5, 7)>,
            <Vector2(4, 8)>, <Vector2(9, 4)>, <Vector2(8, 5)>, <Vector2(7, 6)>, <Vector2(6, 7)>,
            <Vector2(5, 8)>, <Vector2(9, 5)>, <Vector2(8, 6)>, <Vector2(7, 7)>, <Vector2(6, 8)>,
            <Vector2(9, 6)>, <Vector2(8, 7)>, <Vector2(7, 8)>, <Vector2(9, 7)>, <Vector2(8, 8)>,
            <Vector2(9, 8)>]
        """
        print('\n__________________ FREE PATH SEARCH STARTED __________________\n')
        # TRANSFORM THE POSITION TO THE PYGAME VECTOR
        self.goal = goal
        # IMPORT THE DEQUE TO PUT THE NODES
        self.frontier = deque()
        # APPEND THE FRONTIER WITH THE POSITION
        self.frontier.append(self.goal)
        # THE LIST OF VISITED NODES
        self.visited = []
        # THE POSITION WILL BE PUT AT THE VISITED QUEUE (IS WHERE WE ARE)
        self.visited.append(self.goal)
        # START OUR LOOP
        #* As long there's nodes on the frontier do
        while len(self.frontier) > 0:
            # THE CURRENT NODE WE WANT TO LOOK IS THE NEXT NODE
            #* Pop's the next on the queue list
            self.current = self.frontier.popleft()
            # THE NEIGHBOORS OF THE CURRENT TILE
            for next in graph.find_neighbors(self.current):
                #print(self.current)
                # IF THE NEXT NODE IS NOT VISITED
                if next not in self.visited:
                    # ADD THE NODE TO THE FRONTIER LIST
                    self.frontier.append(next)
                    # PUT ON THE VISITED NODES
                    self.visited.append(next)
        global visitedNodes
        visitedNodes = len(self.visited)
        print(f'\nThe Node Cells are:\n{self.visited}')
        return self.visited

    def astar_search(self, graph, start,
                     goal):
         #* START THE TIMER
        start_time = timeit.default_timer()
        start_process_time = process_time()
        # SET THE START AND GOAL VALUES
        self.start = start
        print(f'The Goal Node is located at: {self.start}')
        self.goal = goal
        print(f'The Start Node is located at: {self.goal}')
        # IMPORT THE QUEUE TO PUT THE NODES
        self.frontier = createBigWorld.PriorityQueue()
        #* Put the nodes on the Frontier with cost 0
        self.frontier.put(vec_to_int(self.start), 0)
        #* Starts the Path Dictionary
        self.path = {}
        #* Starts the Cost Dictionary
        self.cost = {}
        # THE START IS NONE SINCE IS WERE WE ARE
        self.path[vec_to_int(self.start)] = None
        self.cost[vec_to_int(self.start)] = 0
        #? Init the While Interactions Variable
        self.while_interactions = 0
        while not self.frontier.empty():
            #? Add 1 interaction for every loop
            self.while_interactions += 1
            #* The next one will be the one with lowest cost
            self.current = self.frontier.get()
            #* If the goal is reached break
            if self.current == self.goal:
                break
            #* Find the neighbors of the current node
            #? Init the For Interactions Variable
            self.for_interactions = 0
            for next in graph.find_neighbors(vec(self.current)):
                #? Add 1 interaction for every loop
                self.for_interactions += 1
                next = vec_to_int(next)
                #* The cost is the atual cost plus the cost to move to the next node
                self.next_cost = self.cost[self.current] + graph.cost(self.current, next)
                #* If not in the cost or have a lower cost
                if next not in self.cost or self.next_cost < self.cost[next]:
                    #* Update the values
                    self.cost[next] = self.next_cost
                    #? Instead of the Dijkstra, the priority will be the heuristic function
                    self.priority = self.next_cost + manhattan_distance(self.goal, vec(next))
                    #* Put in the priority
                    self.frontier.put(next, self.priority)
                    #* Put in the path vector
                    self.path[next] = vec(self.current) - vec(next)
        #* Stop the Default Timer (Wall Timer)
        stop_time = timeit.default_timer()
        #* Stop the Process Timer (Wall Timer)
        stop_process_time = process_time()
        # PRINT ALL THE VISITED NODES
        print(f"\nThe A* Search Path Available Nodes Movement are:\n{self.path}")
        print(f"\nThe  A* Search Path have: {len(self.path)} Available Nodes")
        print(f"\nThe  A* Search Path 'While Loop' Interactions was: {self.while_interactions}")
        print(f"\nThe  A* Search Path 'For Loop' Interactions was: {self.for_interactions}")
        print("\nThe  A* Search Path 'Wall time' was ", stop_time - start_time, 'sec')
        print("\nThe  A* Search Path 'Process Time' was ",
              stop_process_time - start_process_time, 'sec\n')
        
        return self.path


    def cooperative_astar_search(self, graph, start,
                                 goal,
                                 time = world.TIME_LIMIT):
        """
        Reads free nodes in a Wheighted Grid using the find_neighbors(node) function,
        and returns the shortest path using Cooperative A-Star (A*) Search for the
        Weighted Nodes.
        Attributes:
            (graph) (Class)
            (start) (tuple)
            (goal)  (tuple)
            (time)  (int)
        
        Args:
            (graph): A Weighted Grid
            (goal) : Is the position in the Weighted Grid that
                     we want to achieve
            (time) : Is the time limit that to run the algorithm
        
        Vars:
            limit_time (int) = The Maximum Time we Can Run The Search
        
        Returns:
            The Shortest Path Using Cooperative A-Star (CA*) Search for the START
            and GOAL nodes provided
        """
    
        print("\n__________________ COOPERATIVE A* (CA*) SEARCH STARTED __________________\n")    
        #* START THE TIMER
        start_time = timeit.default_timer()
        start_process_time = process_time()
        # SET THE START AND GOAL VALUES
        #* The start is the goal since the A* runs backwards
        self.start = start
        print(f'The Goal Nodes is: {self.start}')
        self.goal = goal
        print(f'The Start Node are located at: {self.goal}')
        self.time = time
        print(f'The Time Limit is: {self.time[1]}')
        # IMPORT THE QUEUE TO PUT THE NODES
        self.frontier = createBigWorld.PriorityQueue()
        #* Put the nodes on the Frontier with cost 0
        self.frontier.put(vec_to_int(self.start), 0)
        #* Starts the Path Dictionary
        self.path = {}
        #* Starts the Cost Dictionary
        self.cost = {}
        # THE START IS NONE SINCE IS WERE WE ARE
        self.path[vec_to_int(self.start)] = None
        self.cost[vec_to_int(self.start)] = 0
        #? Init the While Interactions Variable
        self.while_interactions = 0
        while not self.frontier.empty():
            #? Add 1 interaction for every loop
            self.while_interactions += 1
            #* The next one will be the one with lowest cost
            self.current = self.frontier.get()
            #* If the goal is reached break
            if self.current == self.goal:
                break
            #* Find the neighbors of the current node
            #? Init the For Interactions Variable
            self.for_interactions = 0
            for next in graph.find_neighbors(vec(self.current)):
                #? Add 1 interaction for every loop
                self.for_interactions += 1
                next = vec_to_int(next)
                #* The cost is the atual cost plus the cost to move to the next node
                self.next_cost = self.cost[self.current] + graph.cost(self.current, next)
                #* If not in the cost or have a lower cost
                if next not in self.cost or self.next_cost < self.cost[next]:
                    #* Update the values
                    self.cost[next] = self.next_cost
                    #? Instead of the Dijkstra, the priority will be the heuristic function
                    self.priority = self.next_cost + manhattan_distance(self.goal, vec(next))
                    #* Put in the priority
                    self.frontier.put(next, self.priority)
                    #* Put in the path vector
                    self.path[next] = vec(self.current) - vec(next)
                    #print(self.path)
        #* Stop the Default Timer (Wall Timer)
        stop_time = timeit.default_timer()
        #* Stop the Process Timer (Wall Timer)
        stop_process_time = process_time()
        # PRINT ALL THE VISITED NODES
        print(f"\nThe CA* Search Path Available Nodes Movement are:\n{self.path}")
        print(f"\nThe  CA* Search Path have: {len(self.path)} Available Nodes")
        print(f"\nThe  CA* Search Path 'While Loop' Interactions was: {self.while_interactions}")
        print(f"\nThe  CA* Search Path 'For Loop' Interactions was: {self.for_interactions}")
        print("\nThe  CA* Search Path 'Wall time' was ", stop_time - start_time, 'sec')
        print("\nThe  CA* Search Path 'Process Time' was ",
              stop_process_time - start_process_time, 'sec\n')

        return self.path


def vec_to_int(vector):
    """
    A function that converts a PyGame vector to a integer.
    The Dictionary in python to not accept vector values, this function
    will convert the vector to a int value.
    Attributes:
        (vector) (vec2d)
    
    Args:
        
        (vector): An vector in the 2D format from the PyGame Lib
    
    Returns:
        
        (x,y) (int)
    """
    # RETURN THE VECTOR AS INTEGER
    return (int(vector.x), int(vector.y))


def vec_to_list(vector_list):
    """
    A function that converts a PyGame list of vectors to a list of tuples.

    Attributes:
        (vector_list) (list)
    
    Args:
        
        (vector): An list of vectors in the 2D format from the PyGame Lib
    
    Returns:
        
        (list): An list of tuples
    
    Example:

        vec_to_list([<Vector2(8, 0)>, <Vector2(8, 1)>, <Vector2(9, 1)>])
        >>> [(8, 0), (8, 1), (9, 1)]



    """
    # RETURN THE VECTOR AS AN LIST OF TUPLES
    return list(tuple(map(int, paths)) for paths in vector_list)


def manhattan_distance(node_one, node_two):
    """
    A function that calculate the Manhattan Distance used in the A* Heuristic
    Attributes:
        (node_one) (vec2d)
        (node_two) (vec2d)
    
    Args:
        
        (vector2d): An vector in the 2D format from the PyGame Lib
    
    Returns:
        
        The manhattan Distance between two nodes (int)
    """
    #* Multiply by ten since the Weighted Graph has this constant
    #? See the cost function at the Wheigted Grid World Class
    manhattan_distance = (abs(node_one.x - node_two.x) + abs(node_one.y - node_two.y))*10

    return manhattan_distance


def wait_in(node_value, previous_path):
    temporary_path = previous_path[:]
    try:
        get_collision_index = temporary_path.index(node_value)
        if get_collision_index == 0:
            previous_index = get_collision_index
        else:
            previous_index = int(get_collision_index - 1)
        previous_node = temporary_path[previous_index]
        temporary_path[previous_index:previous_index] = [previous_node]
    except:
        pass
    global new_path
    new_path = []
    for values in temporary_path:
        new_path.append(vec(values))
    return new_path


def find_path_collisions(robot_goal, robot_sprites, planning, time):

    if not ReservationTable:
        #*TODO: CREATE A NEW FUNCTION FOR THIS SINCE REPEATS FROM HERE
        global newWorld
        newWorld = createBigWorld.WeightedGrid()
        #* Use the Find the Free Spaces Available
        free_space = planning.find_free_space(newWorld, robot_goal)
        #* Init the Robot in a Random Position
        random_start = random.choice(free_space)
        #* Init the Robot in a Random Position
        random_start = random.choice(free_space)
        if vec_to_int(random_start) in SelectedStarts:
            change_start = vec_to_int(random_start)             
            while change_start in SelectedStarts:
                change_start = vec_to_int(random.choice(free_space))
            random_start = vec(change_start)
        SelectedStarts.add(vec_to_int(random_start))
        #* Run the Path Planning Algorithm
        path = planning.cooperative_astar_search(newWorld, robot_goal, random_start, time)
        DesiredPaths.append(path)
        #* Run the Paths and Add The Robots
        robot =  createBigWorld.MultiRobot(random_start, robot_goal, path)
        robot_sprites.add(robot)
        #*TODO: TO HERE
        #* Add the Path to the Reservation Table
        add_first_path = vec_to_list(path)
        ReservationTable.append(add_first_path)
        print(f'The Resevation Table is now: {ReservationTable}\n')
        print(ReservationTable)
    else:
        #*TODO: CREATE A NEW FUNCTION FOR THIS SINCE REPEATS FROM HERE
        #newWorld = createBigWorld.WeightedGrid()
        #* Use the Find the Free Spaces Available
        free_space = planning.find_free_space(newWorld, robot_goal)
        #* Init the Robot in a Random Position
        random_start = random.choice(free_space)
        if vec_to_int(random_start) in SelectedStarts:
            change_start = vec_to_int(random_start)             
            while change_start in SelectedStarts:
                change_start = vec_to_int(random.choice(free_space))
            random_start = vec(change_start)
        SelectedStarts.add(vec_to_int(random_start))
        #* Run the Path Planning Algorithm
        atual_temporary_path = planning.cooperative_astar_search(newWorld, robot_goal, random_start, time)
        #* Run the Paths and Add The Robots
        robot =  createBigWorld.MultiRobot(random_start, robot_goal, atual_temporary_path)
        #*TODO: TO HERE
        pretended_path = vec_to_list(atual_temporary_path)
        print(f'Pretended Path: {pretended_path}')
        for path_compared in ReservationTable:
            print('Path Compared: ', path_compared)
            equal_values = set(pretended_path).intersection(path_compared)
            print(f'Equal Nodes: {equal_values}')
            if equal_values:
                indexes_path_compared = [path_compared.index(x) for x in equal_values]
                indexes_new_path = [pretended_path.index(x) for x in equal_values]
                print(f'The indexes on first are: {indexes_path_compared}')
                print(f'The indexes on second are: {indexes_new_path}')
                if len(equal_values) == 1:
                    if (indexes_path_compared == indexes_new_path):
                        print('-------------------------------')
                        print('NEED REPLAN OR WAIT')
                        print('-------------------------------')
                        CbsConstraints.extend(list(equal_values))
                        print(f'CBS Constraints: {CbsConstraints}')
                        #*TODO: CREATE A NEW FUNCTION FOR THIS SINCE REPEATS FROM HERE
                        #newWorld = createBigWorld.WeightedGrid()
                        #* Use the Find the Free Spaces Available
                        free_space = planning.find_free_space(newWorld, robot_goal)
                        #* Init the Robot in a Random Position
                        random_start = random.choice(free_space)
                        #* Init the Robot in a Random Position
                        random_start = random.choice(free_space)
                        if vec_to_int(random_start) in SelectedStarts:
                            change_start = vec_to_int(random_start)             
                            while change_start in SelectedStarts:
                                change_start = vec_to_int(random.choice(free_space))
                            random_start = vec(change_start)
                        SelectedStarts.add(vec_to_int(random_start))
                        #* Run the Path Planning Algorithm
                        path = planning.cooperative_astar_search(newWorld, robot_goal, random_start, time)
                        #*TODO: TO HERE
                        #* Run the Paths and Add The Robots
                        DesiredPaths.append(path)
                        robot =  createBigWorld.MultiRobot(random_start, robot_goal, path)
                        #robot_sprites.add(robot)
                else:
                    if (indexes_path_compared == indexes_new_path) or (indexes_path_compared[0] == indexes_new_path[1]):
                        print('-------------------------------')
                        print('NEED REPLAN OR WAIT')
                        print('-------------------------------')
                        CbsConstraints.extend(list(equal_values))
                        print(f'CBS Constraints: {CbsConstraints}')
                        #*TODO: CREATE A NEW FUNCTION FOR THIS SINCE REPEATS FROM HERE
                        #newWorld = createBigWorld.WeightedGrid()
                        #* Use the Find the Free Spaces Available
                        free_space = planning.find_free_space(newWorld, robot_goal)
                        #* Init the Robot in a Random Position
                        random_start = random.choice(free_space)
                        #* Init the Robot in a Random Position
                        random_start = random.choice(free_space)
                        if vec_to_int(random_start) in SelectedStarts:
                            change_start = vec_to_int(random_start)             
                            while change_start in SelectedStarts:
                                change_start = vec_to_int(random.choice(free_space))
                            random_start = vec(change_start)
                        SelectedStarts.add(vec_to_int(random_start))
                        #* Run the Path Planning Algorithm
                        path = planning.cooperative_astar_search(newWorld, robot_goal, random_start, time)
                        DesiredPaths.append(path)
                        #* Run the Paths and Add The Robots
                        robot =  createBigWorld.MultiRobot(random_start, robot_goal, path)
                        #robot_sprites.add(robot)
                        #*TODO: TO HERE
                
            robot_sprites.add(robot)
        ReservationTable.append(pretended_path)
        print(len(ReservationTable))


def create_neighboorhood(pos, swarm_key):
    """
    A function responsible to update the Swarm Hash Table.
    
    The Function will calculate the neighboors cells around the robot
    and update the Swarm Hash table.

        (pos)           (vec2d)
        (swarm_key)     (str)
    
    Args:
        
        (start)          : The Robot Random Start Position in vec2d format
        (swarm_key)      : The Swarm Neighboorhood
    
    Returns:
        
        The Swarms Neighboorhoods in the Grid
    """
    #* Create the Temporary Cells Swarm
    #? This will prevent long term collisions
    cell_1_x, cell_1_y = pos.x+1, pos.y
    cell_2_x, cell_2_y = pos.x+2, pos.y
    cell_3_x, cell_3_y = pos.x-1, pos.y
    cell_4_x, cell_4_y = pos.x-2, pos.y
    cell_5_x, cell_5_y = pos.x, pos.y+1
    cell_6_x, cell_6_y = pos.x, pos.y+2
    cell_7_x, cell_7_y = pos.x, pos.y-1
    cell_8_x, cell_8_y = pos.x, pos.y-2
    #? This will prevent short term collisions
    cell_9_x, cell_9_y = pos.x-1, pos.y-1
    cell_10_x, cell_10_y = pos.x-1, pos.y+1
    cell_11_x, cell_11_y = pos.x+1, pos.y-1
    cell_12_x, cell_12_y = pos.x+1, pos.y+1
    #* Updates de Swarm Dictionary
    first_swarm = [ vec(cell_1_x,cell_1_y), vec(cell_2_x,cell_2_y),
                    vec(cell_3_x, cell_3_y), vec(cell_4_x, cell_4_y),
                    vec(cell_5_x, cell_5_y), vec(cell_6_x, cell_6_y),
                    vec(cell_7_x, cell_7_y), vec(cell_8_x, cell_8_y),
                    vec(cell_9_x, cell_9_y), vec(cell_10_x, cell_10_y),
                    vec(cell_11_x, cell_11_y), vec(cell_12_x, cell_12_y),
                   ]
    #* Update the Dictionary
    swarmAreas.update({swarm_key:first_swarm})

    return swarmAreas 


def create_swarms_area(robot_goal, robot_sprites, planning, time):
    """
    A function responsible to create new Swarms. If the start position
    is inside a known neighboorhood the function will look the swam he
    will need to enter and update the table. If not, will create a new
    swarm.

    Returns:
        
        The Swarm Hash Table
    """

    global newWorld
    newWorld = createBigWorld.WeightedGrid()
    #* Use the Find the Free Spaces Available
    free_space = planning.find_free_space(newWorld, robot_goal)
    #* Init the Robot in a Random Position
    random_start = random.choice(free_space)
    #* Init the Robot in a Random Position
    random_start = random.choice(free_space)
    if vec_to_int(random_start) in SelectedStarts:
        change_start = vec_to_int(random_start)             
        while change_start in SelectedStarts:
            change_start = vec_to_int(random.choice(free_space))
        random_start = vec(change_start)
    # SWARM CREATION
    global swarm_count
    swarm_count += 1
    swarm_value = ['Swarm', str(swarm_count)]
    swarm_key = "".join(swarm_value)
    #* If the swarm was not create before, create a new one
    if not swarmAreas:
        pygame.draw.rect(createBigWorld.screen, paint.COLOR_GREEN,
                        [(random_start.x-2)*100, (random_start.y-2)*100, 500, 500], 1)
        create_neighboorhood(random_start, swarm_key)
        #*TODO: CREATE A NEW FUNCTION FOR THIS SINCE REPEATS FROM HERE
        #newWorld = createBigWorld.WeightedGrid()
        #* Use the Find the Free Spaces Available
        free_space = planning.find_free_space(newWorld, robot_goal)
        #* Init the Robot in a Random Position
        random_start = random.choice(free_space)
        #* Run the Path Planning Algorithm
        path = planning.astar_search(newWorld, robot_goal, random_start)
        #* Run the Paths and Add The Robots
        robot =  createBigWorld.MultiRobot(random_start, robot_goal, path)
        robot_sprites.add(robot)
    else:
        #* If the swarm was created before, check if the new swam area
        #* is inside the previous swarm
        if any(random_start in values for values in swarmAreas.values()):
            #* Check in with Swarm the Position was Found
            keys = [key for key, value in swarmAreas.items() if random_start in value]
            print('\nFound a new Robot neighboors in inside a known Swarm, '+
                    'including the {} into {}...\n'.format(swarm_key, keys))
            swarm_key = str(keys[0])
            create_neighboorhood(random_start, swarm_key)
            #*TODO: CREATE A NEW FUNCTION FOR THIS SINCE REPEATS FROM HERE
            #newWorld = createBigWorld.WeightedGrid(world_nodes_constraints=CbsConstraints)
            #* Use the Find the Free Spaces Available
            #free_space = planning.find_free_space(newWorld, robot_goal)
            #* Init the Robot in a Random Position
            #random_start = random.choice(free_space)
            #* Run the Path Planning Algorithm
            #path = planning.astar_search(newWorld, robot_goal, random_start)
            #* Run the Paths and Add The Robots
            #robot =  createBigWorld.MultiRobot(random_start, robot_goal, path)
            find_path_collisions(robot_goal, robot_sprites, planning, time)
            #robot_sprites.add(robot)
            #*TODO: TO HERE
            
        else:
            #* Create a new Swarm Neighboor
            create_neighboorhood(random_start, swarm_key)
            #*TODO: CREATE A NEW FUNCTION FOR THIS SINCE REPEATS FROM HERE
            #newWorld = createBigWorld.WeightedGrid()
            #* Use the Find the Free Spaces Available
            #free_space = planning.find_free_space(newWorld, robot_goal)
            #* Init the Robot in a Random Position
            #random_start = random.choice(free_space)
            #* Run the Path Planning Algorithm
            path = planning.astar_search(newWorld, robot_goal, random_start)
            #* Run the Paths and Add The Robots
            robot =  createBigWorld.MultiRobot(random_start, robot_goal, path)
            robot_sprites.add(robot)
            #*TODO: TO HERE
        #robot_sprites.add(robot)

def run_swarm(robots_qtd, goals, time, newWorld, planning, all_robots):
    """
    A function responsible to init the path search with swarms or not.

        (robots_qtd)  (vec2d)
        (goals)       (deque)
        (time)        (tuple)
        (newWorld)    (class)
        (planning)    (class)
        (all_robots)  (sprite)
    
    Args:
        
        (robots_qtd)     : The Number of Robots Desired in the World
        (goals)          : The Desired Goals in Deque Format
        (time)           : The maximum time limit to find a solution
        (newWorld)       : The Class responsible to create the World
        (planning)       : The Class responsible to run the Path Planning
        (all_robots)     : The Robots in Sprite format

    Returns:
        
        The Swarm Creation and Path Planning Execution
    """
    #* Init with zero Robots
    robot_count = 0
    #* Create The Robots and Paths based on the Number Required
    for _ in range(robots_qtd):
        robot_count += 1
        #* Pop up the Goal
        goal_poped = vec(goals.popleft())

        create_swarms_area(goal_poped, all_robots, planning, time)

    print(swarmAreas)

def run_multiagent_astar(goal=world.GOAL,time=world.TIME_LIMIT,
                         robots_qtd=world.ROBOTS_QTD):
    """
    The Multi-Agent A-Star Search function of the Path Planning Library, is
    responsible to run the Weighted World and calculate the shortest path in a
    PyGame Screen with the A-Star Algorithm for Multiple Robots
        
    Attributes:
        (goal)       (tuple)
        (robots_qrd) (int)
    
    Args:
        (goal)      : Is the position in the Weighted Grid that
                      we want to achieve
        (robots_qtd): Is the number of robots with want to run the simulation

    Returns:
       The Shortest Path from the A* Algorithm for multiple robots loaded in a
       PyGame Screen with World Inputed by the program or user
    """
    # CALL THE CLASS WORLD GRID
    #* ADJUST HERE THE WORLD YOU WANT
    #? IF YOU NEED TO TEST THE WORLD FIRST USE THE "createGrids.py" module
    goals = goal
    print(f'\nThe Number of Robots Choosen was: {robots_qtd}\n')
    newWorld = createBigWorld.WeightedGrid()
    planning = PathPlanning()
    #* Start the Treadmill Class in a Group
    all_treadmill_items = pygame.sprite.Group()
    #* Init the treadmill Sprite
    treadmill_items = createBigWorld.TreadmillItems()
    all_treadmill_items.add(treadmill_items)
    #* Add the Robots Sprite Classes to the Groups
    all_robots = pygame.sprite.Group()
    # SET THE MULTIPLE PATHS
    global pathsGlobal, robotsStartPos
    pathsGlobal = deque([])
    robotsStartPos = deque([])
    robot_count = 0
    for _ in range(robots_qtd):
        robot_count += 1
        #* Pop up the Goal
        goal_poped = vec(goals.popleft())
        #* Use the Find the Free Spaces Available
        free_space = planning.find_free_space(newWorld, goal_poped)
        #* Init the Robot in a Random Position
        random_start = random.choice(free_space)
        if vec_to_int(random_start) in SelectedStarts:
            change_start = vec_to_int(random_start)             
            while change_start in SelectedStarts:
                change_start = vec_to_int(random.choice(free_space))
            random_start = vec(change_start)
        SelectedStarts.add(vec_to_int(random_start))
        #* Run the Path Planning Algorithm
        path = planning.astar_search(newWorld, goal_poped, random_start)
        #* Run the Paths and Add The Robots
        robots =  createBigWorld.MultiRobot(random_start, goal_poped, path)
        all_robots.add(robots)
    # CREATE A LOOP AND RUN THE WORLD IN A SCREEN CONTINUALLY
    # * If still running do the Loop
    running = True
    global FLAG
    FLAG = False
    while running:
        # ADJUST THE CLOCK
        createBigWorld.clock.tick(default.FPS)
        # IF THE PYGAME RECEIVES AN EVENT
        for event in pygame.event.get():
            # IF THE EVENT IS TO QUIT THE APPLICATION
            if event.type == pygame.QUIT:
                #* Break the LOOP
                running == False
                #* Shutdown the PyGame
                pygame.quit()
                #* Closes the program and doesn't crete any dialogue
                sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    #* Break the LOOP
                    running == False
                    #* Shutdown the PyGame
                    pygame.quit()
                    #* Closes the program and doesn't crete any dialogue
                    sys.exit(0)
                if event.key == pygame.K_s:
                    # Dump the wall list for saving (if needed)
                    #* Use the command to show the actual obstacles values if modified
                    print('The obstacle tuples drawn is:\n',
                            [(int(loc.x), int(loc.y)) for loc in createBigWorld.obstaclesPositionGlobal])
                if event.key == pygame.K_SPACE:
                    #* Run the robot movement simulation
                    pygame.event.clear()
                    with concurrent.futures.ProcessPoolExecutor() as executor:
                       executor.map(all_robots.update(all_robots, all_treadmill_items, newWorld))
            # CHECKS IF THERE'S A MOUSE BUTTON EVENT IN THE SCREEN
            if event.type == pygame.MOUSEBUTTONDOWN:
                # PICK THE GRID LOCATION WHERE THE MOUSE WAS PUSHED AND STORE
                mouse_pos = vec(pygame.mouse.get_pos())//newWorld.cell_size_width
                # IF THE BUTTON WAS PRESSED
                if event.button == 1:
                    # IF THE MOUSE POSITION IS IN THE OBSTACLES TUPLE
                    if mouse_pos in newWorld.obstaclesPosition:
                        # REMOVE THE OBSTACLE
                        newWorld.obstaclesPosition.remove(mouse_pos)
                    else:
                        # ADD A OBSTACLE
                        newWorld.obstaclesPosition.append(mouse_pos)
                # FOR EVERY NEW CLICK  OR OBSTACLE ADD, WE RECALCULATE THE PATH
                #* RIGHT MOUSE TO CHANGE THE GOAL
                if event.button == 3:
                    goal_node = mouse_pos
                #path = planning.astar_search(newWorld, goal_node, start_node) 
        #pygame.event.pump()
        # DRAW THE SCREEN CAPTION DISPLAY WITH FPS
        pygame.display.set_caption("World Grid Representation [{:.2f}]".format(createBigWorld.clock.get_fps()))
        # FILLS THE SCREEN WITH A BLANK DISPLAY
        createBigWorld.screen.fill(paint.COLOR_WHITE)
        # UPDATE THE DISPLAY
        #* Draw the grid
        newWorld.draw_grid()
        #* Draw the obstacles
        newWorld.draw_obstacles()
        #* Draw the Treadmill Zone
        newWorld.draw_treadmill_zone()
        #* Draw the Workers Zone
        newWorld.draw_workers_zone()
        #* Draw the Delivery Zone
        newWorld.draw_delivery_zone()
        #* Draw the Recharge Zone
        newWorld.draw_recharge_zone()
        #* Draw the Pickup Zone
        newWorld.draw_pickup_zone()
        #* Draw the Arrows
        newWorld.draw_arrows()
        #* Draw Don't Move Zone
        newWorld.draw_dont_move()
        #* Update the Spriters
        all_treadmill_items.update()
        #* Draw the Spriters in the Grid
        all_robots.draw(createBigWorld.screen)
        all_treadmill_items.draw(createBigWorld.screen)
        #*Update the full display Surface to the screen
        #create_swarms_area()
        pygame.display.flip()

def run_cbs(goal=world.GOAL,time=world.TIME_LIMIT,
            robots_qtd=world.ROBOTS_QTD):
    """
    The Cooperative A-Star (STA*) Search function of the Path Planning Library, is
    responsible to run the Weighted World and calculate the shortest path in a
    PyGame Screen with Discretized Time for Multiple Robots
        
    Attributes:
        (goal)       (tuple)
        (time)       (tuple)
        (robots_qrd) (int)
    
    Args:
        (goal)      : Is the position in the Weighted Grid that
                      we want to achieve
        (time)      : Is the discretized time limit that we will run the algorithm
        (robots_qtd): Is the number of robots with want to run the simulation
    
    Vars:
        limit_time (int) = The Maximun Time to Run the Algorithm
    Returns:
       The Shortest Path from the CA* Algorithm for multiple robots loaded in a
       PyGame Screen with World Inputed by the program or user
    """
    # CALL THE CLASS WORLD GRID
    #* ADJUST HERE THE WORLD YOU WANT
    #? IF YOU NEED TO TEST THE WORLD FIRST USE THE "createGrids.py" module
    time_limit = time
    time = (0, time_limit)
    goals = goal
    print(f'\nThe Time Limit to Run the  Function is: {time_limit}\n')
    print(f'\nThe Number of Robots Choosen was: {robots_qtd}\n')
    newWorld = createBigWorld.WeightedGrid(world_nodes_constraints=CbsConstraints)
    planning = PathPlanning()
    #* Start the Treadmill Class in a Group
    all_treadmill_items = pygame.sprite.Group()
    #* Init the treadmill Sprite
    treadmill_items = createBigWorld.TreadmillItems()
    all_treadmill_items.add(treadmill_items)
    #* Add the Robots Sprite Classes to the Groups
    all_robots = pygame.sprite.Group()
    # SET THE MULTIPLE PATHS
    global pathsGlobal, robotsStartPos, boidsStartsPos
    pathsGlobal = deque([])
    robotsStartPos = deque([])
    boidsStartsPos = []
    robot_count = 0
    start_time = timer()
    for _ in range(robots_qtd):
        robot_count += 1
        #* Pop up the Goal
        goal_poped = vec(goals.popleft())
        #* Use the Find the Free Spaces Available
        #free_space = planning.find_free_space(newWorld, goal_poped)
        #* Init the Robot in a Random Position
        #random_start = random.choice(free_space)
        #* Run the Path Planning Algorithm
        #path = planning.cooperative_astar_search(newWorld, goal_poped, random_start, time)
        #* Run the Paths and Add The Robots
        #temporary_robot =  createWorld.MultiRobot(random_start, goal_poped, path)
        #* Discretized the Path in Time Steps
        #temporary_path = createWorld.temporaryPath.popleft()
        find_path_collisions(goal_poped, all_robots, planning, time)
        #all_robots.add(new_robot)
    end_time = timer()
    print('----------------- SIMULATION FINISHED ----------------\n')
    print(f'The CBS takes {end_time-start_time} seconds to run')
    all_paths = createBigWorld.AllPaths
    print(f'The CBS Runned {len(all_paths)} Paths')
    makespan = max(map(len, all_paths))
    print(f'The Makespan is:  {makespan}')
    sum_of_costs = sum(map(len, all_paths))
    print(f'The Sum of Costs is:  {sum_of_costs}')

    # CREATE A LOOP AND RUN THE WORLD IN A SCREEN CONTINUALLY
    # * If still running do the Loop
    running = True
    while running:
        # ADJUST THE CLOCK
        createBigWorld.clock.tick(default.FPS)
        # IF THE PYGAME RECEIVES AN EVENT
        for event in pygame.event.get():
            # IF THE EVENT IS TO QUIT THE APPLICATION
            if event.type == pygame.QUIT:
                #* Break the LOOP
                running == False
                #* Shutdown the PyGame
                pygame.quit()
                #* Closes the program and doesn't crete any dialogue
                sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    #* Break the LOOP
                    running == False
                    #* Shutdown the PyGame
                    pygame.quit()
                    #* Closes the program and doesn't crete any dialogue
                    sys.exit(0)
                if event.key == pygame.K_s:
                    # Dump the wall list for saving (if needed)
                    #* Use the command to show the actual obstacles values if modified
                    print('The obstacle tuples drawn is:\n',
                            [(int(loc.x), int(loc.y)) for loc in newWorld.obstaclesPosition])
                if event.key == pygame.K_SPACE:
                    #* Run the robot movement simulation
                    pygame.event.clear()
                    with concurrent.futures.ProcessPoolExecutor() as executor:
                        executor.map(all_robots.update(all_robots, all_treadmill_items, newWorld))
                    #all_robots.update(all_robots, all_treadmill_items, newWorld)
            # CHECKS IF THERE'S A MOUSE BUTTON EVENT IN THE SCREEN
            if event.type == pygame.MOUSEBUTTONDOWN:
                # PICK THE GRID LOCATION WHERE THE MOUSE WAS PUSHED AND STORE
                mouse_pos = vec(pygame.mouse.get_pos())//newWorld.cell_size_width
                # IF THE BUTTON WAS PRESSED
                if event.button == 1:
                    # IF THE MOUSE POSITION IS IN THE OBSTACLES TUPLE
                    if mouse_pos in newWorld.obstaclesPosition:
                        # REMOVE THE OBSTACLE
                        newWorld.obstaclesPosition.remove(mouse_pos)
                    else:
                        # ADD A OBSTACLE
                        newWorld.obstaclesPosition.append(mouse_pos)
                # FOR EVERY NEW CLICK  OR OBSTACLE ADD, WE RECALCULATE THE PATH
                #* RIGHT MOUSE TO CHANGE THE GOAL
                if event.button == 3:
                    goal_node = mouse_pos
        #pygame.event.pump()
        # DRAW THE SCREEN CAPTION DISPLAY WITH FPS
        pygame.display.set_caption("World Grid Representation [{:.2f}]".format(createBigWorld.clock.get_fps()))
        # FILLS THE SCREEN WITH A BLANK DISPLAY
        createBigWorld.screen.fill(paint.COLOR_WHITE)
        # UPDATE THE DISPLAY
        #* Draw the grid
        newWorld.draw_grid()
        #* Draw the obstacles
        newWorld.draw_obstacles()
        #* Draw the Treadmill Zone
        newWorld.draw_treadmill_zone()
        #* Draw the Workers Zone
        newWorld.draw_workers_zone()
        #* Draw the Delivery Zone
        newWorld.draw_delivery_zone()
        #* Draw the Recharge Zone
        newWorld.draw_recharge_zone()
        #* Draw the Pickup Zone
        newWorld.draw_pickup_zone()
        #* Draw the Arrows
        newWorld.draw_arrows()
        #* Draw Don't Move Zone
        newWorld.draw_dont_move()
        #* Update the Spriters
        all_treadmill_items.update()
        #* Draw the Spriters in the Grid
        all_robots.draw(createBigWorld.screen)
        all_treadmill_items.draw(createBigWorld.screen)
        #*Update the full display Surface to the screen
        #create_swarms_area()
        pygame.display.flip()


def run_mapf_swarm(goal=world.GOAL,time=world.TIME_LIMIT,
                   robots_qtd=world.ROBOTS_QTD):
    """
    The MAPF-S (Multi-Agent Pathfinding Swarm) framework, with is
    responsible to run the Weighted World with Swarms and calculate
    the shortest path in a PyGame Screen with Discretized Time for
    Multiple Robots
        
    Attributes:
        (goal)       (tuple)
        (time)       (tuple)
        (robots_qrd) (int)
    
    Args:
        (goal)      : Is the position in the Weighted Grid that
                      we want to achieve
        (time)      : Is the discretized time limit that we will run the algorithm
        (robots_qtd): Is the number of robots with want to run the simulation
    
    Vars:
        limit_time (int) = The Maximun Time to Run the Algorithm
    Returns:
       The Shortest Path from the CA* Algorithm for multiple robots loaded in a
       PyGame Screen with World Inputed by the program or user
    """
    # CALL THE CLASS WORLD GRID
    #* ADJUST HERE THE WORLD YOU WANT
    #? IF YOU NEED TO TEST THE WORLD FIRST USE THE "createGrids.py" module
    time_limit = time
    time = (0, time_limit)
    goals = goal
    print(f'\nThe Time Limit to Run the  Function is: {time_limit}\n')
    print(f'\nThe Number of Robots Choosen was: {robots_qtd}\n')
    newWorld = createBigWorld.WeightedGrid()
    planning = PathPlanning()
    #* Start the Treadmill Class in a Group
    all_treadmill_items = pygame.sprite.Group()
    #* Init the treadmill Sprite
    treadmill_items = createBigWorld.TreadmillItems()
    all_treadmill_items.add(treadmill_items)
    #* Add the Robots Sprite Classes to the Groups
    all_robots = pygame.sprite.Group()
    # SET THE MULTIPLE PATHS
    global pathsGlobal
    pathsGlobal = deque([])
    start_time = timer()
    run_swarm(robots_qtd, goals, time, newWorld, planning, all_robots)
    end_time = timer()
    print('----------------- SIMULATION FINISHED ----------------\n')
    print(f'The Swarm takes {end_time-start_time} seconds to run')
    all_paths = createBigWorld.AllPaths
    print(f'The Swarm Runned {len(all_paths)} Paths')
    makespan = max(map(len, all_paths))
    print(f'The Makespan is:  {makespan}')
    sum_of_costs = sum(map(len, all_paths))
    print(f'The Sum of Costs is:  {sum_of_costs}')
    # CREATE A LOOP AND RUN THE WORLD IN A SCREEN CONTINUALLY
    # * If still running do the Loop
    running = True
    while running:
        # ADJUST THE CLOCK
        createBigWorld.clock.tick(default.FPS)
        # IF THE PYGAME RECEIVES AN EVENT
        for event in pygame.event.get():
            # IF THE EVENT IS TO QUIT THE APPLICATION
            if event.type == pygame.QUIT:
                #* Break the LOOP
                running == False
                #* Shutdown the PyGame
                pygame.quit()
                #* Closes the program and doesn't crete any dialogue
                sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    #* Break the LOOP
                    running == False
                    #* Shutdown the PyGame
                    pygame.quit()
                    #* Closes the program and doesn't crete any dialogue
                    sys.exit(0)
                if event.key == pygame.K_s:
                    # Dump the wall list for saving (if needed)
                    #* Use the command to show the actual obstacles values if modified
                    print('The obstacle tuples drawn is:\n',
                            [(int(loc.x), int(loc.y)) for loc in createBigWorld.obstaclesPositionGlobal])
                if event.key == pygame.K_SPACE:
                    #* Run the robot movement simulation
                    pygame.event.clear()
                    with concurrent.futures.ProcessPoolExecutor(max_workers=2) as executor:
                        executor.map(all_robots.update(all_robots, all_treadmill_items, newWorld))
                        #time.sleep(1)
            # CHECKS IF THERE'S A MOUSE BUTTON EVENT IN THE SCREEN
            if event.type == pygame.MOUSEBUTTONDOWN:
                # PICK THE GRID LOCATION WHERE THE MOUSE WAS PUSHED AND STORE
                mouse_pos = vec(pygame.mouse.get_pos())//newWorld.cell_size_width
                # IF THE BUTTON WAS PRESSED
                if event.button == 1:
                    # IF THE MOUSE POSITION IS IN THE OBSTACLES TUPLE
                    if mouse_pos in newWorld.obstaclesPosition:
                        # REMOVE THE OBSTACLE
                        newWorld.obstaclesPosition.remove(mouse_pos)
                    else:
                        # ADD A OBSTACLE
                        newWorld.obstaclesPosition.append(mouse_pos)
                # FOR EVERY NEW CLICK  OR OBSTACLE ADD, WE RECALCULATE THE PATH
                #* RIGHT MOUSE TO CHANGE THE GOAL
                if event.button == 3:
                    goal_node = mouse_pos
                #path = planning.astar_search(newWorld, goal_node, start_node) 
        #pygame.event.pump()
        # DRAW THE SCREEN CAPTION DISPLAY WITH FPS
        pygame.display.set_caption("World Grid Representation [{:.2f}]".format(createBigWorld.clock.get_fps()))
        # FILLS THE SCREEN WITH A BLANK DISPLAY
        createBigWorld.screen.fill(paint.COLOR_WHITE)
        # UPDATE THE DISPLAY
        #* Draw the grid
        newWorld.draw_grid()
        #* Draw the obstacles
        newWorld.draw_obstacles()
        #* Draw the Treadmill Zone
        newWorld.draw_treadmill_zone()
        #* Draw the Workers Zone
        newWorld.draw_workers_zone()
        #* Draw the Delivery Zone
        newWorld.draw_delivery_zone()
        #* Draw the Recharge Zone
        newWorld.draw_recharge_zone()
        #* Draw the Pickup Zone
        newWorld.draw_pickup_zone()
        #* Draw the Arrows
        newWorld.draw_arrows()
        #* Draw Don't Move Zone
        newWorld.draw_dont_move()
        #* Update the Spriters
        all_treadmill_items.update()
        #* Draw the Spriters in the Grid
        all_robots.draw(createBigWorld.screen)
        all_treadmill_items.draw(createBigWorld.screen)
        #*Update the full display Surface to the screen
        #create_swarms_area()
        pygame.display.flip()


def main():
    """
    The main function, responsible to deal with the Args provided by
    the user at the terminal
    Returns:
        The simulation required by the user
    """
    # LIST OF COMMAND LINE ARGUMENTS
    parser = argparse.ArgumentParser(description="PathPlanningPy Library")
    parser.add_argument_group(title='Run Options')

    parser.add_argument('--mastar', action='store_true',
                        help='Runs the Multi-Agent A* Search')

    parser.add_argument('--cbs', action='store_true',
                        help='Runs the Cooperative A* Search')

    parser.add_argument('--mapfs', action='store_true',
                        help='Runs the MAPF-S Framework')

    parser.add_argument('--version', action='version',
                        version='Path Planning with Python = v1.0')

    args = parser.parse_args()

    if args.mastar:
        run_multiagent_astar()
    if args.cbs:
        run_cbs()
    if args.mapfs:
        run_mapf_swarm()

if __name__ == '__main__':
    main()
