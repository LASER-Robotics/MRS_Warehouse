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

import os
import sys
import argparse
import pygame
import random
import time
import timeit
import schedule
import concurrent.futures
import multiprocessing
#? Using the collections module since is the most efficient
#? to implement and manipulate a queue list
from collections import deque
#* To see the sum of the system and user CPU Time of
#* of the current process
from time import process_time
#* To see the memory usage in each line
import memory_profiler
#* Import the createWorld Module
from world import createWorld, settings


# LOAD THE WORLD VARIABLES
#* Horizontal Layout
world = createWorld.create
# LOAD THE PYGAME DEFAULTS VARIABLES
default = settings.PygameDefaults()
# LOAD THE COLORS FROM THE SETTINGS
paint = settings.Colors()
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
            print(f'Current: {self.current}')
            print(graph.find_neighbors(vec(self.current)))
            # THE NEIGHBOORS OF THE CURRENT TILE
            for next in  graph.find_neighbors(self.current):
                # IF THE NEXT NODE IS NOT VISITED
                if next not in self.visited:
                    # ADD THE NODE TO THE FRONTIER LIST
                    self.frontier.append(next)
                    # PUT ON THE VISITED NODES
                    self.visited.append(next)
        # PRINT ALL THE VISITED NODES
        global visitedNodes
        visitedNodes = len(self.visited)
        print(f'\nThe Free Node Cells Available are:\n{self.visited}')
        print(f'\nFree Node Cells availabe: {visitedNodes} Grid Cells')
        return self.visited

    def breath_first_search(self, graph, start = world.START,
                            goal = world.GOAL):
        """
        Reads free nodes in World Grid using the find_neighbors(node) function,
        and returns the Breath First Search for the Node inputed in the header.
        Attributes:
            (graph) (Class)
            (start) (tuple)
            (goal)  (tuple)
        
        Args:
            (graph): A World Grid
            (start): Is the position in the Weighted Grid that
                     we start the Path Plannign Algorithm
            (goal) : Is the position in the Weighted Grid that
                     we want to achieve
        
        Returns:
            The Shortest Path Using Breath First Search for the START
            and GOAL nodes provided
        """
    
        print('\n__________________ BREATH FIRST SEARCH STARTED __________________\n')
        #* START THE TIMER
        start_time = timeit.default_timer()
        start_process_time = process_time()
        # SET THE START AND GOAL VALUES
        self.start = start
        print(f'The Start Node is located at: {self.start}')
        self.goal = goal
        print(f'The Goal Node is located at: {self.goal}')
        # IMPORT THE DEQUE TO PUT THE NODES
        self.frontier = deque()
        # APPEND THE FRONTIER WITH THE POSITION
        self.frontier.append(self.start)
        # IN THIS CASE WE ARE ONLY INTERESTED IF WE ARE MOVING TO THE CELL
        #* OK, I visited the node, but I Will move to it?
        #? Create a Path Dictionary were the Key will be the cell and the value
        #? is the Cell that we came from
        self.path = {}
        # THE START IS NONE SINCE IS WERE WE ARE
        #? Converts the start vector to integers
        self.path[vec_to_int(self.start)] = None
        # START OUR LOOP
        #* As long there's nodes on the frontier do
        #? Init the While Interactions Variable
        self.while_interactions = 0
        while len(self.frontier) > 0:
            #? Add 1 interaction for every loop
            self.while_interactions += 1
            # THE CURRENT NODE WE WANT TO LOOK IS THE NEXT NODE
            #* Pop's the next on the queue list
            self.current = self.frontier.popleft()
            print(f'Current: {self.current}')
            if self.current == self.goal:
                break
            # THE NEIGHBOORS OF THE CURRENT TILE
            #? Init the For Interactions Variable
            self.for_interactions = 0
            for next in graph.find_neighbors(self.current):
                #? Add 1 interaction for every loop
                self.for_interactions += 1
                # IF THE NEXT NODE IS IN THE PATH DIC
                if vec_to_int(next) not in self.path:
                    # ADD THE NODE TO THE FRONTIER LIST
                    self.frontier.append(next)
                    # CREATE A DIRECTION VECTOR POINTING WERE TO GO
                    #? The current is a vector, if we subtract to the
                    #? next we will have a direction vector pointing
                    #? to the direction we wanna go
                    self.path[vec_to_int(next)] = self.current - next
        #* Stop the Default Timer (Wall Timer)
        stop_time = timeit.default_timer()
        #* Stop the Process Timer (Wall Timer)
        stop_process_time = process_time()
        # PRINT ALL THE VISITED NODES
        print(f'\nThe Breadth First Search Path Available Nodes Movement are:\n{self.path}')
        print(f'\nThe Breadth First Search Path have: {len(self.path)} Available Nodes')
        print(f'\nThe Breadth First Search Path "While Loop" Interactions was: {self.while_interactions}')
        print(f'\nThe Breadth First Search Path "For Loop" Interactions was: {self.for_interactions}')
        print('\nThe Breadth First Search Path "Wall time" was ', stop_time - start_time, 'sec')
        print('\nThe Breadth First Search Path "Process Time" was ',
              stop_process_time - start_process_time, 'sec\n')
        return self.path

    def dijkstras_search(self, graph, start = world.START,
                         goal = world.GOAL):
        """
        Reads free nodes in World Grid using the find_neighbors(node) function,
        and returns the the shortest path using Dijkstra Search for the Weighted
        Nodes.
        Attributes:
            (graph) (Class)
            (start) (tuple)
            (goal)  (tuple)
        
        Args:
            (graph): A Weighted Grid
            (start): Is the position in the Weighted Grid that
                     we start the Path Plannign Algorithm
            (goal) : Is the position in the Weighted Grid that
                     we want to achieve
        
        Returns:
            The Shortest Path Using Dijkstra Search for the START
            and GOAL nodes provided
        """
    
        print("\n__________________ DIJKSTRA'S SEARCH STARTED __________________\n")    
        #* START THE TIMER
        start_time = timeit.default_timer()
        start_process_time = process_time()
        # SET THE START AND GOAL VALUES
        self.start = start
        print(f'The Start Node is located at: {self.start}')
        self.goal = goal
        print(f'The Goal Node is located at: {self.goal}')
        # IMPORT THE QUEUE TO PUT THE NODES
        self.frontier = createWorld.PriorityQueue()
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
                    self.priority = self.next_cost
                    #* Put in the priority
                    self.frontier.put(next, self.priority)
                    #* Put in the path vector
                    self.path[next] = vec(self.current) - vec(next)
        #* Stop the Default Timer (Wall Timer)
        stop_time = timeit.default_timer()
        #* Stop the Process Timer (Wall Timer)
        stop_process_time = process_time()
        # PRINT ALL THE VISITED NODES
        print(f"\nThe Dijkstra's Search Path Available Nodes Movement are:\n{self.path}")
        print(f"\nThe  Dijkstra's Search Path have: {len(self.path)} Available Nodes")
        print(f"\nThe  Dijkstra's Search Path 'While Loop' Interactions was: {self.while_interactions}")
        print(f"\nThe  Dijkstra's Search Path 'For Loop' Interactions was: {self.for_interactions}")
        print("\nThe  Dijkstra's Search Path 'Wall time' was ", stop_time - start_time, 'sec')
        print("\nThe  Dijkstra's Search Path 'Process Time' was ",
              stop_process_time - start_process_time, 'sec\n')
        
        return self.path

    def astar_search(self, graph, start = world.START,
                     goal = world.GOAL):
        """
        Reads free nodes in a Wheighted Grid using the find_neighbors(node) function,
        and returns the shortest path using A-Star (A*) Search for the Weighted Nodes.
        Attributes:
            (graph) (Class)
            (start) (tuple)
            (goal)  (tuple)
        
        Args:
            (graph): A Weighted Grid
            (start): Is the position in the Weighted Grid that
                     we start the Path Plannign Algorithm
            (goal) : Is the position in the Weighted Grid that
                     we want to achieve
        
        Returns:
            The Shortest Path Using A-Star (A*) Search for the START
            and GOAL nodes provided
        """
    
        print("\n__________________ A-Star (A*) SEARCH STARTED __________________\n")    
        #* START THE TIMER
        start_time = timeit.default_timer()
        start_process_time = process_time()
        # SET THE START AND GOAL VALUES
        self.start = start
        print(f'The Goal Node is located at: {self.start}')
        self.goal = goal
        print(f'The Start Node is located at: {self.goal}')
        # IMPORT THE QUEUE TO PUT THE NODES
        self.frontier = createWorld.PriorityQueue()
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

    def space_astar_search(self, graph, start=world.START,
                           goal = world.GOAL,
                           time = world.TIME_LIMIT):
        """
        Reads free nodes in a Wheighted Grid using the find_neighbors(node) function,
        and returns the shortest path using Space Time A-Star (A*) Search for the
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
            The Shortest Path Using A-Star (A*) Search for the START
            and GOAL nodes provided
        """
    
        print("\n__________________ SPACE TIME A* (STA*) SEARCH STARTED __________________\n")    
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
        self.frontier = createWorld.PriorityQueue()
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

def run_breadth_search(start=world.START, goal=world.GOAL):
    """
    The Breadth Search function of the Path Planning Library, responsible
    to run the Grid World and calculate the shortest path in a PyGame Screen
        
    Attributes:
        (start) (tuple)
        (goal) (tuple)
    
    Args:
        (start): Is the position in the Weighted Grid that
                 we start the Path Plannign Algorithm
        (goal) : Is the position in the Weighted Grid that
                 we want to achieve
    Returns:
       The Shortest Path from the Breadth Search Algorithm loaded in a
       PyGame Screen with World Inputed Variables by the program or user
    """
    # CALL THE CLASS WORLD GRID
    #* ADJUST HERE THE WORLD YOU WANT
    #? IF YOU NEED TO TEST THE WORLD FIRST USE THE "createGrids.py" module
    global start_node, goal_node
    start_node = vec(start)
    print(start_node)
    goal_node = vec(goal.popleft())
    print(goal_node)
    newWorld = createWorld.WorldGrid()
    planning = PathPlanning()
    # PUT THE FUNCTIONS THAT YOU WANT BELLOW
    #* THE POSTIION WE WANT
    #* Using the Find Free Space to find the free space availabe in the
    free_space = planning.find_free_space(newWorld, goal_node)
    #* Using the Breath First Search to find the paths
    #? Inverted since the Breath Search works from goal to start
    path = planning.breath_first_search(newWorld, goal_node, start_node)
    #* Start all the Sprite Class in a Group
    all_robots = pygame.sprite.Group()
    all_treadmill_items = pygame.sprite.Group()
    #* Init the Spriter Class
    robot = createWorld.SingleRobot(start_node, goal_node, path)
    treadmill_items = createWorld.TreadmillItems()
    #* Add the Sprite Classes to the Groups
    all_robots.add(robot)       
    all_treadmill_items.add(treadmill_items)
    # CREATE A LOOP AND RUN THE WORLD IN A SCREEN CONTINUALLY
    # * If still running do the Loop
    running = True
    while running:
        # ADJUST THE CLOCK
        createWorld.clock.tick(default.FPS)
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
                            [(int(loc.x), int(loc.y)) for loc in createWorld.obstaclesPositionGlobal])
                if event.key == pygame.K_SPACE:
                    #* Run the robot movement simulation
                    all_robots.update()
            # CHECKS IF THERE'S A MOUSE BUTTON EVENT IN THE SCREEN
            if event.type == pygame.MOUSEBUTTONDOWN:
                # PICK THE GRID LOCATION WHERE THE MOUSE WAS PUSHED AND STORE
                mouse_pos = vec(pygame.mouse.get_pos()) // newWorld.cell_size_width
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
                #* MIDDLE MOUSE TO CHANGE THE CURRENT START POSITION
                if event.button == 2:
                    start_node = mouse_pos
                    print(start_node)
                #* RIGHT MOUSE TO CHANGE THE GOAL
                if event.button == 3:
                    goal_node = mouse_pos
                path = planning.breath_first_search(newWorld, goal_node, start_node)
                all_robots = pygame.sprite.Group()
                robot = createWorld.SingleRobot(start_node, goal_node, path)
                all_robots.add(robot)
                all_robots.draw(createWorld.screen)
        # DRAW THE SCREEN CAPTION DISPLAY WITH FPS
        pygame.display.set_caption("World Grid Representation [{:.2f}]".format(createWorld.clock.get_fps()))
        # FILLS THE SCREEN WITH A BLANK DISPLAY
        createWorld.screen.fill(paint.COLOR_WHITE)
        # FILL THE EXPLORED AREA
        for node in path:
            x, y = node
            rect = pygame.Rect(x * createWorld.cellSizeWidth,
                               y * createWorld.cellSizeWidth,
                               createWorld.cellSizeWidth,
                               createWorld.cellSizeWidth)
            pygame.draw.rect(createWorld.screen,
                             paint.COLOR_STATEGRAY, rect)
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
        #* Draws the Don't Move Zone
        newWorld.draw_dont_move()
        #* Draw the Path Planning Arrows
        newWorld.draw_arrows()
        #* Update the Spriters
        all_treadmill_items.update()
        #* Draw the Spriters in the Grid
        all_robots.draw(createWorld.screen)
        all_treadmill_items.draw(createWorld.screen)
        #* Draw the Path from Start to Goal
        current = start_node + path[vec_to_int(start_node)]
        #* Empty Variable to Calculates the Path Lenght
        global pathLength
        pathLength = 1
        #* As long we never reached the Goal
        while current != goal_node:
            path_vector = path[(current.x, current.y)]
            #? Uses the Length Squared function of PyGame to return the
            #? Euclidean length of the vector
            if path_vector.length_squared() == 1:
                pathLength += 1
            x = (current.x * createWorld.cellSizeWidth) + (createWorld.cellSizeWidth/2)
            y = (current.y * createWorld.cellSizeWidth) + (createWorld.cellSizeWidth/2)
            img = newWorld.arrows[vec_to_int(path[(current.x, current.y)])]
            #* Center the image in the cell
            arrow_rec = img.get_rect(center=(x,y))
            #* Show in the screen
            createWorld.screen.blit(img, arrow_rec)
            # FIND THE NEXT NODE IN THE PATH
            #* Add to the current node the next node in the path
            current = current + path[vec_to_int(current)]
        #* Load the Draw Start Node Function
        newWorld.draw_start(start_node)
        #* Load the Draw Goal Function
        newWorld.draw_goal(goal_node)
        #* Draws the Path Size in the screen
        newWorld.draw_text('Breadth First Search', 15,
                           paint.COLOR_WHITE,
                           createWorld.screenSizeWidth-110,
                           createWorld.screenSizeHeight-30,
                           align="bottomright")
        newWorld.draw_text('Path Lenght: {} Node Cells'.format(pathLength), 13,
                           paint.COLOR_WHITE,
                           createWorld.screenSizeWidth-110,
                           createWorld.screenSizeHeight-10,
                           align="bottomright")
        #*Update the full display Surface to the screen
        pygame.display.flip()

def run_dijkstras_search(start=world.START, goal=world.GOAL):
    """
    The Dijkstras Search function of the Path Planning Library, responsible
    to run the Weighted World and calculate the shortest path in a PyGame Screen
        
    Attributes:
        (start) (tuple)
        (goal) (tuple)
    
    Args:
        (start): Is the position in the Weighted Grid that
                 we start the Path Plannign Algorithm
        (goal) : Is the position in the Weighted Grid that
                 we want to achieve
    Returns:
       The Shortest Path from the Dijkstra Algorithm loaded in a
       PyGame Screen with World Inputed Variables by the program or user
    """
    # CALL THE CLASS WORLD GRID
    #* ADJUST HERE THE WORLD YOU WANT
    #? IF YOU NEED TO TEST THE WORLD FIRST USE THE "createGrids.py" module
    global start_node, goal_node
    start_node = vec(start)
    print(start_node)
    goal_node = vec(goal.popleft())
    print(goal_node)
    newWorld = createWorld.WeightedGrid()
    planning = PathPlanning()
    # PUT THE FUNCTIONS THAT YOU WANT BELLOW
    #* THE POSTIION WE WANT
    #* Using the Find Free Space to find the free space availabe in the
    free_space = planning.find_free_space(newWorld, goal_node)
    #* Using the Breath First Search to find the paths
    #? Inverted since the Breath Search works from goal to start
    path = planning.dijkstras_search(newWorld, goal_node, start_node)
    #* Start all the Sprite Class in a Group
    all_robots = pygame.sprite.Group()
    all_treadmill_items = pygame.sprite.Group()
    #* Init the Spriter Class
    robot = createWorld.SingleRobot(start_node, goal_node, path)
    treadmill_items = createWorld.TreadmillItems()
    #* Add the Sprite Classes to the Groups
    all_robots.add(robot)
    all_treadmill_items.add(treadmill_items)
    # CREATE A LOOP AND RUN THE WORLD IN A SCREEN CONTINUALLY
    # * If still running do the Loop
    running = True
    while running:
        # ADJUST THE CLOCK
        createWorld.clock.tick(default.FPS)
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
                            [(int(loc.x), int(loc.y)) for loc in createWorld.obstaclesPositionGlobal])
                if event.key == pygame.K_SPACE:
                    #* Run the robot movement simulation
                    all_robots.update()
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
                #* MIDDLE MOUSE TO CHANGE THE CURRENT START POSITION
                if event.button == 2:
                    start_node = mouse_pos
                #* RIGHT MOUSE TO CHANGE THE GOAL
                if event.button == 3:
                    goal_node = mouse_pos
                path = planning.dijkstras_search(newWorld, goal_node, start_node)
                all_robots = pygame.sprite.Group()
                robot = createWorld.SingleRobot(start_node, goal_node, path)
                all_robots.add(robot)
                all_robots.draw(createWorld.screen)
        # DRAW THE SCREEN CAPTION DISPLAY WITH FPS
        pygame.display.set_caption("World Grid Representation [{:.2f}]".format(createWorld.clock.get_fps()))
        # FILLS THE SCREEN WITH A BLANK DISPLAY
        createWorld.screen.fill(paint.COLOR_WHITE)
        # FILL THE EXPLORED AREA
        for node in path:
            x, y = node
            rect = pygame.Rect(x * createWorld.cellSizeWidth,
                               y * createWorld.cellSizeWidth,
                               createWorld.cellSizeWidth,
                               createWorld.cellSizeWidth)
            pygame.draw.rect(createWorld.screen,
                             paint.COLOR_STATEGRAY, rect)
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
        #* Update the Treadmill Spriter
        all_treadmill_items.update()
        #* Draw the Spriters in the Grid
        all_robots.draw(createWorld.screen)
        all_treadmill_items.draw(createWorld.screen)
        #* Draw the Path from Start to Goal
        current = start_node + path[vec_to_int(start_node)]
        #* Empty Variable to Calculates the Path Lenght
        global pathLength
        pathLength = 1
        #* As long we never reached the Goal
        while current != goal_node:
            path_vector = path[(current.x, current.y)]
            #? Uses the Length Squared function of PyGame to return the
            #? Euclidean length of the vector
            if path_vector.length_squared() == 1:
                pathLength += 1
            x = (current.x * createWorld.cellSizeWidth) + (createWorld.cellSizeWidth/2)
            y = (current.y * createWorld.cellSizeWidth) + (createWorld.cellSizeWidth/2)
            img = newWorld.arrows[vec_to_int(path[(current.x, current.y)])]
            #* Center the image in the cell
            arrow_rec = img.get_rect(center=(x,y))
            #* Show in the screen
            createWorld.screen.blit(img, arrow_rec)
            # FIND THE NEXT NODE IN THE PATH
            #* Add to the current node the next node in the path
            current = current + path[vec_to_int(current)]
        #* Load the Draw Robot Function
        newWorld.draw_start(start_node)
        #* Load the Draw Goal Function
        newWorld.draw_goal(goal_node)
        #* Draws the Path Size in the screen
        newWorld.draw_text('Dijkstra Search', 15,
                           paint.COLOR_WHITE,
                           createWorld.screenSizeWidth-110,
                           createWorld.screenSizeHeight-30,
                           align="bottomright")
        newWorld.draw_text('Path Lenght: {} Node Cells'.format(pathLength), 13,
                           paint.COLOR_WHITE,
                           createWorld.screenSizeWidth-110,
                           createWorld.screenSizeHeight-10,
                           align="bottomright")
        #*Update the full display Surface to the screen
        pygame.display.flip()

def run_astar_search(start=world.START, goal=world.GOAL):
    """
    The A-Star (A*) Search function of the Path Planning Library, responsible
    to run the Weighted World and calculate the shortest path in a PyGame Screen
        
    Attributes:
        (start) (tuple)
        (goal) (tuple)
    
    Args:
        (start): Is the position in the Weighted Grid that
                 we start the Path Plannign Algorithm
        (goal) : Is the position in the Weighted Grid that
                 we want to achieve
    Returns:
       The Shortest Path from the A-Star* Algorithm loaded in a
       PyGame Screen with World Inputed by the program or user
    """
    # CALL THE CLASS WORLD GRID
    #* ADJUST HERE THE WORLD YOU WANT
    #? IF YOU NEED TO TEST THE WORLD FIRST USE THE "createGrids.py" module
    global start_node, goal_node
    start_node = vec(start)
    print(start_node)
    goal_node = vec(goal.popleft())
    print(goal_node)
    newWorld = createWorld.WeightedGrid()
    planning = PathPlanning()
    # PUT THE FUNCTIONS THAT YOU WANT BELLOW
    #* THE POSTIION WE WANT
    #* Using the Find Free Space to find the free space availabe in the
    free_space = planning.find_free_space(newWorld, goal_node)
    #* Using the A* Search to find the paths
    #? Inverted since the A* Search works from goal to start
    path = planning.astar_search(newWorld, goal_node, start_node)
    #* Start all the Sprite Class in a Group
    all_robots = pygame.sprite.Group()
    all_treadmill_items = pygame.sprite.Group()
    #* Init the Spriter Class
    robot = createWorld.SingleRobot(start_node, goal_node, path)
    treadmill_items = createWorld.TreadmillItems()
    #* Add the Sprite Classes to the Groups
    all_robots.add(robot)
    all_treadmill_items.add(treadmill_items)
    # CREATE A LOOP AND RUN THE WORLD IN A SCREEN CONTINUALLY
    # * If still running do the Loop
    running = True
    while running:
        # ADJUST THE CLOCK
        createWorld.clock.tick(default.FPS)
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
                            [(int(loc.x), int(loc.y)) for loc in createWorld.obstaclesPositionGlobal])
                if event.key == pygame.K_SPACE:
                    #* Run the robot movement simulation
                    all_robots.update()
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
                #* MIDDLE MOUSE TO CHANGE THE CURRENT START POSITION
                if event.button == 2:
                    start_node = mouse_pos
                #* RIGHT MOUSE TO CHANGE THE GOAL
                if event.button == 3:
                    goal_node = mouse_pos
                path = planning.astar_search(newWorld, goal_node, start_node)
                all_robots = pygame.sprite.Group()
                robot = createWorld.SingleRobot(start_node, goal_node, path)
                all_robots.add(robot)
                all_robots.draw(createWorld.screen)
        # DRAW THE SCREEN CAPTION DISPLAY WITH FPS
        pygame.display.set_caption("World Grid Representation [{:.2f}]".format(createWorld.clock.get_fps()))
        # FILLS THE SCREEN WITH A BLANK DISPLAY
        createWorld.screen.fill(paint.COLOR_WHITE)
        # FILL THE EXPLORED AREA
        for node in path:
            x, y = node
            rect = pygame.Rect(x * createWorld.cellSizeWidth,
                               y * createWorld.cellSizeWidth,
                               createWorld.cellSizeWidth,
                               createWorld.cellSizeWidth)
            pygame.draw.rect(createWorld.screen,
                             paint.COLOR_STATEGRAY, rect)
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
        all_robots.draw(createWorld.screen)
        all_treadmill_items.draw(createWorld.screen)
        #* Draw the Path from Start to Goal
        current = start_node + path[vec_to_int(start_node)]
        #* Empty Variable to Calculates the Path Lenght
        global pathLength
        pathLength = 1
        #* As long we never reached the Goal
        while current != goal_node:
            path_vector = path[(current.x, current.y)]
            #? Uses the Length Squared function of PyGame to return the
            #? Euclidean length of the vector
            if path_vector.length_squared() == 1:
                pathLength += 1
            x = (current.x * createWorld.cellSizeWidth) + (createWorld.cellSizeWidth/2)
            y = (current.y * createWorld.cellSizeWidth) + (createWorld.cellSizeWidth/2)
            img = newWorld.arrows[vec_to_int(path[(current.x, current.y)])]
            #* Center the image in the cell
            arrow_rec = img.get_rect(center=(x,y))
            #* Show in the screen
            createWorld.screen.blit(img, arrow_rec)
            # FIND THE NEXT NODE IN THE PATH
            #* Add to the current node the next node in the path
            current = current + path[vec_to_int(current)]
        #* Load the Draw Robot Function
        newWorld.draw_start(start_node)
        #* Load the Draw Goal Function
        newWorld.draw_goal(goal_node)
        #* Draws the Path Size in the screen
        newWorld.draw_text('A* Search', 15,
                           paint.COLOR_WHITE,
                           createWorld.screenSizeWidth-110,
                           createWorld.screenSizeHeight-30,
                           align="bottomright")
        newWorld.draw_text('Path Lenght: {} Node Cells'.format(pathLength), 13,
                           paint.COLOR_WHITE,
                           createWorld.screenSizeWidth-110,
                           createWorld.screenSizeHeight-10,
                           align="bottomright")
        #*Update the full display Surface to the screen
        pygame.display.flip()

def run_space_astar_search(goal=world.GOAL,time=world.TIME_LIMIT,
                           robots_qtd=world.ROBOTS_QTD):
    """
    The Space Time A-Star (STA*) Search function of the Path Planning Library, is
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
       The Shortest Path from the STA* Algorithm for multiple robots loaded in a
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
    newWorld = createWorld.WeightedGrid()
    planning = PathPlanning()
    #* Start the Treadmill Class in a Group
    all_treadmill_items = pygame.sprite.Group()
    #* Init the treadmill Sprite
    treadmill_items = createWorld.TreadmillItems()
    all_treadmill_items.add(treadmill_items)
    #* Add the Robots Sprite Classes to the Groups
    all_robots = pygame.sprite.Group()
    # SET THE MULTIPLE PATHS
    global pathsGlobal, robotsStartPos
    pathsGlobal = deque([])
    robotsStartPos = deque([])
    for _ in range(robots_qtd):
        goal_poped = vec(goals.popleft())
        #* Using the Find Free Space to find the free space availabe in the
        free_space = planning.find_free_space(newWorld, goal_poped)
        random_start = random.choice(free_space)
        robotsStartPos.extend([random_start])
        pathsGlobal.append([planning.space_astar_search(newWorld, goal_poped, random_start, time)])
        #* Run the Paths and Add The Robots
        start = robotsStartPos.popleft()
        path = pathsGlobal.popleft()
        robots =  createWorld.MultiRobot(start, goal_poped, path)
        all_robots.add(robots)
    # CREATE A LOOP AND RUN THE WORLD IN A SCREEN CONTINUALLY
    # * If still running do the Loop
    running = True
    while running:
        # ADJUST THE CLOCK
        createWorld.clock.tick(default.FPS)
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
                            [(int(loc.x), int(loc.y)) for loc in createWorld.obstaclesPositionGlobal])
                if event.key == pygame.K_SPACE:
                    #* Run the robot movement simulation
                    pygame.event.clear()
                    with concurrent.futures.ProcessPoolExecutor(max_workers=5) as executor:
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
        pygame.display.set_caption("World Grid Representation [{:.2f}]".format(createWorld.clock.get_fps()))
        # FILLS THE SCREEN WITH A BLANK DISPLAY
        createWorld.screen.fill(paint.COLOR_WHITE)
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
        all_robots.draw(createWorld.screen)
        all_treadmill_items.draw(createWorld.screen)
        #*Update the full display Surface to the screen
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
    parser.add_argument('--breadth', action='store_true',
                        help='Runs the Breath First Search')

    parser.add_argument('--dijkstra', action='store_true',
                        help='Runs the Dijkstra Search')

    parser.add_argument('--astar', action='store_true',
                        help='Runs the A* Search')

    parser.add_argument('--stastar', action='store_true',
                        help='Runs the Space Time A* Search')

    parser.add_argument('--version', action='version',
                        version='Path Planning with Python = v1.0')

    args = parser.parse_args()

    if args.breadth:
        run_breadth_search()
    if args.dijkstra:
        run_dijkstras_search()
    if args.astar:
        run_astar_search()
    if args.stastar:
        run_space_astar_search()

if __name__ == '__main__':
    main()
