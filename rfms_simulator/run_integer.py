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
from world import createIntegerWorld
from world.settings import settingsIntBigWorld
# LOAD THE WORLD VARIABLES
#* Horizontal Layout
world = createIntegerWorld.create
# LOAD THE PYGAME DEFAULTS VARIABLES
default = settingsIntBigWorld.PygameDefaults()
# LOAD THE COLORS FROM THE SETTINGS
paint = settingsIntBigWorld.Colors()
# LOAD THE COLORS FROM THE SETTINGS
# THE SELECTED STARTS
SelectedStarts = set()
# ADJACENCY LIST
AdjacencyList = []
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



def create_adjacency_list(world_width=world.GRID_WIDTH, world_height=world.GRID_HEIGHT):

    number_of_nodes = int(world_width * world_height)
    list_of_nodes = set([x for x in range(0, number_of_nodes)])
    for node in list_of_nodes:
        upper_node = node - world_width
        bottom_node = node + world_width
        #* Checks if the node is in the Left Limits
        if node % world_width == 0:
            left_node = None
            right_node = node + 1
        #* Checks if the node is in the Left Limits
        elif (node + 1) % world_width == 0:
            left_node = node - 1
            right_node = None
        else:
            left_node = node - 1
            right_node = node + 1
        nodes_around_raw = set([upper_node, bottom_node, left_node, right_node])
        #* Removes the undesired nodes (negative numbers or number outside limits)
        real_nodes_around = list(nodes_around_raw & list_of_nodes)
        global AdjacencyList
        AdjacencyList.append(real_nodes_around)

def find_all_pygame_nodes(graph, goal):

    # TRANSFORM THE POSITION TO THE PYGAME VECTOR
    goal = goal
    # IMPORT THE DEQUE TO PUT THE NODES
    frontier = deque()
    # APPEND THE FRONTIER WITH THE POSITION
    frontier.append(goal)
    # THE LIST OF VISITED NODES
    visited = []
    # THE POSITION WILL BE PUT AT THE VISITED QUEUE (IS WHERE WE ARE)
    visited.append(goal)
    # START OUR LOOP
    #* As long there's nodes on the frontier do
    while len(frontier) > 0:
        # THE CURRENT NODE WE WANT TO LOOK IS THE NEXT NODE
        #* Pop's the next on the queue list
        current = frontier.popleft()
        # THE NEIGHBOORS OF THE CURRENT TILE
        for next in graph.find_all_neighbors(current):
            # IF THE NEXT NODE IS NOT VISITED
            if next not in visited:
                # ADD THE NODE TO THE FRONTIER LIST
                frontier.append(next)
                # PUT ON THE VISITED NODES
                visited.append(next)
    print(f'\nThe All PyGame Node Cells Available are:\n{visited}')
    return visited

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

def run_base_world(goal=world.GOAL,time=world.TIME_LIMIT,
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
    newWorld = createIntegerWorld.WeightedGrid()
    #* Start the Treadmill Class in a Group
    all_treadmill_items = pygame.sprite.Group()
    #* Init the treadmill Sprite
    treadmill_items = createIntegerWorld.TreadmillItems()
    all_treadmill_items.add(treadmill_items)
    #* Add the Robots Sprite Classes to the Groups
    all_robots = pygame.sprite.Group()
    #* Create the Nodes as Integers
    create_adjacency_list()
    global AdjacencyList
    print(f'\nThe Adjacency List is: {AdjacencyList}\n')
    #* Show PyGame Nodes (vec format) and to ordenated Tuples
    all_pygame_nodes_vec = find_all_pygame_nodes(newWorld, vec(0,0))
    all_pygame_nodes_list = vec_to_list(all_pygame_nodes_vec)
    pygame_node_list = sorted(all_pygame_nodes_list, key=itemgetter(1))
    print(f'The Arranged All Nodes List is: {pygame_node_list}\n')
    #* Create a Dict to create relation between the integer graph and PyGame Graph
    relate_pygame_int_graph = dict(zip(pygame_node_list, AdjacencyList))
    print(f'The Relation Between PyGame and Int Graph is: {relate_pygame_int_graph}\n')
    # CREATE A LOOP AND RUN THE WORLD IN A SCREEN CONTINUALLY
    # * If still running do the Loop
    running = True
    while running:
        # ADJUST THE CLOCK
        createIntegerWorld.clock.tick(default.FPS)
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
                            [(int(loc.x), int(loc.y)) for loc in createIntegerWorld.obstaclesPositionGlobal])
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
        pygame.display.set_caption("World Grid Representation [{:.2f}]".format(createIntegerWorld.clock.get_fps()))
        # FILLS THE SCREEN WITH A BLANK DISPLAY
        createIntegerWorld.screen.fill(paint.COLOR_WHITE)
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
        all_robots.draw(createIntegerWorld.screen)
        all_treadmill_items.draw(createIntegerWorld.screen)
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

    parser.add_argument('--base', action='store_true',
                        help='Shows the Base World')

    parser.add_argument('--version', action='version',
                        version='Path Planning with Python = v1.0')

    args = parser.parse_args()

    if args.base:
        run_base_world()

if __name__ == '__main__':
    main()