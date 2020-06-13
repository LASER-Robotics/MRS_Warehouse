#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Author: Italo Barros
Email: ircbarros@pm.me
License: MIT

Am example of how the system is implemented using the Breadth

You can run this module using the command line:

$ python3 -m memory_profiler example.py

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
import time
import pygame
import timeit
#* To see the memory usage in each line
import memory_profiler
#* Import the createWorld Module
import world.createWorld
#? Using the collections module since is the most efficient
#? to implement and manipulate a queue list
from collections import deque
#* To see the sum of the system and user CPU Time of
#* of the current process
from time import process_time

# GLOBAL VARIABLES
#* Load the PyGame Vector2 lib
vec = pygame.math.Vector2
#* SET the DEFAULT position X in the grid
pos_x = 0
#* SET the DEFAULT position Y in the grid
pos_y = 0

class PathPlanning:
    """
    The Class Responsible to the Path Planning Algorithm and functions,
    responsible to return or execute the Free Space and the Breadth-First Search
    Planning

    For more informations of each method, see the specified docstring!

    Functions:

        find_free_space(graph, position): returns the free space available in
                                          the grid

        breath_first_search: returns the node path executed by the algorithm

    """
    def __init__(self):

        pass


    def find_free_space(self, graph, position=(pos_x, pos_y)):
        """
        Reads free nodes in World Grid using the find_neighbors(node) function,
        and returns a list of free nodes that can be explored starting from the
        position inputed.

        Attributes:

            (graph) (Class)
            (position) (tuple)
        
        Args:

            (graph): A World Grid Class
            (pos_x, pos_y): Is the start position in the World Grid that
                            we want to find the free space as a tuple
        
        Vars:

            pos_x = The node position in the X axys (column)
            pos_y = The node position in the Y axys (row)
        
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
        # SET THE POSITION USING LIST COMPREHENSION
        self.position_x, self.position_y = position[0], position[1]
        # TRANSFORM THE POSITION TO THE PYGAME VECTOR
        self.position = vec(self.position_x, self.position_y)
        # IMPORT THE DEQUE TO PUT THE NODES
        self.frontier = deque()
        # APPEND THE FRONTIER WITH THE POSITION
        self.frontier.append(position)
        # THE LIST OF VISITED NODES
        self.visited = []
        # THE POSITION WILL BE PUT AT THE VISITED QUEUE (IS WHERE WE ARE)
        self.visited.append(position)
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
        print(f'\nThe Free Node Cells Available are:\n{self.visited}')
        print(f'\nFree Node Cells availabe: {len(self.visited)} Grid Cells')
        return self.visited

    #? The memory profile generator
    @profile
    def breath_first_search(self, graph, position=(pos_x, pos_y)):
        """
        Reads free nodes in World Grid using the find_neighbors(node) function,
        and returns the Breath Firs Search for the Node inputed in the header.

        Attributes:

            (graph) (Class)
            (position) (tuple)
        
        Args:

            (graph): A World Grid Class
            (pos_x, pos_y): Is the position in the World Grid that
                            we want to find path to it
        
        Vars:

            pos_x = The node position in the X axys (column)
            pos_y = The node position in the Y axys (row)
        
        Returns:

            A list of Path nodes available to the Position Inputed

        """
    
        print('\n__________________ BREATH FIRST SEARCH STARTED __________________\n')
        #* START THE TIMER
        start_time = timeit.default_timer()
        start_process_time = process_time()
        # SET THE POSITION USING LIST COMPREHENSION
        self.position_x, self.position_y = position[0], position[1]
        # TRANSFORM THE POSITION TO THE PYGAME VECTOR
        self.position = vec(self.position_x, self.position_y)
        # IMPORT THE DEQUE TO PUT THE NODES
        self.frontier = deque()
        # APPEND THE FRONTIER WITH THE POSITION
        self.frontier.append(position)
        # IN THIS CASE WE ARE ONLY INTERESTED IF WE ARE MOVING TO THE CELL
        #* OK, I visited the node, but I Will move to it?
        #? Create a Path Dictionary were the Key will be the cell and the value
        #? is the Cell that we came from
        self.path = {}
        # THE START IS NONE SINCE IS WERE WE ARE
        #? Converts the start vector to integers
        self.path[vec_to_int(self.position)] = None
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

def vec_to_int(vector):
    """
    A function that converts a vector to a integer.

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


def main():
    """
    The main function of the Path Planning Library, responsible to run
    the World e execute the desired functions. Put here the world you
    want to run, the functions, and the program will execute the
    simulation and open a new screen.

    """
    # CALL THE CLASS WORLD GRID
    #* ADJUST HERE THE WORLD YOU WANT
    #? IF YOU NEED TO TEST THE WORLD FIRST USE THE "createGrids.py" module
    newWorld = world.createWorld.WorldGrid()
    #* Init the Path Planning Class
    planning = PathPlanning()
    #* Init the draw Arrows Function from the World Grid
    newWorld.draw_arrows()
    # PUT THE FUNCTIONS THAT YOU WANT BELLOW
    #* THE POSTIION WE WANT
    start = (0,0)
    #* Using the Find Free Space to find the free space availabe in the grid
    free_space = planning.find_free_space(newWorld, start)
    #* Using the Breath First Search to find the paths
    path = planning.breath_first_search(newWorld, start)
    # CREATE A LOOP AND RUN THE WORLD IN A SCREEN CONTINUALLY
    # * If still running do the Loop
    running = True
    while running:
        # ADJUST THE CLOCK
        world.createWorld.clock.tick(world.createWorld.FPS)
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
                            [(int(loc.x), int(loc.y)) for loc in world.createWorld.obstaclesPosition])
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
                if event.button == 3:
                    start = mouse_pos
                path = planning.breath_first_search(newWorld, start)
        # DRAW THE SCREEN CAPTION DISPLAY WITH FPS
        pygame.display.set_caption("World Grid Representation [{:.2f}]".format(world.createWorld.clock.get_fps()))
        # FILLS THE SCREEN WITH A BLANK DISPLAY
        world.createWorld.screen.fill(world.createWorld.COLOR_WHITE)
        # UPDATE THE DISPLAY
        #* Draw the grid
        newWorld.draw_grid()
        #* Draw the obstacles
        newWorld.draw_obstacles()
        #* Draw the Recharge Zone
        newWorld.draw_recharge_zone()
        #* Draw the Treadmill Zone
        newWorld.draw_treadmill_zone()
        #* Draw the Workers Zone
        newWorld.draw_workers_zone()
        #* Draw the Delivery Zone
        newWorld.draw_delivery_zone()
        #* Draw the Pickup Zone
        newWorld.draw_pickup_zone()
        #* Draw the Arrows in the Screen
        #* For each node and direction in the path dictionary do
        for node, direction in path.items():
            #* If there's a direction
            if direction:
                #* Find the X and Y of the node
                x, y = node
                #* Find the center point of the node X
                #? The center will be the (x * Cell Size) + (Cell Size / 2)
                x = (x * world.createWorld.cellSizeWidth) + (world.createWorld.cellSizeHeight/2)
                #* Find the center point of the node Y
                #? The center will be the (y * Cell Size) + (Cell Size / 2)
                y = (y * world.createWorld.cellSizeWidth) + (world.createWorld.cellSizeHeight/2)
                #* The image that we want to use is the arrow pointing to the direction we need
                img = newWorld.arrows[vec_to_int(direction)]
                #* Center the image in the cell
                arrow_rec = img.get_rect(center=(x,y))
                #* Show in the screen
                world.createWorld.screen.blit(img, arrow_rec)
        #*Update the full display Surface to the screen
        pygame.display.flip()

if __name__ == '__main__':
    main()
