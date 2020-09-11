#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Author: Italo Barros
Email: ircbarros@pm.me
License: MIT
An grid generator using the PyGame module!

This module will create a New World Grid to run the Path Planning Algorithms.

For more informations take a look at the "WorldGrid" Class.

Classes:

    WorldGrid: Create a new World Grid with specified or inputed Arguments

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
import heapq
import random
import schedule
import threading
import concurrent.futures
from world import settings
from multiprocessing import Process
from collections import deque
from .settings import settingsBigWorld

# GLOBAL VARIABLES
# LOAD THE PyGAME VETOR2 LIB
vec = pygame.math.Vector2
# LOAD THE WORLD VARIABLES
#? You can change here the world you want to launch. To create
#? a new world, just add a new Class to the settings module
#* Horizontal Layout
create = settingsBigWorld.TradGridHorizontal()
#* Vertical Layout
#create = settingsBigWorld.TradGridVertical()
#* Flying-V Layout
#create = settingsBigWorld.FlyingVGrid()
#* Fishbone Layout
#create = settingsBigWorld.FishboneGrid()
# LOAD THE COLORS FROM THE SETTINGS
paint = settingsBigWorld.Colors()
# LOAD THE PYGAME DEFAULTS VARIABLES
default = settingsBigWorld.PygameDefaults()
# CENTER THE GRID TO THE MIDDLE OF THE SCREEN
os.environ['SDL_VIDEO_CENTERED'] = '1'
# INIT THE PYGAME MODULE
pygame.init()
# INIT THE PYGAME CLOCK
clock = pygame.time.Clock()


class WorldGrid:
    '''
    Create a Word Grid in a screen size defined by the user
    or by default using the cells and the grid size. Use this function
    if you DO NOT WANT THE SCREEN and ONLY THE NODE VALUES

    Attributes:

     (GRID_WIDTH, GRID_HEIGHT) (tuple)
     (CELL_WIDTH, CELL_HEIGHT) (tuple)
     [(OBSTACLE_1_x, OBSTACLE_1_y), ... ,(OBSTACLE_X_x, OBSTACLE_X_y)]     (list)
     [(RECHARGE_1_x, RECHARGE_1_y), ... ,(RECHARGE_X_x, RECHARGE_X_y)]     (list)
     [(TREADMILL_1_x, TREADMILL_1_y), ... ,(TREADMILL_X_x, TREADMILL_X_y)] (list)
     [(WORKERS_1_x, WORKERS_1_y), ... ,(WORKERS_X_x, WORKERS_X_y)]         (list)
     [(DELIVERY_1_x, DELIVERY_1_y), ... ,(DELIVERY_X_x, DELIVERY_X_y)]     (list)
     [(PICKUP_1_x, PICKUP_1_y), ... ,(PICKUP_X_x, PICKUP_X_y)]             (list)
     [(DONT_MOVE_1_x, DONT_MOVE_1_y), ... ,(DONT_MOVE_X_x, DONT_MOVE_X_y)] (list)
    
        Where:

            GRID_WIDTH  (int)
            GRID_HEIGHT (int)
            CELL_WIDTH  (int)
            CELL_HEIGHT (int)
            OBSTACLE_x  (int)
            OBSTACLE_y  (int)
            RECHARGE_x  (int)
            RECHARGE_y  (int)
            TREADMILL_x (int)
            TREADMILL_y (int)
            WORKERS_x   (int)
            WORKERS_y   (int)
            DELIVERY_x  (int)
            DELIVERY_y  (int)
            PICKUP_x    (int)
            PICKUP_y    (int)
            DONT_MOVE_x (int)
            DONT_MOVE_y (int)

    Args:

        (GRID_WIDTH, GRID_SIZE):          An tuple with the desired GRID size
        (CELL_WIDTH, CELL_HEIGHT):        An tuple with the desired CELL size
        (OBSTACLE_X_x, OBSTACLE_X_y):     An tuple with the desired OBSTACLES positions
        (RECHARGE_X_x, RECHARGE_X_y):     An tuple with the desired RECHARGE positions
        (TREADMILL_X_x, TREADMILL_X_y):   An tuple with the desired TREADMILL position
        (WORKERS_X_x, WORKERS_X_y):       An tuple with the desired WORKERS positions
        (DELIVERY_X_x, DELIVERY_X_y):     An tuple with the desired DELIVERY positions
        (PICKUP_X_x, WORKERS_X_y):        An tuple with the desired PICKUP positions
        (DONT_MOVE_X_x, DONT_MOVE_X_y):   An tuple with the desired DON'T MOVE positions

    Vars:

        GRID_WIDTH    = The desired GRID WIDTH
        GRID_HEIGHT   = The desired GRID HEIGHT
        CELL_WIDTH    = The desired CELL WIDTH
        CELL_HEIGHT   = The desired CELL HEIGHT
        OBSTACLE_X_x  = The OBSTACLE location "X" at Grid pos. X  (column)
        OBSTACLE_X_y  = The  OBSTACLE location  "X" at Grid in pos. Y (row)
        RECHARGE_X_x  = The RECHARGE location "X" at Grid pos. X  (column)
        RECHARGE_X_y  = The RECHARGE location  "X" at Grid in pos. Y (row)
        TREADMILL_X_x = The TREADMILL location "X" at Grid pos. X  (column)
        TREADMILL_X_y = The TREADMILL location  "X" at Grid in pos. Y (row)
        WORKERS_X_x   = The WORKERS location "X" at Grid pos. X  (column)
        WORKERS_X_y   = The WORKERS location  "X" at Grid in pos. Y (row)
        DELIVERY_X_x  = The DELIVERY location "X" at Grid pos. X  (column)
        DELIVERY_X_y  = The DELIVERY  location  "X" at Grid in pos. Y (row)
        PICKUP_X_x    = The PICKUP location "X" at Grid pos. X (column)
        PICKUP_X_y    = The PICKUP location  "X" at Grid in pos. Y (row)

    Returns:

        A Node Grid Graph with Obstacles, Recharge Zone, Treadmill Zone,
        Workers Zone, Delivery Zone, Pickup Zone and Don't Move zones with
        size e height inputed by the user WITHOUT SCREEN VISUALIZATION!

    [Examples]

        USER DEFINED VALUES:
        import grid
        world = grid.WorldGrid((10,10), (100,100), (2,2), (6,6), (9,9), (8,9), (8,8)) 
        >>> Will show a 10x10 Grid Screen with cells size 100x100 and obstacle
            at positions (2,2), recharge zone at position (6,6), treadmill
            at position (9,9), worker at position (8,9), and delivery at position
            (8,8)

        DEFAULT VALUES:
        import grid
        world = grid.WorldGrid()
        >>> Will show the DEFAULT grid (12x8) with cell size 100x100 and
            similar to the image on the github
    '''

    def __init__(self, world_grid_size = (create.GRID_WIDTH, create.GRID_HEIGHT),
                 world_cell_size = (create.CELL_WIDTH, create.CELL_HEIGHT),
                 world_obstacles_positions = create.OBSTACLES,
                 world_recharge_positions = create.RECHARGE_ZONE,
                 world_treadmill_positions = create.TREADMILL_ZONE,
                 world_workers_positions = create.WORKERS_POS,
                 world_delivery_positions = create.DELIVERY_ZONE,
                 world_pickup_positions = create.PICKUP_ZONE,
                 world_dont_move = create.DONT_MOVE,
                 world_nodes_constraints = create.NODES_CONSTRAINTS):
        # SET THE GRID SIZE USING LIST COMPREHENSION
        self.grid_size_width, self.grid_size_height = world_grid_size[0], world_grid_size[1]
        # SET THE CELL SIZE USING LIST COMPREHENSION
        self.cell_size_width, self.cell_size_height = world_cell_size[0], world_cell_size[1]
        # SET  THE OBSTACLES POSITION
        self.world_obstacles_position = world_obstacles_positions
        # SET THE RECHARGE ZONE POSITION
        self.world_recharge_position = world_recharge_positions
        # SET THE TREADMILL POSITION
        self.world_treadmill_position = world_treadmill_positions
        # SET THE WORKERS POSITIONS
        self.world_workers_positions = world_workers_positions
        # SET THE DELIVERY POSITIONS
        self.world_delivery_positions = world_delivery_positions
        # SET THE PICKUP POSITIONS
        self.world_pickup_positions = world_pickup_positions
        # SET THE CANT MOVE POSITIONS
        self.world_dont_move = world_dont_move
        # SET THE NODE WORLD CONSTRAINTS
        self.world_nodes_constraints = world_nodes_constraints
        # SET THE OBSTACLES VECTOR
        self.obstaclesPosition = []
        for obstacle in self.world_obstacles_position:
            self.obstaclesPosition.append(vec(obstacle))
        global obstaclesPositionGlobal
        obstaclesPositionGlobal = self.obstaclesPosition
        # SET THE TREADMILL VECTOR
        global treadmillPositionGlobal
        treadmillPositionGlobal = []
        for treadmill_zone in self.world_treadmill_position:
            treadmillPositionGlobal.append(vec(treadmill_zone))
        # SET THE WORKERS ZONE VECTOR
        global workersPositionGlobal
        workersPositionGlobal = []
        for worker_position in self.world_workers_positions:
            workersPositionGlobal.append(vec(worker_position))
        # SET THE DELIVERY AS GLOBAL
        global deliveryPositionGlobal
        deliveryPositionGlobal = []
        for delivery_position in self.world_delivery_positions:
            deliveryPositionGlobal.append(vec(delivery_position))
       # SET THE CANT MOVE ZONE VECTOR
        global dontMoveGlobal
        dontMoveGlobal = []
        for dont_move in self.world_dont_move:
            dontMoveGlobal.append(vec(dont_move))
       #* SET THE RECHARGE AND PICKUP ZONE VECTOR
        global rechargePositionGlobal, pickupPositionGlobal
        rechargePositionGlobal = []
        for recharge_position in self.world_recharge_position:
            rechargePositionGlobal.append(vec(recharge_position))
        pickupPositionGlobal = []
        for pickup_position in self.world_pickup_positions:
            pickupPositionGlobal.append(vec(pickup_position))
       # SET THE WORLD CONSTRAINTS VECTOR
        global nodesConstraintsGlobal
        nodesConstraintsGlobal = []
        for nodes_constraints in self.world_nodes_constraints:
            nodesConstraintsGlobal.append(vec(nodes_constraints))
        # CALCULATE THE SCREEN SIZE
        #* The Width will be the grid width size * the cell witdh size
        self.screen_width = self.grid_size_width * self.cell_size_width
        #* The Height will be the grid height size * the cell height size
        self.screen_height = self.grid_size_height * self.cell_size_height
        #* The screen will be a tuple with the previous values W x H
        self.screen = (self.screen_width, self.screen_height)
        # ? Allow acces to the screenSize, and cellSize variables
        # ? as global, Maybe there's a better way to do it?
        global screenSizeWidth, screenSizeHeight, cellSizeWidth, cellSizeHeight, screenSize
        screenSizeWidth = self.screen_width
        screenSizeHeight = self.screen_height
        cellSizeWidth = self.cell_size_width
        cellSizeHeight = self.cell_size_height
        screenSize = self.screen
        # SET ALLOWED CONNECTIONS
        # *Where I can travel? In this case LEFT, RIGHT, TOP, BOTTOM
        self.travel_left = vec(1, 0)
        self.travel_right = vec(-1, 0)
        self.travel_bottom = vec(0, 1)
        self.travel_top = vec(0, -1)
        self.travel_none = vec(0, 0)
        # * In this way the allowed connections are
        self.allowed_connections = [self.travel_left, self.travel_right,
                                    self.travel_bottom, self.travel_top,
                                    self.travel_none]
        # SETUP THE SCREEN
        global screen
        screen = pygame.display.set_mode(screenSize)
        print(
        '''
                         _           _____ _____________ 
                        | |         /  ___|  ___| ___  /
                        | |     __ _\ `--.| |__ | |_/ /
                        | |    / _` |`--. |  __||    / 
                        | |___| (_| /\__/ | |___| |\ \ 
                        \_____/\__,_\____/\____/\_| \_|
                  Laboratory of Systems Engineering and Robotics
                            João Pessoa, PB - Brazil
                            https://laser.ci.ufpb.br/
                        Dev.: https://github.com/Ircbarros
       ===================================================================
                Welcome to the Path Planning Grid World Simulator
            Remember to correctly discretize the grid! If you place a 
             10x10 grid (10m x 10m) with cells of size 100x100, the
               ratio will be 1 px = 1 cm. If you want a 100x100
                 (100m x 100m) grid, the blocks must be 10x10,
                               that is 10px = 1m
                    
             IF YOU DO NOT DISCRETIZE CORRECTLY THE PC WILL FREEEZE!
               (if you put a long grid with long cell values e.g.)
           * TO QUIT THE APPLICATION: PRESS 'ESC' OR EXIT
           * TO DRAWN OR REMOVE A OBSTACLE PRESS THE LEFT MOUSE BUTTON
           * TO SHOW THE OBSTACLES DRAWN: PRESS 'CTRL+S'
           * TO ADD AN OBSTACLE (GREY CELLS) IN THE SCREEN PRESS THE
             LEFT MOUSE BUTTON
           * TO REMOVE AN OBSTACLE (GREY CELLS) IN THE SCREEN PRESS
             THE MOUSE BUTTON AT THE CELL YOU WANT TO REMOVE
           * TO CHANGE THE GOAL POSITION PRESS THE RIGHT MOUSE BUTTON

       ===================================================================
        ''')
        print(f'\nCreating a Grid with Screen Size: {screenSizeWidth}px x {screenSizeHeight}px....\n')
        print(f'The Grid Size Values are: {self.grid_size_width}px x {self.grid_size_height}px\n')
        print(f'The Grid Cell Size Values are: {cellSizeWidth}px x {cellSizeHeight}px\n')
        print('Grid Objects:')
        print('.................')
        print(f'The Obstacles are located in:\n {self.obstaclesPosition}\n')
        print(f'The Recharge Zone are located in:\n {rechargePositionGlobal}\n')
        print(f'The Treadmill Zone are located in:\n {treadmillPositionGlobal}\n')
        print(f'The Workers Zone are located in:\n {workersPositionGlobal}\n')
        print(f'The Delivery Zone are located in:\n {deliveryPositionGlobal}\n')
        print(f'The Pickup Zone are located in:\n {pickupPositionGlobal}\n')
        print(f"The Grid Cells block to robot movement are:\n {dontMoveGlobal}\n")

    def grid_in_bounds(self, node):
        ''' Define the Grid bounds an limits that the robot can visit
        '''
        #* If the robot is inside the bounds of the grid
        if 0 <= node.x < self.grid_size_width and 0 <= node.y < self.grid_size_height:
            #? The function will return TRUE anyway, just to add more readability
            return True

    def is_passable(self, node):
        ''' Define if a Grid Node is passable or if is a obstacle

        Returns:

            True: If the Node is Passable
            False: If Node is Not Passable
        '''
        #* If the node is not in the obstacles vector
        if node not in self.obstaclesPosition:
            #* If the node is not in the treadmill vector
            if node not in treadmillPositionGlobal:
                #* If the node is not in the workers position vector
                if node not in workersPositionGlobal:
                    #* If the node is not in the don't move vector
                    if node not in dontMoveGlobal:
                        #* If the node is in the node constraints
                            if node not in nodesConstraintsGlobal:
                                # TO USE THE BREATH SEARCH ONLY, USE THE FOLLOWS IFs
                                #if node not in rechargePositionGlobal
                                    #if node note in pickupPositionGlobal
                                #? The function will return TRUE anyway,
                                #? just to add more readability
                                return True

    def find_neighbors(self, node):
        '''
        Find the node Neighbors filtering the walls and points outside the grid

        Returns:


            (neigbors) (filter object): The available neighbors node filtered in the
                                        Vec2d (PyGame) format

        '''
        #* The neighbors will be the connections available on that node
        neighbors = [node + connection for connection in self.allowed_connections]
        # FILTER THE NEIGHBORS
        #* Filter the Neigbors points outside the Grid and that are not passable
        neighbors = filter(self.grid_in_bounds, neighbors)    
        neighbors = filter(self.is_passable, neighbors)
        #! DO NOT USE THE PRINT BELLOW! WILL BREAK THE FUNCTION!
        # print(list(neighbors))
        return neighbors
    
    def find_all_neighbors(self, node):
        '''
        Find the all node Neighbors without filtering the grid

        Returns:


            (neigbors) : The available neighbors node in Vec2d (PyGame) format

        '''
        #* The neighbors will be the connections available on that node
        all_neighbors = [node + connection for connection in self.allowed_connections]
        all_neighbors = filter(self.grid_in_bounds, all_neighbors)

        return all_neighbors

    def draw_obstacles(self):
        '''
        Draw the obstacles (pods) in the grid

        Returns:

            The Obstacles cells (Pods) inserted in the Main Screen
        '''
        # For the obstacles in the obstacles zone do
        for obstacle in self.obstaclesPosition:
            # Draw a obstacle as a retancle with format of the cell size
            rect_obstacle = pygame.Rect(obstacle * cellSizeWidth,
                                        (cellSizeWidth-1, cellSizeHeight-1))
            pygame.draw.rect(screen, paint.COLOR_LIGHTGRAY, rect_obstacle)
            self.obstacle_vector = vec(obstacle)
            # THE DIRECTORY WERE THE ROBOT IMG FILE IS LOCATED
            self.icon_dir = os.path.join(os.path.dirname(__file__), '../icons')
            # LOAD THE ROBOT IMG TO THE PYGAME MODULE
            self.obstacle_img = pygame.image.load(os.path.join(self.icon_dir, 'pod.png')).convert_alpha()
            # SCALE THE IMAGE
            self.obstacle_img = pygame.transform.scale(self.obstacle_img, (70,70))
            # FILL THE IMAGE WITH THE BLEND MODULE
            self.start_center = ((self.obstacle_vector.x * cellSizeWidth) + (cellSizeHeight / 2),
                                (self.obstacle_vector.y * cellSizeWidth) + (cellSizeHeight / 2))
            screen.blit(self.obstacle_img,
                        self.obstacle_img.get_rect(center=self.start_center))

    def draw_treadmill_zone(self):
        '''
        Draw the Treadmill zone in the grid

        Returns:

            The Treadmill Zone cells inserted in the Main Screen
        '''
        # For the cells in the treadmill zone do
        for treadmill_zone in treadmillPositionGlobal:
            self.treadmill_vector = vec(treadmill_zone)
            # THE DIRECTORY WERE THE ROBOT IMG FILE IS LOCATED
            self.icon_dir = os.path.join(os.path.dirname(__file__), '../icons')
            # LOAD THE ROBOT IMG TO THE PYGAME MODULE
            self.treadmill_img = pygame.image.load(os.path.join(self.icon_dir,
                                                   'treadmill.png')).convert_alpha()
            # SCALE THE IMAGE
            self.treadmill_img = pygame.transform.scale(self.treadmill_img, (100,100))
            # FILL THE IMAGE WITH THE BLEND MODULE
            self.start_center = ((self.treadmill_vector.x * cellSizeWidth) + (cellSizeHeight / 2),
                                (self.treadmill_vector.y * cellSizeWidth) + (cellSizeHeight / 2))
            screen.blit(self.treadmill_img,
                        self.treadmill_img.get_rect(center=self.start_center))

    def draw_workers_zone(self):
        '''
        Draws the Workers zone in the grid

        Returns:

            The Workers Zone cells inserted in the Main Screen
        '''
        # For the cells in the worker zone do
        for workers_zone in workersPositionGlobal:
            self.workers_vector = vec(workers_zone)
            # Draw a obstacle as a retancle with format of the cell size
            rect_workers = pygame.Rect(workers_zone * cellSizeWidth,
                                       (cellSizeWidth-1, cellSizeHeight-1))
            pygame.draw.rect(screen, paint.COLOR_INDEPENDENCE, rect_workers)
            self.icon_dir = os.path.join(os.path.dirname(__file__), '../icons')
            # LOAD THE ROBOT IMG TO THE PYGAME MODULE
            self.worker_img = pygame.image.load(os.path.join(self.icon_dir, 'worker.png')).convert_alpha()
            # SCALE THE IMAGE
            self.worker_img = pygame.transform.scale(self.worker_img, (50,50))
            # FILL THE IMAGE WITH THE BLEND MODULE
            self.worker_center = ((self.workers_vector.x * cellSizeWidth) + (cellSizeHeight / 2),
                                  (self.workers_vector.y *cellSizeWidth) + (cellSizeHeight / 2))
            screen.blit(self.worker_img,
                        self.worker_img.get_rect(center = self.worker_center))

    def draw_delivery_zone(self):
        '''
        Draw the Delivery zone cells in the grid

        Returns:

            The Delivery Zone cells inserted in the Main Screen
        '''
        # For the cells in the delivery zone do
        for delivery_zone in deliveryPositionGlobal:
            # Draw a obstacle as a retancle with format of the cell size
            rect_delivery = pygame.Rect(delivery_zone * cellSizeWidth,
                                        (cellSizeWidth-1, cellSizeHeight-1))
            pygame.draw.rect(screen, paint.COLOR_CAROLINA, rect_delivery)

    def draw_grid(self):
        '''
        Draws the Grid Cells

        Returns:

            The Grid cells inserted in the Main Screen
        '''
        # FOR X VALUES IN RANGE OF SCREEN WIDTH AND CELL WIDTH DO
        for x in range(0, screenSizeWidth, cellSizeWidth):
            # DRAW A LINE IN THE SCREEN WITH COLOR FROM 0 TO THE HEIGHT
            pygame.draw.line(screen, paint.COLOR_DARKGRAY, (x, 0), (x, screenSizeHeight))
        # FOR X VALUES IN RANGE OF SCREEN HEIGHT AND CELL HEIGHT DO
        for y in range(0, screenSizeHeight, cellSizeHeight):
            # DRAW A LINE IN THE SCREEN WITH COLOR FROM 0 TO THE WIDTH
            pygame.draw.line(screen, paint.COLOR_DARKGRAY, (0, y), (screenSizeWidth, y))

    def draw_recharge_zone(self):
        '''
        Draw the Recharge Zone cells in the grid
    
        Returns:

            The Recharge Zone cells inserted in the Main Screen
        '''
        # For the cells in the pickup zone do
        for recharge_zone in rechargePositionGlobal:
            rect_recharge = pygame.Rect(recharge_zone * cellSizeWidth,
                                        (cellSizeWidth-1, cellSizeHeight-1))
            pygame.draw.rect(screen, paint.COLOR_VERMILION, rect_recharge)
            self.recharge_vector = vec(recharge_zone)
            # THE DIRECTORY WERE THE ROBOT IMG FILE IS LOCATED
            self.icon_dir = os.path.join(os.path.dirname(__file__), '../icons')
            # LOAD THE ROBOT IMG TO THE PYGAME MODULE
            self.recharge_img = pygame.image.load(os.path.join(self.icon_dir,
                                                          'arrow_white.png')).convert_alpha()
            # SCALE THE IMAGE
            self.recharge_img = pygame.transform.scale(self.recharge_img,
                                                       (30,30))
            # FILL THE IMAGE WITH THE BLEND MODULE
            self.start_center = ((self.recharge_vector.x * cellSizeWidth) + (cellSizeHeight / 2),
                                (self.recharge_vector.y * cellSizeWidth) + (cellSizeHeight / 2))
            screen.blit(self.recharge_img,
                        self.recharge_img.get_rect(center=self.start_center))

    def draw_pickup_zone(self):
        '''
        Draw the Pickup Zone cells in the grid
    
        Returns:

            The Pickup Zone cells inserted in the Main Screen
        '''
        # For the cells in the pickup zone do
        for pickup_zone in pickupPositionGlobal:
            # Draw a obstacle as a retancle with format of the cell size
            rect_pickup = pygame.Rect(pickup_zone * cellSizeWidth,
                                      (cellSizeWidth-1, cellSizeHeight-1))
            pygame.draw.rect(screen, paint.COLOR_SOFT_YELLOW, rect_pickup)
            self.pickup_vector = vec(pickup_zone)
            # THE DIRECTORY WERE THE ROBOT IMG FILE IS LOCATED
            self.icon_dir = os.path.join(os.path.dirname(__file__), '../icons')
            # LOAD THE ROBOT IMG TO THE PYGAME MODULE
            self.pickup_img = pygame.image.load(os.path.join(self.icon_dir,
                                                             'arrow_white.png')).convert_alpha()
            # SCALE THE IMAGE
            self.pickup_img = pygame.transform.scale(self.pickup_img,
                                                             (30,30))
            # FILL THE IMAGE WITH THE BLEND MODULE
            self.start_center = ((self.pickup_vector.x * cellSizeWidth) + (cellSizeHeight / 2),
                                (self.pickup_vector.y * cellSizeWidth) + (cellSizeHeight / 2))
            screen.blit(self.pickup_img,
                        self.pickup_img.get_rect(center=self.start_center))

    def draw_dont_move(self):
        '''
        Draw the DON'T MOVE zones cells in the grid

        Returns:

            The Don't Move cells draw and inserted in the Main Screen
        '''
        # For the cells in the delivery zone do
        for dont_move_zone in dontMoveGlobal:
            # Draw a obstacle as a retancle with format of the cell size
            rect_dont_move = pygame.Rect(dont_move_zone * cellSizeWidth,
                                         (cellSizeWidth-1, cellSizeHeight-1))
            pygame.draw.rect(screen, paint.COLOR_PRUSSIAN, rect_dont_move)

    def draw_arrows(self):
        '''
        Draws Arrows in the Grid

        Returns:

            An arrow in the Grid Cell with Defined Direction
        '''
        # THE DIRECTORY WERE THE ARROW IMG FILE IS LOCATED
        self.icon_dir = os.path.join(os.path.dirname(__file__), '../icons')
        # CREATE A GLOBAL DIC WITH THE ARROWS
        self.arrows = {}
        # LOAD THE ARROW IMG TO THE PYGAME MODULE
        self.arrow_img = pygame.image.load(os.path.join(self.icon_dir, 'arrow_black.png')).convert_alpha()
        # SCALE THE IMAGE
        self.arrow_img = pygame.transform.scale(self.arrow_img, (20,20))
        # SET ARROW DIRECTIONS
        #* Since the arrow icon is pointing to right we need the arrows in other directions
        self.arrow_point_up = (1,0)
        self.arrow_point_down = (0,1)
        self.arrow_point_left = (-1,0)
        self.arrow_pont_right = (0,-1)
        # CREATE A LIST WITH THE ARROWS POINTING IN ALL DIRECTIONS
        for direction in [self.arrow_point_up, self.arrow_point_down,
                          self.arrow_point_left, self.arrow_pont_right]:
            # ROTATE THE ARROWS
            self.arrows[direction] = pygame.transform.rotate(self.arrow_img,
                                                             vec(direction).angle_to(vec(self.arrow_point_up)))

    def draw_start(self, start):
        '''
        Adjust the Start Icon and draw in the Grid


        Args:

            start (tuple): The start Position (X,Y)

        Returns:

            The Start icon translated to the PyGame Library and Draw
            at the specified START location
        '''
        # SET THE GOAL
        self.start = vec(start)
        # THE DIRECTORY WERE THE ROBOT IMG FILE IS LOCATED
        self.icon_dir = os.path.join(os.path.dirname(__file__), '../icons')
        # LOAD THE ROBOT IMG TO THE PYGAME MODULE
        self.start_img = pygame.image.load(os.path.join(self.icon_dir, 'start.png')).convert_alpha()
        # SCALE THE IMAGE
        self.start_img = pygame.transform.scale(self.start_img, (50,50))
        # FILL THE IMAGE WITH THE BLEND MODULE
        self.start_img.fill((255, 0, 0, 255), special_flags = pygame.BLEND_RGBA_MULT)
        self.start_center = ((self.start.x * cellSizeWidth) + (cellSizeHeight / 2),
                             (self.start.y *cellSizeWidth) + (cellSizeHeight / 2))
        screen.blit(self.start_img,
                    self.start_img.get_rect(center = self.start_center))

    def draw_goal(self, goal):
        '''
        Adjust the Goals Icon and draw in the Grid

        Args:

            goal (tuple): The goal Position (X,Y)

        Returns:

            The Goal icon translated to the PyGame Library and Draw
            at the specified GOAL location
        '''
        # SET THE GOAL
        self.goal = vec(goal)
        # THE DIRECTORY WERE THE ROBOT IMG FILE IS LOCATED
        self.icon_dir = os.path.join(os.path.dirname(__file__), '../icons')
        # LOAD THE ROBOT IMG TO THE PYGAME MODULE
        self.goal_img = pygame.image.load(os.path.join(self.icon_dir, 'goal.png')).convert_alpha()
        # SCALE THE IMAGE
        self.goal_img = pygame.transform.scale(self.goal_img, (50,50))
        # FILL THE IMAGE WITH THE BLEND MODULE
        #self.goal_img.fill(special_flags = pygame.BLEND_RGBA_MULT)
        self.goal_center = ((self.goal.x * cellSizeWidth) + (cellSizeHeight / 2),
                            (self.goal.y *cellSizeWidth) + (cellSizeHeight / 2))
        screen.blit(self.goal_img,
                    self.goal_img.get_rect(center = self.goal_center))

    def draw_text(self, text, size, color, x, y, align="topleft"):
        '''
        Draws the Text in the Screen using the Pygame Module

        Args:

            text (Pygame function): The text desired
            size (int): The size of the Text
            color (tuple): The Desired Color
            x (int): The X position in the Screen
            y (int): The Y position in the Screen
            align (str): The Alignment Desired

        '''
        self.font = pygame.font.Font(default.font_name, size)
        self.text_surface = self.font.render(text, True, color)
        self.text_rect = self.text_surface.get_rect(**{align: (x, y)})
        screen.blit(self.text_surface, self.text_rect)


class WeightedGrid(WorldGrid):
    '''
    Create a Word Grid in a screen size defined by the user
    or by default using the cells and the grid size with Inheritance
    from the WorldGrid Class, creating then a Weighted Graph. The
    Recharge Zone and the Pickup Zone have more weight than the other
    nodes, and aren't obstacles.

    Attributes:

     (GRID_WIDTH, GRID_HEIGHT) (tuple)
     (CELL_WIDTH, CELL_HEIGHT) (tuple)
     [(OBSTACLE_1_x, OBSTACLE_1_y), ... ,(OBSTACLE_X_x, OBSTACLE_X_y)]     (list)
     [(RECHARGE_1_x, RECHARGE_1_y), ... ,(RECHARGE_X_x, RECHARGE_X_y)]     (list)
     [(TREADMILL_1_x, TREADMILL_1_y), ... ,(TREADMILL_X_x, TREADMILL_X_y)] (list)
     [(WORKERS_1_x, WORKERS_1_y), ... ,(WORKERS_X_x, WORKERS_X_y)]         (list)
     [(DELIVERY_1_x, DELIVERY_1_y), ... ,(DELIVERY_X_x, DELIVERY_X_y)]     (list)
     [(PICKUP_1_x, PICKUP_1_y), ... ,(PICKUP_X_x, PICKUP_X_y)]             (list)
     [(DONT_MOVE_1_x, DONT_MOVE_1_y), ... ,(DONT_MOVE_X_x, DONT_MOVE_X_y)] (list)
    
        Where:

            GRID_WIDTH  (int)
            GRID_HEIGHT (int)
            CELL_WIDTH  (int)
            CELL_HEIGHT (int)
            OBSTACLE_x  (int)
            OBSTACLE_y  (int)
            RECHARGE_x  (int)
            RECHARGE_y  (int)
            TREADMILL_x (int)
            TREADMILL_y (int)
            WORKERS_x   (int)
            WORKERS_y   (int)
            DELIVERY_x  (int)
            DELIVERY_y  (int)
            PICKUP_x    (int)
            PICKUP_y    (int)
            DONT_MOVE_x (int)
            DONT_MOVE_y (int)

    Args:

        (GRID_WIDTH, GRID_SIZE):          An tuple with the desired GRID size
        (CELL_WIDTH, CELL_HEIGHT):        An tuple with the desired CELL size
        (OBSTACLE_X_x, OBSTACLE_X_y):     An tuple with the desired OBSTACLES positions
        (RECHARGE_X_x, RECHARGE_X_y):     An tuple with the desired RECHARGE positions
        (TREADMILL_X_x, TREADMILL_X_y):   An tuple with the desired TREADMILL position
        (WORKERS_X_x, WORKERS_X_y):       An tuple with the desired WORKERS positions
        (DELIVERY_X_x, DELIVERY_X_y):     An tuple with the desired DELIVERY positions
        (PICKUP_X_x, WORKERS_X_y):        An tuple with the desired PICKUP positions
        (DONT_MOVE_X_x, DONT_MOVE_X_y):   An tuple with the desired DON'T MOVE positions

    Vars:

        GRID_WIDTH    = The desired GRID WIDTH
        GRID_HEIGHT   = The desired GRID HEIGHT
        CELL_WIDTH    = The desired CELL WIDTH
        CELL_HEIGHT   = The desired CELL HEIGHT
        OBSTACLE_X_x  = The OBSTACLE location "X" at Grid pos. X  (column)
        OBSTACLE_X_y  = The  OBSTACLE location  "X" at Grid in pos. Y (row)
        RECHARGE_X_x  = The RECHARGE location "X" at Grid pos. X  (column)
        RECHARGE_X_y  = The RECHARGE location  "X" at Grid in pos. Y (row)
        TREADMILL_X_x = The TREADMILL location "X" at Grid pos. X  (column)
        TREADMILL_X_y = The TREADMILL location  "X" at Grid in pos. Y (row)
        WORKERS_X_x   = The WORKERS location "X" at Grid pos. X  (column)
        WORKERS_X_y   = The WORKERS location  "X" at Grid in pos. Y (row)
        DELIVERY_X_x  = The DELIVERY location "X" at Grid pos. X  (column)
        DELIVERY_X_y  = The DELIVERY  location  "X" at Grid in pos. Y (row)
        PICKUP_X_x    = The PICKUP location "X" at Grid pos. X (column)
        PICKUP_X_y    = The PICKUP location  "X" at Grid in pos. Y (row)

    Returns:

        A Weighted Node Grid Graph with Obstacles, Recharge Zone, Treadmill
        Zone, Workers Zone, Delivery Zone with size e height inputed by the
        user.
    '''

    def __init__(self, world_grid_size = (create.GRID_WIDTH, create.GRID_HEIGHT),
                 world_cell_size = (create.CELL_WIDTH, create.CELL_HEIGHT),
                 world_obstacles_positions = create.OBSTACLES,
                 world_recharge_positions = create.RECHARGE_ZONE,
                 world_treadmill_positions = create.TREADMILL_ZONE,
                 world_workers_positions = create.WORKERS_POS,
                 world_delivery_positions = create.DELIVERY_ZONE,
                 world_pickup_positions = create.PICKUP_ZONE,
                 world_dont_move = create.DONT_MOVE,
                 world_nodes_constraints = create.NODES_CONSTRAINTS):
 
        super().__init__()
        # CREATE A DIC TO SAVE THE NODE WEIGHTS
        self.weights = {}
        # CREATE WEIGHTS FOR THE RECHARGE ZONE
        for Recharge in self.world_recharge_position:
            self.weights[Recharge] = 40
        # CREATE WEIGHTS FOR THE PICKUP ZONE
        for Pickup in self.world_pickup_positions:
            self.weights[Pickup] = 40

    # COST FUNCTION
    def cost(self, from_node, to_node):
        '''
        The cost function responsible to put weights at each grid cell
        node.

        Args:

            from_node (tuple)
            to_node (tuple)
        
        Vars:

            from_node: A tuple with the actual node cell position (X,Y)
            to_node: A tuple with the desired node cell position (X,Y)


        Returns:

            The cell node weighted in 10 if moving horizontal/vertical
            and 14 if moving diagonal
        '''
        #* If the move is horizontal or vertical the cost is 1
        #? Uses the Length Squared function of PyGame to return the
        #? Euclidean length of the vector
        if (vec(to_node) - vec(from_node)).length_squared() == 1:
            #* Multiply to 10 to work only with integers
            return self.weights.get(to_node, 0) + 10
        #* If the move is diagonal, the cost is 1.4 (diagonal move in squares)
        else:
            #* Multiply to 10 to work only with integers
            return self.weights.get(to_node, 0) + 14


class PriorityQueue:
    '''
    Class responsible to create de Priority Queue using the heapq
    library to elaborate the path planning algorithm based on the
    cells with lower cost values

    '''
    def __init__(self):
        #* Init with a empty list of nodes
        self.nodes = []
    
    def put(self, node, cost):
        '''
        Function responsible to insert a node with a cost to the
        searched nodes

        Args:

            node (tuple)
            cost (int)
        
        Vars:

            node: A tuple with the actual node cell position (X,Y)
            cost: A integer with the actual cell cost value


        Returns:

            The cell node weighted in 10 if moving horizontal/vertical
            and 14 if moving diagonal inserted to the heapq node list
        '''
        #* Put the nodes and the cost at the Heap Queue
        heapq.heappush(self.nodes, (cost, node))
    
    def get(self):
        '''
        Function responsible to get a node with his cost

        Returns:

            The cell node cost
        '''
        #* Get the node in the Priority Queue
        return heapq.heappop(self.nodes)[1]

    def empty(self):
        '''
        Function responsible to check if the search is finished
        and that's no more frontier nodes available

        Returns:

            True if the search finishes
        '''
        #* Check if the search ends
        return len(self.nodes) == 0


class SingleRobot(pygame.sprite.Sprite):
    '''
    Adjust the Robot Icon and draw in the Grid as a Sprite

    Args:

        start (vector2d): The Robot start node position (X , Y)
                          in the Vector2d PyGame format
        goal (vector2d):  The Robot goal position (X, Y) in the
                          Vector2d PyGame format
        path (vector2d):  The Shortest Path the robot will run

    Returns:

        The Robot icon translated to the PyGame Library and Draw
        at the specified START location
    '''
    # The Sprite for the Robots
    def __init__(self, start, goal, path):

        pygame.sprite.Sprite.__init__(self)
        # THE PATH VARIABLES
        self.path = path
        self.start = start
        #* The Start used to Move the Robot
        self.start_pos = (int(start.x), int(start.y))
        self.goal_pos = goal
        # THE DIRECTORY WERE THE ROBOT IMG FILE IS LOCATED
        self.icon_dir = os.path.join(os.path.dirname(__file__), '../icons')
        # LOAD THE ROBOT IMG TO THE PYGAME MODULE
        self.image = pygame.image.load(os.path.join(self.icon_dir, 'robot.png')).convert_alpha()
        # SCALE THE IMAGE
        self.image = pygame.transform.scale(self.image, (50,50))
        # DRAWS A RECTANGLE FOR THE FIGURE
        self.rect = self.image.get_rect()
        # FILL THE IMAGE WITH THE BLEND MODULE
        self.rect.center = ((self.start.x * cellSizeWidth) + (cellSizeHeight / 2),
                            (self.start.y * cellSizeWidth) + (cellSizeHeight / 2))
        # SET THE ROTATION
        self.rotation = 0
        # SET THE SPEED
        self.x_speed = create.MAX_VELOCITY
        self.y_speed = create.MAX_VELOCITY
        # SET THE PATH HEAP WHERE THE ROBOT WILL MOVE
        self.robot_path_pop = deque([])
        # SET the PATH for the Robot as Global
        global robotPath, pathVector
        robotPath = deque([])
        pathVector = deque([])
        #* Where I can travel? In this case LEFT, RIGHT, TOP, BOTTOM
        self.move_left = vec(1, 0)
        self.move_bottom = vec(0, 1)
        self.move_right = vec(-1, 0)
        self.move_top = vec(0, -1)
        #* Set the Current position on the Node
        self.current_pos = self.path[(self.start_pos)] + self.start
        robotPath.extend([self.current_pos])
        path_vector_start = self.current_pos - self.start
        pathVector.extend([path_vector_start])
        while self.current_pos != self.goal_pos:
            self.path_vector = self.path[(self.current_pos.x, self.current_pos.y)]
            pathVector.extend([self.path_vector])
            self.current_pos = self.current_pos + self.path[(int(self.current_pos.x),
                                                             int(self.current_pos.y))]
            robotPath.extend([self.current_pos])         

        print(f'The robot path was the nodes: {list(robotPath)}\n')
        print(f'The robot movements desired is: {list(pathVector)}\n')

    def update(self):
        '''
        Update the Robot Location at every Frame when SPACE is Pressed

        Args:

            None
        
        Returns:

            The Robot icon translated to the PyGame Library and Draw
            at the Screen moving to START to GOAL along the shortest
            path found by the Search Algorithm.
        '''
        try:
            #* Pick the Path Vector 
            vector = pathVector.popleft()
            atual_pos = robotPath.popleft()
            print(f'The Current Robot Position is: {atual_pos}\n')
            #* Move to Left
            if (vector.x == self.move_left.x) and (vector.y == self.move_left.y):

                self.rect.x += create.MAX_VELOCITY
            #* Move to Right
            elif (vector.x == self.move_right.x) and (vector.y == self.move_right.y):

                self.rect.x -= create.MAX_VELOCITY
            #* Move to Top
            elif (vector.x == self.move_top.x) and (vector.y == self.move_top.y):

                self.rect.y -= create.MAX_VELOCITY
            #* Move to Bottom
            else:

                self.rect.y += create.MAX_VELOCITY
        #* Finish the Animation
        except:
            print('------------------')
            print('Animation Finised!')
            print('------------------\n')
            print('To RESTART change the robot start or goal position and press SPACE\n')
            print('To QUIT close the Screen\n')


global allPathsVectors, allRobotsPaths, temporaryPath
allPathsVectors = deque([])
allRobotsPaths = deque([])
temporaryPath = deque()

class MultiRobot(pygame.sprite.Sprite):
    '''
    Adjust the Robots Icons and draw in the Grid as a Sprite

    Args:

        start (vector2d):  The Robot start node position (X , Y)
                           in the Vector2d PyGame format
        goal (vector2d):   The Robot goal position (X, Y) in the
                           Vector2d PyGame format
        path (vector2d):   The Shortest Path the robot will run
        robot_count (int): The atual robot number

    Returns:

        The Robot icon translated to the PyGame Library and Draw
        at the specified START location
    '''
    # The Sprite for the Robots

    def __init__(self, start, goal, path):

        pygame.sprite.Sprite.__init__(self)
        print("\n__________________ STARTING A NEW ROBOT __________________\n")  
        # THE MULTI ROBOT VARIABLES
        self.start = start
        self.start_pos = (int(start.x), int(start.y))
        self.goal_pos = goal
        self.path = path
        print(f'Robot Start Position: {self.start}')
        print(f'Robot Goal Position: {self.goal_pos}')
        print(f'Robot Path: {self.path}')
        # THE DIRECTORY WERE THE ROBOT IMG FILE IS LOCATED
        self.icon_dir = os.path.join(os.path.dirname(__file__), '../icons')
        # LOAD THE ROBOT IMG TO THE PYGAME MODULE
        self.image = pygame.image.load(os.path.join(self.icon_dir, 'robot.png')).convert_alpha()
        # SCALE THE IMAGE
        self.image = pygame.transform.scale(self.image, (50,50))
        # DRAWS A RECTANGLE FOR THE FIGURE
        self.rect = self.image.get_rect()
        # FILL THE IMAGE WITH THE BLEND MODULE
        self.rect.center = ((self.start.x * cellSizeWidth) + (cellSizeHeight / 2),
                            (self.start.y * cellSizeWidth) + (cellSizeHeight / 2))
        # SET THE ROTATION
        self.rotation = 0
        # SET THE ACCELERATION AND VELOCITY
        self.vel = vec(create.MAX_VELOCITY).rotate(0)
        self.acc = vec(0,0)
        # BOOLEAN TURN CHECK
        self.turn_left = False
        self.turn_right = False
        self.turn_bottom = False
        self.turn_top = True
        # SET the PATH and the PATH VECTOR as Global
        global robotPath, pathVector
        robotPath = deque([])
        pathVector = deque([])
        #* Where I can travel? In this case LEFT, RIGHT, TOP, BOTTOM
        self.move_left = vec(1, 0)
        self.move_bottom = vec(0, 1)
        self.move_right = vec(-1, 0)
        self.move_top = vec(0, -1)
        #self.wait = vec(0, 0)
        #* Set the Current position on the Node
        self.current_pos = self.path[(self.start_pos)] + self.start
        robotPath.extend([self.current_pos])
        path_vector_start = self.current_pos - self.start
        pathVector.extend([path_vector_start])
        while self.current_pos != self.goal_pos:
            self.path_vector = self.path[(self.current_pos.x, self.current_pos.y)]
            pathVector.extend([self.path_vector])
            self.current_pos = self.current_pos + self.path[(int(self.current_pos.x),
                                                             int(self.current_pos.y))]
            robotPath.extend([self.current_pos])
        # PUT THE PATH AND VECTORS AS GLOBAL
        allPathsVectors.extend([list(pathVector)])
        allRobotsPaths.extend([list(robotPath)])
        temporaryPath.append(list(robotPath))
        print(f'The robot path was the nodes: {list(robotPath)}')
        print(f'The robot movements desired is: {list(pathVector)}\n')


    def update(self, robots, treadmill, worldGrid):
        '''
        Update the Robot Location at every Frame when SPACE is Pressed

        Args:

            None
        
        Returns:

            The Robot icon translated to the PyGame Library and Draw
            at the Screen moving to START to GOAL along the shortest
            path found by the Search Algorithm.
        '''
        # INICIO DA INSTANCIA DO RELÓGIO
        clock = pygame.time.Clock()
        dt = clock.tick(default.FPS)
        #* The Robot Paths
        robot_path_poped = allRobotsPaths.popleft()
        print(f'Robot Path: {robot_path_poped}\n')
        #* The Vector Movement Desired
        path_poped = allPathsVectors.popleft()
        print(f'\nRobot Path Movement: {path_poped}')

        def run():
            pygame.event.clear()
            path_iterator = iter(robot_path_poped)
            for vector in path_poped:
                pygame.event.pump()
                current_pos = vec(next(path_iterator, 'Goal Reached'))
                #print(f'\nThe Current Robot Movements is: {vector}')
                #print(f'The Current Robot Position is: {current_pos}\n')
                robots.draw(screen)
                vec_pos = current_pos
                #* Draws the Area were the Robots look for Boids
                #pygame.draw.rect(screen, paint.COLOR_GREEN, [(vec_pos.x-2)*100,
                                 #(vec_pos.y-2)*100, 500, 500], 0)
                #* Move to Left
                if (vector.x == self.move_left.x) and (vector.y == self.move_left.y):
                    #* Checks if the Turn was alredy Made
                    if not self.turn_left:
                        #* If not rotate
                        self.image = pygame.transform.rotate(self.image, -90)
                        self.turn_left = True
                        self.turn_right = False
                        self.turn_bottom = False
                        self.turn_top = False
                        #* Now move
                        self.rect.x += create.MAX_VELOCITY
                    else:
                        self.rect.x += create.MAX_VELOCITY
                #* Move to Right
                elif (vector.x == self.move_right.x) and (vector.y == self.move_right.y):
                    #* Checks if the Turn was alredy Made
                    if not self.turn_right:
                        #* If not rotate
                        self.image = pygame.transform.rotate(self.image, 90)
                        self.turn_right = True
                        self.turn_left = False
                        self.turn_bottom = False
                        self.turn_top = False
                        #* Now move
                        self.rect.x += create.MAX_VELOCITY
                    else:
                        self.rect.x -= create.MAX_VELOCITY

                #* Move to Top
                elif (vector.x == self.move_top.x) and (vector.y == self.move_top.y):
                    #* Checks if the Turn was alredy Made
                    if not self.turn_top:
                        #* If not rotate
                        self.image = pygame.transform.rotate(self.image, 90)
                        self.turn_right = True
                        self.turn_left = False
                        self.turn_bottom = False
                        self.turn_top = True
                        #* Now move
                        self.rect.y -= create.MAX_VELOCITY
                    else:
                        self.rect.y -= create.MAX_VELOCITY

                #* Move to Bottom
                else:
                    if not self.turn_bottom:
                        #* If not rotate
                        self.image = pygame.transform.rotate(self.image, 180)
                        self.turn_right = True
                        self.turn_left = False
                        self.turn_bottom = True
                        self.turn_top = False
                        #* Now move
                        self.rect.y += create.MAX_VELOCITY
                    else:
                        self.rect.y += create.MAX_VELOCITY
                
                time.sleep(0.5)
            
        # RUN THREADED
        thread = concurrent.futures.ThreadPoolExecutor(max_workers=5)
        thread.submit(run)


global boidsPathsVectors, boidsPaths
boidsPathsVectors = deque([])
boidsPaths = deque([])
class Boids(pygame.sprite.Sprite):

    def __init__(self, start, goal, path):
        pygame.sprite.Sprite.__init__(self)
        print("\n__________________ STARTING A NEW ROBOT __________________\n")
        # THE MULTI ROBOT VARIABLES
        self.start = start
        self.start_pos = (int(start.x), int(start.y))
        self.goal_pos = goal
        self.path = path
        print(f'Robot Start Position: {self.start}')
        print(f'Robot Goal Position: {self.goal_pos}')
        print(f'Robot Path: {self.path}')
        # THE DIRECTORY WERE THE ROBOT IMG FILE IS LOCATED
        self.icon_dir = os.path.join(os.path.dirname(__file__), '../icons')
        # LOAD THE ROBOT IMG TO THE PYGAME MODULE
        self.image = pygame.image.load(os.path.join(self.icon_dir, 'robot.png')).convert_alpha()
        # SCALE THE IMAGE
        self.image = pygame.transform.scale(self.image, (50,50))
        # DRAWS A RECTANGLE FOR THE FIGURE
        self.rect = self.image.get_rect()
        # FILL THE IMAGE WITH THE BLEND MODULE
        self.rect.center = ((self.start.x * cellSizeWidth) + (cellSizeHeight / 2),
                            (self.start.y * cellSizeWidth) + (cellSizeHeight / 2))
        # SET THE ROTATION
        self.rotation = 0
        # SET THE ACCELERATION AND VELOCITY
        self.vel = vec(create.MAX_VELOCITY).rotate(0)
        self.acc = vec(0,0)
        # SET the PATH and the PATH VECTOR as Global
        global robotPath, pathVector
        robotPath = deque([])
        pathVector = deque([])
        #* Where I can travel? In this case LEFT, RIGHT, TOP, BOTTOM
        self.move_left = vec(1, 0)
        self.move_bottom = vec(0, 1)
        self.move_right = vec(-1, 0)
        self.move_top = vec(0, -1)
        # BOOLEAN TURN CHECK
        self.turn_left = False
        self.turn_right = False
        self.turn_bottom = False
        self.turn_top = True
        # SET the PATH and the PATH VECTOR as Global
        global boidPath, boidVector
        boidPath = deque([])
        boidVector = deque([])
        #* Where I can travel? In this case LEFT, RIGHT, TOP, BOTTOM
        self.move_left = vec(1, 0)
        self.move_bottom = vec(0, 1)
        self.move_right = vec(-1, 0)
        self.move_top = vec(0, -1)
        #self.wait = vec(0, 0)
        #* Set the Current position on the Node
        self.current_pos = self.path[(self.start_pos)] + self.start
        robotPath.extend([self.current_pos])
        path_vector_start = self.current_pos - self.start
        pathVector.extend([path_vector_start])
        while self.current_pos != self.goal_pos:
            self.path_vector = self.path[(self.current_pos.x, self.current_pos.y)]
            pathVector.extend([self.path_vector])
            self.current_pos = self.current_pos + self.path[(int(self.current_pos.x),
                                                             int(self.current_pos.y))]
            robotPath.extend([self.current_pos])
        # PUT THE PATH AND VECTORS AS GLOBAL
        boidsPathsVectors.extend([list(pathVector)])
        boidsPaths.extend([list(robotPath)])
        print(f'The robot path was the nodes: {list(robotPath)}')
        print(f'The robot movements desired is: {list(pathVector)}\n')


    def update(self, robots, treadmill, worldGrid):
        '''
        Update the Robot Location at every Frame when SPACE is Pressed

        Args:

            None
        
        Returns:

            The Robot icon translated to the PyGame Library and Draw
            at the Screen moving to START to GOAL along the shortest
            path found by the Search Algorithm.
        '''
        # INICIO DA INSTANCIA DO RELÓGIO
        clock = pygame.time.Clock()
        dt = clock.tick(default.FPS)
        #* The Robot Paths
        boid_path_poped = boidsPaths.popleft()
        print(f'Robot Path: {boid_path_poped}\n')
        #* The Vector Movement Desired
        path_poped = boidsPathsVectors.popleft()
        print(f'\nRobot Path Movement: {path_poped}')

        def run():
            pygame.event.clear()
            path_iterator = iter(boid_path_poped)
            for vector in path_poped:
                pygame.event.pump()
                current_pos = vec(next(path_iterator, 'Goal Reached'))
                #print(f'\nThe Current Robot Movements is: {vector}')
                #print(f'The Current Robot Position is: {current_pos}\n')
                robots.draw(screen)
                vec_pos = current_pos
                #* Draws the Area were the Robots look for Boids
                pygame.draw.rect(screen, paint.COLOR_ORANGE, [(vec_pos.x-2)*100,
                                 (vec_pos.y-2)*100, 500, 500], 0)
                #* Move to Left
                if (vector.x == self.move_left.x) and (vector.y == self.move_left.y):
                    #* Checks if the Turn was alredy Made
                    if not self.turn_left:
                        #* If not rotate
                        self.image = pygame.transform.rotate(self.image, -90)
                        self.turn_left = True
                        self.turn_right = False
                        self.turn_bottom = False
                        self.turn_top = False
                        #* Now move
                        self.rect.x += create.MAX_VELOCITY
                    else:
                        self.rect.x += create.MAX_VELOCITY
                #* Move to Right
                elif (vector.x == self.move_right.x) and (vector.y == self.move_right.y):
                    #* Checks if the Turn was alredy Made
                    if not self.turn_right:
                        #* If not rotate
                        self.image = pygame.transform.rotate(self.image, 90)
                        self.turn_right = True
                        self.turn_left = False
                        self.turn_bottom = False
                        self.turn_top = False
                        #* Now move
                        self.rect.x += create.MAX_VELOCITY
                    else:
                        self.rect.x -= create.MAX_VELOCITY

                #* Move to Top
                elif (vector.x == self.move_top.x) and (vector.y == self.move_top.y):
                    #* Checks if the Turn was alredy Made
                    if not self.turn_top:
                        #* If not rotate
                        self.image = pygame.transform.rotate(self.image, 90)
                        self.turn_right = True
                        self.turn_left = False
                        self.turn_bottom = False
                        self.turn_top = True
                        #* Now move
                        self.rect.y -= create.MAX_VELOCITY
                    else:
                        self.rect.y -= create.MAX_VELOCITY

                #* Move to Bottom
                else:
                    if not self.turn_bottom:
                        #* If not rotate
                        self.image = pygame.transform.rotate(self.image, 180)
                        self.turn_right = True
                        self.turn_left = False
                        self.turn_bottom = True
                        self.turn_top = False
                        #* Now move
                        self.rect.y += create.MAX_VELOCITY
                    else:
                        self.rect.y += create.MAX_VELOCITY
                
                time.sleep(0.5)
            
        # RUN THREADED
        thread = concurrent.futures.ThreadPoolExecutor()
        thread.submit(run)


class TreadmillItems(pygame.sprite.Sprite):
    '''
    Adjust the Treadmill Items and draw in the Grid as a Sprite

    Args:
        items_start (Tuple): The items start position (X, Y) in
                            the treadmill

    Returns:

        The Items icon are translated to the PyGame Library and Drawn
        at the specified START location
    '''
    # The Sprite for the Robots
    def __init__(self, items_start=(11,0)):

        pygame.sprite.Sprite.__init__(self)
    
        # SET THE START NODE
        self.start = vec(items_start)
        # THE DIRECTORY WERE THE ROBOT IMG FILE IS LOCATED
        self.icon_dir = os.path.join(os.path.dirname(__file__), '../icons')
        # LOAD THE ROBOT IMG TO THE PYGAME MODULE
        self.image = pygame.image.load(os.path.join(self.icon_dir, 'box.png')).convert_alpha()
        # SCALE THE IMAGE
        self.image = pygame.transform.scale(self.image, (50,50))
        # DRAWS A RECTANGLE FOR THE FIGURE
        self.rect = self.image.get_rect()
        # FILL THE IMAGE WITH THE BLEND MODULE
        self.rect.center = ((self.start.x * cellSizeWidth) + (cellSizeHeight / 2),
                                (self.start.y * cellSizeWidth) + (cellSizeHeight / 2))
    
    def update(self):

        self.rect.y += 2

        if self.rect.top > screenSizeHeight:
            self.rect.bottom = 0