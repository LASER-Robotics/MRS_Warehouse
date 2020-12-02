#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Author: Italo Barros
Email: ircbarros@pm.me
License: MIT
An grid generator using the PyGame module!

This module is responsible to save the variables used in the Create World
Module. Here you can change the variables to run the world you want!

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

import pygame
from collections import deque

# SET THE DELIVERY ZONE DEQUE
DELIVERY_ZONE_DEQUE = deque([])


class TradGridHorizontal(object):
    '''
    Class responsible to save the World Variables for
    the Bigger simulation
    '''
    def __init__(self):
        #* START POINT
        self.STARTS = [(1,1), (1,7), (4,7), (6,7)]
        #* GOALS POINTS
        self.GOALS = [(9,0), (9, 2), (9,4), (9,6)]
        #* SET the DEFAULT Number of Robots
        self.ROBOTS_QTD = 4
        #* SET the DEFAULT TIME Start and Limit
        self.TIME_LIMIT = 10000
        # #* SET the DEFAULT Grid Width
        self.GRID_WIDTH = 12 # 100
        # #* SET the DEFAULT Grid Height
        self.GRID_HEIGHT = 8 # 50
        # #* SET the DEFAULT Width and Height of the Grid Cells
        self.CELL_WIDTH = 20
        self.CELL_HEIGHT = 20
        #* For a bigger stage use the variables bellow
        # self.GRID_WIDTH = 200
        # self.GRID_HEIGHT = 100
        # self.CELL_WIDTH = 10
        # self.CELL_HEIGHT = 10
        #* For a REALLY heavy large stage use the variables bellow
        # self.GRID_WIDTH = 400
        # self.GRID_HEIGHT = 200
        # self.CELL_WIDTH = 5
        # self.CELL_HEIGHT = 5
        # SET the DEFAULT Margin value
        self.DEFAULT_CELL_MARGIN = 1
        #* SET the DEFAUL VELOCITY
        self.MAX_VELOCITY = self.CELL_WIDTH
        # OBSTACLES AND SOME CELL POSITIONS IN THE GRID
        #! BE CAREFUL AND NOT TO SET CELL VALUE TO MORE THAN ONE VARIABLE!
        #! ALSO CHECK THE FREE SPACE AVAILABLE!
        #* SET the DEFAULT Obstacles (Pods)
        self.OBSTACLES = [(2,1), (3,1), (1,2), (2,2), (3,2),
                          (5,1), (6,1), (7,1), (5,2), (6,2), (7,2),
                          (1,4), (2,4), (3,4), (1,5), (2,5), (3,5),
                          (5,4), (6,4), (7,4), (5,5), (6,5), (7,5)]
        #* SET the DEFAULT Treadmill Zone
        self.TREADMILL_ZONE = []
        for treadmill in range(self.GRID_HEIGHT):
            self.TREADMILL_ZONE.append((self.GRID_WIDTH-1, treadmill))
        #* SET the DEFAULT Workers Zone
        self.WORKERS_POS = []
        for workers in range(0, self.GRID_HEIGHT-1, 2):
            self.WORKERS_POS.append((self.GRID_WIDTH-2, workers))
        #* SET the DEFAULT Delivery Zone
        self.DELIVERY_ZONE = []
        for delivery in range(0, self.GRID_HEIGHT-1, 2):
            self.DELIVERY_ZONE.append((self.GRID_WIDTH-3, delivery))
        #* SET the DEFAULT Recharge Zone Queue
        self.PICKUP_ZONE = []
        for pickup in range(int(self.GRID_WIDTH/2), self.GRID_WIDTH-3):
            self.PICKUP_ZONE.append((pickup, self.GRID_HEIGHT-1))
        #* SET the DEFAULT Pickup Zone Queue (Where the robots fills the empty pods)
        self.RECHARGE_ZONE = []
        for recharge in range(int(self.GRID_WIDTH/2)):
            self.RECHARGE_ZONE.append((recharge, self.GRID_HEIGHT-1))
        #* SET the DEFAUL Cells that the Robot Can't Move
        #? In this case is the Column where the worker is located
        self.DONT_MOVE = []
        for dont_move in range(1, self.GRID_HEIGHT, 2):
            self.DONT_MOVE.append((self.GRID_WIDTH-2, dont_move))
        #* SET THE NODES CONSTRAINTS
        self.NODES_CONSTRAINTS = []
        #* SWARM RADIUS
        self.SWARM_AREA = 200


class TradGridVertical(object):
    '''
    Class responsible to save the World Variables for
    the simulation based at the LaSER available space

    (HERE 1 CELL = 1 M)
    '''
    def __init__(self):
        #* START POINT
        self.START = (6,7)
        #* GOALS POINTS
        self.GOAL = deque([(9,1), (9, 3), (9,5), (9,0), (9,2), (9,4), (9,6), (0, 7), (1, 7), (2, 7), (3, 7), (4, 7),(5, 7), (6, 7), (7, 7), (8, 7), (9,7)])
        #* SET the DEFAULT Number of Robots
        self.ROBOTS_QTD = 3
        self.OBSTACLES = [(0, 1), (0, 2), (0, 3), (0, 4), (0, 5),
                          (2, 1), (2, 2), (2, 3), (2, 4), (2, 5),
                          (3, 1), (3, 2), (3, 3), (3, 4), (3, 5),
                          (5, 1), (5, 2), (5, 3), (5, 5), (5, 4),
                          (6, 1), (6, 2),  (6, 4), (6, 5), (6, 3)]
        #* SET the DEFAULT Number of Robots
        self.ROBOTS_QTD = 3
        #* SET the DEFAULT TIME Start and Limit
        self.TIME_LIMIT = 10000
        # #* SET the DEFAULT Grid Width
        self.GRID_WIDTH = 12
        # #* SET the DEFAULT Grid Height
        self.GRID_HEIGHT = 8
        # #* SET the DEFAULT Width and Height of the Grid Cells
        self.CELL_WIDTH = 20
        self.CELL_HEIGHT = 20
        #* For a bigger stage use the variables bellow
        # self.GRID_WIDTH = 200
        # self.GRID_HEIGHT = 100
        # self.CELL_WIDTH = 10
        # self.CELL_HEIGHT = 10
        #* For a REALLY heavy large stage use the variables bellow
        # self.GRID_WIDTH = 400
        # self.GRID_HEIGHT = 200
        # self.CELL_WIDTH = 5
        # self.CELL_HEIGHT = 5
        # SET the DEFAULT Margin value
        self.DEFAULT_CELL_MARGIN = 1
        #* SET the DEFAUL VELOCITY
        self.MAX_VELOCITY = self.CELL_WIDTH
        # OBSTACLES AND SOME CELL POSITIONS IN THE GRID
        #! BE CAREFUL AND NOT TO SET CELL VALUE TO MORE THAN ONE VARIABLE!
        #! ALSO CHECK THE FREE SPACE AVAILABLE!
        #* SET the DEFAULT Treadmill Zone
        self.TREADMILL_ZONE = []
        for treadmill in range(self.GRID_HEIGHT):
            self.TREADMILL_ZONE.append((self.GRID_WIDTH-1, treadmill))
        #* SET the DEFAULT Workers Zone
        self.WORKERS_POS = []
        for workers in range(0, self.GRID_HEIGHT-1, 2):
            self.WORKERS_POS.append((self.GRID_WIDTH-2, workers))
        #* SET the DEFAULT Delivery Zone
        self.DELIVERY_ZONE = []
        for delivery in range(0, self.GRID_HEIGHT-1, 2):
            self.DELIVERY_ZONE.append((self.GRID_WIDTH-3, delivery))
        #* SET the DEFAULT Recharge Zone Queue
        self.PICKUP_ZONE = []
        for pickup in range(int(self.GRID_WIDTH/2), self.GRID_WIDTH-3):
            self.PICKUP_ZONE.append((pickup, self.GRID_HEIGHT-1))
        #* SET the DEFAULT Pickup Zone Queue (Where the robots fills the empty pods)
        self.RECHARGE_ZONE = []
        for recharge in range(int(self.GRID_WIDTH/2)):
            self.RECHARGE_ZONE.append((recharge, self.GRID_HEIGHT-1))
        #* SET the DEFAUL Cells that the Robot Can't Move
        #? In this case is the Column where the worker is located
        self.DONT_MOVE = []
        for dont_move in range(1, self.GRID_HEIGHT, 2):
            self.DONT_MOVE.append((self.GRID_WIDTH-2, dont_move))
        #* SET THE NODES CONSTRAINTS
        self.NODES_CONSTRAINTS = []
        #* SWARM RADIUS
        self.SWARM_AREA = 200


class FlyingVGrid(object):
    '''
    Class responsible to save the World Variables for
    the simulation based at the LaSER available space

    (HERE 1 CELL = 1 M)
    '''
    def __init__(self):
        #* START POINT
        self.START = (6,7)
        #* GOALS POINTS
        self.GOAL = [(9,0), (9, 2), (9,4), (9,6)]
        #* SET the DEFAULT Number of Robots
        self.ROBOTS_QTD = 3
        self.OBSTACLES = [(1, 0), (1, 1), (1, 3), (1, 4), (1, 5),
                          (3, 0), (3, 2), (3, 3), (3, 4), (3, 5),
                          (5, 0), (5, 1), (5, 3), (5, 4), (5, 5),
                          (7, 0), (7, 1), (7, 2), (7, 4), (7, 5)]
        #* SET the DEFAULT Number of Robots
        self.ROBOTS_QTD = 3
        #* SET the DEFAULT TIME Start and Limit
        self.TIME_LIMIT = 10000
        # #* SET the DEFAULT Grid Width
        self.GRID_WIDTH = 12
        # #* SET the DEFAULT Grid Height
        self.GRID_HEIGHT = 8
        # #* SET the DEFAULT Width and Height of the Grid Cells
        self.CELL_WIDTH = 20
        self.CELL_HEIGHT = 20
        #* For a bigger stage use the variables bellow
        # self.GRID_WIDTH = 200
        # self.GRID_HEIGHT = 100
        # self.CELL_WIDTH = 10
        # self.CELL_HEIGHT = 10
        #* For a REALLY heavy large stage use the variables bellow
        # self.GRID_WIDTH = 400
        # self.GRID_HEIGHT = 200
        # self.CELL_WIDTH = 5
        # self.CELL_HEIGHT = 5
        # SET the DEFAULT Margin value
        self.DEFAULT_CELL_MARGIN = 1
        #* SET the DEFAUL VELOCITY
        self.MAX_VELOCITY = self.CELL_WIDTH
        # OBSTACLES AND SOME CELL POSITIONS IN THE GRID
        #! BE CAREFUL AND NOT TO SET CELL VALUE TO MORE THAN ONE VARIABLE!
        #! ALSO CHECK THE FREE SPACE AVAILABLE!
        #* SET the DEFAULT Treadmill Zone
        self.TREADMILL_ZONE = []
        for treadmill in range(self.GRID_HEIGHT):
            self.TREADMILL_ZONE.append((self.GRID_WIDTH-1, treadmill))
        #* SET the DEFAULT Workers Zone
        self.WORKERS_POS = []
        for workers in range(0, self.GRID_HEIGHT-1, 2):
            self.WORKERS_POS.append((self.GRID_WIDTH-2, workers))
        #* SET the DEFAULT Delivery Zone
        self.DELIVERY_ZONE = []
        for delivery in range(0, self.GRID_HEIGHT-1, 2):
            self.DELIVERY_ZONE.append((self.GRID_WIDTH-3, delivery))
        #* SET the DEFAULT Recharge Zone Queue
        self.PICKUP_ZONE = []
        for pickup in range(int(self.GRID_WIDTH/2), self.GRID_WIDTH-3):
            self.PICKUP_ZONE.append((pickup, self.GRID_HEIGHT-1))
        #* SET the DEFAULT Pickup Zone Queue (Where the robots fills the empty pods)
        self.RECHARGE_ZONE = []
        for recharge in range(int(self.GRID_WIDTH/2)):
            self.RECHARGE_ZONE.append((recharge, self.GRID_HEIGHT-1))
        #* SET the DEFAUL Cells that the Robot Can't Move
        #? In this case is the Column where the worker is located
        self.DONT_MOVE = []
        for dont_move in range(1, self.GRID_HEIGHT, 2):
            self.DONT_MOVE.append((self.GRID_WIDTH-2, dont_move))
        #* SET THE NODES CONSTRAINTS
        self.NODES_CONSTRAINTS = []
        #* SWARM RADIUS
        self.SWARM_AREA = 200


class FishboneGrid(object):
    '''
    Class responsible to save the World Variables for
    the simulation based at the LaSER available space

    (HERE 1 CELL = 1 M)
    '''
    def __init__(self):
        #* START POINT
        self.START = (6,7)
        #* GOALS POINTS
        self.GOAL = deque([(9,1), (9, 3), (9,5), (9,0), (9,2), (9,4), (9,6), (0, 7), (1, 7), (2, 7), (3, 7), (4, 7),(5, 7), (6, 7), (7, 7), (8, 7), (9,7)])
        #* SET the DEFAULT Number of Robots
        self.ROBOTS_QTD = 3
        self.OBSTACLES = [(0, 0), (0, 1), (0, 5), (0, 4),
                          (1, 0), (1, 1), (1, 2), (1, 4), (1, 5),
                          (2, 4), (2, 5),
                          (3, 0), (3, 1), (3, 4), (3, 5),
                          (4, 0), (4, 1), (4, 2), (4, 4), (4, 5),
                          (5, 4), (5, 5),
                          (6, 0), (6, 1), (6, 4), (6, 5),
                          (7, 0), (7, 1), (7, 2), (7, 5)]
        #* SET the DEFAULT Number of Robots
        self.ROBOTS_QTD = 3
        #* SET the DEFAULT TIME Start and Limit
        self.TIME_LIMIT = 10000
        # #* SET the DEFAULT Grid Width
        self.GRID_WIDTH = 12
        # #* SET the DEFAULT Grid Height
        self.GRID_HEIGHT = 8
        # #* SET the DEFAULT Width and Height of the Grid Cells
        self.CELL_WIDTH = 20
        self.CELL_HEIGHT = 20
        #* For a bigger stage use the variables bellow
        # self.GRID_WIDTH = 200
        # self.GRID_HEIGHT = 100
        # self.CELL_WIDTH = 10
        # self.CELL_HEIGHT = 10
        #* For a REALLY heavy large stage use the variables bellow
        # self.GRID_WIDTH = 400
        # self.GRID_HEIGHT = 200
        # self.CELL_WIDTH = 5
        # self.CELL_HEIGHT = 5
        # SET the DEFAULT Margin value
        self.DEFAULT_CELL_MARGIN = 1
        #* SET the DEFAUL VELOCITY
        self.MAX_VELOCITY = self.CELL_WIDTH
        # OBSTACLES AND SOME CELL POSITIONS IN THE GRID
        #! BE CAREFUL AND NOT TO SET CELL VALUE TO MORE THAN ONE VARIABLE!
        #! ALSO CHECK THE FREE SPACE AVAILABLE!
        #* SET the DEFAULT Treadmill Zone
        self.TREADMILL_ZONE = []
        for treadmill in range(self.GRID_HEIGHT):
            self.TREADMILL_ZONE.append((self.GRID_WIDTH-1, treadmill))
        #* SET the DEFAULT Workers Zone
        self.WORKERS_POS = []
        for workers in range(0, self.GRID_HEIGHT-1, 2):
            self.WORKERS_POS.append((self.GRID_WIDTH-2, workers))
        #* SET the DEFAULT Delivery Zone
        self.DELIVERY_ZONE = []
        for delivery in range(0, self.GRID_HEIGHT-1, 2):
            self.DELIVERY_ZONE.append((self.GRID_WIDTH-3, delivery))
        #* SET the DEFAULT Recharge Zone Queue
        self.PICKUP_ZONE = []
        for pickup in range(int(self.GRID_WIDTH/2), self.GRID_WIDTH-3):
            self.PICKUP_ZONE.append((pickup, self.GRID_HEIGHT-1))
        #* SET the DEFAULT Pickup Zone Queue (Where the robots fills the empty pods)
        self.RECHARGE_ZONE = []
        for recharge in range(int(self.GRID_WIDTH/2)):
            self.RECHARGE_ZONE.append((recharge, self.GRID_HEIGHT-1))
        #* SET the DEFAUL Cells that the Robot Can't Move
        #? In this case is the Column where the worker is located
        self.DONT_MOVE = []
        for dont_move in range(1, self.GRID_HEIGHT, 2):
            self.DONT_MOVE.append((self.GRID_WIDTH-2, dont_move))
        #* SET THE NODES CONSTRAINTS
        self.NODES_CONSTRAINTS = []
        #* SWARM RADIUS
        self.SWARM_AREA = 200


class PygameDefaults(object):
    '''
    Class responsible to save some Pygame Variables
    '''
    def __init__(self):
        # DEFINE THE FPS
        self.FPS = 15
        # DEFINE THE FONT NAME
        self.font_name = pygame.font.match_font('dejavusans')


class Colors(object):
    '''
    Class responsible to save the colors used to paint the Screen
    '''
    def __init__(self):
        #? For more RGB Colors go to:
        # https://graf1x.com/shades-of-red-color-palette-hex-rgb-code/
        #? To translate Corors to the Decimal RGB Format go to:
        # https://www.colorhexa.com/

        self.COLOR_BLACK = (0, 0, 0)
        self.COLOR_WHITE = (255, 255, 255)
        self.COLOR_GREEN = (0, 255, 0)
        self.COLOR_RED = (255, 0, 0)
        self.COLOR_CYAN = (0, 255, 255)
        self.COLOR_MAGENTA = (255, 0, 255)
        self.COLOR_YELLOW = (255, 255, 0)
        self.COLOR_YELLOW_ROYAL = (250, 218, 94)
        self.COLOR_ORANGE = (230, 149, 0)
        #* The Recharge Zone Color
        self.COLOR_SOFT_YELLOW = (239, 217, 127)
        self.COLOR_DARKGRAY = (40, 40, 40)
        self.COLOR_MEDGRAY = (75, 75, 75)
        self.COLOR_STATEGRAY = (211,211,211)
        #* The Obstacles Color
        self.COLOR_LIGHTGRAY = (170, 170, 169)
        #* The Workers Zone Color
        self.COLOR_PRUSSIAN = (9, 71, 99)
        #* The Delivery Zone Color
        self.COLOR_CAROLINA = (118, 180, 214)
        #* The Treadmill Zone Color
        self.COLOR_INDEPENDENCE = (64, 90, 155)
        #* The Pickup Zone Color
        self.COLOR_VERMILION = (163, 44, 50)
        #* The Don't Move Zone Color
        self.COLOR_GRAYISH = (226, 230, 230)