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
import math
import copy
import argparse
import pygame
import random
import timeit
from memory_profiler import profile
import numpy as np
import dill
from threading import Thread
from operator import itemgetter
from itertools import islice 
#? Using the collections module since is the most efficient
#? to implement and manipulate a queue list
from collections import deque
#* To see the sum of the system and user CPU Time of
#* of the current process
from time import process_time
from timeit import default_timer as timer
from tabulate import tabulate
#* To see the memory usage in each line
import memory_profiler
#* Import the createWorld Module
from world import createIntegerWorld
from world.settings import settingsIntBigWorld
from spacetime_astar import Spacetime_Astar
# from spacetime_bfs import Spacetime_BFS
# LOAD THE WORLD VARIABLES
#* Horizontal Layout
world = createIntegerWorld.create
# LOAD THE PYGAME DEFAULTS VARIABLES
default = settingsIntBigWorld.PygameDefaults()
# LOAD THE COLORS FROM THE SETTINGS
paint = settingsIntBigWorld.Colors()
# THE SAVED PATHS
DesiredPaths = []
# THE MATRIX INNER CONSTANT
MatrixDimmension = 0
#* Load the PyGame Vector2 lib
vec = pygame.math.Vector2
#* Set the numpy print threshold
np.set_printoptions(threshold=np.inf)
#* Controle de posição do nó
FREE = -1
#* The paths values, used later to translate to pygame,
#* and also to calculate the sum of costs/makespan
MAPFS_PATHS = []


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
    print(f'\nThe All PyGame Node Cells Available are: {visited}\n')
    return visited


def create_adjacency_list(world_width, world_height, world_nodes, world_obstables,
                          world_recharge_nodes, world_pickup_nodes):
    # * The list with all nodes neighbors
    adjacency_list = []
    # * The width will be -2 since the last two are zones that the robots cannot visit
    real_width = world_width - 2
    number_of_nodes = int(real_width * world_height)
    list_of_nodes = set([x for x in range(0, number_of_nodes)])
    # * Convert the world nodes to a dict to find the index pos of obstacles
    world_nodes_indexed = {item: idx for idx, item in enumerate(world_nodes)}
    # * Find the obstacles position in (int) format
    obstacles_in_int = [world_nodes_indexed.get(item) for item in world_obstables]
    # * Find the recharge zone position in (int) format
    recharge_in_int = [world_nodes_indexed.get(item) for item in world_recharge_nodes]
    # * Find the recharge zone position in (int) format
    pickup_in_int = [world_nodes_indexed.get(item) for item in world_pickup_nodes]
    for node in list_of_nodes:
        # * If the node is in recharge zone the possible movements is
        # * only to the upper node
        if node in recharge_in_int:
                upper_node = node - real_width
                bottom_node = None
                left_node = None
                right_node = None
        # * If the node is in pickup zone the possible movements is
        # * only to the upper node
        elif node in pickup_in_int:
                upper_node = node - real_width
                bottom_node = None
                left_node = None
                right_node = None
        # * If is a obstacle we ignore since the robot can only enter in it
        # ? Remove this if we want to move bellow the pods
        elif node in obstacles_in_int:
            # * Get the available moves
            moves = check_nodes_neighbors(node, obstacles_in_int, real_width)
            # * Creates a iterator to return none for empty values
            moves_iterator = iter(moves)
            upper_node = next(moves_iterator, None)
            bottom_node = next(moves_iterator, None)
            left_node = next(moves_iterator, None)
            right_node = next(moves_iterator, None)
        else:
            upper_node = node - real_width
            bottom_node = node + real_width
            #* Checks if the node is in the Left Limits
            if node % real_width == 0:
                left_node = None
                right_node = node + 1
            #* Checks if the node is in the Right Limits
            elif (node + 1) % real_width == 0:
                left_node = node - 1
                right_node = None
            else:
                left_node = node - 1
                right_node = node + 1
        nodes_around_raw = set([upper_node, bottom_node, left_node, right_node])
        #* Removes the undesired nodes (negative numbers or number outside limits)
        real_nodes_around = list(nodes_around_raw & list_of_nodes)
        adjacency_list.append(real_nodes_around)
    
    return adjacency_list, number_of_nodes, world_nodes_indexed

def check_nodes_neighbors(node, obstacles, world_width):
    # * Left, Right, Top, Bottom
    allowed_connections = [node-1, node+1, node-world_width, node+world_width]
    # * The allowed movements are the movements in free space nodes
    allowed_movements = set(allowed_connections) - set(obstacles)

    return allowed_movements


def create_adjacency_matrix(adjacency_list):
    global MatrixDimmension
    # * The MatrixDimmension is the matrix inner dimensions (i, j)
    MatrixDimmension = len(adjacency_list)
    # * Create a empty matrix of NaN values
    adj_matrix = np.nan * np.ones((MatrixDimmension, MatrixDimmension))
    np.fill_diagonal(adj_matrix, 0)

    for i in range(MatrixDimmension):
        for j in adjacency_list[i]:
            adj_matrix[i,j] = 1
    # * Convert NaN values to zero
    adj_matrix[np.isnan(adj_matrix)] = 0

    return adj_matrix.astype(int)

def create_weighted_adjacency_matrix(adjacency_weighted_list):
    """
    adjacency_weighted_list = [{1:0.2,2:0.5},{2:1},{},{0:0.1,1:0.6}]
    """
    # * The n is the matrix inner dimensions (i, j)
    n = len(adjacency_weighted_list)

    # * Create a empty matrix of NaN values
    weighted_adj_matrix = np.nan * np.ones((n, n))
    np.fill_diagonal(weighted_adj_matrix, 0)

    for i in range(n):
        for j, weight in adjacency_weighted_list[i].items():
            weighted_adj_matrix[i,j] = weight
    # * Convert NaN values to zero
    weighted_adj_matrix[np.isnan(weighted_adj_matrix)] = 0

    return weighted_adj_matrix.astype(int)

def create_instance_array(matrix):
    # * Create the array values based on the diagonal (Matrix Dimension-1 to 1)
    reduced_rows = np.arange(MatrixDimmension-1, 0, -1)
    # * Get the Upper Triangular Part
    upper_values = iter(matrix[np.triu_indices(matrix.shape[0], k = 1)]) # offset
    instance_arrays = [list(islice(upper_values, int(elem))) for elem in reduced_rows]

    return instance_arrays

def create_instance_file(starts_pygame, goals_pygame, adjacency_list, instance_arrays, nodes_idx, pygame_relation, agents):
    print('Creating Instance File....\n')
    start_positions = [nodes_idx.get(item) for item in starts_pygame]
    goal_positions = [nodes_idx.get(item) for item in goals_pygame]
    no_of_vertices = len(adjacency_list)
    with open('instance.txt', 'w') as writedfile:
        writedfile.write(f'{world.ROBOTS_QTD} {no_of_vertices}\n')
        for agent in range(0, agents):
            writedfile.write('%s ' % start_positions[agent])
        writedfile.write('\n')
        for agent in range(0, agents):
            writedfile.write('%s ' % goal_positions[agent])        
        writedfile.write('\n')
        x_vectors = [x[0][0] for x in pygame_relation.items()]
        # writedfile.write(tabulate(x_vectors, tablefmt="plain"))
        writedfile.writelines(map(lambda x:str(x)+' ', x_vectors))
        writedfile.write('\n')
        y_vectors = [y[0][1] for y in pygame_relation.items()]
        # writedfile.write(tabulate(y_vectors, tablefmt="plain"))
        writedfile.writelines(map(lambda y:str(y)+' ', y_vectors))
        writedfile.write('\n')
        writedfile.write(tabulate(instance_arrays, tablefmt="plain"))
    print('OK!\n')


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

# * Cria o caminho da origem de agent até seu destino utilizando a heurística STA*
def stastar_search(agent, input_instance, solution_instance, t_init=0, forbidden_vertex=1, agent_start=None, agent_goal=None, swarm=False):
    # * An empty map with all nodes
    came_from = [-1 for x in range(input_instance.no_nodes)]
    is_visited = [False for x in range(input_instance.no_nodes)]
    if swarm:
        start = agent_start
        goal = agent_goal
        # * Se houve conflito, start = local que o agente estava antes do conflito
        if t_init > 0:
            start = solution_instance.space[0][t_init - 1]
    else:
        start = input_instance.start_positions[agent]
        goal = input_instance.goal_positions[agent]
        # * Se houve conflito, start = local que o agente estava antes do conflito
        if t_init > 0:
            start = solution_instance.space[agent][t_init - 1]
    # IMPORT THE QUEUE TO PUT THE NODES
    frontier = createIntegerWorld.PriorityQueue()
    # * Put the nodes on the Frontier with cost 0
    frontier.put(start, 0)
    # * The cost of the cheapest path from start to n currently known.
    max_g_score = 2 * input_instance.no_nodes
    g_score = [max_g_score for x in range(input_instance.no_nodes)]
    g_score[start] = 0
    if t_init > 0:
        g_score[start] = t_init - 1 
    # * Our current best guess
    f_score = [max_g_score for x in range(input_instance.no_nodes)]
    f_score[start] = g_score[start] + input_instance.manhattan_distance(goal, start)
    # * The start is None since is were whe are
    while not frontier.empty():
        #* The next one will be the one with lowest cost
        current = frontier.get()
        #* If the goal is reached break
        if current == goal:
            path = reconstruct_path(start, came_from, goal)
            return path
        for next in input_instance.adj_list[current]:
            if(forbidden_vertex >= 0) and (forbidden_vertex == next):
                continue
            if not solution_instance.is_valid_moviment(agent, current, next, g_score[current]):
                continue
            tentative_g_score = g_score[current] + 1
            if tentative_g_score < g_score[next]:
                came_from[next] = current
                g_score[next] = tentative_g_score
                f_score[next] = tentative_g_score + input_instance.manhattan_distance(goal, next)
                if not is_visited[next]:
                    frontier.put(next, f_score[next])

        is_visited[current] = True
    # * Se não encontrar caminho para goal retorna false
    return None


def cbs_search(input_instance, solution_instance=None, swarm_starts=None, swarm_goals=None, swarm=False):
    if not swarm:
        solution = Solution(input_instance)
        for a in range(input_instance.no_robots):
            path = stastar_search(agent=a, input_instance=input_instance, solution_instance=solution)
            if path != None:
                solution.add_path(a, path)
            else:
                return None
        print(f'\n-------- STA* RESULTS:')
        solution.print()
        print(f'\n-------- CBS STARTED:\n')
        # # * Meu robô atual
        makespan = solution.makespan
        for t in range(makespan+1):
            for a1 in range(input_instance.no_robots - 1):
                for a2 in range(a1 + 1, input_instance.no_robots):
                    # print(f'Makespan 1: {solution.makespan}')
                        # * Pegar minha posição atual e comparar com o do próximo agente
                        if solution.space[a1][t] ==  solution.space[a2][t]:
                            print(f'Node Colision at: {solution.space[a1][t]}')
                            print(f'Time: {t}')
                            print(f'First Agent: {a1}')
                            print(f'Second Agent: {a2}')
                            path = stastar_search(agent=a1, input_instance=input_instance, solution_instance=solution, t_init=t)
                            print(path)
                            if path != None:
                                # * Se eu cheguei primeiro quem deve mudar o caminho é o segundo agente
                                # * caso contrário, quem muda sou eu
                                agent = solution.checks_last_in(a1, a2, t)
                                solution.clear_path(t, agent)
                                solution.append_path(agent, path, t)
                            else:
                                print('Cannot find solution!')
                                return None
                            break

        print(f'\n-------- CBS RESULTS:')
        solution.print_last_cbs()
    else:
        solution = solution_instance
        solution.input.no_robots = len(swarm_starts)
        solution.input.start_positions = swarm_starts
        solution.input.goal_positions = swarm_goals
        solution.space =  [[FREE for t in range(solution.input.max_sim_time)] for a in range(solution.input.no_robots)]
        solution.agent_makespan = [[0] for a in range(solution.input.no_robots)]

        for a in range(solution.input.no_robots):
            path = stastar_search(agent=a, input_instance=input_instance, solution_instance=solution)
            if path != None:
                solution.add_path(a, path)
            else:
                return None
        print(f'\n-------- STA* RESULTS:')
        solution.print()
        print(f'\n-------- CBS STARTED:\n')
        # # * Meu robô atual
        makespan = solution.makespan
        for t in range(makespan+1):
            for a1 in range(input_instance.no_robots - 1):
                for a2 in range(a1 + 1, input_instance.no_robots):
                    # print(f'Makespan 1: {solution.makespan}')
                        # * Pegar minha posição atual e comparar com o do próximo agente
                        if solution.space[a1][t] ==  solution.space[a2][t]:
                            print(f'Node Colision at: {solution.space[a1][t]}')
                            print(f'Time: {t}')
                            print(f'First Agent: {a1}')
                            print(f'Second Agent: {a2}')
                            path = stastar_search(agent=a1, input_instance=input_instance, solution_instance=solution, t_init=t)
                            print(path)
                            if path != None:
                                # * Se eu cheguei primeiro quem deve mudar o caminho é o segundo agente
                                # * caso contrário, quem muda sou eu
                                agent = solution.checks_last_in(a1, a2, t)
                                solution.clear_path(t, agent)
                                solution.append_path(agent, path, t)
                            else:
                                print('--------------------------------------')
                                raise ValueError('Cannot find solution!')
                            break

        print(f'\n-------- CBS RESULTS:')
        solution.print_last_cbs()

def reconstruct_path(start, came_from, current):
    total_path = [current]
    while current != start:
        current = came_from[current]
        total_path.insert(0, current)
    return total_path

class Swarm:
    def __init__(self, input_instance, world_nodes):
        print(f'\n-------- MAPF-S STARTED:\n')
        self.input_instance = input_instance
        self.solutions_inst = []
        self.swarm_zones = {}
        self.swarm_groups_starts = []
        self.swarm_groups_goals = []                                                                                    
        self.run_swarms(input_instance, world_nodes, swarm_count=0)
        self.run_planning(input_instance)

    def run_swarms(self, input_instance, world_nodes, swarm_count=0):
        for a in range(input_instance.no_robots):
            try:
                agent_position = input_instance.start_positions[a]
                swarm_key = 'SWARM_' + str(swarm_count)
                print(f'> Starting {swarm_key} zone:')
                print(f'Agent: {a}')
                print(f'Start Position: {agent_position}')
                print('Checking if agent is inside another swarm...')
                is_inside = any(agent_position in values for values in self.swarm_zones.values())
                # if any(agent_position in values for values in self.swarm_zones.values()):
                if is_inside:
                    del is_inside
                    print('Agent is inside another swarm!')
                    #* Check in with Swarm the agent_position was Found
                    keys = [key for key, value in self.swarm_zones.items() if agent_position in value]
                    print('\nFound a new Robot neighboors inside a known Swarm, '+
                            'including the {} zone into {} zone...\n'.format(swarm_key, keys))
                    swarm_key = str(keys[0])
                    new_swarm, nodes_list = self.create_swarm(a, agent_position, swarm_key, world_nodes)
                    new_zone_swarm = (set(new_swarm) & nodes_list) ^ set(self.swarm_zones[swarm_key])
                    self.swarm_zones.update({swarm_key:list(new_zone_swarm)})
                    # * Add this agent to the desired group
                    agent_group = int(swarm_key.rpartition('_')[-1])
                    self.swarm_groups_starts[agent_group].append(agent_position)
                    self.swarm_groups_goals[agent_group].append(input_instance.goal_positions[a])
                    
                    del agent_group, new_zone_swarm, keys

                else:
                    print('Agent is alone, creating another swarm...')
                    new_swarm, nodes_list = self.create_swarm(a, agent_position, swarm_key, world_nodes)
                    # * We need to eliminate the nodes that are not present on the grid
                    print('Deleting nodes not present on grid...')
                    new_zone = list(set(new_swarm) & nodes_list)
                    print(f'Updated Swarm Zone: {new_zone}\n')
                    self.swarm_zones.update({swarm_key:new_zone})
                    self.swarm_groups_starts.append([agent_position])
                    self.swarm_groups_goals.append([input_instance.goal_positions[a]])
                    self.solutions_inst.append(Solution(input_instance))
                    # pickle_filename = ''.join([swarm_key, '.pickle'])
                    # dill.dump(Solution(input_instance),file=open(pickle_filename, 'wb'))
                    swarm_count += 1
            except ValueError:
                    print(f'Was not possible to perform MAPF-S to agent: {a}, Swarm: {swarm_count}')
        print(f'\n-------- SWARM Zones:\n')
        print(self.swarm_zones)
        print(f'\n-------- SWARM Groups (By Start):\n')
        print(self.swarm_groups_starts)
        print(f'\n-------- SWARM Groups (By Goals):\n')
        print(self.swarm_groups_goals)
        # print(f'\n-------- Agent instance at:\n')
        # print(self.solutions_inst)

    def run_planning(self, input_instance):
        for id, swarm_pos in enumerate(self.swarm_groups_starts):
            # * Loads the solution
            # solution_filename = ''.join(['SWARM_', str(id), '.pickle'])
            # solution_reloaded = dill.load(open(solution_filename, 'rb'))
            # * If int, the agent is alone, run STA*
            if len(swarm_pos) == 1:
                #TODO: Run single agent using STA* or join single agents and run?
                path = stastar_search(agent=0, input_instance=input_instance, solution_instance=self.solutions_inst[id], agent_start=self.swarm_groups_starts[id][0], agent_goal=self.swarm_groups_goals[id][0], swarm=True)
                if path != None:
                    global MAPFS_PATHS
                    MAPFS_PATHS.append(path)
                else:
                    return None
            # * The agent is inside a group, use CBS
            else:
                    # st_star = Spacetime_Astar(createInputs)
                    # sol = st_star.multi_start(100)
                    # if(sol != None):
                    #     sol.print()
                    #     st_star.local_search(sol)
                    #     sol.print()
                cbs_search(input_instance, solution_instance=self.solutions_inst[id], swarm_starts=self.swarm_groups_starts[id], swarm_goals=self.swarm_groups_goals[id], swarm=True)

    def create_swarm(self, agent, position, swarm_key, world_nodes, world_width=world.GRID_WIDTH, world_height=world.GRID_HEIGHT,):
        """
        A function responsible to update the Swarm Hash Table.
        
        The Function will calculate the neighboors cells around the robot
        and update the Swarm Hash table.

        
        Args:
            
            (position)      (int) : The agent position on the graph
            (swarm_key)     (str) : The swarm value the agent is inserted
            (world_width)   (int) : The world width desired by the user
            (world_height)  (int) : The world height desired by the user
        
        Returns:
            
            The Swarms Neighboorhoods in the Grid

        """
        # * The width will be -2 since the last two are zones that the robots cannot visit
        real_width = world_width - 2
        number_of_nodes = int(real_width * world_height)
        list_of_nodes = set([x for x in range(0, number_of_nodes)])
        # * Our top swarm nodes will be the 2 nodes upper
        swarm_cell_top_1, swarm_cell_top_2 = [position - real_width], [position - (real_width * 2)]
        # * Our top swarm nodes will be the 2 nodes down
        swarm_celll_bottom_1, swarm_celll_bottom_2 = [position + real_width], [position + (real_width * 2)]
        # * Used to not include the swarm number bellow this value
        left_boundaries = [x for x in range(0, number_of_nodes, real_width)]
        # * Used to not include the swarm number after this value
        right_boundaries = [x + (real_width - 1) for x in left_boundaries]
        if position in left_boundaries:
            # * Include only the two cells on the right
            swarm_cells_sides = [position + 1, position + 2]
            swarm_cell_diagonals = [position - real_width + 1, position + (real_width + 1)]
        elif position in right_boundaries:
            # * Include only the two cells on the left
            swarm_cells_sides = [position - 1, position - 2]
            swarm_cell_diagonals = [position - real_width - 1]
        # * If position is 1 node far way left boundaries
        elif (position - 1) in left_boundaries:
            # * Include only 1 cell on left, and two cells on right
            swarm_cells_sides = [position - 1, position + 1, position + 2]
            swarm_cell_diagonals = [position - real_width - 1,
                                    position - real_width + 1,
                                    position + real_width -1,
                                    position + real_width + 1,
                                    ]
        elif (position + 1) in right_boundaries:
            # * Include only 1 cell on right, and two cells on right
            swarm_cells_sides = [position + 1, position - 1, position - 2]
            swarm_cell_diagonals = [position - real_width - 1,
                                    position - real_width + 1,
                                    position + real_width -1,
                                    position + real_width + 1,
                                    ]
        else:
            # * Include both cells on right and left
            swarm_cells_sides = [position + 1, position + 2, position - 1, position - 2]
            swarm_cell_diagonals = [position - real_width - 1,
                                    position - real_width + 1,
                                    position + real_width -1,
                                    position + real_width + 1,
                                    ]

        # * Our swarm zone in raw format (containing nodes not found on grid)
        swarm_zone_raw = [*swarm_cell_top_1, *swarm_cell_top_2,
                        *swarm_celll_bottom_1, *swarm_celll_bottom_2,
                        *swarm_cell_diagonals,
                        *swarm_cells_sides]
        print(f'Agent Swarm: {swarm_zone_raw}')
        
        return swarm_zone_raw, list_of_nodes


class Solution:
    def __init__(self, inputs):
        self.input = inputs
        self.space = [[FREE for t in range(self.input.max_sim_time)] for a in range(self.input.no_robots)]
        self.makespan = 0
        self.agent_makespan = [[0] for a in range(self.input.no_robots)]
        self.agent_max_makespan = None
        self.agent = [[FREE for t in range(self.input.max_sim_time)] for a in range(self.input.no_nodes)]

    def add_path(self, agent, path):
        t = 0
        for v in path:
            self.space[agent][t] = v
            t += 1
        # * Colocando a última posição após o tempo final para sempre
        for ta in range(t, self.input.max_sim_time):
            self.space[agent][ta] = self.input.goal_positions[agent]
        for tb in range(self.input.max_sim_time):
            # * Descobrindo a localização do agente
            v = self.space[agent][tb]
            # * Definindo qual agente está em cada posição em cada tempo
            self.agent[v][tb] = agent
        # * Para garantir que tenha o tempo final
        t = t - 1
        self.agent_makespan[agent] = t
        if t > self.makespan:
            self.makespan = t
            self.agent_max_makespan = agent
    
    def append_path(self, agent, path, t_append):
    
        # * O tempo em que irá ser dado o append
        t = t_append - 1
        print(f'New Path: {path}\n')
        for v in path:
            self.space[agent][t] = v
            t += 1
        for ta in range(t, self.input.max_sim_time):
            self.space[agent][ta] = self.input.goal_positions[agent]
        for tb in range(t_append, self.input.max_sim_time):
            # * Descobrindo a localização do agente
            v = self.space[agent][tb]
            # * Definindo qual agente está em cada posição em cada tempo
            self.agent[v][tb] = agent
        # * Para garantir que tenha o tempo final
        t = t - 1
        self.agent_makespan[agent] = t
        self.makespan = 0
        
        for a in range(self.input.no_robots):

            if self.agent_makespan[a] > self.makespan:
                self.makespan = self.agent_makespan[a]
                self.agent_max_makespan = a
    
    def clear_path(self, t_init, agent):
        for t in range(t_init, self.input.max_sim_time):
            v = self.space[agent][t]
            if v != FREE:
                self.space[agent][t] = FREE
                # * Se o agente for o próprio agente, ignora, se não apaga o espaço
                if self.agent[v][t] == agent:
                    # * Informando que existe um agente nesta pos
                    self.agent[v][t] = FREE
            else:
                break

    def checks_last_in(self, agent_1, agent_2, time):
        # * Encontra o tempo mínimo onde o agente 1 chegou ao goal
        agent_1_t = self.space[agent_1].index(self.space[agent_1][time])
        # * Encontra o tempo mínimo onde o agente 2 chegou ao goal
        agent_2_t = self.space[agent_2].index(self.space[agent_2][time])
        # * Se o agent 1 chegou primeiro, quem deve mudar o caminho é o 2
        if agent_1_t <= agent_2_t:
            return agent_2
        # * Se não quem deve mudar é o 1
        else:
            return agent_1

    def is_valid_moviment(self, agent, v, u, t):
        # * Pegando o agente do futuro que está na posição vizinha
        a_next_future = self.agent[u][t + 1]
        # * Pegando o agente do passado que está na posição vizinha
        a_next_past = self.agent[u][t]
        
        # * Olhando se no futuro vai ter alguém na minha posição
        #print("Tentando", next,  "Futuro", a_next_future, "Passado", a_next_past)
        if (a_next_future != FREE) and (a_next_future != agent):
            return False
        # * Qual o local que o agente na posição do vizinho vai para o futuro
        if self.space[a_next_past][t + 1] == v:
            return False
        else:
            return True

    def print(self):
        print(f'\nThe Makespan value is {self.makespan}')
        for a in range(self.input.no_robots):
            string = ''
            string = ''.join([string,'{'])
            for t in range(self.makespan+1):
                string = ''.join([string, str(self.space[a][t]), ' '])
            string = ''.join([string,'}'])
            print(string)

    def print_last_cbs(self):
        for a in range(self.input.no_robots):
            tmp_path = []
            string = ''
            string = ''.join([string,'{'])
            for t in range(self.makespan+1):
                string = ''.join([string, str(self.space[a][t]), ' '])
                tmp_path.append(self.space[a][t])
            global MAPFS_PATHS
            MAPFS_PATHS.append(tmp_path)
            string = ''.join([string,'}'])
            print(string)



class Inputs:
    def __init__(self, no_nodes, no_robots, adj_matrix, adj_list, origin_vectors, goal_vectors, nodes_idx, pygame_relation):
        self.max_sim_time = int(no_robots * no_nodes)
        self.no_nodes = no_nodes
        self.no_robots = no_robots
        self.adj_matrix = adj_matrix
        self.adj_list = adj_list
        self.nodes_idx = nodes_idx
        self.pygame_relation = pygame_relation
        print('\n---- CREATING INPUT STRUCTURE:\n')
        print(f'Max. Simulation Time: {self.max_sim_time}')
        print(f'No. of Vertices: {no_nodes}')
        print(f'No. of Agents: {no_robots}')
        print(f'The World Adj. List: {adj_list}')
        self.start_positions = [nodes_idx.get(item) for item in origin_vectors]
        self.goal_positions = [nodes_idx.get(item) for item in goal_vectors]
        print(f'Origin Vector: {self.start_positions}')
        print(f'Goal Vector: {self.goal_positions}')
        self.x_vectors = [x[0][0] for x in pygame_relation.items()]
        self.y_vectors = [y[0][1] for y in pygame_relation.items()]
        print(f'Vectors on X axis: {self.x_vectors}')
        print(f'Vectors on Y axis: {self.y_vectors}')
    
    def manhattan_distance(self, node_origin, node_dest):
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
        manhattan_distance = (abs(self.x_vectors[node_origin] - self.x_vectors[node_dest]) + abs(self.x_vectors[node_origin] - self.x_vectors[node_dest])) * 10

        return manhattan_distance


def run_cbs(goals=world.GOALS, starts=world.STARTS, robots_qtd=world.ROBOTS_QTD):
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
    print(f'\nThe Number of Robots Choosen was: {robots_qtd}\n')
    newWorld = createIntegerWorld.WeightedGrid()
    #* Start the Treadmill Class in a Group
    all_treadmill_items = pygame.sprite.Group()
    #* Init the treadmill Sprite
    treadmill_items = createIntegerWorld.TreadmillItems()
    all_treadmill_items.add(treadmill_items)
    #* Add the Robots Sprite Classes to the Groups
    all_robots = pygame.sprite.Group()
    #* Show PyGame Nodes (vec format) and to ordenated Tuples
    all_pygame_nodes_vec = find_all_pygame_nodes(newWorld, vec(0,0))
    all_pygame_nodes_list = vec_to_list(all_pygame_nodes_vec)
    # * The nodes with the robot cannot move on
    non_desired_nodes = world.TREADMILL_ZONE + world.WORKERS_POS + world.DONT_MOVE
    # * The nodes available for movement
    available_pygame_nodes = list(set(all_pygame_nodes_list) - set(non_desired_nodes))
    pygame_possible_nodes  = sorted(sorted(available_pygame_nodes), key=itemgetter(1))
    #* Create the Nodes as Integers
    world_adj_list, world_nodes, world_nodes_indexed = create_adjacency_list(world_width=world.GRID_WIDTH, world_height=world.GRID_HEIGHT,
                                                                             world_nodes=pygame_possible_nodes, world_obstables=world.OBSTACLES,
                                                                             world_recharge_nodes=world.RECHARGE_ZONE, world_pickup_nodes=world.PICKUP_ZONE)
    #* Create a Dict to create relation between the integer graph and PyGame Graph
    relate_pygame_int_graph = dict(zip(pygame_possible_nodes, world_adj_list))
    #* Creates a non-weighted adj matrix based on the adj list
    non_weighted_matrix = create_adjacency_matrix(world_adj_list)
    #* Creates the instance array and saves as .txt
    # instance_array = create_instance_array(non_weighted_matrix)
    # create_instance_file(starts_pygame=starts, goals_pygame=goals, adjacency_list=world_adj_list, instance_arrays=instance_array, nodes_idx=world_nodes_indexed, pygame_relation=relate_pygame_int_graph)
    createInputs = Inputs(no_nodes=world_nodes, no_robots=robots_qtd, adj_matrix=non_weighted_matrix, adj_list=world_adj_list, origin_vectors=starts, goal_vectors=goals, nodes_idx=world_nodes_indexed, pygame_relation=relate_pygame_int_graph)
    # STAstar_Search(createInputs)
    start_time = timer()
    cbs_search(createInputs)
    end_time = timer()
    global MAPFS_PATHS
    print('\n----------------- SIMULATION FINISHED ----------------\n')
    print(f'The CBS takes {end_time-start_time} seconds to run')
    print(f'The CBS Runned {len(MAPFS_PATHS)} Paths')
    makespan = max(map(len, MAPFS_PATHS))
    print(f'The Makespan is:  {makespan}')
    sum_of_costs = sum(map(len, MAPFS_PATHS))
    print(f'The Sum of Costs is:  {sum_of_costs}')
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

def run_stms(goals=world.GOALS, starts=world.STARTS, robots_qtd=world.ROBOTS_QTD):
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
    print(f'\nThe Number of Robots Choosen was: {robots_qtd}\n')
    newWorld = createIntegerWorld.WeightedGrid()
    #* Start the Treadmill Class in a Group
    all_treadmill_items = pygame.sprite.Group()
    #* Init the treadmill Sprite
    treadmill_items = createIntegerWorld.TreadmillItems()
    all_treadmill_items.add(treadmill_items)
    #* Add the Robots Sprite Classes to the Groups
    all_robots = pygame.sprite.Group()
    #* Show PyGame Nodes (vec format) and to ordenated Tuples
    all_pygame_nodes_vec = find_all_pygame_nodes(newWorld, vec(0,0))
    all_pygame_nodes_list = vec_to_list(all_pygame_nodes_vec)
    # * The nodes with the robot cannot move on
    non_desired_nodes = world.TREADMILL_ZONE + world.WORKERS_POS + world.DONT_MOVE
    # * The nodes available for movement
    available_pygame_nodes = list(set(all_pygame_nodes_list) - set(non_desired_nodes))
    pygame_possible_nodes  = sorted(sorted(available_pygame_nodes), key=itemgetter(1))
    #* Create the Nodes as Integers
    world_adj_list, world_nodes, world_nodes_indexed = create_adjacency_list(world_width=world.GRID_WIDTH, world_height=world.GRID_HEIGHT,
                                                                             world_nodes=pygame_possible_nodes, world_obstables=world.OBSTACLES,
                                                                             world_recharge_nodes=world.RECHARGE_ZONE, world_pickup_nodes=world.PICKUP_ZONE)
    #* Create a Dict to create relation between the integer graph and PyGame Graph
    relate_pygame_int_graph = dict(zip(pygame_possible_nodes, world_adj_list))
    #* Creates a non-weighted adj matrix based on the adj list
    non_weighted_matrix = create_adjacency_matrix(world_adj_list)
    #* Creates the instance array and saves as .txt
    instance_array = create_instance_array(non_weighted_matrix)
    createInputs = Inputs(no_nodes=world_nodes, no_robots=robots_qtd, adj_matrix=non_weighted_matrix, adj_list=world_adj_list, origin_vectors=starts, goal_vectors=goals, nodes_idx=world_nodes_indexed, pygame_relation=relate_pygame_int_graph)
    # create_instance_file(starts_pygame=starts, goals_pygame=goals, adjacency_list=world_adj_list, instance_arrays=instance_array, nodes_idx=world_nodes_indexed, pygame_relation=relate_pygame_int_graph, agents=createInputs.no_robots)

    start_time = timer()
    st_star = Spacetime_Astar(createInputs)
    sol = st_star.multi_start(100)
    if(sol != None):
        sol.print()
        st_star.local_search(sol)
        sol.print()
    # sol = Spacetime_Multistart
    end_time = timer()
    global MAPFS_PATHS
    print('\n----------------- SIMULATION FINISHED ----------------\n')
    print(f'The STMS takes {end_time-start_time} seconds to run')
    # print(f'The CBS Runned {len(MAPFS_PATHS)} Paths')
    # makespan = max(map(len, MAPFS_PATHS))
    # print(f'The Makespan is:  {makespan}')
    # sum_of_costs = sum(map(len, MAPFS_PATHS))
    # print(f'The Sum of Costs is:  {sum_of_costs}')
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

def run_mapfs(goals=world.GOALS, starts=world.STARTS, robots_qtd=world.ROBOTS_QTD):
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
    print(f'\nThe Number of Robots Choosen was: {robots_qtd}\n')
    newWorld = createIntegerWorld.WeightedGrid()
    #* Start the Treadmill Class in a Group
    all_treadmill_items = pygame.sprite.Group()
    #* Init the treadmill Sprite
    treadmill_items = createIntegerWorld.TreadmillItems()
    all_treadmill_items.add(treadmill_items)
    #* Add the Robots Sprite Classes to the Groups
    all_robots = pygame.sprite.Group()
    #* Show PyGame Nodes (vec format) and to ordenated Tuples
    all_pygame_nodes_vec = find_all_pygame_nodes(newWorld, vec(0,0))
    all_pygame_nodes_list = vec_to_list(all_pygame_nodes_vec)
    # * The nodes with the robot cannot move on
    non_desired_nodes = world.TREADMILL_ZONE + world.WORKERS_POS + world.DONT_MOVE
    # * The nodes available for movement
    available_pygame_nodes = list(set(all_pygame_nodes_list) - set(non_desired_nodes))
    pygame_possible_nodes  = sorted(sorted(available_pygame_nodes), key=itemgetter(1))
    #* Create the Nodes as Integers
    world_adj_list, world_nodes, world_nodes_indexed = create_adjacency_list(world_width=world.GRID_WIDTH, world_height=world.GRID_HEIGHT,
                                                                             world_nodes=pygame_possible_nodes, world_obstables=world.OBSTACLES,
                                                                             world_recharge_nodes=world.RECHARGE_ZONE, world_pickup_nodes=world.PICKUP_ZONE)
    #* Create a Dict to create relation between the integer graph and PyGame Graph
    relate_pygame_int_graph = dict(zip(pygame_possible_nodes, world_adj_list))
    #* Creates a non-weighted adj matrix based on the adj list
    non_weighted_matrix = create_adjacency_matrix(world_adj_list)
    #* Creates the instance array and saves as .txt
    # instance_array = create_instance_array(non_weighted_matrix)
    # create_instance_file(starts_pygame=starts, goals_pygame=goals, adjacency_list=world_adj_list, instance_arrays=instance_array, nodes_idx=world_nodes_indexed, pygame_relation=relate_pygame_int_graph)
    createInputs = Inputs(no_nodes=world_nodes, no_robots=robots_qtd, adj_matrix=non_weighted_matrix, adj_list=world_adj_list, origin_vectors=starts, goal_vectors=goals, nodes_idx=world_nodes_indexed, pygame_relation=relate_pygame_int_graph)
    start_time = timer()
    Swarm(createInputs, world_nodes=pygame_possible_nodes)
    end_time = timer()
    global MAPFS_PATHS
    print(f'\nThe MAPF-S Paths are: {MAPFS_PATHS}\n')
    print('\n----------------- SIMULATION FINISHED ----------------\n')
    print(f'The MAPF-S takes {end_time-start_time} seconds to run')
    print(f'The Swarm Runned {len(MAPFS_PATHS)} Paths')
    makespan = max(map(len, MAPFS_PATHS))
    print(f'The Makespan is:  {makespan}')
    sum_of_costs = sum(map(len, MAPFS_PATHS))
    print(f'The Sum of Costs is:  {sum_of_costs}')
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

    parser.add_argument('--cbs', action='store_true',
                        help='Runs CBS')
    parser.add_argument('--stms', action='store_true',
                        help='Runs STMS')
    parser.add_argument('--mapfs', action='store_true',
                        help='Runs ST-SPF')

    parser.add_argument('--version', action='version',
                        version='Path Planning with Python = v1.0')

    args = parser.parse_args()

    if args.cbs:
        run_cbs()
    if args.mapfs:
        run_mapfs()
    if args.stms:
        run_stms()

if __name__ == '__main__':
    main()