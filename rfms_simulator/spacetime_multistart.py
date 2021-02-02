import random
import math
import copy
from solution import Solution

def reconstruct_path(start, came_from, current):
    total_path = [current]
    while current != start:
        current = came_from[current]
        total_path.insert(0, current)
    return total_path

class Spacetime_Multistart:
    def __init__(self, input_instance):
        self.input = input_instance
        
    def create_path(self, solution, agent, t_ini=0, forbidden_vertex=-1):
        raise NotImplementedError()

    def create_solution(self):
        solution = Solution(self.input)
        for a in range(self.input.no_robots):
            path = self.create_path(solution, a)
            if(path != None):
                solution.add_path(a, path)
            else:
                return None
            
        self.resolve_conflit(solution)
        return solution
    
    
    def multi_start(self, num_iter):
        best_sol = None
        best_makespan = float('inf')
        LC = []
        for a in range(self.input.no_robots):
            LC.append((self.input.manhattan_distance(self.input.start_positions[a], self.input.goal_positions[a]), a))
        LC.sort()
        
        for i in range(num_iter):
            solution = Solution(self.input)
            LRC = copy.deepcopy(LC)
        
            while len(LRC) > 0:
                a = LRC.pop(random.randint(0, math.floor(len(LRC)/2)))[1]
                path = self.create_path(solution, a)
                if(path != None):
                    solution.add_path(a, path)
                else:
                    solution = None
                    break;
            if(solution != None):
                self.resolve_conflit(solution)
            else:
                continue
                
            if(solution.makespan < best_makespan):
                best_makespan = solution.makespan
                if(best_sol != None):
                    del best_sol
                best_sol = solution
                
        return best_sol
        
    def resolve_conflit(self, solution):
        #Procura algum conflito entre agentes gerado pelo a_star
        makespan = solution.makespan
        for t in range(makespan+1):
            for a1 in range(self.input.no_robots - 1):
                for a2 in range(a1 + 1, self.input.no_robots):
                    # print(f'Makespan 1: {solution.makespan}')
                    # * Pegar minha posição atual e comparar com o do próximo agente
                    if solution.space[a1][t] ==  solution.space[a2][t]:
                        #print(f'Node Colision at: {solution.space[a1][t]}')
                        #print(f'Time: {t}')
                        #print(f'First Agent: {a1}')
                        #print(f'Second Agent: {a2}')
                        # * Se eu cheguei primeiro quem deve mudar o caminho é o segundo agente
                        # * caso contrário, quem muda sou eu
                        # agent = solution.checks_last_in(a1, a2, t)
                        # print(f'The Agent {a+1} checks in at goal first, changing the Agent {agent} path...')
                        path = self.create_path(solution, a1, t)
                        if(path != None):
                            solution.clear_path(t, a1)
                            solution.append_path(a1, path, t)
                        else:
                            return None
                        # print(f'Makespan 2: {solution.makespan}')
                        # solution.print()
                        # return
                        break
    
    def corner_neighbor(self, sol):
        agent = sol.agent_max_makespan
        #print("Agent: " + str(agent))
        for t in range(1, sol.makespan-1):
            #print("Tempo: " + str(t))
            v = sol.space[agent][t]
            
            #print("Forbidden V: " + str(v))
            path = self.create_path(sol, agent, t, v)
            #print("Path:" + str(path))
            if(path != None) and (sol.makespan > (t-1) + len(path)):
                print(t-1, len(path), sol.makespan)
                sol.clear_path(t, agent)
                sol.append_path(agent, path, t)
                return True
        
        return False
    
    def local_search(self, sol): 
        while(self.corner_neighbor(sol)):
            pass
