from queue import PriorityQueue
from spacetime_multistart import Spacetime_Multistart, reconstruct_path

class Spacetime_Astar(Spacetime_Multistart):
    def __init__(self, input):
        super().__init__(input)
    
    #Cria o caminho da origem de agent até seu destino utilizando
    #a heurística a_star
    def create_path(self, solution, agent, t_ini=0, forbidden_vertex=-1):
        came_from = [-1 for x in range(self.input.no_nodes)]
        is_visited = [False for x in range(self.input.no_nodes)]
        start = self.input.start_positions[agent]
        goal = self.input.goal_positions[agent]
        
        # * Se houve conflito, start = local que o agente estava antes do conflito
        if t_ini > 0:
            start = solution.space[agent][t_ini - 1]
        
        queue = PriorityQueue()
        queue.put((0, start))
    
        # * The cost of the cheapest path from start to n currently known.
        max_g_score = 2 * self.input.no_nodes
        g_score = [max_g_score for x in range(self.input.no_nodes)]
        g_score[start] = 0
        if t_ini > 0:
            g_score[start] = t_ini - 1 
        
        f_score = [max_g_score for x in range(self.input.no_nodes)]
        f_score[start] = g_score[start] + self.input.manhattan_distance(goal, start)
       
        while not queue.empty():
            #* The next one will be the one with lowest cost
            current = queue.get()[1]
            #print("Current", current)
            #* If the goal is reached break
            if current == goal:
                path = reconstruct_path(start, came_from, goal)
                return path
            for next in self.input.adj_list[current]:
                
                if(forbidden_vertex >= 0) and (forbidden_vertex == next):
                    continue
                if not solution.is_valid_moviment(agent, current, next, g_score[current]):
                    continue    
                
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score[next]:
                    
                    came_from[next] = current
                    g_score[next] = tentative_g_score
                    f_score[next] = tentative_g_score + self.input.manhattan_distance(goal, next)
                    #print("Vizinho", next,"fscore", f_score[next])
                    if not is_visited[next]:
                        #* Put in the priority
                        queue.put((f_score[next], next))
            is_visited[current] = True
        #Se não encontrar caminho para goal retorna false
        return None
