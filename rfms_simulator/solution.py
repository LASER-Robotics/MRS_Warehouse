FREE = -1

class Solution:
    def __init__(self, input):
        self.input = input
        self.space = [[FREE for t in range(self.input.max_sim_time)] for a in range(input.no_robots)]
        self.makespan = 0
        self.agent_makespan = [self.input.max_sim_time for a in range(input.no_robots)]
        self.agent_max_makespan = None
        self.agent = [[FREE for t in range(self.input.max_sim_time)] for a in range(input.no_nodes)]

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
        #print(f'New Path: {path}\n')
        for v in path:
            self.space[agent][t] = v
            t += 1
        for ta in range(t, self.input.max_sim_time):
            self.space[agent][ta] =  self.input.goal_positions[agent]
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
