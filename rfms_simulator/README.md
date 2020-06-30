# WarehouseSim.Py

Simulação e discretização de grids para análise de algoritmos de planejamento de trajetória

## Pacotes Necessários

- [Python 3+](https://docs.python-guide.org/starting/install3/linux/)
- [PyGame](https://pypi.org/project/pygame/)
- [Memory Profiler](https://pypi.org/project/memory-profiler/)
- [Schedule](https://schedule.readthedocs.io/en/stable/)

## Como o Pacote é dividido?

**examples**: Nesta pasta você pode encontrar um exemplo prático de como o mundo e um algoritmo de trajetória funciona
na simulação, e como o mundo é gerado. Para trajetória é utilizado o algoritmo Breadth First Search.

**world**: Pasta em que se encontra a Classe responsável por gerar o mundo discretizado

**old_simulator**: Pasta em que se encontra o primeiro código feito para o mundo conforme a figura abaixo (Backup)

<a href=""><img src="https://i.ibb.co/25SF3LC/simulator-complete.png" align="center" width="300" style="margin-right: 20px;"></a>

## Quais os Módulos disponívels?

**createWorld**: Este módulo é responsável por gerar dois mundos em formato Grid, um Wheighted e outro não, onde
as configurações e design deste mundo pode ser modificadas pelo módulo a seguir.

**settings**: Este módulo é responsável por armazenar as informações que você deseja gerar para o mundo, onde é
possível adicionar novas classes de mundo, e são armazenados valores como Quantidade de Robôs, Tamanho do Mundo,
Cores Utilizadas, Locais dos Obstáculos, Locais dos Trabalhadores, etc.

## Como utilizar?

### Sobre a Simulação para Apenas um Robô
Para utilizar o pacote de exemplo basta executar o arquivo "example.py" com o Python 3 e o Memory Profiler:

```
python3 -m memory_profiler example.py
```

Para executar o arquivo principal utilize o argpase -h ou --help para mais informações sobre as simulações disponíveis:

```
python3 main.py -h 
```  

Durante a simulação single-robot (A-star, Dijkstra's, Breadth First Search) é possível modificar a localização do robô, o goal desejado e adicionar novos obstáculos. O cálculo do Path Planning será feito em tempo real, e no terminal será possível verificar o tempo que levou para o cálculo do caminho, assim como a quantidade de nodes visitadas pelo robô. Algumas informações importantes:

- O _botão esquerdo do mouse_ é responśavel por adicionar obstáculos

- O _botão direito do mouse_ é responśavel por mudar a localização do goal

- O _botão central (scroll) do mouse_ é responśavel por mudar a localização do robô

### Sobre a Simulação Multi-Robôs

As seguintes simulações utilizam sistemas multi-robôs, onde é possível executar a simulação ao pressionar "ESPAÇO" no teclado:

```
python3 main.py -stastar
```

Informações sobre a visualização:

- A *área em cinza escuro* é o caminho que foi explorado pelo algorítmo
- A *área em cinza claro* são os obstáculos (pods)
- A *área em amarelo* é a zona de recarga
- A *área em vermelho* é a zona de pick
- A *área azul claro* é a zona de delivery

### Erros Conhecidos

Caso encontre o erro abaixo após iniciar a simulação multi-robôs, basta ignorar e rodar o código novamente.

```
Traceback (most recent call last):
  File "main.py", line 1343, in <module>
    main()
  File "main.py", line 1340, in main
    run_space_astar_search()
  File "main.py", line 1218, in run_space_astar_search
    robots =  createWorld.MultiRobot(start, goal_poped, path)
  File "/home/"your_username"/MRS_Warehouse/rfms_simulator/world/createWorld.py", line 999, in __init__
    self.current_pos = self.path[(self.start_pos)] + self.start
TypeError: unsupported operand type(s) for +: 'NoneType' and 'pygame.math.Vector2'
```
