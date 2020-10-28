# WarehousePy

Simulação e discretização de grids para análise de algoritmos de planejamento de trajetória

## Pacotes Necessários

- [Python 3+](https://docs.python-guide.org/starting/install3/linux/)
- [PyGame](https://pypi.org/project/pygame/)
- [Memory Profiler](https://pypi.org/project/memory-profiler/)
- [Parallel Execute](https://pypi.org/project/parallel-execute/)

## Como o Pacote é dividido?

**Examples**: Nesta pasta você pode encontrar um exemplo prático de como o mundo e um algoritmo de trajetória funciona
na simulação, e como o mundo é gerado. Para trajetória é utilizado o algoritmo Breadth First Search.

**world**: Pasta em que se encontra a Classe responsável por gerar o mundo discretizado

## Como utilizar?

Para utilizar o pacote de exemplo basta executar o arquivo "example.py" com o Python 3 e o Memory Profiler:

```
python3 -m memory_profiler example.py
```

Para executar o arquivo principal (Em desenv.) utilize o argpase -h ou --help para mais informações sobre as simulações disponíveis:


```
python3 main.py -h 
```  
Durante a simulação é possível modificar a localização do robô, o goal desejado e adicionar novos obstáculos. O cálculo do path planning será feito em tempo real, e no terminal será possível verificar o tempo que levou para o mesmo e o número do caminho. Algumas informações importantes:

- O _botão esquerdo do mouse_ é responśavel por adicionar obstáculos

- O _botão direito do mouse_ é responśavel por mudar a localização do goal

- O _botão central (scroll) do mouse_ é responśavel por mudar a localização do robô

Informações sobre a visualização:

- A *área em cinza escuro* é o caminho que foi explorado pelo algorítmo
- A *área em cinza claro* são os obstáculos (pods)
- A *área em amarelo* é a zona de recarga
- A *área em vermelho* é a zona de pick
- A *área azul claro* é a zona de delivery
