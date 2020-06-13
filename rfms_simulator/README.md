# WarehousePy

Simulação e discretização de grids para análise de algoritmos de planejamento de trajetória

## Pacotes Necessários

- [Python 3+](https://docs.python-guide.org/starting/install3/linux/)
- [PyGame](https://pypi.org/project/pygame/)
- [Memory Profiler](https://pypi.org/project/memory-profiler/)

## Como o Pacote é dividido?

**Examples**: Nesta pasta você pode encontrar um exemplo prático de como o mundo e um algoritmo de trajetória funciona
na simulação, e como o mundo é gerado. Para trajetória é utilizado o algoritmo Breadth First Search.

**world**: Pasta em que se encontra a Classe responsável por gerar o mundo discretizado

**some_tests**: Algoritmos que serão implementados na função principal em breve (A* e Dijkstra)

## Como utilizar?

Para utilizar o pacote de exemplo basta executar o arquivo "example.py" com o Python 3 e o Memory Profiler:

```
python3 -m memory_profiler example.py
```

Para executar os arquivos de teste:


```
python3 nome_do_arquivo.py
```

Para executar o arquivo principal (Em desenv.):


```
python3 -m memory_profiler main.py
```  
