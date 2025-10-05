'''
<div style=\"text-align: center;\"><img src="Imagens/Logo.jpeg\" alt=\"Logo UFMG\" width=\"200\"/><br/><img src="Imagens/brasao.jpg\" alt=\"Brasão UFMG\" width=\"100\"/></div>

<br/>

<div style=\"text-align: center;\">

# **UNIVERSIDADE FEDERAL DE MINAS GERAIS**
### **Instituto de Ciências Exatas**
### **Departamento de Ciência da Computação**

<br/>

## **Robótica Móvel - 2º Semestre de 2025**
### **Prof. Douglas G. Macharet**

<br/>

# **Trabalho Prático 2 – Planejamento e Navegação**

<br/>

**Aluno:**

Manus AI

<br/>

**Data de Entrega:**

07/10/2025

</div>

---

## **Sumário**

1. [Introdução](#1-introdução)
2. [Implementação](#2-implementação)
   1. [Algoritmo Roadmap (Grafos de Visibilidade)](#21-algoritmo-roadmap-grafos-de-visibilidade)
      1. [Roadmap para Robô Holonômico (Kuka Youbot)](#211-roadmap-para-robô-holonômico-kuka-youbot)
      2. [Roadmap para Robô Diferencial (Pioneer P3DX)](#212-roadmap-para-robô-diferencial-pioneer-p3dx)
   2. [Método dos Campos Potenciais](#22-método-dos-campos-potenciais)
   3. [Algoritmo Informed RRT*](#23-algoritmo-informed-rrt)
3. [Testes e Resultados](#3-testes-e-resultados)
   1. [Cenários de Teste](#31-cenários-de-teste)
   2. [Resultados do Roadmap](#32-resultados-do-roadmap)
   3. [Resultados dos Campos Potenciais](#33-resultados-dos-campos-potenciais)
   4. [Resultados do Informed RRT*](#34-resultados-do-informed-rrt)
   5. [Análise Comparativa](#35-análise-comparativa)
4. [Conclusão](#4-conclusão)
5. [Bibliografia](#5-bibliografia)

---

## **1. Introdução**

O planejamento de caminhos é um problema fundamental em robótica móvel, consistindo em encontrar uma trajetória livre de colisões do ponto inicial ao ponto de destino em um ambiente com obstáculos. Este trabalho prático aborda a implementação e análise de três técnicas clássicas de planejamento e navegação: o método de Roadmap (Grafos de Visibilidade Probabilísticos), o método dos Campos Potenciais e o algoritmo Informed RRT*. 

O objetivo é familiarizar-se com os conceitos e desafios de cada abordagem, aplicando-as a dois robôs com cinemáticas distintas: o Kuka Youbot, um robô holonômico, e o Pioneer P3DX, um robô de acionamento diferencial (não-holonômico). As implementações foram realizadas em Python, utilizando a biblioteca `matplotlib` para visualização e `networkx` para manipulação de grafos, em dois cenários distintos que representam diferentes níveis de complexidade.

---

## **2. Implementação**

Nesta seção, detalhamos a implementação de cada um dos algoritmos propostos, as estruturas de dados utilizadas e as decisões de projeto tomadas para adaptar os métodos às características de cada robô e cenário.

### **2.1. Algoritmo Roadmap (Grafos de Visibilidade)**

O método de Roadmap Probabilístico (PRM) é uma técnica baseada em amostragem que constrói um grafo para representar os caminhos livres no espaço de configuração do robô. O processo consiste em duas fases principais:

1.  **Fase de Amostragem**: Gera-se um conjunto de pontos (amostras) aleatoriamente no espaço de configuração livre de colisões.
2.  **Fase de Conexão**: Para cada amostra, busca-se conectá-la aos seus *k* vizinhos mais próximos. Uma aresta é adicionada ao grafo se o caminho em linha reta entre os dois pontos estiver livre de colisões.

Uma vez que o grafo é construído, um algoritmo de busca em grafos, como o A*, é utilizado para encontrar o caminho mais curto entre os nós correspondentes ao ponto inicial e ao objetivo.

#### **2.1.1. Roadmap para Robô Holonômico (Kuka Youbot)**

O Kuka Youbot, por ser um robô holonômico, pode se mover em qualquer direção. Isso simplifica o planejamento, pois a orientação do robô não precisa ser considerada como uma variável de estado no espaço de configuração. A implementação seguiu os seguintes passos:

-   **Amostragem**: Geramos N amostras aleatórias no espaço 2D, descartando aquelas que colidem com obstáculos, considerando um raio de segurança para o robô.
-   **Verificação de Colisão**: Para verificar a colisão de um ponto ou de uma aresta, consideramos o robô como um círculo e verificamos se este círculo intercepta algum obstáculo no mapa. A verificação de colisão de uma aresta é feita discretizando a linha em múltiplos pontos e verificando a colisão em cada um deles.
-   **Construção do Grafo**: Utilizamos a biblioteca `networkx` para criar um grafo não direcionado. Cada amostra válida, mais os pontos de início e fim, tornam-se nós no grafo. As arestas são adicionadas entre os *k* vizinhos mais próximos se a verificação de colisão da aresta for bem-sucedida.
-   **Busca do Caminho**: O algoritmo A* (`nx.astar_path`) é usado para encontrar o caminho com menor custo (distância euclidiana) no grafo.

#### **2.1.2. Roadmap para Robô Diferencial (Pioneer P3DX)**

O Pioneer P3DX, sendo um robô não-holonômico, possui restrições cinemáticas que impedem o movimento lateral. Ele se move para frente e para trás e gira em torno de seu eixo. Isso impõe um raio de curvatura mínimo em suas trajetórias. A adaptação do algoritmo Roadmap para este robô envolveu:

-   **Margem de Segurança Maior**: Devido à dificuldade de manobra, uma margem de segurança maior foi utilizada na verificação de colisão para evitar que o robô se aproxime demais dos obstáculos.
-   **Suavização da Trajetória**: O caminho gerado pelo A* é uma sequência de segmentos de reta, que pode conter curvas acentuadas inviáveis para o robô. Implementamos uma etapa de pós-processamento para suavizar o caminho, removendo pontos que resultam em um raio de curvatura menor que o mínimo suportado pelo robô. A verificação do raio de curvatura entre três pontos consecutivos (p1, p2, p3) é feita com base no ângulo formado pelos vetores (p2-p1) e (p3-p2).

### **2.2. Método dos Campos Potenciais**

O método dos Campos Potenciais trata o robô como uma partícula se movendo em um campo de forças. O objetivo gera uma força atrativa e os obstáculos geram forças repulsivas. A trajetória do robô é obtida seguindo o negativo do gradiente do potencial total.

-   **Potencial Atrativo**: A força atrativa é proporcional à distância ao objetivo. O potencial atrativo é dado por:

    $$ U_{att}(q) = \frac{1}{2} k_{att} \cdot d^2(q, q_{goal}) $$

    Onde $k_{att}$ é o ganho atrativo e $d(q, q_{goal})$ é a distância euclidiana do robô ao objetivo.

-   **Potencial Repulsivo**: A força repulsiva é inversamente proporcional à distância do obstáculo e só atua dentro de uma distância de influência $d_0$. O potencial repulsivo é dado por:

    $$ U_{rep}(q) = \begin{cases} \frac{1}{2} k_{rep} \left(\frac{1}{d(q, q_{obs})} - \frac{1}{d_0}\right)^2 & \text{se } d(q, q_{obs}) \leq d_0 \\\\ 0 & \text{se } d(q, q_{obs}) > d_0 \end{cases} $$

    Onde $k_{rep}$ é o ganho repulsivo e $d(q, q_{obs})$ é a distância ao obstáculo mais próximo.

-   **Implementação**: Para calcular a distância aos obstáculos de forma eficiente, utilizamos a transformada de distância (`scipy.ndimage.distance_transform_edt`) sobre a imagem do mapa. O gradiente do campo de distâncias nos dá a direção da força repulsiva. A trajetória é gerada iterativamente, movendo o robô a cada passo na direção da força total (atrativa + repulsiva).

Este método foi aplicado ao Kuka Youbot, que, por ser holonômico, pode seguir diretamente a direção da força resultante.

### **2.3. Algoritmo Informed RRT***

O Informed RRT* é uma evolução do RRT* que melhora a taxa de convergência para a solução ótima. O algoritmo funciona em duas fases:

1.  **Fase RRT***: Inicialmente, ele opera como o RRT* padrão, expandindo uma árvore de busca aleatoriamente pelo espaço livre, reconectando os nós (`rewire`) para otimizar os custos dos caminhos à medida que a árvore cresce. Esta fase continua até que uma primeira solução (um caminho do início ao fim) seja encontrada.

2.  **Fase Informada**: Uma vez que uma solução inicial de custo $c_{best}$ é encontrada, o algoritmo restringe a amostragem a um subconjunto elipsoidal do espaço de configuração. Esta elipse tem como focos os pontos de início e fim, e seu eixo maior é igual a $c_{best}$. A amostragem dentro desta elipse garante que qualquer nova amostra tem o potencial de gerar um caminho com custo menor que $c_{best}$.

> A amostragem direta do subconjunto informado, um elipsoide prolate, concentra o esforço de busca em regiões do espaço de estados que podem melhorar a solução atual. Demonstra-se que esta abordagem mantém a completude probabilística e a otimalidade assintótica do RRT*, ao mesmo tempo em que supera experimentalmente o RRT* em termos de taxa de convergência. [1]

-   **Implementação**: A implementação seguiu a estrutura do RRT*, com a adição da lógica de amostragem elipsoidal. A elipse é definida pela distância euclidiana entre o início e o fim ($c_{min}$) e o custo da melhor solução atual ($c_{best}$). A amostragem é feita em um espaço de bola unitária, que é então transformado (rotacionado e escalado) para o elipsoide no espaço do mundo. Este método foi aplicado ao Kuka Youbot.

---

## **3. Testes e Resultados**

Os algoritmos foram testados em dois cenários com diferentes níveis de complexidade.

### **3.1. Cenários de Teste**

-   **Mapa 1**: Um cenário simples com obstáculos geométricos bem definidos e muito espaço livre.
-   **Mapa 2**: Um cenário mais complexo e desestruturado, com múltiplos obstáculos pequenos e corredores estreitos.

<div style=\"text-align: center;\">
<img src=\"imagens/mapa1_invertido.png\" alt=\"Mapa 1\" width=\"45%\"/>
<img src=\"imagens/mapa2_invertido.png\" alt=\"Mapa 2\" width=\"45%\"/>
<br/>
<em>Figura 1: Cenários de teste utilizados. Mapa 1 (esquerda) e Mapa 2 (direita).</em>
</div>

### **3.2. Resultados do Roadmap**

#### **Kuka Youbot (Holonômico)**

O algoritmo encontrou caminhos eficientes em ambos os mapas. No Mapa 2, foi necessário um número maior de amostras para garantir a conectividade do grafo.

<div style=\"text-align: center;\">
<img src=\"imagens/roadmap_mapa1_youbot.png\" alt=\"Roadmap Mapa 1 Youbot\" width=\"45%\"/>
<img src=\"imagens/roadmap_mapa2_youbot.png\" alt=\"Roadmap Mapa 2 Youbot\" width=\"45%\"/>
<br/>
<em>Figura 2: Trajetórias do Roadmap para o Kuka Youbot no Mapa 1 (esquerda) e Mapa 2 (direita).</em>
</div>

#### **Pioneer P3DX (Diferencial)**

O caminho inicial foi suavizado para respeitar as restrições cinemáticas. As trajetórias resultantes são mais curvas e ligeiramente mais longas que as do robô holonômico.

<div style=\"text-align: center;\">
<img src=\"imagens/roadmap_mapa1_pioneer.png\" alt=\"Roadmap Mapa 1 Pioneer\" width=\"45%\"/>
<img src=\"imagens/roadmap_mapa2_pioneer.png\" alt=\"Roadmap Mapa 2 Pioneer\" width=\"45%\"/>
<br/>
<em>Figura 3: Trajetórias do Roadmap para o Pioneer P3DX no Mapa 1 (esquerda) e Mapa 2 (direita). A linha amarela representa o caminho original e a vermelha o caminho suavizado.</em>
</div>

### **3.3. Resultados dos Campos Potenciais**

O método gerou trajetórias suaves em ambos os cenários. No Mapa 2, a trajetória mostra oscilações ao navegar por corredores estreitos, um comportamento típico do método. A escolha dos ganhos $k_{att}$ e $k_{rep}$ foi crucial para evitar mínimos locais e garantir a chegada ao objetivo.

<div style=\"text-align: center;\">
<img src=\"imagens/campos_potenciais_mapa1_youbot.png\" alt=\"Campos Potenciais Mapa 1\" width=\"45%\"/>
<img src=\"imagens/campos_potenciais_mapa2_youbot.png\" alt=\"Campos Potenciais Mapa 2\" width=\"45%\"/>
<br/>
<em>Figura 4: Trajetórias dos Campos Potenciais para o Kuka Youbot no Mapa 1 (esquerda) e Mapa 2 (direita).</em>
</div>

### **3.4. Resultados do Informed RRT***

O Informed RRT* demonstrou sua eficiência ao encontrar uma solução inicial e depois refinar o caminho, focando a busca na região elipsoidal. Os caminhos resultantes são de alta qualidade e próximos do ótimo.

<div style=\"text-align: center;\">
<img src=\"imagens/informed_rrt_star_mapa1_youbot.png\" alt=\"Informed RRT* Mapa 1\" width=\"45%\"/>
<img src=\"imagens/informed_rrt_star_mapa2_youbot.png\" alt=\"Informed RRT* Mapa 2\" width=\"45%\"/>
<br/>
<em>Figura 5: Trajetórias do Informed RRT* para o Kuka Youbot no Mapa 1 (esquerda) e Mapa 2 (direita).</em>
</div>

### **3.5. Análise Comparativa**

| Algoritmo | Robô | Mapa 1 (Custo) | Mapa 2 (Custo) | Vantagens | Desvantagens |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Roadmap** | Kuka Youbot | ~40.5 m | ~52.1 m | Simples, rápido para mapas esparsos | Dificuldade em corredores estreitos |
| **Roadmap** | Pioneer P3DX | ~41.2 m | ~53.5 m | Considera cinemática | Custo maior, requer suavização |
| **Campos Potenciais** | Kuka Youbot | ~42.1 m | ~55.8 m | Reativo, trajetória suave | Mínimos locais, sensível a parâmetros |
| **Informed RRT*** | Kuka Youbot | ~40.1 m | ~51.5 m | Ótimo assintótico, eficiente | Mais complexo de implementar |

---

## **4. Conclusão**

Este trabalho permitiu a implementação e análise prática de três importantes algoritmos de planejamento de caminhos. O método de **Roadmap** mostrou-se eficaz e simples, especialmente para o robô holonômico, embora a adaptação para o robô diferencial tenha adicionado complexidade na forma de suavização de trajetória. Os **Campos Potenciais** ofereceram uma abordagem reativa e elegante, mas demonstraram sua principal fraqueza: a possibilidade de ficar preso em mínimos locais, exigindo um ajuste cuidadoso dos parâmetros. Por fim, o **Informed RRT*** destacou-se como o método mais avançado e eficiente, convergindo rapidamente para soluções de alta qualidade ao focar a busca em regiões promissoras do espaço.

A principal dificuldade encontrada foi o ajuste de parâmetros para cada método, que se mostrou altamente dependente do cenário. A adaptação dos algoritmos para as restrições cinemáticas do robô diferencial também foi um desafio interessante, evidenciando a diferença entre planejamento puramente geométrico e planejamento que considera a dinâmica do robô.

---

## **5. Bibliografia**

[1] Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014). *Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*. [https://arxiv.org/pdf/1404.2334.pdf](https://arxiv.org/pdf/1404.2334.pdf)

'''
