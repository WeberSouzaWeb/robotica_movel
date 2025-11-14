# TP3 – Exploração e Mapeamento (Documentação)

**Aluno(s):** _Preencha aqui_  
**Matrícula:** _Preencha aqui_  
**Disciplina:** Robótica Móvel — 2º/2025  
**Professor:** Prof. Douglas G. Macharet

---

## 1. Introdução
Descreva o problema: exploração do ambiente com um robô Kobuki equipado com laser e construção de um mapa de ocupação (Occupancy Grid). Inclua uma visão geral do programa, robô e cenas usadas. Cite dimensões (ex.: **L = 0.230 m**, **r = 0.035 m**) quando relevantes para o modelo diferencial.

## 2. Execução
Dependências (Python, `requirements.txt`) e **como executar** para cada experimento. Explique como ajustar nomes de objetos/sinais na cena e como selecionar o tamanho da célula (`--cell-size`).

## 3. Navegação
Explique a estratégia: combinação de *wall-following* (evitar colisões e contornar paredes) com orientação periódica para regiões de maior incerteza (fronteiras do mapa). Indique parâmetros de controle e condições de troca de comportamento.

## 4. Implementação
- **Transformações** entre frames (laser→robô→mundo) e discretização no grid.
- **Occupancy Grid** com **log-odds**: atualizações `l(x) += L_occ` (célula de impacto) e `l(x) += L_free` (células ao longo do feixe), com `L_occ = logit(p_occ) - logit(p0)`, `L_free = logit(p_free) - logit(p0)` e *clipping* `[l_min, l_max]`.  
- **Ruído** no laser (ângulo e alcance), motivação e impactos.
- Estruturas de dados: matriz `numpy` para log-odds; utilidades de *ray tracing*; salvamento de imagens e trilhas.

Inclua **diagramas** (opcional) e trechos mínimos de código **apenas** quando imprescindíveis.

## 5. Testes
### 5.1 Avaliação do tamanho de célula
Apresente **três mapas** (0.01, 0.1, 0.5 m), **mesmo cenário** e condições semelhantes (mesma semente de ruído). Discuta trade-offs:
- 0.01 m: alta definição, maior custo computacional e ruído espúrio aparente;
- 0.1 m: balanço entre detalhe e robustez;
- 0.5 m: rápido, porém perda de detalhes e “escadização” de paredes.

Inclua **plots incrementais** da trajetória + feixes.

### 5.2 Experimentos estático e dinâmico
Com o melhor `--cell-size`, execute **dois testes** por cenário com **posições iniciais diferentes**. Mostre **mapas finais** e **plots incrementais**. Comente o efeito de obstáculos móveis no mapa (fantasmas, borrões).

## 6. Conclusão
Síntese das observações sobre **eficiência** (tempo, custo) e **eficácia** (qualidade do mapa), limitações e melhorias (frontier-based planning completo, SLAM, filtros de ruído, etc.).

## 7. Bibliografia
- Elfes, A. (1989). Using occupancy grids for mobile robot perception and navigation.  
- Thrun, S., Burgard, W., Fox, D. (2005). *Probabilistic Robotics*. MIT Press.  
- Documentação do CoppeliaSim (ZMQ Remote API) e do modelo Hokuyo.
