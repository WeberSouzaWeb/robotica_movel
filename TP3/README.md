# TP3 – Exploração e Mapeamento (Kobuki + CoppeliaSim)

Este pacote contém uma **implementação completa de Occupancy Grid com log-odds**, uma **estratégia simples de exploração** e **scripts** para gerar os mapas e os gráficos solicitados.

> **Importante:** Ajuste os nomes dos objetos/sinais da sua cena (juntas das rodas, laser e base do robô) no arquivo `configs/config.yaml`. O código tenta detectar nomes comuns automaticamente, mas cada cena pode variar.

---

## Como executar

1) **Abra sua cena** no CoppeliaSim (cenário estático ou dinâmico com o Kobuki + laser).
2) Com a cena aberta, **inicie a simulação**.
3) Em um terminal, crie e ative um ambiente virtual (opcional) e instale as dependências:

```bash
python -m venv .venv
source .venv/bin/activate  # (Linux/Mac)
# .\.venv\Scripts\Activate.ps1  # (Windows PowerShell)

pip install -r requirements.txt
```

4) Edite `configs/config.yaml` se necessário (nomes dos objetos/sinais na sua cena).

5) Execute o script principal para **um teste** (ex.: resolução da célula = 0.1 m, duração 180 s, ruído moderado):
```bash
python tp3_main.py --cell-size 0.1 --duration 180 --noise-range 0.01 --noise-angle-deg 1.5 --scenario static
```
Parâmetros úteis:
- `--cell-size`  tamanho da célula (m). Teste **0.01**, **0.1**, **0.5** para a seção de avaliação do enunciado.
- `--duration`   tempo de navegação (s) para construir um bom mapa.
- `--noise-range` desvio-padrão do ruído aditivo na **distância** (m).
- `--noise-angle-deg` desvio-padrão do ruído aditivo no **ângulo** (graus).
- `--scenario` rótulo para organizar saídas (`static` ou `dynamic`).

Saídas são gravadas em `output/`:
- `map_<cenario>_res<cell>.png` – mapa de ocupação (cinza: claro = livre, escuro = ocupado).
- `path_laser_<cenario>_res<cell>.png` – **plot incremental** da trajetória e dos feixes do laser.
- `map_<cenario>_res<cell>.npy` – matriz de probabilidades (0–1) para análise posterior.

### Rodando os **3 tamanhos de célula** (0.01, 0.1, 0.5)
> Deixe o robô mapeando tempo suficiente em **mesma semente de ruído** para isolar o efeito da resolução.

```bash
python tp3_main.py --cell-size 0.01 --duration 120 --scenario static
python tp3_main.py --cell-size 0.1  --duration 120 --scenario static
python tp3_main.py --cell-size 0.5  --duration 120 --scenario static
```

Depois, escolha o melhor `--cell-size` e repita **dois experimentos** com **posições iniciais diferentes** para cada cenário (`static` e `dynamic`).

---

## Notas sobre integração com a cena

- O script usa a **ZMQ Remote API** do CoppeliaSim (client Python `coppeliasim_zmqremoteapi_client`).
- Por padrão, ele tenta estes nomes comuns:
  - Base do robô: `"/kobuki"` ou `"/Kobuki"`
  - Motor esquerdo: `"/leftMotor"`  (também tenta `"/Kobuki/leftMotor"` etc.)
  - Motor direito: `"/rightMotor"`
  - Laser: `"/laser"` ou `"/Hokuyo"` ou `"/fastHokuyo"`
  - **Sinal de varredura** (string): tenta `["hokuyo_data", "laser_scan", "scan_data"]`
- Caso a sua cena use outros nomes/sinais, **configure em** `configs/config.yaml`.
- Se a sua cena **não publicar** um sinal de laser, inclua um child script no laser que faça:
  - empacote os *ranges* (e opcionalmente `angle_min`, `angle_increment`, `range_max`) com `sim.packFloatTable(...)` e
  - publique com `sim.setStringSignal('<nome_do_sinal>', packed)` a cada passo de simulação.
  - O Python lê e desserializa com `sim.unpackFloatTable(...)` via ZMQ.

> A localização do robô é lida continuamente via `sim.getObjectPose(...)` (frame global).

---

## O que este código **entrega**
- **Occupancy Grid** com **log-odds** (clipping e `p0`, `p_occ`, `p_free` configuráveis).
- **Ruído** no laser (ângulo e alcance) conforme solicitado.
- **Exploração reativa** simples: *wall-following* + escolha periódica de rumo para maximizar ganho de informação local (fronteiras).
- **Transformações** laser→robô→mundo e discretização no *grid*.
- **Gravação de imagens** do grid e do **plot incremental** (trajetória + leituras).
- Código **modular e comentado** para facilitar ajustes/estudos.

---

## Troubleshooting rápido
- **Não conecta no CoppeliaSim**: abra o CoppeliaSim, inicie a simulação e verifique a porta do ZMQ (padrão 23000). Edite `configs/config.yaml` se necessário.
- **Sem dados de laser**: ajuste `laser_signal_names` no `config.yaml` ou adicione um child script no laser para publicar o sinal.
- **Robô não se move**: confira nomes das juntas no `config.yaml` e se estão em modo de controle de velocidade.

---

## Licença
Uso acadêmico. Sinta-se à vontade para adaptar para o seu TP3.
