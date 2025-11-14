#!/usr/bin/env python3
import argparse, time, math, os, sys, json
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
import yaml

# ========= Utilidades =========

def logit(p: float) -> float:
    eps = 1e-9
    p = min(max(p, eps), 1 - eps)
    return math.log(p / (1 - p))

def sigmoid(l: float) -> float:
    return 1.0 / (1.0 + math.exp(-l))

def rotation_matrix(theta: float):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])

def world_to_grid(x, y, xmin, ymin, res):
    # retorna índices i,j a partir de (x,y)
    i = int((y - ymin) / res)  # linha
    j = int((x - xmin) / res)  # coluna
    return i, j

def bresenham_line(x0, y0, x1, y1, xmin, ymin, res, shape):
    """Discretiza uma reta (x0,y0)->(x1,y1) em índices de células usando passos do tamanho da célula."""
    dx = x1 - x0
    dy = y1 - y0
    L = max(abs(dx), abs(dy)) / max(res, 1e-6)
    n = int(max(1, L))
    xs = np.linspace(x0, x1, n)
    ys = np.linspace(y0, y1, n)
    cells = []
    for k in range(n):
        i, j = world_to_grid(xs[k], ys[k], xmin, ymin, res)
        if 0 <= i < shape[0] and 0 <= j < shape[1]:
            if len(cells) == 0 or cells[-1] != (i, j):
                cells.append((i, j))
    return cells



# ========= Occupancy Grid =========

@dataclass
class GridParams:
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    res: float
    p0: float = 0.5
    p_occ: float = 0.7
    p_free: float = 0.3
    l_min: float = -4.0
    l_max: float = 4.0

class OccupancyGrid:
    def __init__(self, params: GridParams):
        self.params = params
        self.w = int(math.ceil((params.xmax - params.xmin) / params.res))
        self.h = int(math.ceil((params.ymax - params.ymin) / params.res))
        self.L0 = logit(params.p0)
        self.L_occ = logit(params.p_occ) - self.L0
        self.L_free = logit(params.p_free) - self.L0
        self.grid = np.zeros((self.h, self.w), dtype=np.float32) + self.L0

    def update_ray(self, ox, oy, hx, hy, hit=True):
        # ox,oy: origem (robô); hx,hy: ponto de impacto
        cells = bresenham_line(ox, oy, hx, hy, self.params.xmin, self.params.ymin, self.params.res, self.grid.shape)
        if not cells:
            return
        # Células livres ao longo do feixe (exceto a última, se houve impacto)
        free_cells = cells[:-1] if hit else cells
        for (i, j) in free_cells:
            self.grid[i, j] = np.clip(self.grid[i, j] + self.L_free, self.params.l_min, self.params.l_max)
        # Célula de impacto (ocupada)
        if hit and len(cells) > 0:
            i, j = cells[-1]
            self.grid[i, j] = np.clip(self.grid[i, j] + self.L_occ, self.params.l_min, self.params.l_max)

    def probability(self):
        # Convert log-odds para probabilidade
        sig = 1.0 / (1.0 + np.exp(-self.grid))
        return sig

    def save_image(self, path_png):
        p = self.probability()
        # 0=preto (ocupado) ... 1=branco (livre)
        img = (1.0 - p)  # invertido p/ ficar "ocupado=escuro"
        plt.figure()
        plt.imshow(img, origin='lower', extent=[self.params.xmin, self.params.xmax, self.params.ymin, self.params.ymax],cmap='gray',vmin=0.0, vmax=1.0,)
        plt.title("Occupancy Grid (mais escuro = maior probabilidade de ocupação)")
        plt.xlabel("x [m]"); plt.ylabel("y [m]")
        plt.savefig(path_png, bbox_inches='tight', dpi=180)
        plt.close()
        
# ========= Utilidades para colisão no Occupancy Grid =========

def is_in_free_space(x, y, prob_grid, params: GridParams, occ_threshold=0.6):
    """
    Retorna True se (x,y) estiver dentro dos limites e em célula com prob. de ocupação < occ_threshold.
    prob_grid: matriz de probabilidades de ocupação (OccupancyGrid.probability()).
    """
    i, j = world_to_grid(x, y, params.xmin, params.ymin, params.res)
    h, w = prob_grid.shape
    if i < 0 or j < 0 or i >= h or j >= w:
        return False
    return prob_grid[i, j] < occ_threshold  # > limiar => tratado como obstáculo


def is_segment_collision_free(x0, y0, x1, y1, prob_grid, params: GridParams, occ_threshold=0.6):
    """
    Usa Bresenham no grid para checar se o segmento (x0,y0)->(x1,y1) passa por alguma célula ocupada.
    """
    cells = bresenham_line(x0, y0, x1, y1,
                        params.xmin, params.ymin,
                        params.res, prob_grid.shape)
    if not cells:
        return False  # sem células, considere inválido
    h, w = prob_grid.shape
    for (i, j) in cells:
        if i < 0 or j < 0 or i >= h or j >= w:
            return False  # saiu do mapa
        if prob_grid[i, j] >= occ_threshold:
            return False  # colisão
    return True

def sample_random_free_goal(prob_grid, params: GridParams,
                            occ_threshold=0.5,
                            current_pose=None,
                            min_dist=0.5,
                            max_tries=500):
    """
    Escolhe aleatoriamente um ponto em área livre (prob < occ_threshold)
    e devolve em coordenadas do mundo (x_goal, y_goal).

    - prob_grid: matriz de probabilidades (grid.probability())
    - params: GridParams do OccupancyGrid
    - current_pose: (x, y) do robô, para evitar goals muito perto
    - min_dist: distância mínima do robô até o goal
    """
    prob = np.asarray(prob_grid, dtype=float)
    h, w = prob.shape

    # células livres: prob < limiar (0.5 => 0.5 é obstáculo/indefinido)
    free_mask = prob < occ_threshold
    free_indices = np.argwhere(free_mask)

    if free_indices.size == 0:
        return None  # ainda não tem área livre mapeada

    xmin, ymin, res = params.xmin, params.ymin, params.res

    for _ in range(max_tries):
        i, j = free_indices[np.random.randint(len(free_indices))]
        # centro da célula em coordenadas do mundo
        x = xmin + (j + 0.5) * res
        y = ymin + (i + 0.5) * res

        if current_pose is not None:
            xr, yr = current_pose
            if math.hypot(x - xr, y - yr) < min_dist:
                continue  # muito perto, tenta outra célula

        return (x, y)

    return None  # não achou nada que respeite min_dist
# ========= Interface com CoppeliaSim =========

class CoppeliaInterface:
    def __init__(self, host="localhost", port=23000, scene_cfg=None):
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        self.client = RemoteAPIClient(host=host, port=port)
        self.sim = self.client.getObject('sim')
        self.scene_cfg = scene_cfg or {}
        self._resolve_handles()

    def _resolve_handles(self):
        sim = self.sim
        def try_get(name):
            try:
                return sim.getObject(name)
            except Exception:
                return None

        def resolve_any(candidates):
            for c in candidates:
                h = try_get(c)
                if h is not None:
                    return h
            raise RuntimeError(f"Não encontrei nenhum dos nomes: {candidates}")

        self.robot = resolve_any(self.scene_cfg.get("robot_base_candidates", ["/kobuki"]))
        self.left_motor = resolve_any(self.scene_cfg.get("left_motor_candidates", ["/leftMotor"]))
        self.right_motor = resolve_any(self.scene_cfg.get("right_motor_candidates", ["/rightMotor"]))
        self.laser = resolve_any(self.scene_cfg.get("laser_candidates", ["/Hokuyo"]))
        
        angle_key = self.scene_cfg.get("hokuyo_angle_data", "hokuyo_angle_data")
        range_key = self.scene_cfg.get("hokuyo_range_data", "hokuyo_range_data")
        self.laser_signal = [angle_key, range_key]

        

    # ---- pose e controle ----
    def get_pose(self):
        # Pose global do robô (x,y,theta)
        pose = self.sim.getObjectPose(self.robot, -1)  # [x y z qx qy qz qw]
        x, y = float(pose[0]), float(pose[1])
        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
        # yaw a partir de quaternion (z-axis)
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        return x, y, yaw

    def set_velocity(self, v_l, v_r, wheel_radius=0.035):
        # Define velocidade angular das rodas a partir de v linear (m/s) por roda → ω = v/r
        wl = float(v_l / wheel_radius)
        wr = float(v_r / wheel_radius)
        self.sim.setJointTargetVelocity(self.left_motor, wl)
        self.sim.setJointTargetVelocity(self.right_motor, wr)

    def read_laser_scan(self):
        sim = self.sim

        def _try_decode_float_table(ft):
            if not ft or len(ft) < 5:
                return None
            # Formato A: [angle_min, angle_increment, range_max, N, r0...]
            if ft[3] > 0 and int(ft[3]) == ft[3]:
                angle_min = float(ft[0])
                angle_inc = float(ft[1])
                range_max = float(ft[2])
                N = int(ft[3])
                ranges = ft[4:4+N]
                return angle_min, angle_inc, 0.0, range_max, np.array(ranges, dtype=np.float32)
            # Formato B: [angle_min, angle_max, angle_increment, range_min, range_max, N, r0...]
            if len(ft) >= 6 and int(ft[5]) == ft[5]:
                angle_min  = float(ft[0])
                angle_max  = float(ft[1])
                angle_inc  = float(ft[2])
                range_min  = float(ft[3])
                range_max  = float(ft[4])
                N = int(ft[5])
                ranges = ft[6:6+N]
                return angle_min, angle_inc, range_min, range_max, np.array(ranges, dtype=np.float32)
            return None

        # 1) tentar pelos string signals (caso alguém use texto/JSON no futuro)
        for sig in self.laser_signal:
            try:
                s = sim.getStringSignal(sig)
                if s:
                    ft = sim.unpackFloatTable(s)
                    decoded = _try_decode_float_table(ft)
                    if decoded:
                        return decoded
            except Exception:
                # pode falhar se o sinal não existir / não for string UTF-8
                pass

        # 2) tentar pelos buffer properties (o que o seu Lua está usando)
        scene_h = sim.handle_scene
        for sig in self.laser_signal:
            key = f"signal.{sig}"
            try:
                b = sim.getBufferProperty(scene_h, key)
                if b:
                    ft = sim.unpackFloatTable(b)
                    decoded = _try_decode_float_table(ft)
                    if decoded:
                        return decoded
            except Exception:
                pass

        raise RuntimeError(
            "Não foi possível ler o laser nem como stringSignal nem como bufferProperty. "
            "Verifique se o script Lua está chamando sim.setBufferProperty(sim.handle_scene, 'signal.hokuyo_range_data', ...) "
            "e se o nome do sinal bate com o config.yaml."
        )

# ========= Controle baseado em trajetória (RRT*) =========
@dataclass
class RRTStarParams:
    max_iters: int = 2000          # Nº máx. de iterações
    step_size: float = 0.25        # passo máximo entre nós consecutivos (m)
    goal_sample_rate: float = 0.1  # prob. de amostrar diretamente o goal
    neighbor_radius: float = 0.5   # raio p/ rewire (m)
    occ_threshold: float = 0.5     # prob. acima disso = obstáculo
    goal_tolerance: float = 0.25   # distância aceitável até o goal (m)


@dataclass
class RRTNode:
    x: float
    y: float
    parent: int   # índice do nó pai na lista 'nodes' (-1 para raiz)
    cost: float   # custo acumulado desde o start


def _extract_path(nodes, idx_goal):
    """Remonta caminho [ (x0,y0), ..., (xg,yg) ] a partir do índice idx_goal."""
    path = []
    i = idx_goal
    while i != -1:
        n = nodes[i]
        path.append((n.x, n.y))
        i = n.parent
    path.reverse()
    return path


def rrt_star(start, goal, occupancy_grid: OccupancyGrid,
                world_bounds=None,
                params: RRTStarParams = None):
    """
    RRT* 2D sobre o OccupancyGrid.
    start, goal: tuplas (x,y) em coordenadas do mundo.
    occupancy_grid: instância de OccupancyGrid já preenchida.
    world_bounds: (xmin, xmax, ymin, ymax). Se None, usa bounds do grid.
    Retorna: lista de (x,y) do start até (aprox.) goal.
    """
    if params is None:
        params = RRTStarParams()

    gp = occupancy_grid.params
    prob = occupancy_grid.probability()

    if world_bounds is None:
        xmin, xmax, ymin, ymax = gp.xmin, gp.xmax, gp.ymin, gp.ymax
    else:
        xmin, xmax, ymin, ymax = world_bounds

    # Nó raiz
    nodes = [RRTNode(start[0], start[1], parent=-1, cost=0.0)]

    def sample_free():
        # goal bias
        if np.random.rand() < params.goal_sample_rate:
            return goal[0], goal[1]
        for _ in range(100):
            xr = np.random.uniform(xmin, xmax)
            yr = np.random.uniform(ymin, ymax)
            if is_in_free_space(xr, yr, prob, gp, params.occ_threshold):
                return xr, yr
        # fallback se não achar nada "bom"
        return goal[0], goal[1]

    for _ in range(params.max_iters):
        x_rand, y_rand = sample_free()

        # Nó mais próximo
        dists = [math.hypot(n.x - x_rand, n.y - y_rand) for n in nodes]
        idx_near = int(np.argmin(dists))
        near = nodes[idx_near]

        # Steer para o amostrado
        theta = math.atan2(y_rand - near.y, x_rand - near.x)
        dist = min(params.step_size, dists[idx_near])
        x_new = near.x + dist * math.cos(theta)
        y_new = near.y + dist * math.sin(theta)

        # Verifica se novo ponto é livre e se o segmento está livre
        if not is_in_free_space(x_new, y_new, prob, gp, params.occ_threshold):
            continue
        if not is_segment_collision_free(near.x, near.y, x_new, y_new, prob, gp, params.occ_threshold):
            continue

        new_node = RRTNode(x_new, y_new, parent=idx_near, cost=near.cost + dist)

        # Região de vizinhança p/ RRT* (rewire)
        neighbor_indices = []
        for i, n in enumerate(nodes):
            d = math.hypot(n.x - x_new, n.y - y_new)
            if d <= params.neighbor_radius:
                if is_segment_collision_free(n.x, n.y, x_new, y_new, prob, gp, params.occ_threshold):
                    neighbor_indices.append(i)

        # Escolhe melhor pai entre vizinhos (menor custo)
        best_parent = idx_near
        best_cost = new_node.cost
        for i in neighbor_indices:
            n = nodes[i]
            c = n.cost + math.hypot(n.x - x_new, n.y - y_new)
            if c < best_cost:
                best_cost = c
                best_parent = i
        new_node.parent = best_parent
        new_node.cost = best_cost

        nodes.append(new_node)
        new_idx = len(nodes) - 1

        # Rewire: tenta melhorar vizinhos passando pelo novo nó
        for i in neighbor_indices:
            n = nodes[i]
            c_through_new = new_node.cost + math.hypot(n.x - new_node.x, n.y - new_node.y)
            if c_through_new + 1e-6 < n.cost:
                if is_segment_collision_free(n.x, n.y, new_node.x, new_node.y, prob, gp, params.occ_threshold):
                    n.parent = new_idx
                    n.cost = c_through_new

        # Verifica se chegou (ou quase) no goal
        d_goal = math.hypot(new_node.x - goal[0], new_node.y - goal[1])
        if d_goal <= params.goal_tolerance:
            # Checa segmento final new -> goal
            if is_segment_collision_free(new_node.x, new_node.y,
                                        goal[0], goal[1],
                                        prob, gp, params.occ_threshold):
                goal_node = RRTNode(goal[0], goal[1],
                                    parent=new_idx,
                                    cost=new_node.cost + d_goal)
                nodes.append(goal_node)
                return _extract_path(nodes, len(nodes) - 1)

    # Se não conectou o goal, devolve melhor nó mais próximo do goal
    best_idx = None
    best_dist = float("inf")
    for i, n in enumerate(nodes):
        d = math.hypot(n.x - goal[0], n.y - goal[1])
        if d < best_dist:
            best_dist = d
            best_idx = i

    if best_idx is not None:
        return _extract_path(nodes, best_idx)

    # fallback bem simples
    return [start, goal]

class RRTStarController:
    """
    Segue uma trajetória (lista de waypoints) gerada por um planejador RRT* externo.

    Use:
        controller.set_path(path)
    onde:
        path = [(x0, y0), (x1, y1), ...] em coordenadas do mundo.
    """
    def __init__(self, v_linear=0.25, v_angular_max=0.8, waypoint_tol=0.10):
        self.v_linear = v_linear
        self.v_angular_max = v_angular_max
        self.waypoint_tol = waypoint_tol
        self.path = []
        self.idx = 0

    def set_path(self, path):
        self.path = list(path) if path is not None else []
        self.idx = 0

    def is_finished(self):
        return self.idx >= len(self.path)

    def control(self, x, y, yaw):
        # se não tiver caminho ou já terminou, para o robô
        if self.is_finished() or not self.path:
            return 0.0, 0.0

        # waypoint corrente
        tx, ty = self.path[self.idx]
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)

        # se já chegou nesse waypoint, vai para o próximo
        if dist < self.waypoint_tol:
            self.idx += 1
            return self.control(x, y, yaw)

        # direção desejada no mundo e erro relativo ao robô
        target_heading = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(target_heading - yaw),
            math.cos(target_heading - yaw)
        )

        # controle proporcional simples em w, saturado
        k_ang = 2.0
        w_cmd = k_ang * heading_error
        w_cmd = max(-self.v_angular_max, min(self.v_angular_max, w_cmd))

        # se o erro de heading for grande, reduz v
        if abs(heading_error) > math.radians(80):
            v_cmd = 0.0
        else:
            v_cmd = self.v_linear

        return v_cmd, w_cmd

def diff_drive(v, w, L=0.230):
    # Converte v,w (no centro do eixo) para velocidades lineares por roda
    v_l = v - (L/2.0)*w
    v_r = v + (L/2.0)*w
    return v_l, v_r

def next_indexed_filename(base_without_ext, ext=".png"):
    """Retorna base_01.png, base_02.png, ... sem sobrescrever arquivos existentes."""
    idx = 1
    while True:
        fname = f"{base_without_ext}_{idx:02d}{ext}"
        if not os.path.exists(fname):
            return fname
        idx += 1

# ========= Loop principal =========

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cell-size", type=float, required=True, help="Tamanho da célula (m). Ex.: 0.01, 0.1, 0.5")
    parser.add_argument("--duration", type=float, default=180.0, help="Duração (s) da exploração")
    parser.add_argument("--noise-range", type=float, default=0.0, help="Desvio-padrão do ruído no alcance (m)")
    parser.add_argument("--noise-angle-deg", type=float, default=0.0, help="Desvio-padrão do ruído no ângulo (graus)")
    parser.add_argument("--scenario", type=str, default="static", help="Rótulo do cenário (static/dynamic)")
    parser.add_argument("--config", type=str, default="configs/config.yaml")
    args = parser.parse_args()

    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)

    # Interface com CoppeliaSim
    ci = CoppeliaInterface(host=cfg["coppeliasim"]["host"], port=cfg["coppeliasim"]["port"], scene_cfg=cfg.get("scene", {}))

    # Grid
    xmin, xmax, ymin, ymax = cfg["mapping"]["world_bounds"]
    gp = GridParams(xmin, xmax, ymin, ymax, res=args.cell_size,
                    p0=cfg["mapping"]["p0"], p_occ=cfg["mapping"]["p_occ"], p_free=cfg["mapping"]["p_free"],
                    l_min=cfg["mapping"]["l_min"], l_max=cfg["mapping"]["l_max"])
    grid = OccupancyGrid(gp)

    # ===== Controle via RRT* =====
    nav_cfg = cfg["navigation"]
    controller = RRTStarController(
        v_linear=nav_cfg.get("v_linear", 0.25),
        v_angular_max=nav_cfg.get("v_angular", 0.8),
        waypoint_tol=nav_cfg.get("waypoint_tolerance", 0.10),
    )
    rrt_params = RRTStarParams(
        max_iters=nav_cfg.get("rrt_max_iters", 2000),
        step_size=nav_cfg.get("rrt_step_size", 0.25),
        goal_sample_rate=nav_cfg.get("rrt_goal_sample_rate", 0.1),
        neighbor_radius=nav_cfg.get("rrt_neighbor_radius", 0.5),
        occ_threshold=nav_cfg.get("rrt_occ_threshold", 0.5),
        goal_tolerance=nav_cfg.get("rrt_goal_tolerance", 0.25),
    )

    # distância mínima entre robô e um novo goal
    min_goal_dist = nav_cfg.get("min_goal_distance", 0.5)

    current_goal = None

    # Logs para plot incremental
    path_xy = []
    laser_endpoints = []  # lista de segmentos [(x0,y0,x1,y1), ...] amostrados

    t0 = time.time()
    last_plot = t0
    angle_noise_std = math.radians(args.noise_angle_deg)
    L_wheel = 0.230
    r_wheel = 0.035

    try:
        while True:
            tnow = time.time()
            if (tnow - t0) > args.duration:
                break

            # Pose do robô
            x, y, yaw = ci.get_pose()
            path_xy.append((x, y))

            # Leitura do laser
            angle_min, angle_inc, range_min, range_max, ranges = ci.read_laser_scan()

            # Aplica ruídos (ângulo e alcance)
            if angle_noise_std > 0.0 or args.noise_range > 0.0:
                N = len(ranges)
                angle_offsets = np.random.normal(0.0, angle_noise_std, size=N)
                range_offsets = np.random.normal(0.0, args.noise_range, size=N)
            else:
                N = len(ranges)
                angle_offsets = np.zeros(N)
                range_offsets = np.zeros(N)

            # Atualiza grid feixe a feixe
            for k in range(len(ranges)):
                r = float(ranges[k] + range_offsets[k])
                r = max(0.0, min(r, range_max))
                ang = angle_min + k*angle_inc + angle_offsets[k]

                # Ponto do impacto no mundo
                # Laser assumido na base do robô (ajuste aqui se seu laser tiver offset)
                lx, ly = x, y
                hx = lx + r * math.cos(yaw + ang)
                hy = ly + r * math.sin(yaw + ang)

                hit = (r < (range_max - 1e-3))
                grid.update_ray(lx, ly, hx, hy, hit=hit)

                # Salva amostras esparsas para o plot incremental
                if k % max(1, len(ranges)//64) == 0:
                    laser_endpoints.append((lx, ly, hx, hy))
                    
            # Depois de atualizar o Occupancy Grid, pegamos probabilidades
            prob = grid.probability()

            # Se o caminho atual acabou (ou ainda não existe), escolhe um novo goal livre
            if controller.is_finished():
                goal = sample_random_free_goal(
                    prob_grid=prob,
                    params=gp,
                    occ_threshold=rrt_params.occ_threshold,
                    current_pose=(x, y),
                    min_dist=min_goal_dist,
                )

                if goal is not None:
                    print(f"[RRT*] Novo goal aleatório em área livre: {goal}")
                    rrt_path = rrt_star(
                        start=(x, y),
                        goal=goal,
                        occupancy_grid=grid,
                        world_bounds=(xmin, xmax, ymin, ymax),
                        params=rrt_params,
                    )
                    controller.set_path(rrt_path)
                    current_goal = goal
                else:
                    # ainda não há células livres mapeadas: o robô fica parado
                    print("[RRT*] Nenhuma área livre suficiente ainda; aguardando mais medições.")
            # Controle de locomoção
            v_cmd, w_cmd = controller.control(x, y, yaw)
            v_l, v_r = diff_drive(v_cmd, w_cmd, L=L_wheel)
            ci.set_velocity(v_l, v_r, wheel_radius=r_wheel)

            # Pequena cadência para não sobrecarregar
            time.sleep(0.05)

    finally:
        # Para o robô ao fim
        ci.set_velocity(0.0, 0.0, wheel_radius=r_wheel)

    # ---- Salva resultados ----
    os.makedirs("output", exist_ok=True)
    scenario = args.scenario
    res_str = str(args.cell_size).replace(".", "p")
    map_png = f"output/map_{scenario}_res{res_str}.png"
    grid.save_image(map_png)

    # Matriz de probabilidades
    np.save(f"output/map_{scenario}_res{res_str}.npy", grid.probability())
    base = os.path.join("output", f"path_laser_{scenario}_res{res_str}")
    path_plot_png = next_indexed_filename(base, ext=".png")
    
    plt.figure()
    ax = plt.gca()
    ax.set_facecolor("1.0")  # fundo branco
    # trajetória em preto
    if len(path_xy) > 1:
        xs, ys = zip(*path_xy)
        plt.plot(xs, ys, color="0.0", linewidth=1.5)
    # feixes em cinza
    for (x0, y0, x1, y1) in laser_endpoints[::2]:
        plt.plot([x0, x1], [y0, y1], color="0.6", linewidth=0.5)
    plt.axis('equal')
    plt.xlabel("x [m]"); plt.ylabel("y [m]")
    plt.title("Plot incremental: trajetória e feixes do laser (escala de cinza)")
    plt.savefig(path_plot_png, bbox_inches='tight', dpi=180)
    plt.close()

    print("Concluído.")
    print("Arquivos gerados:")
    print(" -", map_png)
    print(" -", path_plot_png)
    print(" -", f"output/map_{scenario}_res{res_str}.npy")
    
if __name__ == "__main__":
    main()
