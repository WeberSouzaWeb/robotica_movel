#!/usr/bin/env python3
import argparse, time, math, os, json
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
import yaml

def logit(p: float) -> float:
    eps = 1e-9
    p = min(max(p, eps), 1 - eps)
    return math.log(p / (1 - p))

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
        def world_to_grid(x, y, xmin, ymin, res):
            i = int((y - ymin) / res)
            j = int((x - xmin) / res)
            return i, j
        def bresenham_line(x0, y0, x1, y1, xmin, ymin, res, shape):
            dx = x1 - x0; dy = y1 - y0
            L = max(abs(dx), abs(dy)) / max(res, 1e-6)
            n = int(max(1, L))
            xs = np.linspace(x0, x1, n); ys = np.linspace(y0, y1, n)
            cells = []
            for k in range(n):
                i, j = world_to_grid(xs[k], ys[k], xmin, ymin, res)
                if 0 <= i < shape[0] and 0 <= j < shape[1]:
                    if not cells or cells[-1] != (i, j):
                        cells.append((i, j))
            return cells

        cells = bresenham_line(ox, oy, hx, hy, self.params.xmin, self.params.ymin, self.params.res, self.grid.shape)
        if not cells: return
        free_cells = cells[:-1] if hit else cells
        for (i, j) in free_cells:
            self.grid[i, j] = np.clip(self.grid[i, j] + self.L_free, self.params.l_min, self.params.l_max)
        if hit and len(cells) > 0:
            i, j = cells[-1]
            self.grid[i, j] = np.clip(self.grid[i, j] + self.L_occ, self.params.l_min, self.params.l_max)

    def probability(self):
        return 1.0 / (1.0 + np.exp(-self.grid))

    def save_image(self, path_png):
        p = self.probability()
        img = (1.0 - p)
        plt.figure()
        plt.imshow(img, origin='lower', extent=[self.params.xmin, self.params.xmax, self.params.ymin, self.params.ymax])
        plt.title("Occupancy Grid (escuro=ocupado)")
        plt.xlabel("x [m]"); plt.ylabel("y [m]")
        plt.savefig(path_png, bbox_inches='tight', dpi=180)
        plt.close()

class CoppeliaInterface:
    def __init__(self, host="localhost", port=23000, scene_cfg=None):
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        self.client = RemoteAPIClient(host=host, port=port)
        self.sim = self.client.getObject('sim')
        self.scene_cfg = scene_cfg or {}
        self._resolve_handles()

    def _try_get(self, name):
        try: return self.sim.getObject(name)
        except Exception: return None

    def _resolve_any(self, candidates, what):
        for c in candidates:
            h = self._try_get(c)
            if h is not None: return h
        raise RuntimeError(f"Não encontrei nenhum dos nomes para {what}: {candidates}")

    def _resolve_handles(self):
        sim = self.sim
        self.robot = self._resolve_any(self.scene_cfg.get("robot_base_candidates", ["/kobuki", "/Kobuki", "/Robot", "/Base"]), "base do robô")
        # motores
        self.left_motor = None; self.right_motor = None
        for c in self.scene_cfg.get("left_motor_candidates", []):
            self.left_motor = self._try_get(c) or (self._try_get(c[1:]) if c.startswith('/') else None) or self.left_motor
        for c in self.scene_cfg.get("right_motor_candidates", []):
            self.right_motor = self._try_get(c) or (self._try_get(c[1:]) if c.startswith('/') else None) or self.right_motor
        if self.left_motor is None or self.right_motor is None:
            jt = sim.object_joint_type
            joints = sim.getObjectsInTree(self.robot, jt, 0)
            if len(joints) >= 2:
                joints = [(h, sim.getObjectAlias(h,1), sim.getObjectPose(h,self.robot)[1]) for h in joints]
                joints.sort(key=lambda t: t[2])
                self.right_motor = self.right_motor or joints[0][0]
                self.left_motor  = self.left_motor  or joints[-1][0]
        # laser
        self.laser = None
        try:
            self.laser = self._resolve_any(self.scene_cfg.get("laser_candidates", ["/Hokuyo", "/fastHokuyo", "/laser"]), "laser")
        except Exception:
            self.laser = None
        self.laser_signal = self.scene_cfg.get("laser_signal", ["hokuyo_data", "laser_scan", "scan_data", "hokuyo_range_data"])

    def _range_max_from_child_sensor(self):
        try:
            if self.laser is None: return None
            vs_type = self.sim.object_visionsensor_type
            sensors = self.sim.getObjectsInTree(self.laser, vs_type, 0)
            if sensors:
                return float(self.sim.getObjectFloatParam(sensors[0], self.sim.visionfloatparam_far_clipping))
        except Exception:
            pass
        return None

    def _guess_range_max(self, dists):
        try: return max(float(max(dists)), 0.01)
        except Exception: return 10.0

    def get_pose(self):
        pose = self.sim.getObjectPose(self.robot, -1)
        x, y = float(pose[0]), float(pose[1])
        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        return x, y, yaw

    def set_velocity(self, v_l, v_r, wheel_radius=0.035):
        wl = float(v_l / wheel_radius); wr = float(v_r / wheel_radius)
        self.sim.setJointTargetVelocity(self.left_motor, wl)
        self.sim.setJointTargetVelocity(self.right_motor, wr)

    def read_laser_scan(self):
        # 1) Sinais compactos: Formato A/B
        for sig in self.laser_signal:
            try:
                s = self.sim.getStringSignal(sig)
                if not s: continue
                data = self.sim.unpackFloatTable(s)
                if len(data) < 5: continue
                if data[3] > 0 and int(data[3]) == data[3]:
                    angle_min, angle_increment, range_max, N = data[0], data[1], data[2], int(data[3])
                    ranges = data[4:4+N]
                    return angle_min, angle_increment, None, range_max, np.array(ranges, dtype=np.float32)
                if len(data) >= 6 and int(data[5]) == data[5]:
                    angle_min, angle_max, angle_increment, range_min, range_max, N = data[:6]
                    ranges = data[6:6+int(N)]
                    return angle_min, angle_increment, range_min, range_max, np.array(ranges, dtype=np.float32)
            except Exception:
                pass
        # 2) Pares ângulos+distâncias do fastHokuyo
        try:
            sA = self.sim.getStringSignal('hokuyo_angle_data')
            sR = self.sim.getStringSignal('hokuyo_range_data')
            if sA and sR:
                angles = self.sim.unpackFloatTable(sA)
                dists  = self.sim.unpackFloatTable(sR)
                if angles and dists and len(angles) == len(dists) and len(angles) >= 2:
                    N = len(angles)
                    a0 = float(angles[0]); a1 = float(angles[-1])
                    angle_min = a0
                    angle_inc = (a1 - a0) / (N - 1)
                    range_max = self._range_max_from_child_sensor() or self._guess_range_max(dists)
                    return angle_min, angle_inc, None, float(range_max), np.array(dists, dtype=np.float32)
        except Exception:
            pass
        raise RuntimeError("Não foi possível ler o sinal do laser. Publique 'hokuyo_data' (Formato A/B) ou 'hokuyo_angle_data'+'hokuyo_range_data'.")

class Explorer:
    def __init__(self, nav_cfg):
        self.v_lin = nav_cfg.get("v_linear", 0.25)
        self.v_ang = nav_cfg.get("v_angular", 0.8)
        self.min_obs = nav_cfg.get("min_obstacle_distance", 0.35)
        self.recalc_period = nav_cfg.get("frontier_recalc_period", 3.0)
        self.last_recalc = 0.0
        self.target_bearing = 0.0

    def pick_bearing(self, ranges, angle_min, angle_inc):
        N = len(ranges)
        if N == 0: return 0.0
        w = max(3, N//36)
        smooth = np.convolve(ranges, np.ones(w)/w, mode='same')
        k_best = int(np.argmax(smooth))
        return angle_min + k_best * angle_inc

    def control(self, ranges, angle_min, angle_inc, tnow):
        rmin = float(np.min(ranges)) if len(ranges) else 10.0
        if (tnow - self.last_recalc) > self.recalc_period:
            self.target_bearing = self.pick_bearing(ranges, angle_min, angle_inc)
            self.last_recalc = tnow
        ang_cmd = float(np.clip(self.target_bearing, -self.v_ang, self.v_ang))
        if rmin < self.min_obs:
            return 0.0, math.copysign(self.v_ang, self.target_bearing if self.target_bearing!=0 else 1.0)
        return self.v_lin, ang_cmd

def diff_drive(v, w, L=0.230):
    return v - (L/2.0)*w, v + (L/2.0)*w

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cell-size", type=float, default=0.1)
    p.add_argument("--duration", type=float, default=120.0)
    p.add_argument("--noise-range", type=float, default=0.0)
    p.add_argument("--noise-angle-deg", type=float, default=0.0)
    p.add_argument("--scenario", type=str, default="static")
    p.add_argument("--config", type=str, default="configs/config.yaml")
    p.add_argument("--list-handles", action="store_true")
    args = p.parse_args()

    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)

    ci = CoppeliaInterface(host=cfg["coppeliasim"]["host"], port=cfg["coppeliasim"]["port"], scene_cfg=cfg.get("scene", {}))

    if args.list-handles:
        info = {"robot_base": None, "laser_signal_to_try": None}
        try:
            info["robot_base"] = ci.sim.getObjectAlias(ci.robot, 1)
        except Exception: pass
        info["laser_signal_to_try"] = ci.laser_signal
        print(json.dumps(info, indent=2, ensure_ascii=False))
        return

    xmin, xmax, ymin, ymax = cfg["mapping"]["world_bounds"]
    gp = GridParams(xmin, xmax, ymin, ymax, res=args.cell_size,
                    p0=cfg["mapping"]["p0"], p_occ=cfg["mapping"]["p_occ"], p_free=cfg["mapping"]["p_free"],
                    l_min=cfg["mapping"]["l_min"], l_max=cfg["mapping"]["l_max"])
    grid = OccupancyGrid(gp)

    explorer = Explorer(cfg["navigation"])

    path_xy = []
    laser_endpoints = []

    t0 = time.time()
    angle_noise_std = math.radians(args.noise_angle_deg)
    L_wheel = 0.230; r_wheel = 0.035

    try:
        while time.time() - t0 < args.duration:
            x, y, yaw = ci.get_pose()
            path_xy.append((x, y))

            angle_min, angle_inc, range_min, range_max, ranges = ci.read_laser_scan()

            N = len(ranges)
            if N > 0 and (angle_noise_std > 0 or args.noise_range > 0):
                angle_offsets = np.random.normal(0.0, angle_noise_std, size=N)
                range_offsets = np.random.normal(0.0, args.noise_range, size=N)
            else:
                angle_offsets = np.zeros(N); range_offsets = np.zeros(N)

            for k in range(N):
                r = float(max(0.0, min(ranges[k] + range_offsets[k], range_max)))
                ang = angle_min + k*angle_inc + angle_offsets[k]
                lx, ly = x, y
                hx = lx + r * math.cos(yaw + ang)
                hy = ly + r * math.sin(yaw + ang)
                hit = (r < (range_max - 1e-3))
                grid.update_ray(lx, ly, hx, hy, hit=hit)
                if k % max(1, N//64) == 0:
                    laser_endpoints.append((lx, ly, hx, hy))

            v_cmd, w_cmd = explorer.control(ranges, angle_min, angle_inc, time.time())
            v_l, v_r = diff_drive(v_cmd, w_cmd, L=L_wheel)
            ci.set_velocity(v_l, v_r, wheel_radius=r_wheel)

            time.sleep(0.05)
    finally:
        ci.set_velocity(0.0, 0.0, wheel_radius=r_wheel)

    os.makedirs("output", exist_ok=True)
    res_str = str(args.cell_size).replace(".", "p")
    map_png = f"output/map_{args.scenario}_res{res_str}.png"
    grid.save_image(map_png)
    npy_path = f"output/map_{args.scenario}_res{res_str}.npy"
    np.save(npy_path, grid.probability())

    import matplotlib.pyplot as plt
    plt.figure()
    if len(path_xy) > 1:
        xs, ys = zip(*path_xy)
        plt.plot(xs, ys, linewidth=1.5)
    for (x0,y0,x1,y1) in laser_endpoints[::2]:
        plt.plot([x0, x1], [y0, y1], linewidth=0.5)
    plt.axis('equal'); plt.title('Plot incremental: trajetória e feixes')
    plt.xlabel('x [m]'); plt.ylabel('y [m]')
    plt.savefig(f"output/path_laser_{args.scenario}_res{res_str}.png", bbox_inches='tight', dpi=180)
    plt.close()

    print("Concluído.")
