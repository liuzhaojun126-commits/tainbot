#!/usr/bin/env python3
"""
Robust offline minimum-time race plan generator (F1TENTH tianracer_racetrack).

Key design (conservatism first):
  - Corridor: ray-cast along centerline normals on BOTH the PGM occupancy grid
    and the guardrail wall.STL; take the smaller (conservative) distance.
  - Racing line: minimum-curvature convex QP (Heilmeier method) with box bounds
    (clearance >= DESIGN_CLEAR) and a hard finite-difference curvature limit.
  - A "trust scale" lambda shrinks the QP lateral offsets toward the feasible
    centerline until the FINAL dense line passes every gate.
  - The curvature used for BOTH the gates and the velocity profile is the
    robust circumcircle curvature plus a rolling upper envelope (the same
    estimator that gives the battle-tested centerline kmax=2.31), so isolated
    kink artifacts can never be missed by the speed planner.
"""
import os, math, struct, json, argparse
import numpy as np
from scipy.interpolate import splprep, splev
from scipy.spatial import cKDTree
import cvxpy as cp

# ---------------- verified geometry constants ----------------
STL_YAW     = 1.57141
STL_TRANS   = np.array([-1.43091, -1.91728])
MODEL_TRANS = np.array([0.071364, 0.211162])
PGM_RES     = 0.025
PGM_ORIGIN  = np.array([-12.2, -12.5])

WHEELBASE   = 0.26
DELTA_DESIGN= 0.58
KAPPA_HARD  = math.tan(DELTA_DESIGN) / WHEELBASE   # ~2.50 absolute gate
DESIGN_CLEAR= 0.18      # target wall clearance (car half-width 0.10 + margin 0.08)
ABS_CLEAR   = 0.15      # absolute floor
STATION_DS  = 0.10      # QP station spacing
RESAMPLE_DS = 0.12      # final line spacing
QP_KAPPA_LIM= 2.20      # hard FD curvature limit inside QP
FINAL_KAPPA = 2.40      # preferred final robust-kappa gate
FINAL_KAPPA_HARD = 2.45 # absolute final robust-kappa gate

A_LAT = [8.0, 9.0, 10.0, 12.0]
VMAX  = [6.0, 7.0, 8.0]
ACC, DEC = 6.0, 9.0


# ---------------- geometry loaders ----------------
def load_centerline(path):
    return np.array([[float(a), float(b)] for a, b in
                     (l.split(',') for l in open(path) if l.strip() and not l.startswith('#'))])

def load_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        data = f.read(w * h)
    grid = np.frombuffer(data, np.uint8).reshape(h, w)[::-1, :]
    return grid

def load_wall_world(path):
    d = open(path, 'rb').read()
    nt = struct.unpack('<I', d[80:84])[0]
    off, pts = 84, []
    for _ in range(nt):
        v = struct.unpack('<12fH', d[off:off + 50])
        pts += [v[3:5], v[6:8], v[9:11]]
        off += 50
    pts = np.array(pts)
    c, s = math.cos(STL_YAW), math.sin(STL_YAW)
    R = np.array([[c, -s], [s, c]])
    return (R @ pts.T).T + STL_TRANS + MODEL_TRANS

def world_to_map(x, y):
    col = int((x - PGM_ORIGIN[0]) / PGM_RES)
    row = int((y - PGM_ORIGIN[1]) / PGM_RES)
    return col, row

def occupied(grid, col, row):
    h, w = grid.shape
    if col < 0 or col >= w or row < 0 or row >= h:
        return True
    return grid[row, col] < 254   # anything not clearly free


# ---------------- densify the feasible centerline ----------------
def densify(pts, ds):
    seg = np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)
    total = seg.sum()
    tck, _ = splprep([pts[:, 0], pts[:, 1]], s=0, per=True, k=3)
    n = int(total / ds)
    u = np.linspace(0, 1, n, endpoint=False)
    x, y = splev(u, tck)
    return np.column_stack([x, y])

def tangents_normals(pts):
    dx = np.roll(pts[:, 0], -1) - np.roll(pts[:, 0], 1)
    dy = np.roll(pts[:, 1], -1) - np.roll(pts[:, 1], 1)
    t = np.column_stack([dx, dy])
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    nm = np.column_stack([-t[:, 1], t[:, 0]])
    return t, nm


# ---------------- corridors ----------------
def corridor_pgm(pts, nm, grid, max_d=3.0):
    n = len(pts)
    L = np.full(n, max_d); Rr = np.full(n, max_d)
    step = PGM_RES * 0.5
    for i in range(n):
        d = step
        while d < max_d:
            c, r = world_to_map(pts[i, 0] + nm[i, 0] * d, pts[i, 1] + nm[i, 1] * d)
            if occupied(grid, c, r):
                L[i] = d; break
            d += step
        d = step
        while d < max_d:
            c, r = world_to_map(pts[i, 0] - nm[i, 0] * d, pts[i, 1] - nm[i, 1] * d)
            if occupied(grid, c, r):
                Rr[i] = d; break
            d += step
    return L, Rr

def corridor_stl(pts, nm, tg, wall, tree, max_d=2.6, tang_gate=0.5):
    n = len(pts)
    L = np.full(n, max_d); Rr = np.full(n, max_d)
    for i in range(n):
        idx = tree.query_ball_point(pts[i], max_d)
        if not idx:
            continue
        rel = wall[idx] - pts[i]
        an = rel @ nm[i]; at = rel @ tg[i]
        m = np.abs(at) < tang_gate
        pos, neg = an[m & (an > 0.02)], an[m & (an < -0.02)]
        if len(pos): L[i] = pos.min()
        if len(neg): Rr[i] = abs(neg.max())
    return L, Rr


# ---------------- minimum curvature QP ----------------
def solve_qp(ref, nm, kref, L, Rr, clear=DESIGN_CLEAR):
    n = len(ref)
    ds = STATION_DS
    up = L - clear
    lo = -(Rr - clear)
    up = np.minimum(up, 1.2); lo = np.maximum(lo, -1.2)
    lo = np.minimum(lo, up - 0.02)

    x = cp.Variable(n)
    kappa = []
    for i in range(n):
        ip, im = (i + 1) % n, (i - 1) % n
        kappa.append(kref[i] + (x[ip] - 2 * x[i] + x[im]) / ds ** 2)
    cost = cp.sum_squares(cp.hstack(kappa)) * ds
    cost += 0.02 * cp.sum_squares(x)
    cons = [x >= lo, x <= up]
    for k in kappa:
        cons += [k <= QP_KAPPA_LIM, k >= -QP_KAPPA_LIM]
    prob = cp.Problem(cp.Minimize(cost), cons)
    prob.solve(solver=cp.OSQP, max_iter=40000, eps_abs=1e-8, eps_rel=1e-8, polish=True)
    if x.value is None:
        prob.solve(solver=cp.SCS, max_iter=20000)
    xv = x.value
    if xv is None:
        xv = np.zeros(n)
    return xv


# ---------------- resample + robust curvature ----------------
def resample_arc(pts, ds_target=RESAMPLE_DS):
    seg = np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    closed = np.vstack([pts, pts[0]])
    total = s[-1]
    n = max(int(total / ds_target), 100)
    sd = np.linspace(0, total, n, endpoint=False)
    xi = np.interp(sd, s, closed[:, 0])
    yi = np.interp(sd, s, closed[:, 1])
    out = np.column_stack([xi, yi])
    dsv = np.linalg.norm(np.roll(out, -1, axis=0) - out, axis=1)
    return out, dsv

def circumcircle_kappa(pts):
    n = len(pts); k = np.zeros(n)
    for i in range(n):
        p0, p1, p2 = pts[i - 1], pts[i], pts[(i + 1) % n]
        a = np.linalg.norm(p1 - p0); b = np.linalg.norm(p2 - p1); c = np.linalg.norm(p2 - p0)
        area = abs((p1[0] - p0[0]) * (p2[1] - p0[1]) -
                   (p1[1] - p0[1]) * (p2[0] - p0[0])) / 2
        if area > 1e-9:
            k[i] = 4 * area / (a * b * c)
    return k

def rolling_envelope(k, passes_=2):
    e = k.copy()
    for _ in range(passes_):
        e = np.maximum(e, np.roll(e, 1))
        e = np.maximum(e, np.roll(e, -1))
    return e


# ---------------- velocity profile ----------------
def velocity(kappa, dsv, alat, vmax, acc=ACC, dec=DEC):
    n = len(kappa)
    vc = np.minimum(vmax, np.sqrt(alat / np.maximum(kappa, 1e-6)))
    v = vc.copy()
    for _ in range(40):
        vold = v.copy()
        v[0] = min(vc[0], math.sqrt(v[n - 1] ** 2 + 2 * acc * dsv[n - 1]))
        for i in range(1, n):
            v[i] = min(vc[i], math.sqrt(v[i - 1] ** 2 + 2 * acc * dsv[i - 1]))
        v[n - 1] = min(v[n - 1], math.sqrt(v[0] ** 2 + 2 * dec * dsv[0]))
        for i in range(n - 2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * dec * dsv[i]))
        if np.abs(v - vold).max() < 0.005:
            break
    t = float(np.sum(dsv / np.maximum(v, 0.01)))
    return v, t


def main():
    ap = argparse.ArgumentParser()
    sd0 = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.abspath(os.path.join(sd0, '..', '..'))
    ap.add_argument('--pgm', default=os.path.join(pkg, 'maps', 'tianracer_racetrack.pgm'))
    ap.add_argument('--centerline', default=os.path.join(sd0, 'tianracer_racetrack_centerline.csv'))
    ap.add_argument('--stl', default=os.path.join(pkg, 'worlds', 'tianracer_racetrack', 'meshes', 'wall.STL'))
    ap.add_argument('--outdir', default=sd0)
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    cl = load_centerline(a.centerline)
    grid = load_pgm(a.pgm)
    wall = load_wall_world(a.stl)
    tree = cKDTree(wall)

    ref = densify(cl, STATION_DS)
    tg, nm = tangents_normals(ref)
    kref = rolling_envelope(circumcircle_kappa(ref), 1)

    Lp, Rp = corridor_pgm(ref, nm, grid)
    Ls, Rs = corridor_stl(ref, nm, tg, wall, tree)
    L = np.minimum(Lp, Ls); Rr = np.minimum(Rp, Rs)

    x = solve_qp(ref, nm, kref, L, Rr)

    def build(lam):
        line = ref + nm * (lam * x)[:, None]
        out, dsv = resample_arc(line)
        kraw = circumcircle_kappa(out)
        kenv = rolling_envelope(kraw)
        clear, _ = tree.query(out)
        return out, dsv, kraw, kenv, clear

    chosen = None
    for q in range(51):
        lam = 1.0 - 0.02 * q
        out, dsv, kraw, kenv, clear = build(lam)
        if kenv.max() <= FINAL_KAPPA_HARD and clear.min() >= ABS_CLEAR:
            chosen = lam
            break
    if chosen is None:
        raise RuntimeError('no feasible lambda')
    out, dsv, kraw, kenv, clear = build(chosen)

    gates = {
        'kappa_hard(<=%.2f)' % FINAL_KAPPA_HARD: bool(kenv.max() <= FINAL_KAPPA_HARD),
        'kappa_target(<=%.2f)' % FINAL_KAPPA: bool(kenv.max() <= FINAL_KAPPA),
        'clear_hard(>=%.2f)' % ABS_CLEAR: bool(clear.min() >= ABS_CLEAR),
        'clear_target(>=%.2f)' % DESIGN_CLEAR: bool(np.percentile(clear, 5) >= DESIGN_CLEAR),
    }
    length = float(dsv.sum())
    print('lambda %.2f  n=%d length=%.2f' % (chosen, len(out), length))
    print('robust kappa max %.3f (raw %.3f); delta %.2f deg' %
          (kenv.max(), kraw.max(), math.degrees(math.atan(WHEELBASE * kenv.max()))))
    print('clearance min/p5/mean %.3f/%.3f/%.3f' % (clear.min(), np.percentile(clear, 5), clear.mean()))
    print('gates', gates)

    tiers = {}
    for al in A_LAT:
        for vm in VMAX:
            v, t = velocity(kenv, dsv, al, vm)
            tiers[(al, vm)] = (v, t)
            print('alat=%2d vmax=%d t=%.2f vmin=%.2f cov=%.1f%%' %
                  (al, vm, t, v.min(), 100 * np.mean(v >= vm * 0.95)))

    def write(path, v, tag):
        with open(path, 'w') as f:
            f.write('# x,y,v_target,kappa  %s\n' % tag)
            for i in range(len(out)):
                f.write('%.6f,%.6f,%.6f,%.6f\n' % (out[i, 0], out[i, 1], v[i], kenv[i]))

    for (al, vm), (v, t) in tiers.items():
        write(os.path.join(a.outdir, 'tianracer_racetrack_raceline_alat%d_vmax%d.csv' % (int(al), int(vm))),
              v, 'alat=%d vmax=%d t_kin=%.2f' % (int(al), int(vm), t))
    write(os.path.join(a.outdir, 'tianracer_racetrack_raceline.csv'), tiers[(8., 6.)][0],
          'default alat=8 vmax=6')

    report = {
        'chosen_lambda': chosen, 'length_m': length,
        'kappa_robust_max': float(kenv.max()), 'kappa_raw_max': float(kraw.max()),
        'delta_max_deg': math.degrees(math.atan(WHEELBASE * kenv.max())),
        'clearance_min': float(clear.min()), 'clearance_p5': float(np.percentile(clear, 5)),
        'clearance_mean': float(clear.mean()), 'n_points': len(out),
        'gates': gates,
        'tiers': {'a%d_v%d' % (int(al), int(vm)):
                  {'t_kin': t, 'vmin': float(v.min()),
                   'vmax_cov_pct': 100 * float(np.mean(v >= vm * .95))}
                  for (al, vm), (v, t) in sorted(tiers.items())},
    }
    json.dump(report, open(os.path.join(a.outdir, 'robust_report.json'), 'w'), indent=2)
    print('wrote CSVs + robust_report.json to', a.outdir)


if __name__ == '__main__':
    main()
