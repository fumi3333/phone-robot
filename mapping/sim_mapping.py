#!/usr/bin/env python3
"""
phone-robot: room mapping simulation (no real hardware).

A virtual car explores a virtual room and builds an occupancy grid map
with frontier-based exploration, mimicking the planned real pipeline:

    ToF/depth fan rays  ->  log-odds occupancy grid  ->  frontier + A*

Coordinate system / units
-------------------------
- World frame: meters. Origin (0, 0) is the south-west corner of the room.
  x grows east (right), y grows north (up).
- Grid: cell size 0.05 m. Room is 6.0 m (x) x 5.0 m (y)
  -> 120 (cols, x) x 100 (rows, y) cells.
  Cell (row, col) covers x in [col*0.05, (col+1)*0.05),
  y in [row*0.05, (row+1)*0.05). Array index is [row, col] = [y, x].
- Headings: radians, 0 = +x (east), counter-clockwise positive.

Sensor model (imitates the phone depth camera / ToF)
----------------------------------------------------
- Forward fan of N_RAYS rays over FOV (60 deg), max range 5.0 m.
- Ray hits are computed against the ground-truth grid from the TRUE pose.
- Measured range = true range + Gaussian noise (sigma RANGE_NOISE).
- Mapping registers the scan using the BELIEVED pose:
    normal mode : believed = true + small per-scan noise
                  (pose sigma 0.02 m, heading sigma 1.5 deg)
    drift mode  : believed pose error random-walks every move
                  (dead-reckoning-like, NO correction) -> map smears.

Mapping
-------
- Log-odds occupancy grid. L_OCC for a ray endpoint (hit),
  L_FREE for cells along the ray, clamped to [-L_CLAMP, +L_CLAMP].
- A cell is "known" if it received at least one update.
- Coverage = known cells / all cells. Note: solid box interiors can
  never be observed, so the theoretical ceiling is ~97.6%.

Exploration
-----------
- Frontier = known-free cell with an unknown 4-neighbour.
- Plan with A* over known-free cells (8-connected, no corner cutting)
  to the nearest reachable frontier (candidates tried in order of
  Euclidean distance). On arrival, do an in-place 360 deg scan
  (12 headings), then re-plan.
- Stop when coverage >= TARGET_COVERAGE, or no frontier is reachable,
  or MAX_STEPS moves is reached.

Usage: python3 sim_mapping.py [--drift] [--seed N] [--out PREFIX]
"""

import argparse
import heapq
import math
import os

import numpy as np
from PIL import Image

# ---------------------------------------------------------------- parameters
CELL = 0.05                 # m per cell
ROOM_W_M = 6.0              # room width  (x)
ROOM_H_M = 5.0              # room height (y)
GRID_W = int(ROOM_W_M / CELL)   # 120 cols
GRID_H = int(ROOM_H_M / CELL)   # 100 rows

MAX_RANGE = 5.0             # m, sensor max range
MIN_RANGE = 0.10            # m
FOV_RAD = math.radians(60)  # sensor fan width
N_RAYS = 21                 # rays per scan
RAY_DS = 0.025              # m, ray marching step for hit detection
RANGE_NOISE = 0.03          # m, sigma of range measurement noise

POSE_POS_NOISE = 0.02       # m, sigma (normal mode, per scan)
POSE_HEAD_NOISE = math.radians(1.5)  # rad, sigma (normal mode, per scan)

DRIFT_POS_STEP = 0.012      # m, random-walk sigma per move (drift mode).
                            # Basis: cheap wheel odometry errs by a few %
                            # of distance travelled; after ~300 moves
                            # (15 m) this gives sigma ~= 0.2 m.
DRIFT_HEAD_STEP = math.radians(0.15)  # rad, per move (drift mode);
                            # a cheap gyro yaw drift of a few deg per run

L_OCC = 0.85                # log-odds increment for a hit cell
L_FREE = -0.40              # log-odds increment for a free cell
L_CLAMP = 4.0
PLAN_FREE_L = -0.8          # planning/frontier "free" needs evidence
                            # stronger than one stray free update, so a
                            # cell with a hit (+0.85) is not routed onto
                            # after a couple of noisy free carves
FREE_MARGIN = 2 * CELL      # never carve free space into the last 10 cm
                            # before a measured endpoint (noise overshoot
                            # would otherwise eat into obstacle surfaces)

TARGET_COVERAGE = 0.95
MAX_STEPS = 20000           # movement steps (cell-to-cell moves)

START_X, START_Y = 0.80, 0.80   # m
START_HEADING = 0.0             # rad (east)

# boxes (obstacles): (x0, y0, x1, y1) in meters, solid rectangles
BOXES = [
    (1.50, 1.20, 2.10, 1.70),
    (3.60, 3.00, 4.30, 3.50),
    (4.60, 0.70, 5.10, 1.50),
]

GIF_EVERY = 10              # record a frame every N moves
GIF_MAX_FRAMES = 240


# ---------------------------------------------------------------- world
def build_ground_truth():
    """Return uint8 grid [row, col]: 1 = occupied (wall/box), 0 = free."""
    gt = np.zeros((GRID_H, GRID_W), dtype=np.uint8)
    gt[0, :] = 1
    gt[-1, :] = 1
    gt[:, 0] = 1
    gt[:, -1] = 1
    for (x0, y0, x1, y1) in BOXES:
        c0, c1 = int(x0 / CELL), int(math.ceil(x1 / CELL))
        r0, r1 = int(y0 / CELL), int(math.ceil(y1 / CELL))
        gt[r0:r1, c0:c1] = 1
    return gt


def cell_of(x, y):
    return int(y / CELL), int(x / CELL)


def cell_center(r, c):
    return (c + 0.5) * CELL, (r + 0.5) * CELL


# ---------------------------------------------------------------- sensor
def cast_scan(gt, true_x, true_y, true_heading, rng):
    """
    Cast the fan from the TRUE pose against ground truth.
    Returns measured ranges (m), shape (N_RAYS,); np.inf where no return.
    """
    offsets = np.linspace(-FOV_RAD / 2, FOV_RAD / 2, N_RAYS)
    angs = true_heading + offsets
    ds = np.arange(RAY_DS, MAX_RANGE, RAY_DS)          # (D,)
    xs = true_x + np.cos(angs)[:, None] * ds[None, :]   # (R, D)
    ys = true_y + np.sin(angs)[:, None] * ds[None, :]
    cols = np.clip((xs / CELL).astype(int), 0, GRID_W - 1)
    rows = np.clip((ys / CELL).astype(int), 0, GRID_H - 1)
    occ = gt[rows, cols] > 0                            # (R, D)
    has_hit = occ.any(axis=1)
    first = np.argmax(occ, axis=1)
    ranges = np.where(has_hit, ds[first], np.inf)
    # measurement noise only on real returns
    noise = rng.normal(0.0, RANGE_NOISE, size=ranges.shape)
    ranges = np.where(has_hit, np.clip(ranges + noise, MIN_RANGE, MAX_RANGE),
                      np.inf)
    return ranges, offsets


def apply_scan(logodds, ranges, offsets, bx, by, bheading):
    """
    Register one scan into the log-odds grid using the BELIEVED pose.
    Cells along each ray -> free update; endpoint cell of a hit -> occ.
    Within one scan a cell is updated at most once; a hit wins over free.
    """
    free_cells = []
    hit_cells = []
    for r_meas, off in zip(ranges, offsets):
        ang = bheading + off
        ca, sa = math.cos(ang), math.sin(ang)
        limit = MAX_RANGE if np.isinf(r_meas) else min(r_meas, MAX_RANGE)
        # free samples along the ray (keep a safety margin at the end)
        s = np.arange(RAY_DS, max(limit - FREE_MARGIN, RAY_DS), CELL)
        if s.size:
            xs = bx + ca * s
            ys = by + sa * s
            cols = np.clip((xs / CELL).astype(int), 0, GRID_W - 1)
            rows = np.clip((ys / CELL).astype(int), 0, GRID_H - 1)
            free_cells.append(rows * GRID_W + cols)
        if not np.isinf(r_meas):
            ex, ey = bx + ca * r_meas, by + sa * r_meas
            er, ec = cell_of(ex, ey)
            if 0 <= er < GRID_H and 0 <= ec < GRID_W:
                hit_cells.append(er * GRID_W + ec)
    if free_cells:
        free_idx = np.unique(np.concatenate(free_cells))
    else:
        free_idx = np.empty(0, dtype=int)
    hit_idx = np.unique(np.array(hit_cells, dtype=int)) if hit_cells \
        else np.empty(0, dtype=int)
    if hit_idx.size:
        free_idx = np.setdiff1d(free_idx, hit_idx, assume_unique=True)
    flat = logodds.ravel()
    if free_idx.size:
        flat[free_idx] += L_FREE
    if hit_idx.size:
        flat[hit_idx] += L_OCC
    np.clip(logodds, -L_CLAMP, L_CLAMP, out=logodds)


# ---------------------------------------------------------------- planning
def frontier_cells(known, free_mask):
    """Known-free cells with at least one unknown 4-neighbour."""
    unknown = ~known
    adj = np.zeros_like(unknown)
    adj[1:, :] |= unknown[:-1, :]
    adj[:-1, :] |= unknown[1:, :]
    adj[:, 1:] |= unknown[:, :-1]
    adj[:, :-1] |= unknown[:, 1:]
    return np.argwhere(free_mask & adj)  # (N, 2) as (row, col)


def astar(free_mask, start, goal):
    """
    A* over free_mask (bool grid), 8-connected, no diagonal corner cutting.
    start/goal are (row, col). Returns list of (row, col) incl. start, or None.
    """
    sr, sc = start
    gr, gc = goal
    if start == goal:
        return [start]
    # Plain Python lists/bytes for the search internals: indexing numpy
    # scalars in the neighbour loop is an order of magnitude slower,
    # which matters when a distorted map forces many failed searches.
    free_b = free_mask.tobytes()          # row-major, 0/1 per cell
    g_score = [math.inf] * (GRID_H * GRID_W)
    parent = [-1] * (GRID_H * GRID_W)
    g_score[sr * GRID_W + sc] = 0.0

    def h(r, c):
        dx, dy = abs(c - gc), abs(r - gr)
        return max(dx, dy) + (math.sqrt(2) - 1.0) * min(dx, dy)

    heap = [(h(sr, sc), 0.0, sr, sc)]
    neigh = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
             (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
    while heap:
        _, g, r, c = heapq.heappop(heap)
        if (r, c) == (gr, gc):
            path = []
            cur = r * GRID_W + c
            while cur != -1:
                path.append((cur // GRID_W, cur % GRID_W))
                cur = parent[cur]
            return path[::-1]
        if g > g_score[r * GRID_W + c] + 1e-6:
            continue
        for dr, dc, cost in neigh:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < GRID_H and 0 <= nc < GRID_W):
                continue
            n_idx = nr * GRID_W + nc
            if not free_b[n_idx]:
                continue
            if dr != 0 and dc != 0:
                # no corner cutting through blocked orthogonal neighbours
                if not (free_b[r * GRID_W + (c + dc)] and
                        free_b[(r + dr) * GRID_W + c]):
                    continue
            ng = g + cost
            if ng < g_score[n_idx] - 1e-6:
                g_score[n_idx] = ng
                parent[n_idx] = r * GRID_W + c
                heapq.heappush(heap, (ng + h(nr, nc), ng, nr, nc))
    return None


# ---------------------------------------------------------------- rendering
def render_map(logodds, known, robot_rc=None, scale=5):
    """RGB image: unknown=gray, free=white, occupied=black, robot=red."""
    img = np.full((GRID_H, GRID_W, 3), 120, dtype=np.uint8)
    img[known & (logodds < 0)] = (245, 245, 245)
    img[known & (logodds > 0)] = (20, 20, 20)
    if robot_rc is not None:
        r, c = robot_rc
        img[max(r - 1, 0):r + 2, max(c - 1, 0):c + 2] = (230, 40, 40)
    img = np.flipud(img)  # row 0 (y=0) at the bottom
    pil = Image.fromarray(img, mode="RGB")
    return pil.resize((GRID_W * scale, GRID_H * scale), Image.NEAREST)


# ---------------------------------------------------------------- main sim
def run(drift=False, seed=7, out_prefix="run", max_steps=MAX_STEPS):
    rng = np.random.default_rng(seed)
    gt = build_ground_truth()
    logodds = np.zeros((GRID_H, GRID_W), dtype=np.float32)

    # true robot state (grid-following controller; pose in meters)
    start_rc = cell_of(START_X, START_Y)
    robot_r, robot_c = start_rc
    true_x, true_y = cell_center(robot_r, robot_c)
    true_heading = START_HEADING
    logodds[robot_r, robot_c] = L_FREE  # the cell we stand on is free

    # drift state (drift mode only): error of the believed pose
    drift_x = drift_y = drift_h = 0.0

    def scan(phys_heading):
        """Cast rays at phys_heading (true frame); register the scan at the
        believed pose = same heading plus the current pose error."""
        ranges, offsets = cast_scan(gt, true_x, true_y, phys_heading, rng)
        if drift:
            bx, by = true_x + drift_x, true_y + drift_y
            bh = phys_heading + drift_h
        else:
            bx = true_x + rng.normal(0, POSE_POS_NOISE)
            by = true_y + rng.normal(0, POSE_POS_NOISE)
            bh = phys_heading + rng.normal(0, POSE_HEAD_NOISE)
        apply_scan(logodds, ranges, offsets, bx, by, bh)

    def rotation_scan():
        for k in range(12):
            scan(true_heading + k * math.pi / 6)

    frames = []
    steps = 0
    scans = 0
    failed_plans = 0

    scan(true_heading); scans += 1
    rotation_scan(); scans += 12

    def coverage():
        return float(np.count_nonzero(logodds)) / logodds.size

    cov = coverage()
    while cov < TARGET_COVERAGE and steps < max_steps:
        known = logodds != 0
        free_mask = known & (logodds <= PLAN_FREE_L)
        free_mask[robot_r, robot_c] = True
        fronts = frontier_cells(known, free_mask)
        if len(fronts) == 0:
            break
        # candidates nearest-first (Euclidean, in cells)
        d2 = (fronts[:, 0] - robot_r) ** 2 + (fronts[:, 1] - robot_c) ** 2
        order = np.argsort(d2)
        path = None
        for idx in order[:60]:
            goal = (int(fronts[idx, 0]), int(fronts[idx, 1]))
            if goal == (robot_r, robot_c):
                continue  # already standing on this frontier cell
            path = astar(free_mask, (robot_r, robot_c), goal)
            if path is not None:
                break
        if path is None or len(path) < 2:
            # No reachable frontier, or the nearest frontier is the
            # cell we already stand on (possible when drift smears a
            # permanent "unknown" sliver next to the robot): either
            # way this planning round made no progress.
            failed_plans += 1
            if failed_plans >= 2:
                # Two no-progress rounds in a row: stop instead of
                # spinning here (smeared scans keep "revealing" cells,
                # so a distorted map can keep this loop alive forever).
                break
            rotation_scan(); scans += 12
            if coverage() <= cov:  # rotation revealed nothing new
                break
            cov = coverage()
            continue
        failed_plans = 0
        # follow the path, scanning after every move
        for (nr, nc) in path[1:]:
            if steps >= max_steps:
                break
            if gt[nr, nc]:
                # Physical failsafe: the car cannot enter an occupied
                # cell (close-range ToF / bumper in the real car). Treat
                # the refused move as strong occupied evidence and
                # abandon this path; the planner will route around.
                logodds[nr, nc] = max(logodds[nr, nc], 2.0)
                break
            # rows grow with +y (north), cols with +x (east)
            true_heading = math.atan2(nr - robot_r, nc - robot_c)
            robot_r, robot_c = nr, nc
            true_x, true_y = cell_center(robot_r, robot_c)
            if drift:
                drift_x += rng.normal(0, DRIFT_POS_STEP)
                drift_y += rng.normal(0, DRIFT_POS_STEP)
                drift_h += rng.normal(0, DRIFT_HEAD_STEP)
            scan(true_heading); scans += 1
            steps += 1
            if steps % GIF_EVERY == 0 and len(frames) < GIF_MAX_FRAMES:
                frames.append(render_map(logodds, logodds != 0,
                                         (robot_r, robot_c), scale=4))
            if steps % 50 == 0:
                cov = coverage()
                if cov >= TARGET_COVERAGE:
                    break
            if steps % 500 == 0:
                print(f"  ... steps={steps} coverage={coverage() * 100:.1f}%",
                      flush=True)
        rotation_scan(); scans += 12
        cov = coverage()

    cov = coverage()
    known = logodds != 0
    # map accuracy against ground truth on known cells
    pred_occ = logodds > 0
    correct = int(np.count_nonzero(known & (pred_occ == (gt > 0))))
    acc = correct / max(int(np.count_nonzero(known)), 1)

    print(f"mode={'drift' if drift else 'normal'} seed={seed}")
    print(f"steps={steps} scans={scans}")
    print(f"coverage={cov * 100:.2f}%  (known "
          f"{int(np.count_nonzero(known))}/{logodds.size} cells)")
    print(f"map_accuracy_on_known={acc * 100:.2f}%")

    out_dir = os.path.dirname(os.path.abspath(out_prefix))
    os.makedirs(out_dir, exist_ok=True)
    final_img = render_map(logodds, known, (robot_r, robot_c), scale=5)
    final_path = f"{out_prefix}_map.png"
    final_img.save(final_path)
    print(f"saved {final_path}")
    if frames:
        gif_path = f"{out_prefix}_progress.gif"
        frames[0].save(gif_path, save_all=True,
                       append_images=frames[1:] + [final_img.resize(
                           (GRID_W * 4, GRID_H * 4), Image.NEAREST)],
                       duration=70, loop=0)
        print(f"saved {gif_path} ({len(frames) + 1} frames)")
    return cov, steps


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--drift", action="store_true",
                    help="dead-reckoning pose error, no correction")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default="run", help="output file prefix")
    ap.add_argument("--max-steps", type=int, default=MAX_STEPS)
    a = ap.parse_args()
    run(drift=a.drift, seed=a.seed, out_prefix=a.out,
        max_steps=a.max_steps)
