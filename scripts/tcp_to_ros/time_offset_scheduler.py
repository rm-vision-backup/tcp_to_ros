"""Pure-computation t_off scheduler for GCS-side MOVE_TO / FAULT_EXIT timing.

Frozen decisions implemented here (implementation_plan_26090106):
- Input is the horizontal (x, y) start from the bridge-forwarded local pose and
  the goal translated to local ENU by the caller (no height is read/compared).
- Per UAV a straight horizontal segment: L = ||goal - start||, T = L / v_xy.
- Pairwise safe sets S_ij = {delta : D_ij,min(delta) >= d_ij} are built by exact
  interval-boundary analysis (piecewise-quadratic roots and breakpoints), never
  by a fixed time-step grid.
- Multi-UAV optimisation fixes tau_1 = 0 and minimises the start-time variance
  under all pairwise difference constraints via DFS interval combination search
  and a convex QP (active-set, pure Python).
- Negative tau is allowed; the result is shifted by -min(tau) so all send times
  are non-negative. The shift preserves relative offsets, variance and safety.
- Search caps: schedule_max_start_delay_s, schedule_max_interval_combinations,
  schedule_max_compute_time_s. A verified feasible solution found within the
  caps is returned even when global optimality is not proven.

This module is pure computation: no rospy, no I/O. schedule_status values only
feed the tcp_to_ros log (plan 6.1).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# Displacement below which a horizontal segment is degenerate (plan 4.2 assumes
# L_i > 0). A zero-length segment cannot define a meaningful timing segment.
_MIN_DISPLACEMENT_M = 1e-6

# Pad applied to "no simultaneous motion" interval boundaries (delta = -T_j /
# +T_i). At those exact deltas the simultaneous-motion interval degenerates to
# a single point where the model distance jumps discontinuously, so an open
# boundary is represented as a closed interval shrunken inward by this pad.
# 1e-3 s at 0.6 m/s is ~0.6 mm: negligible physically, it only keeps the QP
# solution strictly inside the safe (open) region.
_I_EMPTY_BOUNDARY_PAD_S = 1e-3

# schedule_status values (plan 6.1, only logged by the caller).
SCHEDULE_OK = "SCHEDULE_OK"
GEOMETRY_UNSUITABLE = "GEOMETRY_UNSUITABLE"
PAIR_SAFE_SET_EMPTY = "PAIR_SAFE_SET_EMPTY"
GLOBAL_SCHEDULE_INFEASIBLE = "GLOBAL_SCHEDULE_INFEASIBLE"
SCHEDULE_LIMIT_REACHED = "SCHEDULE_LIMIT_REACHED"


@dataclass(frozen=True)
class ScheduleParams:
    """Validated scheduler parameters (canonical values from backend_defaults.yaml)."""

    horizontal_speed_mps: float
    geometric_distance_m: float
    safety_margin_m: float
    time_epsilon_s: float
    max_start_delay_s: float
    max_interval_combinations: int
    max_compute_time_s: float

    def validate(self) -> None:
        if not math.isfinite(self.horizontal_speed_mps) or self.horizontal_speed_mps <= 0.0:
            raise ValueError("horizontal_speed_mps must be a positive finite number")
        if not math.isfinite(self.geometric_distance_m) or self.geometric_distance_m <= 0.0:
            raise ValueError("schedule_geometric_distance_m must be a positive finite number")
        if not math.isfinite(self.safety_margin_m) or self.safety_margin_m < 0.0:
            raise ValueError("schedule_safety_margin_m must be a non-negative finite number")
        if not math.isfinite(self.time_epsilon_s) or self.time_epsilon_s <= 0.0:
            raise ValueError("schedule_time_epsilon_s must be a positive finite number")
        if not math.isfinite(self.max_start_delay_s) or self.max_start_delay_s <= 0.0:
            raise ValueError("schedule_max_start_delay_s must be a positive finite number")
        if self.max_interval_combinations <= 0:
            raise ValueError("schedule_max_interval_combinations must be a positive integer")
        if not math.isfinite(self.max_compute_time_s) or self.max_compute_time_s <= 0.0:
            raise ValueError("schedule_max_compute_time_s must be a positive finite number")

    @property
    def safe_distance_m(self) -> float:
        """d_ij = geometric distance + safety margin (plan 4.3)."""
        return self.geometric_distance_m + self.safety_margin_m


@dataclass(frozen=True)
class UavSegment:
    """One participating UAV's horizontal start/goal in local ENU (plan 4.1)."""

    uav_id: str  # task-layer id (Axx), used for fail ids
    start_xy: Tuple[float, float]
    goal_xy: Tuple[float, float]


@dataclass(frozen=True)
class PairResult:
    """Per-pair safe delta intervals and the predicted min distance of the best tau."""

    uav_i: str
    uav_j: str
    safe_intervals: Tuple[Tuple[float, float], ...]  # delta = tau_j - tau_i
    predicted_min_distance_m: float = float("inf")


@dataclass
class ScheduleResult:
    """Scheduler output (plan 5.1). tau is shifted non-negative send offsets."""

    success: bool
    schedule_status: str
    tau: List[float] = field(default_factory=list)  # shifted tau_i' (non-negative)
    tau_raw: List[float] = field(default_factory=list)  # pre-shift, tau_1 = 0 reference
    variance: float = 0.0
    fail_ids: List[str] = field(default_factory=list)
    failure_reason: str = ""
    pair_results: List[PairResult] = field(default_factory=list)
    max_horizontal_time_s: float = 0.0
    compute_time_s: float = 0.0
    non_global_optimal: bool = False  # feasible solution found but search caps hit

    def max_tau_s(self) -> float:
        return max(self.tau) if self.tau else 0.0


@dataclass(frozen=True)
class _Motion:
    uav_id: str
    p0: Tuple[float, float]
    u: Tuple[float, float]  # horizontal velocity vector
    T: float  # horizontal motion duration

# --------------------------------------------------------------------------
# small linear-algebra helpers (pure Python, no numpy/scipy dependency)
# --------------------------------------------------------------------------


def _dot(a: List[float], b: List[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def _qx(x: List[float]) -> List[float]:
    """Q @ x for Q = (1/N)(I - (1/N)11^T) (centering matrix)."""
    n = len(x)
    mean = sum(x) / n
    return [(v - mean) / n for v in x]


def _variance(tau: List[float]) -> float:
    n = len(tau)
    mean = sum(tau) / n
    return sum((v - mean) ** 2 for v in tau) / n


def _root_of(a0: float, a1: float, c0: float, c1: float) -> Optional[float]:
    """Root of a0 + a1*delta == c0 + c1*delta (None when parallel/coincident)."""
    denom = a1 - c1
    if abs(denom) < 1e-15:
        return None
    return (c0 - a0) / denom


def _solve_ge(c2: float, c1: float, c0: float, eps: float) -> List[Tuple[float, float]]:
    """Closed-form solutions of c2*delta^2 + c1*delta + c0 >= 0 on the real line.

    Returns a list of (l, r) intervals. The threshold is treated as a quadratic
    so all disjoint safe intervals are preserved (plan 4.3, no grid).
    """
    tiny = max(eps * eps, 1e-18)
    if abs(c2) < tiny:
        if abs(c1) < 1e-15:
            return [(-math.inf, math.inf)] if c0 >= -1e-12 else []
        root = -c0 / c1
        if c1 > 0:
            return [(root, math.inf)]
        return [(-math.inf, root)]
    disc = c1 * c1 - 4.0 * c2 * c0
    if disc < -1e-12:
        # no real roots: sign is constant
        return [(-math.inf, math.inf)] if c2 > 0 else []
    disc = max(0.0, disc)
    sq = math.sqrt(disc)
    r1 = (-c1 - sq) / (2.0 * c2)
    r2 = (-c1 + sq) / (2.0 * c2)
    if c2 > 0:
        return [(-math.inf, r1), (r2, math.inf)]
    return [(r1, r2)]


def _merge(intervals: List[Tuple[float, float]], eps: float) -> List[Tuple[float, float]]:
    """Merge adjacent/overlapping closed intervals using an epsilon gap."""
    if not intervals:
        return []
    ordered = sorted(intervals)
    out: List[List[float]] = []
    for (l, r) in ordered:
        if not out:
            out.append([l, r])
            continue
        last = out[-1]
        if l <= last[1] + eps:
            last[1] = max(last[1], r)
        else:
            out.append([l, r])
    return [(item[0], item[1]) for item in out]


def _min_distance(mi: _Motion, mj: _Motion, delta: float) -> Optional[float]:
    """D_ij,min(delta) inside the simultaneous-motion interval.

    Returns None when the two motion intervals do not overlap, i.e. that pair
    has no simultaneous-motion constraint for this delta (theory model section 4).
    """
    lo = max(0.0, delta)
    hi = min(mi.T, delta + mj.T)
    if lo > hi:
        return None
    ax = mi.p0[0] - mj.p0[0] + mj.u[0] * delta
    ay = mi.p0[1] - mj.p0[1] + mj.u[1] * delta
    bx = mi.u[0] - mj.u[0]
    by = mi.u[1] - mj.u[1]
    b2 = bx * bx + by * by
    if b2 < 1e-18:
        t = lo
    else:
        t = -(ax * bx + ay * by) / b2
        t = min(max(t, lo), hi)
    rx = ax + bx * t
    ry = ay + by * t
    return math.hypot(rx, ry)


def _floyd_point(n: int, constraints: List[Tuple[int, int, float, float]]
                 ) -> Tuple[Optional[List[float]], bool]:
    """Feasible point for l <= tau_j - tau_i <= r with tau_0 = 0.

    Difference constraints are solved with all-pairs shortest paths (Floyd-
    Warshall). A negative diagonal means a negative cycle -> infeasible. The
    feasible point is built per connected component: the component containing
    node 0 is anchored at 0 and every other component is anchored at 0 too
    (components are independent), then tau_i = dist(rep -> i) satisfies every
    edge constraint inside the component. No fake "x_i <= 0" bound is added.
    """
    inf = math.inf
    dist = [[inf] * n for _ in range(n)]
    for i in range(n):
        dist[i][i] = 0.0
    for (i, j, l, r) in constraints:
        if r < dist[i][j]:
            dist[i][j] = r   # tau_j <= tau_i + r
        if -l < dist[j][i]:
            dist[j][i] = -l  # tau_i <= tau_j - l
    for k in range(n):
        row_k = dist[k]
        for i in range(n):
            dik = dist[i][k]
            if dik == inf:
                continue
            row_i = dist[i]
            for j in range(n):
                cand = dik + row_k[j]
                if row_k[j] != inf and cand < row_i[j] - 1e-12:
                    row_i[j] = cand
    for i in range(n):
        if dist[i][i] < -1e-9:
            return None, False

    # Connected components: i and j share a component when they are mutually
    # reachable. Node 0's component is anchored at tau_0 = 0.
    comp = [-1] * n
    comp_id = 0
    for start in range(n):
        if comp[start] != -1:
            continue
        comp[start] = comp_id
        stack = [start]
        while stack:
            u = stack.pop()
            for v in range(n):
                if comp[v] == -1 and dist[u][v] != inf and dist[v][u] != inf:
                    comp[v] = comp_id
                    stack.append(v)
        comp_id += 1
    rep: Dict[int, int] = {}
    for c in range(comp_id):
        members = [i for i in range(n) if comp[i] == c]
        rep[c] = 0 if 0 in members else min(members)
    point = [0.0] * n
    for i in range(n):
        r = rep[comp[i]]
        if r == i:
            point[i] = 0.0
        else:
            point[i] = dist[r][i] if dist[r][i] != inf else 0.0
    point[0] = 0.0
    return point, True


def _solve_linear(matrix, rhs) -> Optional[List[float]]:
    """Gaussian elimination with partial pivoting. None when singular."""
    n = len(rhs)
    a = [list(row) for row in matrix]
    b = list(rhs)
    for col in range(n):
        pivot = col
        for row in range(col + 1, n):
            if abs(a[row][col]) > abs(a[pivot][col]):
                pivot = row
        if abs(a[pivot][col]) < 1e-14:
            return None
        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]
            b[col], b[pivot] = b[pivot], b[col]
        inv = 1.0 / a[col][col]
        for row in range(col + 1, n):
            factor = a[row][col] * inv
            if abs(factor) < 1e-16:
                continue
            for c in range(col, n):
                a[row][c] -= factor * a[col][c]
            b[row] -= factor * b[col]
    x = [0.0] * n
    for row in range(n - 1, -1, -1):
        s = b[row]
        for c in range(row + 1, n):
            s -= a[row][c] * x[c]
        x[row] = s / a[row][row]
    return x


# --------------------------------------------------------------------------
# pairwise safe set: exact interval-boundary analysis (plan 4.3)
# --------------------------------------------------------------------------


def _pair_safe_intervals(mi: _Motion, mj: _Motion, d: float,
                         delta_lo: float, delta_hi: float, eps: float
                         ) -> List[Tuple[float, float]]:
    """All safe delta = tau_j - tau_i in [delta_lo, delta_hi].

    A delta is safe when the two horizontal motion intervals do not overlap
    (no simultaneous-motion constraint, theory section 4) or when the minimum
    distance inside the simultaneous interval is >= d. Breakpoints are the
    structural boundaries of lo/hi and the roots of t* == lo / t* == hi, so
    every output interval uses a single branch of the piecewise quadratic.
    """
    t_i, t_j = mi.T, mj.T
    v_i, v_j = mi.u, mj.u
    wx = mi.p0[0] - mj.p0[0]
    wy = mi.p0[1] - mj.p0[1]
    bx = v_i[0] - v_j[0]
    by = v_i[1] - v_j[1]
    b2 = bx * bx + by * by
    d2 = d * d

    breakpoints: set = {delta_lo, delta_hi, 0.0, t_i - t_j, t_i, -t_j}
    if b2 > eps * eps:
        # t*(delta) = alpha + beta*delta
        ab0 = wx * bx + wy * by
        ab1 = v_j[0] * bx + v_j[1] * by
        alpha = -ab0 / b2
        beta = -ab1 / b2
        # structural segments split by 0 (lo) and t_i - t_j (hi)
        bounds = sorted({delta_lo, delta_hi, 0.0, t_i - t_j})
        for idx in range(len(bounds) - 1):
            a, c = bounds[idx], bounds[idx + 1]
            if c - a <= eps * 1e-3:
                continue
            mid = (a + c) / 2.0
            if mid <= 0.0:
                lo0, lo1 = 0.0, 0.0
            else:
                lo0, lo1 = 0.0, 1.0
            if mid <= t_i - t_j:
                hi0, hi1 = t_j, 1.0
            else:
                hi0, hi1 = t_i, 0.0
            root = _root_of(alpha, beta, lo0, lo1)
            if root is not None and a < root < c:
                breakpoints.add(root)
            root = _root_of(alpha, beta, hi0, hi1)
            if root is not None and a < root < c:
                breakpoints.add(root)

    # Breakpoints outside [delta_lo, delta_hi] cannot contribute sub-intervals;
    # keep the range endpoints so the output is always clipped to the window.
    points = sorted(b for b in breakpoints if delta_lo - eps <= b <= delta_hi + eps)
    points = [max(delta_lo, b) for b in points]
    points = [min(delta_hi, b) for b in points]
    points = sorted(set(points))
    safe: List[Tuple[float, float]] = []
    for idx in range(len(points) - 1):
        a, c = points[idx], points[idx + 1]
        if c - a <= eps * 1e-3:
            continue
        # no simultaneous motion over the whole interval -> safe without bound
        if a > t_i or c < -t_j:
            safe.append((a, c))
            continue
        mid = (a + c) / 2.0
        lo = max(0.0, mid)
        hi = min(t_i, mid + t_j)
        if lo > hi:
            # I empty at mid; when an endpoint coincides with the degenerate
            # single-point boundary (-t_j / +t_i) the model distance jumps, so
            # pad those endpoints inward to stay strictly inside the safe set.
            if c <= -t_j:
                safe.append((a, c - _I_EMPTY_BOUNDARY_PAD_S))
            elif a >= t_i:
                safe.append((a + _I_EMPTY_BOUNDARY_PAD_S, c))
            else:
                safe.append((a, c))
            continue

        c2 = c1 = c0 = None
        if b2 <= eps * eps:
            # same velocity vector: relative position is constant, r = w + v_j*delta
            c2 = v_j[0] * v_j[0] + v_j[1] * v_j[1]
            c1 = 2.0 * (wx * v_j[0] + wy * v_j[1])
            c0 = wx * wx + wy * wy - d2
        else:
            t_star = alpha + beta * mid
            if t_star < lo:
                if mid <= 0.0:
                    # t_hat = lo = 0 -> r = w + v_j*delta
                    c2 = v_j[0] * v_j[0] + v_j[1] * v_j[1]
                    c1 = 2.0 * (wx * v_j[0] + wy * v_j[1])
                    c0 = wx * wx + wy * wy - d2
                else:
                    # t_hat = lo = delta -> r = w + v_i*delta
                    c2 = v_i[0] * v_i[0] + v_i[1] * v_i[1]
                    c1 = 2.0 * (wx * v_i[0] + wy * v_i[1])
                    c0 = wx * wx + wy * wy - d2
            elif t_star > hi:
                if mid <= t_i - t_j:
                    # t_hat = hi = delta + t_j -> r = w + (v_i-v_j)*t_j + v_i*delta
                    px = wx + (v_i[0] - v_j[0]) * t_j
                    py = wy + (v_i[1] - v_j[1]) * t_j
                    c2 = v_i[0] * v_i[0] + v_i[1] * v_i[1]
                    c1 = 2.0 * (px * v_i[0] + py * v_i[1])
                    c0 = px * px + py * py - d2
                else:
                    # t_hat = hi = t_i -> r = w + (v_i-v_j)*t_i + v_j*delta
                    px = wx + (v_i[0] - v_j[0]) * t_i
                    py = wy + (v_i[1] - v_j[1]) * t_i
                    c2 = v_j[0] * v_j[0] + v_j[1] * v_j[1]
                    c1 = 2.0 * (px * v_j[0] + py * v_j[1])
                    c0 = px * px + py * py - d2
            else:
                # t_hat = t* -> D_min^2 = |a|^2 - (a.b)^2 / |b|^2, a = w + v_j*delta
                vj2 = v_j[0] * v_j[0] + v_j[1] * v_j[1]
                c2 = vj2 - ab1 * ab1 / b2
                c1 = 2.0 * (wx * v_j[0] + wy * v_j[1]) - 2.0 * ab0 * ab1 / b2
                c0 = wx * wx + wy * wy - ab0 * ab0 / b2 - d2

        for (l, r) in _solve_ge(c2, c1, c0, eps):
            ll = max(a, l)
            rr = min(c, r)
            if rr >= ll:
                safe.append((ll, rr))

    return _merge(safe, eps)


def _shrink_to_safe(intervals: List[Tuple[float, float]], mi: _Motion, mj: _Motion,
                    d: float, eps: float) -> List[Tuple[float, float]]:
    """Numerical backstop: shrink every interval by eps and reject samples that
    are not verified safe. Conservative shrink is tiny; it never invents an
    unsafe delta."""
    out: List[Tuple[float, float]] = []
    tol = d - 1e-9
    for (a, c) in intervals:
        l = a + eps
        r = c - eps
        if r < l:
            continue
        ok = True
        for sample in (l, r, (l + r) / 2.0):
            md = _min_distance(mi, mj, sample)
            if md is not None and md < tol:
                ok = False
                break
        if ok:
            out.append((l, r))
    return _merge(out, eps)


# --------------------------------------------------------------------------
# convex QP: min variance over difference constraints (plan 4.4 step 4)
# --------------------------------------------------------------------------


def _solve_variance_qp(n: int, constraints: List[Tuple[int, int, float, float]],
                       x0: List[float], eps: float = 1e-9) -> Optional[List[float]]:
    """min Var(tau) s.t. tau_0 = 0 and l <= tau_j - tau_i <= r.

    Active-set method on the centering Hessian Q = (1/N)(I - (1/N)11^T). Q is
    positive definite on {tau : tau_0 = 0}. Returns a feasible point (the
    optimum when it converges, otherwise the best reachable feasible point).
    """
    tol = max(1e-9, eps)
    rows: List[Tuple[List[float], float]] = []
    for (i, j, l, r) in constraints:
        a1 = [0.0] * n
        a1[j] = 1.0
        a1[i] = -1.0
        rows.append((a1, r))
        a2 = [0.0] * n
        a2[i] = 1.0
        a2[j] = -1.0
        rows.append((a2, -l))
    eq = [1.0] + [0.0] * (n - 1)
    eq_b = 0.0

    def feasible(x: List[float]) -> bool:
        if abs(eq[0] * x[0] - eq_b) > 1e-6:
            return False
        for (a, b) in rows:
            if _dot(a, x) > b + 1e-6:
                return False
        return True

    if not feasible(x0):
        return None

    x = list(x0)
    working: List[int] = []
    for k in range(len(rows)):
        slack = rows[k][1] - _dot(rows[k][0], x)
        if -tol <= slack <= tol:
            working.append(k)

    def solve_step() -> Optional[Tuple[List[float], List[float]]]:
        """Solve KKT for (p, multipliers) with current working set."""
        a_rows = [eq] + [rows[k][0] for k in working]
        na = len(a_rows)
        dim = n + na
        matrix = [[0.0] * dim for _ in range(dim)]
        rhs = [0.0] * dim
        qx = _qx(x)
        for i in range(n):
            for j in range(n):
                matrix[i][j] = (1.0 if i == j else 0.0) - 1.0 / n
                matrix[i][j] /= n
            rhs[i] = -qx[i]
        for r in range(na):
            for c in range(n):
                # K = [[Q, A^T], [A, 0]]; A is na x n
                matrix[c][n + r] = a_rows[r][c]     # A^T block (n x na)
                matrix[n + r][c] = a_rows[r][c]     # A block (na x n)
        sol = _solve_linear(matrix, rhs)
        if sol is None:
            return None
        return sol[:n], sol[n:]

    for _ in range(200):
        result = solve_step()
        if result is None:
            # degenerate working set: drop one inequality and retry
            if working:
                working.pop()
                continue
            return x if feasible(x) else None
        p, multipliers = result
        pnorm = max(abs(v) for v in p) if p else 0.0
        if pnorm <= tol:
            # check inequality multipliers: negative means the constraint
            # should leave the working set
            worst = None
            worst_val = 0.0
            for idx, k in enumerate(working):
                mu = multipliers[idx + 1]  # index 0 is the equality multiplier
                if mu < -tol and (worst is None or mu < worst_val):
                    worst = idx
                    worst_val = mu
            if worst is None:
                return x
            working.pop(worst)
            continue
        # line search to the first blocking constraint
        alpha = 1.0
        blocking = None
        for k in range(len(rows)):
            if k in working:
                continue
            ap = _dot(rows[k][0], p)
            if ap <= tol:
                continue
            step = (rows[k][1] - _dot(rows[k][0], x)) / ap
            if step < alpha:
                alpha = step
                blocking = k
        x = [x[i] + alpha * p[i] for i in range(n)]
        if alpha < 1.0 - tol and blocking is not None and blocking not in working:
            working.append(blocking)
        else:
            for k in range(len(rows)):
                if k not in working and abs(rows[k][1] - _dot(rows[k][0], x)) <= tol:
                    working.append(k)
    return x if feasible(x) else None


# --------------------------------------------------------------------------
# interval-combination search (plan 4.4) and the public entry point
# --------------------------------------------------------------------------


def _verify_tau(n: int, tau: List[float], motions: List[_Motion],
                pairs: List[Tuple[int, int]], intervals: Dict[Tuple[int, int],
                List[Tuple[float, float]]], d: float, eps: float) -> bool:
    """Confirm every pair's delta lies inside one of its safe intervals and the
    predicted min distance is >= d (with a small numeric tolerance)."""
    for (i, j) in pairs:
        delta = tau[j] - tau[i]
        contained = False
        for (l, r) in intervals[(i, j)]:
            if l - eps <= delta <= r + eps:
                contained = True
                break
        if not contained:
            return False
        md = _min_distance(motions[i], motions[j], delta)
        if md is not None and md < d - 1e-6:
            return False
    return True


def _search_tau(n: int, motions: List[_Motion], pairs: List[Tuple[int, int]],
                intervals: Dict[Tuple[int, int], List[Tuple[float, float]]],
                params: ScheduleParams, start_time: float
                ) -> Tuple[Optional[List[float]], float, bool, int]:
    """DFS over interval choices for the conflicting pairs.

    A pair whose safe set contains delta = 0 is permissive: the optimal
    solution stays near zero anyway, so its intervals are not enumerated in the
    DFS (the leaf still verifies every pair). Only the restrictive pairs are
    searched, which keeps a 12-UAV batch tractable while every returned tau is
    verified against every pair's safe set. The result may be non-global-
    optimal and is marked so when a cap is hit.
    """
    d = params.safe_distance_m
    eps = params.time_epsilon_s
    restrictive: List[Tuple[int, int]] = []
    for (i, j) in pairs:
        contains_zero = any(l - eps <= 0.0 <= r + eps for (l, r) in intervals[(i, j)])
        if not contains_zero:
            restrictive.append((i, j))
    order = sorted(restrictive, key=lambda p: len(intervals[p]))
    best_tau: Optional[List[float]] = None
    best_var = math.inf
    limited = False
    combos = 0
    constraints: List[Tuple[int, int, float, float]] = []

    def dfs(idx: int) -> None:
        nonlocal best_tau, best_var, limited, combos
        if limited:
            return
        if time.monotonic() - start_time > params.max_compute_time_s:
            limited = True
            return
        if idx == len(order):
            combos += 1
            if combos > params.max_interval_combinations:
                limited = True
                return
            point, feasible = _floyd_point(n, constraints)
            if not feasible:
                return
            tau = _solve_variance_qp(n, constraints, point, eps)
            if tau is None:
                tau = point  # QP failed numerically: keep the verified feasible point
            if not _verify_tau(n, tau, motions, pairs, intervals, d, eps):
                return
            var = _variance(tau)
            if var < best_var - 1e-12:
                best_var = var
                best_tau = list(tau)
            return
        (i, j) = order[idx]
        for (l, r) in intervals[(i, j)]:
            constraints.append((i, j, l, r))
            _, feasible = _floyd_point(n, constraints)
            if feasible:
                dfs(idx + 1)
            constraints.pop()
            if limited:
                return

    dfs(0)
    return best_tau, best_var, limited, combos


def compute_schedule(segments: List[UavSegment], params: ScheduleParams
                     ) -> ScheduleResult:
    """Compute validated tau_i' for a batch of horizontal segments (plan 5.1).

    - N == 1: trivially safe, tau = [0.0].
    - Geometry with zero horizontal displacement -> GEOMETRY_UNSUITABLE.
    - A pair with an empty safe set -> PAIR_SAFE_SET_EMPTY.
    - No feasible combination -> GLOBAL_SCHEDULE_INFEASIBLE, or
      SCHEDULE_LIMIT_REACHED when the caps were hit without a feasible solution.
    - A verified feasible solution found within the caps is returned as
      SCHEDULE_OK (non_global_optimal=True when caps cut the search).
    """
    params.validate()
    started = time.monotonic()
    n = len(segments)
    if n == 0:
        raise ValueError("compute_schedule requires at least one segment")
    v = params.horizontal_speed_mps
    d = params.safe_distance_m
    eps = params.time_epsilon_s
    dmax = params.max_start_delay_s

    motions: List[_Motion] = []
    for seg in segments:
        dx = seg.goal_xy[0] - seg.start_xy[0]
        dy = seg.goal_xy[1] - seg.start_xy[1]
        length = math.hypot(dx, dy)
        if length < _MIN_DISPLACEMENT_M:
            return ScheduleResult(
                success=False,
                schedule_status=GEOMETRY_UNSUITABLE,
                fail_ids=[seg.uav_id],
                failure_reason=(
                    "horizontal displacement too small for timing scheduling "
                    "(plan 4.2 requires L_i > 0)"),
                max_horizontal_time_s=0.0,
                compute_time_s=time.monotonic() - started,
            )
        motions.append(_Motion(
            uav_id=seg.uav_id,
            p0=seg.start_xy,
            u=(v * dx / length, v * dy / length),
            T=length / v,
        ))

    max_horizontal_time_s = max(motion.T for motion in motions)
    if n == 1:
        return ScheduleResult(
            success=True,
            schedule_status=SCHEDULE_OK,
            tau=[0.0],
            tau_raw=[0.0],
            variance=0.0,
            max_horizontal_time_s=max_horizontal_time_s,
            compute_time_s=time.monotonic() - started,
        )

    pairs: List[Tuple[int, int]] = [(i, j) for i in range(n) for j in range(i + 1, n)]
    intervals: Dict[Tuple[int, int], List[Tuple[float, float]]] = {}
    for (i, j) in pairs:
        ivs = _pair_safe_intervals(motions[i], motions[j], d, -dmax, dmax, eps)
        ivs = _shrink_to_safe(ivs, motions[i], motions[j], d, eps)
        if not ivs:
            return ScheduleResult(
                success=False,
                schedule_status=PAIR_SAFE_SET_EMPTY,
                fail_ids=[motions[i].uav_id, motions[j].uav_id],
                failure_reason=(
                    "pair {} / {} has an empty safe delta set".format(
                        motions[i].uav_id, motions[j].uav_id)),
                max_horizontal_time_s=max_horizontal_time_s,
                compute_time_s=time.monotonic() - started,
            )
        intervals[(i, j)] = ivs

    # Fast path: tau = 0 is feasible -> zero variance is globally optimal.
    zero_ok = True
    for (i, j) in pairs:
        md = _min_distance(motions[i], motions[j], 0.0)
        if md is not None and md < d - eps:
            zero_ok = False
            break
    if zero_ok:
        return ScheduleResult(
            success=True,
            schedule_status=SCHEDULE_OK,
            tau=[0.0] * n,
            tau_raw=[0.0] * n,
            variance=0.0,
            pair_results=_build_pair_results(motions, pairs, intervals, [0.0] * n, d, eps),
            max_horizontal_time_s=max_horizontal_time_s,
            compute_time_s=time.monotonic() - started,
        )



    best_tau, best_var, limited, _combos = _search_tau(
        n, motions, pairs, intervals, params, started)
    if best_tau is None:
        if limited:
            return ScheduleResult(
                success=False,
                schedule_status=SCHEDULE_LIMIT_REACHED,
                fail_ids=[motion.uav_id for motion in motions],
                failure_reason="search hit combination/compute cap without a feasible solution",
                max_horizontal_time_s=max_horizontal_time_s,
                compute_time_s=time.monotonic() - started,
            )
        return ScheduleResult(
            success=False,
            schedule_status=GLOBAL_SCHEDULE_INFEASIBLE,
            fail_ids=[motion.uav_id for motion in motions],
            failure_reason="no global tau vector satisfies every pair safe set",
            max_horizontal_time_s=max_horizontal_time_s,
            compute_time_s=time.monotonic() - started,
        )

    shift = min(best_tau)
    tau_shifted = [t - shift for t in best_tau]
    return ScheduleResult(
        success=True,
        schedule_status=SCHEDULE_OK,
        tau=tau_shifted,
        tau_raw=list(best_tau),
        variance=best_var,
        pair_results=_build_pair_results(motions, pairs, intervals, best_tau, d, eps),
        max_horizontal_time_s=max_horizontal_time_s,
        compute_time_s=time.monotonic() - started,
        non_global_optimal=limited,
    )


def _build_pair_results(motions: List[_Motion], pairs: List[Tuple[int, int]],
                        intervals: Dict[Tuple[int, int], List[Tuple[float, float]]],
                        tau: List[float], d: float, eps: float) -> List[PairResult]:
    results: List[PairResult] = []
    for (i, j) in pairs:
        delta = tau[j] - tau[i]
        md = _min_distance(motions[i], motions[j], delta)
        results.append(PairResult(
            uav_i=motions[i].uav_id,
            uav_j=motions[j].uav_id,
            safe_intervals=tuple(intervals[(i, j)]),
            predicted_min_distance_m=md if md is not None else float("inf"),
        ))
    return results
