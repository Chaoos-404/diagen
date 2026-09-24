"""Orthogonal wire routing on the integer grid.

Each net is routed as a tree: start from one pin, then repeatedly connect the
nearest unconnected pin to the tree with A* (state = point + heading, so bends
cost extra). The rules that keep a schematic readable are hard constraints:

* wires never enter a symbol body, a label, or another net's pin or stub;
* a wire leaves a pin straight out along the pin's direction;
* two nets may only cross at right angles, never share a segment, and never
  turn or join where another net passes (so a crossing never looks like a
  junction);
* joins onto a net's own wire make a T (a dot is drawn), never a 4-way cross.

If a net cannot be routed the order is shuffled (failed nets first) and
everything is retried; anything still unroutable is drawn as a dashed red
air wire and reported.

A net can come with a seed: a run of points (a bus trunk) laid down before
anything is routed. Its pins then tap onto the seed, and seed ends that no
tap reached are trimmed away.
"""
from __future__ import annotations

import heapq
from collections import defaultdict

from .geom import DIRS, OPPOSITE

BEND = 4.0
CROSS = 8.0
JOIN4 = 8.0  # joining where three wires already meet (a dotted 4-way junction)
HUG = 0.6   # running right beside another net's wire
REACH_CHECK = 200  # search steps after which unreachable targets are ruled out first


class Router:
    def __init__(self, bounds, blocked, pins, stubs):
        """bounds: (x0, y0, x1, y1) inclusive grid window.
        blocked: set of points no wire may use.
        pins: {point: (net, dir)} for every wired pin.
        stubs: {point: net} the point just outside each pin.
        """
        self.bounds = bounds
        self.blocked = blocked
        self.pins = pins
        self.stubs = stubs

    def route(self, nets, max_rounds=6, seeds=None):
        """nets: {net: [pin points]}; seeds: {net: [points in a line]}.
        Returns (paths, failed)."""
        self.seeds = {n: pts for n, pts in (seeds or {}).items() if n in nets}
        order = sorted(nets, key=lambda n: (n not in self.seeds, self._span(nets[n])))
        best = None
        for rnd in range(max_rounds):
            paths, failed = self._route_all(order, nets)
            if best is None or len(failed) < len(best[1]):
                best = (paths, failed)
            if not failed:
                break
            order = [n for n in order if n in failed] + [n for n in order if n not in failed]
        return best

    @staticmethod
    def _span(pts):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return (max(xs) - min(xs)) + (max(ys) - min(ys))

    def _route_all(self, order, nets):
        self.occ = defaultdict(dict)       # point -> {net: set(dirs)}
        paths = {}
        failed = {}
        seeded = {}
        for net, pts in self.seeds.items():   # every trunk first, so taps cross them
            if all(self._free(net, q, "U" if i else "D") for i, q in enumerate(pts)):
                seeded[net] = list(zip(pts, pts[1:]))
                for a, b in seeded[net]:
                    self._add_edge(net, a, b)
        for net in order:
            pts = nets[net]
            edges, missing = self._route_net(net, pts, seeded.get(net))
            paths[net] = edges
            if missing:
                failed[net] = missing
        return paths, failed

    def _add_edge(self, net, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        d = next(k for k, v in DIRS.items() if v == (dx, dy))
        self.occ[a].setdefault(net, set()).add(d)
        self.occ[b].setdefault(net, set()).add(OPPOSITE[d])

    def _route_net(self, net, pins, seed=None):
        pins = sorted(pins)
        if len(pins) < 2:
            return [], []
        tree_pins = [pins[0]]
        todo = pins[1:]
        edges = []
        missing = []
        tree_pts = set()
        if seed:
            edges = list(seed)
            tree_pts = {p for e in seed for p in e}
            tree_pins, todo = [], list(pins)
        while todo:
            # connect the pin nearest to the tree first
            def dist(p):
                cands = list(tree_pts) + tree_pins
                return min(abs(p[0] - q[0]) + abs(p[1] - q[1]) for q in cands)
            todo.sort(key=dist)
            p = todo.pop(0)
            path = self._astar(net, p, tree_pts, tree_pins)
            if path is None:
                missing.append(p)
                continue
            for a, b in zip(path, path[1:]):
                self._add_edge(net, a, b)
                edges.append((a, b))
                tree_pts.add(a)
                tree_pts.add(b)
            tree_pins.append(p)
            for q in tree_pins:
                tree_pts.discard(q)   # pins are never join points
        if seed:
            edges = self._trim(net, edges, set(pins))
        if missing and len(missing) == len(pins) - 1:
            missing = pins[:1] + missing
        return edges, missing

    def _trim(self, net, edges, pins):
        """Drop dead ends: wire ends that are not pins (unused trunk ends)."""
        edges = list(edges)
        while True:
            deg = defaultdict(int)
            for a, b in edges:
                deg[a] += 1
                deg[b] += 1
            dead = {p for p, k in deg.items() if k == 1 and p not in pins}
            if not dead:
                return edges
            keep = []
            for a, b in edges:
                if a in dead or b in dead:
                    for p, q in ((a, b), (b, a)):
                        d = next(k for k, v in DIRS.items() if v == (q[0] - p[0], q[1] - p[1]))
                        dirs = self.occ[p].get(net)
                        if dirs is not None:
                            dirs.discard(d)
                            if not dirs:
                                del self.occ[p][net]
                                if not self.occ[p]:
                                    del self.occ[p]
                else:
                    keep.append((a, b))
            edges = keep

    def _astar(self, net, start, tree_pts, tree_pins):
        x0, y0, x1, y1 = self.bounds
        sdir = self.pins[start][1]
        # arrival into a pin: from its stub, heading opposite to the pin
        pin_goal = {}
        for q in tree_pins:
            qd = self.pins[q][1]
            dx, dy = DIRS[qd]
            pin_goal[(q[0] + dx, q[1] + dy)] = (q, OPPOSITE[qd])
        # the search ends on reaching a tree point or a tree pin's stub
        targets = list(tree_pts) + list(pin_goal)
        memo = {}

        def h(p, d):
            """Admissible estimate: distance plus the bends that are
            unavoidable from heading d (none if the target lies straight
            ahead, one if it is off to the side, two if it is behind)."""
            key = (p, d)
            if key in memo:
                return memo[key]
            hx, hy = DIRS[d]
            best = 1e18
            for q in targets:
                dx, dy = q[0] - p[0], q[1] - p[1]
                ahead = dx * hx + dy * hy
                side = dx * hy - dy * hx
                v = abs(dx) + abs(dy) + (2 * BEND if ahead < 0 else BEND if side else 0.0)
                if v < best:
                    best = v
            memo[key] = best
            return best

        sx, sy = DIRS[sdir]
        first = (start[0] + sx, start[1] + sy)
        if not self._free(net, first, sdir, allow_own_stub=True):
            return None
        startstate = (first, sdir)
        g = {startstate: 1.0}
        came = {startstate: None}
        heap = [(1.0 + h(first, sdir), 1.0, first, sdir)]
        pops = 0
        while heap:
            f, cost, p, d = heapq.heappop(heap)
            pops += 1
            # A search that fails ends only when everything reachable from the
            # start has been expanded, often a wide open area around a target
            # shut in a small pocket. Once the search runs long, check from
            # both ends whether the target can be reached at all.
            if pops == REACH_CHECK and not self._reachable(net, first, targets):
                return None
            if cost > g.get((p, d), 1e18):
                continue
            # reached the tree?
            if p in tree_pts and self._can_join(net, p, d):
                return self._unwind(came, (p, d), start)
            if p in pin_goal and p not in tree_pts:
                # p is the stub of a tree pin: step straight into the pin
                return self._unwind(came, (p, d), start) + [pin_goal[p][0]]
            crossing_here = self._crossing_at(net, p)
            for nd, (dx, dy) in DIRS.items():
                if nd == OPPOSITE[d]:
                    continue
                if crossing_here and nd != d:
                    continue
                q = (p[0] + dx, p[1] + dy)
                if not (x0 <= q[0] <= x1 and y0 <= q[1] <= y1):
                    continue
                if not self._free(net, q, nd, allow_own_stub=True, tree_pts=tree_pts):
                    continue
                step = 1.0 + (BEND if nd != d else 0.0)
                if self._crossing_at(net, q):
                    step += CROSS
                elif len(self.occ.get(q, {}).get(net, ())) == 3:
                    step += JOIN4
                step += HUG * self._hugging(net, q, nd)
                ng = cost + step
                if ng < g.get((q, nd), 1e18):
                    g[(q, nd)] = ng
                    came[(q, nd)] = (p, d)
                    heapq.heappush(heap, (ng + h(q, nd), ng, q, nd))
        return None

    def _reachable(self, net, first, targets):
        """Can any target be reached from `first` at all? Breadth-first from
        both ends at once, on points only and under looser rules than the
        search (a point is open if some step could enter it: any heading, any
        turn), so False is certain. The side that runs out first decides, so
        a target shut in a small pocket is found out without flooding the
        open side."""
        x0, y0, x1, y1 = self.bounds
        occ, blocked, pins, stubs = self.occ, self.blocked, self.pins, self.stubs
        straight = ({"U", "D"}, {"L", "R"})

        def open_(q):
            if not (x0 <= q[0] <= x1 and y0 <= q[1] <= y1) or q in blocked or q in pins:
                return False
            sn = stubs.get(q)
            if sn is not None and sn != net:
                return False
            for other, ds in occ.get(q, {}).items():
                if (len(ds) >= 4) if other == net else (ds not in straight):
                    return False
            return True
        near, far = {first}, set(targets)
        if near & far:
            return True
        fronts = [list(near), list(far)]
        seen = [near, far]
        while fronts[0] and fronts[1]:
            i = 0 if len(fronts[0]) <= len(fronts[1]) else 1
            nxt = []
            for p in fronts[i]:
                for dx, dy in DIRS.values():
                    q = (p[0] + dx, p[1] + dy)
                    if q in seen[i]:
                        continue
                    if q in seen[1 - i]:
                        return True
                    if open_(q):
                        seen[i].add(q)
                        nxt.append(q)
            fronts[i] = nxt
        return False

    def _unwind(self, came, state, start):
        out = []
        while state is not None:
            out.append(state[0])
            state = came[state]
        out.append(start)
        return out[::-1]

    def _free(self, net, q, heading, allow_own_stub=False, tree_pts=()):
        if q in self.blocked:
            return False
        if q in self.pins:
            return False
        sn = self.stubs.get(q)
        if sn is not None and sn != net:
            return False
        here = self.occ.get(q)
        if here:
            for other, dirs in here.items():
                if other == net:
                    if len(dirs) >= 4:
                        return False
                    continue
                # must cross straight, perpendicular
                if heading in ("L", "R"):
                    if dirs != {"U", "D"}:
                        return False
                else:
                    if dirs != {"L", "R"}:
                        return False
        return True

    def _crossing_at(self, net, p):
        here = self.occ.get(p)
        return bool(here) and any(o != net for o in here)

    def _can_join(self, net, p, heading):
        here = self.occ.get(p, {})
        if any(o != net for o in here):
            return False
        dirs = here.get(net, set())
        if len(dirs) >= 4:
            return False
        # arriving along an existing segment would overlap it
        return OPPOSITE[heading] not in dirs

    def _hugging(self, net, q, heading):
        """Count other-net wires running parallel right next to q."""
        n = 0
        if heading in ("L", "R"):
            side = [(q[0], q[1] - 1), (q[0], q[1] + 1)]
            along = {"L", "R"}
        else:
            side = [(q[0] - 1, q[1]), (q[0] + 1, q[1])]
            along = {"U", "D"}
        for s in side:
            for o, dirs in self.occ.get(s, {}).items():
                if o != net and dirs & along:
                    n += 1
        return n
