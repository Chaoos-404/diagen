"""Glue: netlist text -> layout -> routing -> a `Drawing` plus a quality report."""
from __future__ import annotations

import itertools
import math
import time
from collections import defaultdict
from dataclasses import dataclass, field

from .geom import DIRS, Circle, Drawing, Line, Text, Transform, apply, prim_box, union
from .layout import Layout
from .netlist import Circuit, parse
from .router import Router


@dataclass
class Report:
    components: int = 0
    nets: int = 0
    width: float = 0
    height: float = 0
    wire_length: int = 0
    bends: int = 0
    crossings: int = 0
    junctions: int = 0
    unrouted: list = field(default_factory=list)
    warnings: list = field(default_factory=list)

    @property
    def ok(self):
        return not self.unrouted

    def text(self):
        lines = [
            f"{'OK' if self.ok else 'INCOMPLETE'}: {self.components} parts, {self.nets} wired nets, "
            f"{self.width:.0f}x{self.height:.0f} grid units",
            f"wire length {self.wire_length}, bends {self.bends}, crossings {self.crossings}, "
            f"junctions {self.junctions}",
        ]
        for n in self.unrouted:
            lines.append(f"UNROUTED net '{n}' (drawn as a dashed red air wire)")
        for w in self.warnings:
            lines.append(f"warning: {w}")
        return "\n".join(lines)

    def as_dict(self):
        d = dict(self.__dict__)
        d["ok"] = self.ok
        return d


BODY_CLEARANCE = 0.3   # grid points closer than this to a symbol body are off limits


def _grid_points(box, eps):
    x0, y0, x1, y1 = box
    for x in range(math.ceil(x0 - eps), math.floor(x1 + eps) + 1):
        for y in range(math.ceil(y0 - eps), math.floor(y1 + eps) + 1):
            yield (x, y)


# Layout-search budget per render, counted in trials (each one a full place
# + route) so the same netlist always gives the same drawing on any machine.
# Bigger circuits get fewer, slower trials; SEARCH_SECONDS is only a safety
# net against pathological inputs.
SEARCH_TRIALS = 3000
SEARCH_SECONDS = 30.0


def search_budget(ckt):
    parts = sum(1 for c in ckt.components if c.type not in ("input", "output"))
    return max(40, min(300, SEARCH_TRIALS // max(parts, 1)))


START_SLACK = 1.5   # starting layouts worse than this times the best are not searched
NEAR_MARGIN = 0.9   # ports next to their parts must beat edge ports by 10% to win
COMMUTATIVE = ("and", "or", "nand", "nor", "xor", "xnor")


def score(report):
    """Lower is better: what makes a schematic hard to read."""
    return (report.wire_length + 30 * report.crossings + 2 * report.bends
            + 0.05 * report.width * report.height + 1000 * len(report.unrouted))


def build(ckt: Circuit, opts=None):
    """Lay out and route.

    Several starting layouts are built: with ports=auto (the default) ports
    on the outer edges (conventional) and ports next to the parts they
    connect to, and for each, columns ordered by barycentre alone or with
    crossing reduction on top. Each start is improved by local search with a
    fair share of the budget (a local search ends up somewhere quite
    different depending on where it starts), the best one wins (near ports
    only when clearly better), and the rest of the budget goes to it.
    """
    opts = dict(opts or {})
    budget = [search_budget(ckt), time.monotonic() + SEARCH_SECONDS]
    mode = opts.get("ports") or ckt.options.get("ports") or "auto"
    has_ports = any(c.type in ("input", "output") for c in ckt.components)
    modes = (["edge", "near"] if has_ports else ["edge"]) if mode == "auto" else [mode]
    searches = []
    for m in modes:
        plain = _start(ckt, dict(opts, ports=m))
        xmin = _start(ckt, dict(opts, ports=m, xmin=True))
        searches.append(_Search(ckt, plain, budget))
        if _signature(xmin[2]) != _signature(plain[2]):
            searches.append(_Search(ckt, xmin, budget))
    # a start far behind the best rarely catches up: don't spend budget on it
    top = min(score(x.best[1]) for x in searches)
    searches = [x for x in searches if score(x.best[1]) <= START_SLACK * top]
    share = max(1, budget[0] // len(searches))
    for srch in searches:
        srch.run(cap=share, pairs=False)

    def best_of(m):
        return min((x for x in searches if x.opts["ports"] == m),
                   key=lambda x: score(x.best[1]), default=None)
    win = best_of("edge") or best_of(modes[-1])
    near = best_of("near") if len(modes) == 2 else None
    if near and near is not win and score(near.best[1]) < NEAR_MARGIN * score(win.best[1]):
        win = near
    win.run()
    return win.best[:2]


def _signature(lay):
    """What a starting layout is: its column order."""
    return [[n.id for n in col] for col in lay.cols]


def _start(ckt: Circuit, opts):
    """Tight layout first; if a net cannot be routed, widen the channels."""
    best = None
    for extra in (0, 2, 5):
        result = _build_once(ckt, dict(opts, spacing=extra))
        if best is None or len(result[1].unrouted) < len(best[1].unrouted):
            best = result
        if best[1].ok:
            break
    return best


class _Search:
    """Local search from one starting layout. Moves: swap two neighbours in
    a column, or two inputs of a commutative gate. Every move is a full place
    and route, kept when the routed drawing scores better."""

    def __init__(self, ckt, start, budget):
        self.ckt = ckt
        self.budget = budget               # [trials left, wall-clock limit], shared
        self.best = start
        self.opts = dict(start[2].opts, spacing=start[2].opts.get("spacing", 0))
        self.state = ([], [])              # (column swaps, pin swaps) applied so far
        lay = start[2]
        # In repeated stages only the template stage gets moves of its own;
        # the layout (swaps) and the move itself (pins) repeat them in every
        # stage, so the stages stay alike.
        copies = {n.id for n in lay.nodes if n.id in lay.node_stage
                  and lay.node_stage[n.id][1] != lay.template[lay.node_stage[n.id][0]]}
        # a gate fed only from bus trunks taps them in the order written:
        # swapping its inputs changes nothing but the regular look
        bus = {net for nets in lay.bus.values() for net in nets}
        self.gate_moves = []
        for c in ckt.components:
            if getattr(c.spec, "base", None) in COMMUTATIVE and lay.node_of[c.id].id not in copies:
                ins = [p for p in c.spec.order if p != "y" and p in c.pins]
                if bus and all(c.pins[p] in bus for p in ins):
                    continue
                self.gate_moves += [("pin", tuple((o, a, b) for o in [c.id] + lay.peers.get(c.id, [])))
                                    for a, b in zip(ins, ins[1:])]
        # The move list never changes (swaps keep every column's size), so
        # single moves are scanned round-robin: after an improvement the scan
        # carries on with the next move instead of re-trying the ones that
        # just failed.
        # the gates of a bus column keep the netlist's order (y0, y1, ...):
        # they all tap the same trunks, so the order is the reader's, not the wiring's
        self.moves = [("col", l, i) for l, col in enumerate(lay.cols)
                      for i in range(len(col) - 1)
                      if not (col[i].id in copies and col[i + 1].id in copies)
                      and l not in lay.bus] + self.gate_moves
        self.k = 0
        self.stuck = False                 # no single move helps any more

    def _spent(self):
        return self.budget[0] <= 0 or time.monotonic() >= self.budget[1]

    @staticmethod
    def _apply(st, move):
        swaps, pins = st
        if move[0] == "col":
            return (swaps + [move[1:]], pins)
        for m in move[1]:
            pins = _toggle(pins, m)
        return (swaps, pins)

    def _try(self, st):
        self.budget[0] -= 1
        self.used += 1
        cand = _build_once(self.ckt, dict(self.opts, swaps=st[0], pinswaps=st[1]))
        if score(cand[1]) < score(self.best[1]) - 1e-9:
            self.best, self.state = cand, st
            return True
        return False

    def run(self, cap=None, pairs=True):
        """Climb until no move helps, the shared budget runs out, or `cap`
        trials were spent here. Pairs of moves (to escape a local optimum)
        are tried only when `pairs` is set."""
        self.used = 0
        stop = lambda: self._spent() or (cap is not None and self.used >= cap)
        moves = self.moves
        while moves and not stop():
            found = False
            if not self.stuck:
                for _ in range(len(moves)):
                    if stop():
                        return self
                    m = moves[self.k]
                    self.k = (self.k + 1) % len(moves)
                    if self._try(self._apply(self.state, m)):
                        found = True
                        break
                self.stuck = not found
            if not found and pairs:
                for i, m1 in enumerate(moves):
                    for m2 in moves[i + 1:]:
                        if stop():
                            return self
                        if self._try(self._apply(self._apply(self.state, m1), m2)):
                            found = True
                            self.stuck = False
                            break
                    if found:
                        break
            if not found:
                break
        return self


def _toggle(pinswaps, move):
    """Swapping the same two pins twice cancels out."""
    return [m for m in pinswaps if m != move] if move in pinswaps else pinswaps + [move]


def _build_once(ckt: Circuit, opts):
    lay = Layout(ckt, opts).run()
    prims = []
    blocked = set()
    pins = {}
    stubs = {}
    nets = defaultdict(list)
    hollow = {}   # centre -> radius of open circles; wires stop at their rim
    port_x = {"in": [], "out": []}
    report = Report(components=sum(1 for c in ckt.components if c.type not in ("input", "output")))
    report.warnings += ckt.warnings

    for node in lay.nodes:
        for inst in node.members:
            t = inst.moved(node.x, node.y)
            for p in inst.sym.prims:
                q = apply(t, p)
                if isinstance(q, Circle) and q.fill == "none":
                    hollow[(round(q.c[0], 6), round(q.c[1], 6))] = q.r
                prims.append(q)
                eps = 0.2 if isinstance(q, Text) else 0.05
                blocked.update(_grid_points(prim_box(q), eps))
            # wires keep clear of a body: none may graze it on a row a hair away
            # (an inverter's tip sits 0.1 from the rows above and below it)
            blocked.update(_grid_points(prim_box(apply(t, _box_line(inst.sym.body))), BODY_CLEARANCE))
            shift = Transform(0, False, node.x, node.y)
            for p in inst.decor:
                q = apply(shift, p)
                prims.append(q)
                eps = 0.2 if isinstance(q, Text) else 0.05
                blocked.update(_grid_points(prim_box(q), eps))
            for pn, net in inst.comp.pins.items():
                if pn not in inst.sym.pins:
                    continue
                pos = t.pt((inst.sym.pins[pn].x, inst.sym.pins[pn].y))
                pos = (round(pos[0]), round(pos[1]))
                if not lay.is_signal(net):
                    blocked.add(pos)
                    continue
                d = t.dir(inst.sym.pins[pn].dir)
                if inst.comp.type == "input":
                    port_x["in"].append(pos[0])
                elif inst.comp.type == "output":
                    port_x["out"].append(pos[0])
                pins[pos] = (net, d)
                nets[net].append(pos)
                sp = (pos[0] + DIRS[d][0], pos[1] + DIRS[d][1])
                if sp in stubs and stubs[sp] != net:
                    report.warnings.append(f"pins of nets '{net}' and '{stubs[sp]}' touch at {sp}")
                stubs[sp] = net
    for p in pins:
        blocked.discard(p)
    for p in stubs:
        blocked.discard(p)

    box = union([prim_box(p) for p in prims] + [(x, y, x, y) for x, y in pins])
    margin = 4
    x0, x1 = math.floor(box[0]) - margin, math.ceil(box[2]) + margin
    # no wire runs outside the input or output port columns
    if port_x["in"]:
        x0 = max(x0, min(port_x["in"]))
    if port_x["out"]:
        x1 = min(x1, max(port_x["out"]))
    bounds = (x0, math.floor(box[1]) - margin, x1, math.ceil(box[3]) + margin)
    wired = {n: pts for n, pts in nets.items() if len(pts) >= 2}
    router = Router(bounds, blocked, pins, stubs)
    paths, failed = router.route(wired, seeds=_bus_trunks(lay, pins))

    report.nets = len(wired)
    for net, edges in paths.items():
        prims += _wire_prims(edges, hollow)
        deg = defaultdict(int)
        for a, b in edges:
            deg[a] += 1
            deg[b] += 1
        for p, k in deg.items():
            if k >= 3:
                prims.append(Circle(p, 0.14, "black"))
                report.junctions += 1
        report.wire_length += len(edges)
        report.bends += _bends(edges)
    report.crossings = sum(1 for p, d in router.occ.items() if len(d) >= 2)
    for net, missing in failed.items():
        report.unrouted.append(net)
        pts = nets[net]
        for m in missing:
            other = min((q for q in pts if q != m), key=lambda q: abs(q[0] - m[0]) + abs(q[1] - m[1]))
            prims.append(Line([m, other], dashed=True, color="red"))

    bbox = union([prim_box(p) for p in prims])
    pad = 0.6
    bbox = (bbox[0] - pad, bbox[1] - pad, bbox[2] + pad, bbox[3] + pad)
    report.width = bbox[2] - bbox[0]
    report.height = bbox[3] - bbox[1]
    return Drawing(prims, bbox, ckt.title), report, lay


def _bus_trunks(lay, pins):
    """Vertical trunks for the bus nets of each column (see Layout._bus_nets):
    one track each, right of the channel, spanning every pin of the net that
    reaches it from either side. Tracks are ordered to cross as few source
    wires (coming from the left) and taps (from the right) as possible."""
    seeds = {}
    for l, nets in sorted(lay.bus.items()):
        col = [n for n in lay.nodes if n.layer == l]
        xs = [p[0] for p, (net, d) in pins.items() if d == "L"
              and any(n.x + n.box[0] - 1 <= p[0] <= n.x + n.box[2] + 1
                      and n.y + n.box[1] <= p[1] <= n.y + n.box[3] for n in col)]
        if not xs:
            continue
        edge = min(xs)
        span, taps, srcs = {}, {}, {}
        for net in nets:
            pts = [p for p, (n, d) in pins.items() if n == net]
            taps[net] = [p[1] for p in pts if p[0] >= edge]
            srcs[net] = [p[1] for p in pts if p[0] < edge]
            ys = taps[net] + srcs[net]
            span[net] = (min(ys), max(ys))

        def cost(y, lo, hi):
            # crossing a trunk is fine; meeting its end (a corner) forces a detour
            return 1 if lo < y < hi else 6 if y in (lo, hi) else 0

        def crossings(order):
            # order[0] is nearest the column; later trunks lie further left
            c = 0
            for j, net in enumerate(order):
                for k, other in enumerate(order):
                    lo, hi = span[other]
                    if k < j:
                        c += sum(cost(y, lo, hi) for y in taps[net])
                    elif k > j:
                        c += sum(cost(y, lo, hi) for y in srcs[net])
            return c
        if len(nets) <= 6:
            best = min(itertools.permutations(nets), key=lambda o: (crossings(o), o))
        else:
            best = sorted(nets, key=lambda n: (sum(span[n]), n))
        for j, net in enumerate(best):
            x = edge - 2 - j
            lo, hi = span[net]
            seeds[net] = [(x, y) for y in range(lo, hi + 1)]
    return seeds


def _box_line(body):
    x0, y0, x1, y1 = body
    return Line([(x0, y0), (x1, y1)])


def _wire_prims(edges, hollow=None):
    """Merge unit edges into maximal straight segments, stopping short of the
    rim of any open terminal circle a segment ends in."""
    horiz = defaultdict(set)
    vert = defaultdict(set)
    for a, b in edges:
        if a[1] == b[1]:
            horiz[a[1]].add(min(a[0], b[0]))
        else:
            vert[a[0]].add(min(a[1], b[1]))
    out = []
    hollow = hollow or {}
    for y, xs in horiz.items():
        for s, e in _runs(xs):
            a, b = s, e + 1
            a += hollow.get((a, y), 0)
            b -= hollow.get((b, y), 0)
            out.append(Line([(a, y), (b, y)]))
    for x, ys in vert.items():
        for s, e in _runs(ys):
            a, b = s, e + 1
            a += hollow.get((x, a), 0)
            b -= hollow.get((x, b), 0)
            out.append(Line([(x, a), (x, b)]))
    return out


def _runs(vals):
    vals = sorted(vals)
    runs = []
    for v in vals:
        if runs and runs[-1][1] == v - 1:
            runs[-1][1] = v
        else:
            runs.append([v, v])
    return runs


def _bends(edges):
    dirs = defaultdict(set)
    for a, b in edges:
        o = "h" if a[1] == b[1] else "v"
        dirs[a].add(o)
        dirs[b].add(o)
    return sum(1 for d in dirs.values() if len(d) == 2)


def compile_netlist(text: str, opts=None):
    return build(parse(text), opts)
