"""Glue: netlist text -> layout -> routing -> a `Drawing` plus a quality report."""
from __future__ import annotations

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


def _grid_points(box, eps):
    x0, y0, x1, y1 = box
    for x in range(math.ceil(x0 - eps), math.floor(x1 + eps) + 1):
        for y in range(math.ceil(y0 - eps), math.floor(y1 + eps) + 1):
            yield (x, y)


SEARCH_SECONDS = 1.5   # total layout-search budget per render (each trial is a full place + route)
COMMUTATIVE = ("and", "or", "nand", "nor", "xor", "xnor")


def score(report):
    """Lower is better: what makes a schematic hard to read."""
    return (report.wire_length + 30 * report.crossings + 2 * report.bends
            + 0.05 * report.width * report.height + 1000 * len(report.unrouted))


def build(ckt: Circuit, opts=None):
    """Lay out and route. With ports=auto (the default) both port placements
    are tried: ports on the outer edges (conventional) and ports next to the
    parts they connect to; the second wins only when clearly better."""
    opts = dict(opts or {})
    opts["_deadline"] = time.monotonic() + SEARCH_SECONDS   # shared by every variant
    mode = opts.get("ports") or ckt.options.get("ports") or "auto"
    has_ports = any(c.type in ("input", "output") for c in ckt.components)
    if mode != "auto" or not has_ports:
        opts["ports"] = "edge" if mode == "auto" else mode
        return _build(ckt, opts)[:2]
    # compare the two port placements unsearched, then spend the whole
    # search budget on the one that wins
    edge = _build(ckt, dict(opts, ports="edge"), search=False)
    near = _build(ckt, dict(opts, ports="near"), search=False)
    mode = "near" if score(near[1]) < 0.8 * score(edge[1]) else "edge"
    return _build(ckt, dict(opts, ports=mode))[:2]


def _build(ckt: Circuit, opts, search=True):
    """Tight layout first; if a net cannot be routed, widen the channels.
    Then search: swap neighbouring parts within a column and keep every swap
    that makes the routed drawing better."""
    best = None
    for extra in (0, 2, 5):
        result = _build_once(ckt, dict(opts, spacing=extra))
        if best is None or len(result[1].unrouted) < len(best[1].unrouted):
            best = result
        if best[1].ok:
            break
    if not search:
        return best
    opts = dict(opts, spacing=best[2].opts.get("spacing", 0))
    # moves: swap two neighbours in a column, or two inputs of a commutative gate
    gate_moves = []
    for c in ckt.components:
        spec = c.spec
        if getattr(spec, "base", None) in COMMUTATIVE:
            ins = [p for p in spec.order if p != "y" and p in c.pins]
            gate_moves += [("pin", c.id, a, b) for a, b in zip(ins, ins[1:])]
    state = ([], [])                       # (column swaps, pin swaps) applied so far
    deadline = opts.get("_deadline", time.monotonic() + SEARCH_SECONDS)

    def apply(st, move):
        swaps, pins = st
        if move[0] == "col":
            return (swaps + [move[1:]], pins)
        return (swaps, _toggle(pins, move[1:]))

    def attempt(st):
        return _build_once(ckt, dict(opts, swaps=st[0], pinswaps=st[1]))

    while time.monotonic() < deadline:
        moves = [("col", l, i) for l, col in enumerate(best[2].cols)
                 for i in range(len(col) - 1)] + gate_moves
        found = None
        for m in moves:                              # single moves first
            if time.monotonic() >= deadline:
                break
            st = apply(state, m)
            cand = attempt(st)
            if score(cand[1]) < score(best[1]) - 1e-9:
                found = (cand, st)
                break
        if found is None:                            # then pairs: escapes local optima
            for i, m1 in enumerate(moves):
                for m2 in moves[i + 1:]:
                    if time.monotonic() >= deadline:
                        break
                    st = apply(apply(state, m1), m2)
                    cand = attempt(st)
                    if score(cand[1]) < score(best[1]) - 1e-9:
                        found = (cand, st)
                        break
                if found or time.monotonic() >= deadline:
                    break
        if found is None:
            break
        best, state = found
    return best


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
            blocked.update(_grid_points(prim_box(apply(t, _box_line(inst.sym.body))), 0.05))
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
    paths, failed = router.route(wired)

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
