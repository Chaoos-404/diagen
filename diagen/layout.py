"""Placement: turn a `Circuit` into positioned symbol instances.

The pipeline:

1. Orient two-terminal parts. A part between a signal net and a rail hangs
   vertically (a "shunt"); a part between two signal nets lies horizontally
   in the signal path (a "series" part).
2. Group parts into layout nodes. A node is one anchor part plus satellites
   that must sit right next to it: parts in parallel with it, feedback parts
   across an amplifier or gate (plus the shunt hanging from the feedback
   junction in a non-inverting amplifier), and shunts stacked on a
   transistor's collector/emitter/drain/source.
3. Assign nodes to columns by signal flow (longest path over a DAG built from
   driver pins, or BFS distance from the sources for passive nets).
4. Order nodes inside each column (barycentre sweeps, optionally Sugiyama
   crossing reduction with virtual points for long wires) and pick y positions
   that straighten wires (weighted median targets, resolved with isotonic
   regression so nodes never overlap).
"""
from __future__ import annotations

import math
from collections import defaultdict, deque
from dataclasses import dataclass, field

from .geom import DIRS, Line, Text, Transform, apply, prim_box, text_box, union
from .netlist import Circuit, Component
from .symbols import Port

GAP_Y = 2          # minimum empty rows between nodes in one column
TRANSISTORS = ("npn", "pnp", "nmos", "pmos")


def _is_port(n):
    return n.anchor.comp.type in ("input", "output")


def gap_y(a, b):
    """Rows kept free between two stacked nodes. Two ports only need one, so
    a column of ports can match the 2-unit pitch of the pins they feed."""
    return 1 if _is_port(a) and _is_port(b) else GAP_Y
MIN_COL_GAP = 1    # minimum free columns between node boxes (boxes include pin exits)


@dataclass
class Inst:
    comp: Component
    sym: object
    t: Transform = field(default_factory=Transform)
    decor: list = field(default_factory=list)   # rail symbols, labels (local, pre-transform? no: node frame)

    def pin_pos(self, p):
        pin = self.sym.pins[p]
        return self.t.pt((pin.x, pin.y))

    def pin_dir(self, p):
        return self.t.dir(self.sym.pins[p].dir)

    def moved(self, dx, dy):
        return Transform(self.t.rot, self.t.mirror, self.t.dx + dx, self.t.dy + dy)


@dataclass
class Node:
    id: int
    anchor: Inst
    members: list                    # Insts, anchor first
    layer: int = 0
    x: int = 0
    y: int = 0
    box: tuple = (0, 0, 0, 0)        # local bbox of everything drawn

    @property
    def w(self):
        return self.box[2] - self.box[0]

    @property
    def h(self):
        return self.box[3] - self.box[1]


# --- rail symbols ---------------------------------------------------------------

def rail_symbol(kind, net, pos, d):
    """Artwork for a ground/supply symbol attached to a pin at `pos` facing `d`."""
    x, y = pos
    dx, dy = DIRS[d]
    sx, sy = x + dx, y + dy
    prims = [Line([(x, y), (sx, sy)])]
    if kind == "gnd":
        if d != "D":
            prims.append(Line([(sx, sy), (sx, sy + 0.5)]))
            sy += 0.5
        for i, half in enumerate((0.6, 0.38, 0.16)):
            yy = sy + 0.22 * i
            prims.append(Line([(sx - half, yy), (sx + half, yy)]))
        return prims
    label = net
    up = kind == "pos"
    if d in ("L", "R"):
        prims.append(Line([(sx, sy), (sx, sy + (-0.5 if up else 0.5))]))
        sy += -0.5 if up else 0.5
    prims.append(Line([(sx - 0.5, sy), (sx + 0.5, sy)]))
    prims.append(Text((sx, sy + (-0.12 if up else 0.12)), label, "s" if up else "n", 0.9))
    return prims


def label_text(comp, sym):
    mode = comp.attrs.get("labels") or sym.label_default
    name = comp.attrs.get("label", comp.id)
    if mode == "none" and "label" not in comp.attrs:
        return []
    lines = [name]
    if comp.value and mode != "name":
        lines.append(comp.value)
    if comp.attrs.get("label") == "":
        lines = [comp.value] if comp.value else []
    return lines


def make_label(inst: Inst):
    """Label primitives in the instance's final frame (before node translation)."""
    lines = label_text(inst.comp, inst.sym)
    if not lines:
        return []
    sym = inst.sym
    if sym.two_terminal:
        a = inst.pin_pos("a")
        b = inst.pin_pos("b")
        bx = [inst.t.pt((sym.body[0], sym.body[1])), inst.t.pt((sym.body[2], sym.body[3]))]
        if a[1] == b[1]:   # horizontal: stack above
            top = min(p[1] for p in bx) - 0.15
            cx = (a[0] + b[0]) / 2
            out = []
            for i, s in enumerate(reversed(lines)):
                out.append(Text((cx, top - i * 0.85), s, "s"))
            return out
        right = max(p[0] for p in bx) + 0.25
        cy = (a[1] + b[1]) / 2
        n = len(lines)
        return [Text((right, cy + (i - (n - 1) / 2) * 0.85), s, "w") for i, s in enumerate(lines)]
    x, y, anchor = sym.label_spot
    if inst.t.mirror and inst.t.rot == 2:
        # mirrored left-right: the label moves to the other side with the pins
        px, py = inst.t.pt((x, y))
        anchor = {"w": "e", "e": "w"}.get(anchor, anchor)
        return [Text((px, py + i * 0.85), s, anchor) for i, s in enumerate(lines)]
    if inst.t.mirror:
        # Keep the label on the same visual side: place it as if unmirrored,
        # then shift by where the mirrored body centre actually lands.
        plain = Transform(inst.t.rot, False, inst.t.dx, inst.t.dy)
        c = (sym.body[1] + sym.body[3]) / 2
        px, py = plain.pt((x, y))
        mx, my = inst.t.pt((0, c))
        ux, uy = plain.pt((0, c))
        px, py = px + mx - ux, py + my - uy
    else:
        px, py = inst.t.pt((x, y))
    anchor = anchor if inst.t.rot == 0 else "w"
    out = []
    step = -0.85 if "s" in anchor else 0.85
    order = list(reversed(lines)) if "s" in anchor else lines
    for i, s in enumerate(order):
        out.append(Text((px, py + i * step), s, anchor))
    return out


# --- the engine --------------------------------------------------------------------

class Layout:
    def __init__(self, ckt: Circuit, opts=None):
        self.ckt = ckt
        self.opts = dict(ckt.options)
        self.opts.update({k: v for k, v in (opts or {}).items() if v is not None})
        self.insts: dict[str, Inst] = {}
        self.nodes: list[Node] = []
        self.node_of: dict[str, Node] = {}
        self.sat_kind: dict[str, str] = {}

    # helpers
    def rail(self, net):
        return self.ckt.rail_kind(net)

    def is_signal(self, net):
        return self.rail(net) is None

    def orail(self, net):
        """Rail kind used for orientation and grouping. A bus node (see
        _find_bus_nodes) still gets wired, but its parts hang like a supply's."""
        return self.rail(net) or ("pos" if net in self.bus_nodes else None)

    def _find_bus_nodes(self):
        """Signal nets fed by a grounded source that fan out to two or more
        two-terminal branches, like the top of a bridge or a ladder. Treating
        them as a supply turns each branch into a vertical divider."""
        self.bus_nodes = set()
        for net, pins in self.net_pins.items():
            if not self.is_signal(net):
                continue
            fed = branches = 0
            for cid, p in pins:
                inst = self.insts[cid]
                if not inst.sym.two_terminal or len(inst.comp.pins) != 2:
                    continue
                far = inst.comp.pins["b" if p == "a" else "a"]
                if inst.sym.source and self.rail(far) in ("gnd", "neg"):
                    fed += 1
                elif self.is_signal(far):
                    branches += 1
            if fed and branches >= 2:
                self.bus_nodes.add(net)

    def run(self):
        for c in self.ckt.components:
            sym = c.spec.build(c, self.opts)
            # the engine's search may exchange inputs of a commutative gate
            for cid, a, b in self.opts.get("pinswaps", ()):
                if cid == c.id and a in sym.pins and b in sym.pins:
                    pa, pb = sym.pins[a], sym.pins[b]
                    pa.x, pb.x, pa.y, pb.y = pb.x, pa.x, pb.y, pa.y
            self.insts[c.id] = Inst(c, sym)
        self.net_pins = defaultdict(list)       # net -> [(comp_id, pin)]
        for c in self.ckt.components:
            for p, n in c.pins.items():
                if p in self.insts[c.id].sym.pins:
                    self.net_pins[n].append((c.id, p))
        self._find_bus_nodes()
        self._orient_shunts()
        self._group()
        self._layers()
        self._orient_series()
        self._build_nodes()
        self._stage_keys()
        self._bus_nets()
        self._order()
        self._place()
        return self

    # 1. shunt orientation ----------------------------------------------------------
    def _orient_shunts(self):
        for inst in self.insts.values():
            c, sym = inst.comp, inst.sym
            if not sym.two_terminal:
                mirror = c.attrs.get("flip", "") not in ("", "0", "false", "no")
                if c.type in ("opamp", "op", "oa", "comparator") and self._inv_shunt(c):
                    mirror = not mirror
                if c.id in self.opts.get("flips", ()):     # the engine's search may mirror it
                    mirror = not mirror
                inst.t = Transform(0, mirror)
                continue
            na, nb = c.pins.get("a"), c.pins.get("b")
            ra, rb = self.orail(na) if na else None, self.orail(nb) if nb else None
            if ra is None and rb is None:
                inst.t = Transform(0)       # series; may be flipped later
                continue
            # Vertical. Decide which pin goes on top.
            rank = {"pos": 0, None: 1, "neg": 2, "gnd": 2}
            a_top = rank[ra] <= rank[rb]
            if ra is None and rb is None:
                a_top = True
            if ra == rb:
                a_top = True
            if sym.source and ra is None and rb == "gnd":
                a_top = True
            inst.t = Transform(1) if a_top else Transform(3)

    def _inv_shunt(self, c):
        """Non-inverting style: the '-' net has a part to a rail and '+' is not a rail."""
        inv, non = c.pins.get("-"), c.pins.get("+")
        if inv is None or non is None or not self.is_signal(inv) or not self.is_signal(non):
            return False
        for cid, p in self.net_pins[inv]:
            other = self.insts[cid].comp
            if self.insts[cid].sym.two_terminal and len(other.pins) == 2:
                far = other.pins["b" if p == "a" else "a"]
                if not self.is_signal(far):
                    return True
        return False

    # 2. grouping ----------------------------------------------------------------------
    def _shunt_info(self, cid):
        """For a two-terminal part hanging off a rail: (signal pin, signal net, rail net)."""
        inst = self.insts[cid]
        if not inst.sym.two_terminal or len(inst.comp.pins) != 2:
            return None
        a, b = inst.comp.pins.get("a"), inst.comp.pins.get("b")
        if a is None or b is None:
            return None
        ra, rb = self.orail(a), self.orail(b)
        if ra is None and rb is not None:
            return ("a", a, b)
        if rb is None and ra is not None:
            return ("b", b, a)
        return None

    def _attach(self, child, parent, kind):
        self.parent[child] = parent
        self.sat_kind[child] = kind

    def _cells(self):
        """Transistors drawn as one block, the way textbooks draw them.

        * Stacks: a transistor whose top pin meets another's bottom pin sits
          right under it, pins in line (an inverter's pull-up over its
          pull-down, a cascode, one leg of an H-bridge). This is what the
          north/south pins of a transistor are for.
        * Symmetry: two stacks with the same parts that map onto each other
          when some nets are exchanged are drawn side by side. When they are
          in parallel (every vertical pin on the same net: the pull-ups of a
          NAND gate) they are copies, all facing the same way (translational
          symmetry). Otherwise they are mirror images (a differential pair, a
          current mirror, the two halves of an SRAM cell or an H-bridge):
          on each level the base or gate faces inwards when its net is shared
          or cross-coupled, and outwards when it comes from outside.
        * The part that closes a symmetric pair from below or above (the tail
          of a differential pair, the pull-down of a NAND) sits on the axis.

        Members are positioned here and attached to the first transistor
        with kind ("cell",); nets inside a cell are kept in self.cell_nets.
        `option symmetry=off` turns this off.
        """
        self.cell_nets = set()
        self.axis_pins = {}          # (comp, pin) -> (y of the axis, (comp, pin) of the partner)
        if str(self.opts.get("symmetry", "auto")).lower() in ("off", "no", "0", "false"):
            return
        insts = self.insts
        devs = [c for c in self.ckt.components if c.type in TRANSISTORS
                and not any(k in c.attrs for k in ("flip", "rot", "rank"))]
        if len(devs) < 2:
            return
        comp = {c.id: c for c in devs}
        order = {c.id: i for i, c in enumerate(self.ckt.components)}

        def pin_to(cid, d):
            inst = insts[cid]
            return next((p for p in inst.sym.pins if inst.pin_dir(p) == d), None)
        pins = {c.id: {d: pin_to(c.id, d) for d in ("U", "D", "L")} for c in devs}
        net = {cid: {d: comp[cid].pins.get(p) if p else None for d, p in ps.items()}
               for cid, ps in pins.items()}
        top = {i: net[i]["U"] for i in comp}
        bot = {i: net[i]["D"] for i in comp}
        sig = lambda n: n is not None and self.is_signal(n)
        pull_up = ("pmos", "pnp")

        # 1) stacks
        below, above = {}, {}
        seen = []
        for c in devs:
            for n in (top[c.id], bot[c.id]):
                if sig(n) and n not in seen:
                    seen.append(n)
        for n in seen:
            ups = [i for i in comp if bot[i] == n]       # parts above the junction
            downs = [i for i in comp if top[i] == n]     # parts below it

            def pick(cands, far, upper):
                # several parts on one side: the one on a rail, of the kind
                # that belongs there (the pull-up of an inverter, not its
                # access transistor)
                if len(cands) == 1:
                    return cands[0]
                good = [i for i in cands if far[i] is not None and not sig(far[i])
                        and (comp[i].type in pull_up) == upper]
                return good[0] if len(good) == 1 else None
            if not ups or not downs:
                continue
            u, d = pick(ups, top, True), pick(downs, bot, False)
            if u is None or d is None or u in below or d in above:
                continue
            k = u                                        # no loops
            while k in above and k != d:
                k = above[k]
            if k == d:
                continue
            below[u], above[d] = d, u
            self.cell_nets.add(n)
        cols = []
        for c in devs:
            if c.id not in above:
                col = [c.id]
                while col[-1] in below:
                    col.append(below[col[-1]])
                cols.append(col)

        # 2) symmetric groups
        def mapping(A, B):
            if len(A) != len(B):
                return None
            s = {}
            for a, b in zip(A, B):
                if comp[a].type != comp[b].type:
                    return None
                for d in ("U", "D", "L"):
                    x, y = net[a][d], net[b][d]
                    if (x is None) != (y is None):
                        return None
                    if x is None:
                        continue
                    if s.get(x, y) != y or s.get(y, x) != x:
                        return None
                    s[x], s[y] = y, x
            return s

        def parallel(A, B):
            return all(top[a] == top[b] and bot[a] == bot[b] for a, b in zip(A, B))

        def related(A, B, s):
            if s is None:
                # Not an exact image (the diode-connected half of a current
                # mirror load), but the same parts sharing a net on the same
                # pin of the same level are still drawn as a pair.
                return len(A) == len(B) and all(comp[a].type == comp[b].type for a, b in zip(A, B)) \
                    and any(net[a][d] == net[b][d] and sig(net[a][d])
                            for a, b in zip(A, B) for d in ("U", "D", "L"))
            mine = {net[a][d] for a in A for d in ("U", "D", "L")} - {None}
            fixed = any(s[x] == x and sig(x) for x in mine)
            cross = any(s[x] != x and s[x] in mine for x in mine)
            return fixed or cross

        cols.sort(key=lambda col: (-len(col), order[col[0]]))
        used = set()
        cells = []
        for i, A in enumerate(cols):
            if A[0] in used:
                continue
            group, mirror, smap = [A], False, {}
            for B in cols[i + 1:]:
                if B[0] in used:
                    continue
                s = mapping(A, B)
                if not related(A, B, s):
                    continue
                if parallel(A, B):
                    if not mirror:
                        group.append(B)
                elif len(group) == 1:
                    group, mirror, smap = [A, B], True, s or {}
                    break
            if len(group) == 1:
                continue                                 # a lone stack waits: it may close a pair
            for col in group:
                used.add(col[0])
            below_c = above_c = None
            if len(group) > 1:
                nb, nt = bot[A[-1]], top[A[0]]
                if sig(nb) and all(bot[col[-1]] == nb for col in group):
                    below_c = next((C for C in cols if C[0] not in used and top[C[0]] == nb), None) \
                        or self._center_shunt(nb, used)
                if sig(nt) and all(top[col[0]] == nt for col in group):
                    above_c = next((C for C in cols if C[0] not in used and bot[C[-1]] == nt), None) \
                        or self._center_shunt(nt, used)
                for C in (below_c, above_c):
                    if C:
                        used.add(C[0])
                if below_c:
                    self.cell_nets.add(nb)
                if above_c:
                    self.cell_nets.add(nt)
            # a rung: a part between a net of one half and its mirror image
            # in the other (the load of an H-bridge) goes across the middle
            rung = None
            if mirror:
                mine = {net[a][d] for a in group[0] for d in ("U", "D", "L")}
                for c in self.ckt.components:
                    ab = (c.pins.get("a"), c.pins.get("b"))
                    if insts[c.id].sym.two_terminal and len(c.pins) == 2 and c.id not in used \
                            and all(sig(n) for n in ab) and ab[0] != ab[1] \
                            and smap.get(ab[0]) == ab[1] and (ab[0] in mine or ab[1] in mine):
                        rung = c.id
                        used.add(c.id)
                        break
            # Sides: a lone transistor hanging off the junction inside each
            # half by one of its vertical pins (the access transistors of an
            # SRAM cell) lies on its side next to that half, level with the
            # junction, pointing inwards, its gate up.
            sides = []
            if mirror:
                found = []
                for col in group:
                    hits = []
                    for lvl, (u, d) in enumerate(zip(col, col[1:])):
                        j = bot[u]
                        for C in cols:
                            x = C[0]
                            if len(C) == 1 and x not in used and x not in (u, d):
                                for pd in ("U", "D"):
                                    if net[x][pd] == j:
                                        hits.append((x, lvl, pd))
                    found.append(hits)
                if all(len(h) == 1 for h in found):
                    (a, la, pa), (b, lb, pb) = found[0][0], found[1][0]
                    if comp[a].type == comp[b].type and (la, pa) == (lb, pb) and a != b:
                        sides = [(a, 0, la, pa), (b, 1, lb, pb)]
                        used.update((a, b))
            cells.append((group, mirror, below_c, above_c, rung, sides))
        cells += [([col], False, None, None, None, []) for col in cols
                  if len(col) > 1 and col[0] not in used]

        # 3) geometry: level i of every column has its top pin at y = 6i
        for group, mirror, below_c, above_c, rung, sides in cells:
            flips = []
            for k, col in enumerate(group):
                f = []
                for lvl, cid in enumerate(col):
                    if not mirror:
                        f.append(False)
                        continue
                    other = group[1 - k]
                    h = net[cid]["L"]
                    theirs = {net[o][d] for o in other for d in ("U", "D")}
                    inward = h is not None and (h == net[other[lvl]]["L"] or h in theirs)
                    # inward: the left part turns its base right, the right one left
                    f.append(inward == (k == 0))
                flips.append(f)

            def put(cid, hflip, x, y, pin="U"):
                inst = insts[cid]
                inst.t = Transform(2, True) if hflip else Transform(0)
                px, py = inst.pin_pos(pins[cid][pin])
                inst.t = inst.moved(x - px, y - py)

            ext = []                                     # (left, right) of each column's pin line
            for col, f in zip(group, flips):
                for lvl, (cid, hf) in enumerate(zip(col, f)):
                    put(cid, hf, 0, 6 * lvl)
                boxes = [self._inst_box(insts[cid]) for cid in col]
                # room on the right for the label of a part stacked on an end pin
                ext.append((min(b[0] for b in boxes), max(max(b[2] for b in boxes), 3.5)))
            room = 2
            if rung:
                rb = self._inst_box(insts[rung])
                room += math.ceil(max(rb[2] - rb[0], rb[3] - rb[1])) + 2
            xs = [0]
            for (_, r), (l, _) in zip(ext, ext[1:]):
                xs.append(math.ceil(xs[-1] + r - l + room))
            for col, f, x in zip(group, flips, xs):
                for lvl, (cid, hf) in enumerate(zip(col, f)):
                    put(cid, hf, x, 6 * lvl)
            mid = round(sum(xs) / len(xs))
            yb = 6 * (len(group[0]) - 1) + 4
            members = [cid for col in group for cid in col]
            for cid, k, lvl, pd in sides:
                inst = insts[cid]
                inner = pins[cid][pd]
                want = "R" if k == 0 else "L"
                for rot, mir in ((1, False), (1, True), (3, False), (3, True)):
                    inst.t = Transform(rot, mir)
                    if inst.pin_dir(inner) == want and inst.pin_dir(pins[cid]["L"]) == "U":
                        break
                y = 6 * lvl + 5                          # the junction inside the half
                px, py = inst.pin_pos(inner)
                inst.t = inst.moved(-px, y - py)
                b = self._inst_box(inst)
                half = [self._inst_box(insts[c]) for c in group[k]]
                if k == 0:                               # clear of the half, with a free column
                    dx = math.floor(min(h[0] for h in half) - 2 - b[2])
                else:
                    dx = math.ceil(max(h[2] for h in half) + 2 - b[0])
                inst.t = inst.moved(dx, 0)
                members.append(cid)
            if rung:
                left = group[0]
                ab = [insts[rung].comp.pins[p] for p in ("a", "b")]
                n = ab[0] if ab[0] in {net[a][d] for a in left for d in ("U", "D", "L")} else ab[1]
                y = None
                for lvl, cid in enumerate(left):
                    if bot[cid] == n and lvl + 1 < len(left):
                        y = 6 * lvl + 5                  # the junction between two levels
                        break
                    for d in ("U", "D", "L"):
                        if net[cid][d] == n:
                            y = insts[cid].pin_pos(pins[cid][d])[1]
                if y is not None:
                    inst = insts[rung]
                    inst.t = Transform(0) if ab[0] == n else Transform(2)
                    pa = inst.pin_pos("a" if ab[0] == n else "b")
                    pb = inst.pin_pos("b" if ab[0] == n else "a")
                    inst.t = inst.moved(round((xs[0] + xs[-1] - (pa[0] + pb[0])) / 2), y - pa[1])
                    members.append(rung)
            if below_c:
                if below_c[0] in comp:
                    for lvl, cid in enumerate(below_c):
                        put(cid, False, mid, yb + 2 + 6 * lvl)
                else:
                    self._put_shunt(below_c[0], mid, yb + 2)
                members += below_c
            if above_c:
                if above_c[0] in comp:
                    for lvl, cid in enumerate(reversed(above_c)):
                        put(cid, False, mid, -2 - 6 * lvl, pin="D")
                else:
                    self._put_shunt(above_c[0], mid, -2)
                members += above_c
            for cid in members[1:]:
                self._attach(cid, members[0], ("cell",))
            # A complementary pair in a stack (the pull-up over the pull-down
            # of an inverter, the two halves of a push-pull stage) with its
            # gates tied is mirrored top to bottom about the junction: whatever
            # drives the gates lines up with that axis, level with the output.
            stacks = list(group) + [c for c in (below_c, above_c) if c and c[0] in comp]
            for col in stacks:
                for u, d in zip(col, col[1:]):
                    h = net[u]["L"]
                    if (comp[u].type in pull_up) == (comp[d].type in pull_up) \
                            or not sig(h) or h != net[d]["L"]:
                        continue
                    ku, kd = (u, pins[u]["L"]), (d, pins[d]["L"])
                    if insts[u].pin_dir(ku[1]) != insts[d].pin_dir(kd[1]):
                        continue
                    axis = (insts[u].pin_pos(ku[1])[1] + insts[d].pin_pos(kd[1])[1]) / 2
                    self.axis_pins[ku] = (axis, kd)
                    self.axis_pins[kd] = (axis, ku)

    def _center_shunt(self, net, used):
        """A part from `net` to a rail, not yet placed, to sit on a cell's axis."""
        for c in self.ckt.components:
            info = self._shunt_info(c.id)
            if info and info[1] == net and c.id not in used:
                return [c.id]
        return None

    def _put_shunt(self, cid, x, y):
        inst = self.insts[cid]
        px, py = inst.pin_pos(self._shunt_info(cid)[0])
        inst.t = inst.moved(x - px, y - py)

    def _group(self):
        """Pick satellites. Each satellite hangs off an immediate parent part."""
        self.parent: dict[str, str] = {}
        comps = self.ckt.components
        taken = lambda cid: cid in self.parent
        self._cells()

        # a) shunts in parallel (same signal net, same rail): side by side
        shunt_groups = defaultdict(list)
        for c in comps:
            info = self._shunt_info(c.id)
            if info and not taken(c.id):
                shunt_groups[(info[1], info[2])].append(c.id)
        shunt_anchor = {}
        for key, ids in shunt_groups.items():
            for prev, cid in zip(ids, ids[1:]):
                self._attach(cid, prev, "beside")
            shunt_anchor[key] = ids[0]

        # b) shunts stacked on a vertical pin (collector/emitter/drain/source)
        for c in comps:
            inst = self.insts[c.id]
            if inst.sym.two_terminal or c.type in ("input", "output"):
                continue
            for p in inst.sym.pins:
                net = c.pins.get(p)
                if net is None or not self.is_signal(net) or net in self.cell_nets:
                    continue
                d = inst.pin_dir(p)
                if d not in ("U", "D"):
                    continue
                for (snet, rnet), cid in shunt_anchor.items():
                    if snet != net or taken(cid):
                        continue
                    rk = self.orail(rnet)
                    if (d == "U" and rk == "pos") or (d == "D" and rk in ("gnd", "neg")):
                        self._attach(cid, c.id, ("stack", p, self._shunt_info(cid)[0]))
                        break

        # c) voltage dividers: supply-side shunt above, ground-side shunt below
        by_net = defaultdict(lambda: {"pos": [], "low": []})
        for (snet, rnet), cid in shunt_anchor.items():
            if taken(cid):
                continue
            side = "pos" if self.orail(rnet) == "pos" else "low"
            by_net[snet][side].append(cid)
        for net, sides in by_net.items():
            if sides["pos"] and sides["low"]:
                self._attach(sides["low"][0], sides["pos"][0], "under")

        # d) series parts: in parallel with each other, or feedback around a part
        series = [c for c in comps if self.insts[c.id].sym.two_terminal and not taken(c.id)
                  and len(c.pins) == 2 and all(self.orail(n) is None for n in c.pins.values())
                  and c.pins["a"] != c.pins["b"]]
        groups = defaultdict(list)
        for c in series:
            groups[frozenset(c.pins.values())].append(c)
        directed = []
        for c in comps:
            sym = self.insts[c.id].sym
            if sym.two_terminal or c.type in ("input", "output"):
                continue
            ins = {c.pins[p] for p, pin in sym.pins.items() if pin.kind == "in" and p in c.pins}
            outs = {c.pins[p] for p, pin in sym.pins.items() if pin.kind == "out" and p in c.pins}
            if ins and outs:
                directed.append((c, ins, outs))
        for nets, members in groups.items():
            host = None
            for c, ins, outs in directed:
                if nets & ins and nets & outs and not (nets & ins & outs):
                    host = c
                    break
            if host:
                prev = host.id
                for m in members:
                    self._attach(m.id, prev, "above")
                    prev = m.id
                # hang from the outermost one: each goes outside the last
                self._hang_on_feedback(host, members[-1], shunt_anchor)
            else:
                for prev, m in zip(members, members[1:]):
                    self._attach(m.id, prev.id, "below")

    def _feedback_below(self, host, fb):
        """Does feedback part `fb` go under its host (it feeds a lower input)?"""
        nets = set(fb.comp.pins.values())
        ys = [host.pin_pos(p)[1] for p, pin in host.sym.pins.items()
              if pin.kind == "in" and host.comp.pins.get(p) in nets]
        hb = self._inst_box(host)
        return bool(ys) and ys[0] > (hb[1] + hb[3]) / 2

    def _hang_on_feedback(self, host, fb, shunt_anchor):
        """The gain-setting shunt of a non-inverting amplifier (R1 from the
        '-' node to ground) hangs straight down from the feedback part's end,
        so the '-' node is one vertical wire with R2 across and R1 below."""
        host_i, fb_i = self.insts[host.id], self.insts[fb.id]
        ins = {host.pins[p] for p, pin in host_i.sym.pins.items()
               if pin.kind == "in" and p in host.pins}
        below = self._feedback_below(host_i, fb_i)
        for p, net in fb.pins.items():
            if net not in ins:
                continue
            for (snet, rnet), cid in shunt_anchor.items():
                # skip shunts already placed, or carrying a divider of their own
                if snet != net or cid in self.parent or cid in self.parent.values():
                    continue
                if (self.orail(rnet) in ("gnd", "neg")) == below:
                    self._attach(cid, fb.id, ("hang", p, self._shunt_info(cid)[0]))
                    return

    # 3. layering -----------------------------------------------------------------------
    def _layers(self):
        anchors = [c.id for c in self.ckt.components if c.id not in self.parent]

        def root(cid):
            while cid in self.parent:
                cid = self.parent[cid]
            return cid
        self.root = root
        idx = {a: i for i, a in enumerate(anchors)}
        # adjacency between anchors through signal nets
        adj = defaultdict(set)
        net_nodes = {}
        for net, pins in self.net_pins.items():
            if not self.is_signal(net):
                continue
            nodes = []
            for cid, p in pins:
                r = root(cid)
                if r not in nodes:
                    nodes.append(r)
            net_nodes[net] = nodes
            for a in nodes:
                for b in nodes:
                    if a != b:
                        adj[a].add(b)
        # BFS distance from the sources
        comps = {c.id: c for c in self.ckt.components}
        roots = [a for a in anchors if comps[a].type == "input"]
        roots += [a for a in anchors if self.insts[a].sym.source and adj[a] and a not in roots]
        dist = {}
        queue = deque()
        for r in roots:
            dist[r] = 0
            queue.append(r)
        while True:
            while queue:
                u = queue.popleft()
                for v in sorted(adj[u], key=idx.get):
                    if v not in dist:
                        dist[v] = dist[u] + 1
                        queue.append(v)
            rest = [a for a in anchors if a not in dist]
            if not rest:
                break
            # start the next island from a part with outputs but no driven inputs
            dist[rest[0]] = 0
            queue.append(rest[0])
        node_nets = defaultdict(set)   # anchor -> signal nets it touches
        for net, nodes in net_nodes.items():
            for a in nodes:
                node_nets[a].add(net)
        # directed edges
        edges = set()
        for net, nodes in net_nodes.items():
            drivers = set()
            for cid, p in self.net_pins[net]:
                kind = self.insts[cid].sym.pins[p].kind
                r = root(cid)
                if cid == r and kind == "out":
                    drivers.add(r)
            if drivers:
                for d in drivers:
                    for v in nodes:
                        if v == d or v in drivers:
                            continue
                        # a passive part reached before its driver is feedback
                        # from a later stage (Sallen-Key C1): don't pull it after
                        if self.insts[v].sym.two_terminal and dist[v] < dist[d]:
                            continue
                        edges.add((d, v))
            else:
                # passive parts come before the inputs they feed; otherwise BFS order
                sink = set()
                for cid, p in self.net_pins[net]:
                    if cid == root(cid) and self.insts[cid].sym.pins[p].kind == "in":
                        sink.add(cid)
                for a in nodes:
                    for b in nodes:
                        if a == b:
                            continue
                        if a not in sink and b in sink:
                            edges.add((a, b))
                        elif (a in sink) == (b in sink) and dist[a] < dist[b]:
                            edges.add((a, b))
                        elif (a in sink) == (b in sink) and dist[a] == dist[b] \
                                and node_nets[a] == {net} and node_nets[b] != {net}:
                            # a part that only hangs off this net (a shunt) goes
                            # before the part that carries the signal onward
                            edges.add((a, b))
        outputs = {a for a in anchors if comps[a].type == "output"}
        edges = {(a, b) for a, b in edges if a not in outputs and comps[b].type != "input"}
        # cross-coupled pairs (latches) share a column: merge them for layering
        rep = {a: a for a in anchors}
        for a, b in sorted(edges, key=lambda e: (idx[e[0]], idx[e[1]])):
            if (b, a) in edges and rep[a] == a and rep[b] == b and idx[a] < idx[b]:
                sa, sb = self.insts[a].sym, self.insts[b].sym
                if not sa.two_terminal and not sb.two_terminal and \
                        comps[a].type not in ("input", "output") and comps[b].type not in ("input", "output"):
                    rep[b] = a
        edges = {(rep[a], rep[b]) for a, b in edges if rep[a] != rep[b]}
        members_of = defaultdict(list)
        for a in anchors:
            members_of[rep[a]].append(a)
        all_anchors = anchors
        anchors = [a for a in anchors if rep[a] == a]
        # break cycles with DFS in BFS order
        succ = defaultdict(list)
        for a, b in sorted(edges, key=lambda e: (idx[e[0]], idx[e[1]])):
            succ[a].append(b)
        state = {}
        dag = defaultdict(set)
        order = sorted(anchors, key=lambda a: (dist[a], idx[a]))

        def dfs(u):
            state[u] = 1
            for v in succ[u]:
                if state.get(v) == 1:
                    continue          # back edge: drop it
                dag[u].add(v)
                if v not in state:
                    dfs(v)
            state[u] = 2
        import sys
        sys.setrecursionlimit(max(10000, 4 * len(anchors)))
        for a in order:
            if a not in state:
                dfs(a)
        # longest path
        indeg = defaultdict(int)
        for u in dag:
            for v in dag[u]:
                indeg[v] += 1
        layer = {a: 0 for a in anchors}
        topo = []
        q = deque(a for a in order if indeg[a] == 0)
        while q:
            u = q.popleft()
            topo.append(u)
            for v in dag[u]:
                layer[v] = max(layer[v], layer[u] + 1)
                indeg[v] -= 1
                if indeg[v] == 0:
                    q.append(v)
        # pull sources toward what they drive (not input ports)
        pred = defaultdict(set)
        for u in dag:
            for v in dag[u]:
                pred[v].add(u)
        near = self.opts.get("ports", "edge") == "near"
        for u in reversed(topo):
            if (comps[u].type == "input" and not near) or not dag[u]:
                continue
            layer[u] = max(layer[u], min(layer[v] for v in dag[u]) - 1)
        # A part that only drives output ports has nothing to pull it right,
        # so it would stay back beside its inputs. Line it up with its
        # siblings instead: the parts that read the same nets (the last AND
        # of a decoder joins the other three).
        readers = defaultdict(set)          # net -> anchors reading it
        for net, pins in self.net_pins.items():
            for cid, p in pins:
                if cid in rep and self.insts[cid].sym.pins[p].kind == "in":
                    readers[net].add(rep[cid])
        for u in anchors:
            if comps[u].type in ("input", "output") or not dag[u] \
                    or any(comps[v].type != "output" for v in dag[u]):
                continue
            mine = [n for n, rs in readers.items() if u in rs]
            sib = [layer[v] for n in mine for v in readers[n]
                   if v != u and comps[v].type not in ("input", "output")]
            if sib:
                layer[u] = max(layer[u], max(sib))
        # a source sharing a column with other parts moves one column left
        for u in anchors:
            if self.insts[u].sym.source and comps[u].type != "input":
                if any(layer[o] == layer[u] and not self.insts[o].sym.source
                       and comps[o].type not in ("input", "output") for o in anchors if o != u):
                    layer[u] -= 1
        self.stages = []
        if self.opts.get("stages", "auto") not in ("off", "no", "0", "false"):
            chains = self._find_stages(anchors, net_nodes, rep, members_of, comps, dag)
            if chains and self._stage_layers(layer, anchors, dag, topo, chains, comps):
                self.stages = chains
        for r in list(anchors):
            for m in members_of[r]:
                layer[m] = layer[r]
        anchors = all_anchors
        self._spread_bridges(layer, anchors, root, comps)
        # manual ranks
        for a in anchors:
            if "rank" in comps[a].attrs:
                try:
                    layer[a] = int(comps[a].attrs["rank"])
                except ValueError:
                    pass
        # ports on the outer columns
        inner = [layer[a] for a in anchors if comps[a].type not in ("input", "output")]
        lo = min(inner) if inner else 0
        hi = max(inner) if inner else 0
        # An input that only feeds pins facing right (the base of the mirrored
        # half of a differential pair) comes in from the right of that part.
        from_right = {}
        for a in anchors:
            if comps[a].type != "input":
                continue
            ends = [(cid, p) for n in comps[a].pins.values() for cid, p in self.net_pins[n]
                    if cid != a]
            if ends and all(self.insts[cid].pin_dir(p) == "R" for cid, p in ends):
                from_right[a] = max(layer[root(cid)] for cid, _ in ends)
                # drawn like an output port (pin on the left, name on the right),
                # still the source of its net
                sym = Port("output").build(comps[a], self.opts)
                sym.type, sym.pins["a"].kind = "input", "out"
                self.insts[a].sym = sym
        for a in anchors:
            if a in from_right:
                layer[a] = from_right[a] + 1
            elif comps[a].type == "input":
                layer[a] = layer[a] if near else lo - 1
            elif comps[a].type == "output":
                drv = [layer[p] for p in anchors if a in dag.get(p, ())]
                layer[a] = (max(drv) + 1 if drv else hi + 1) if near else hi + 1
        if near:
            # Ports get thin columns of their own, right beside what they
            # connect to, instead of sharing a column with parts (which would
            # push those parts out of line). Outputs of one stage and inputs
            # of the next get separate columns: stacked in one column they
            # would force the next stage below the previous one's outputs.
            for a in anchors:
                if comps[a].type not in ("input", "output"):
                    layer[a] *= 3
            for a in anchors:
                if a in from_right:
                    layer[a] = 3 * from_right[a] + 1
                elif comps[a].type == "input":
                    used = [layer[v] for v in dag.get(a, ())]
                    layer[a] = min(used) - 1 if used else -1
                elif comps[a].type == "output":
                    drv = [layer[p] for p in anchors if a in dag.get(p, ())
                           and comps[p].type not in ("input", "output")]
                    layer[a] = max(drv) + 1 if drv else 3 * hi + 1
            used = sorted(set(layer.values()))
            rank = {v: i for i, v in enumerate(used)}
            layer = {a: rank[layer[a]] for a in layer}
        base = min(layer.values())
        self.layer = {a: layer[a] - base for a in anchors}
        self.anchors = anchors
        self.adj = adj
        self.dag = dag
        self.dist = dist

    def _find_stages(self, parts, net_nodes, rep, members_of, comps, dag):
        """Repeated stages: groups of parts that follow one another in a
        chain, each linked to the next by a single net (the carry of a
        ripple adder, the clock of a ripple counter), with the same parts in
        each. Parts tagged `stage=` are grouped by the tag instead.

        Returns chains, each a list of stages (lists of part ids) in signal
        order. Stages of a single part need no special layout and are left
        out."""
        parts = [p for p in parts if comps[p].type not in ("input", "output")]
        order = {p: i for i, p in enumerate(parts)}
        partset = set(parts)
        nets = {}
        for net, nodes in net_nodes.items():
            ps = sorted({rep[a] for a in nodes if rep[a] in partset}, key=order.get)
            if len(ps) >= 2:
                nets[net] = ps
        part_nets = defaultdict(list)
        for net, ps in nets.items():
            for q in ps:
                part_nets[q].append(net)

        def reach(start, skip):
            seen, todo = {start}, [start]
            while todo:
                u = todo.pop()
                for n in part_nets[u]:
                    if n in skip:
                        continue
                    for v in nets[n]:
                        if v not in seen:
                            seen.add(v)
                            todo.append(v)
            return seen

        def signature(piece):
            kinds = []
            for q in piece:
                for a in members_of[q]:
                    kinds += [c.type for c in self.ckt.components if self.root(c.id) == a]
            return tuple(sorted(kinds))

        def flows(a, b):
            return any(v in b for u in a for v in dag.get(u, ()))

        tags = {p: comps[p].attrs["stage"] for p in parts if "stage" in comps[p].attrs}
        if tags:
            def natural(t):
                return (0, int(t), "") if t.lstrip("-").isdigit() else (1, 0, t)
            groups = defaultdict(list)
            for q, t in tags.items():
                groups[t].append(q)
            chain = [sorted(groups[t], key=order.get) for t in sorted(groups, key=natural)]
            return [chain] if len(chain) >= 2 else []
        # a cut net: without it, some of its parts can no longer reach the others
        cuts = {n for n, ps in nets.items()
                if any(v not in reach(ps[0], {n}) for v in ps[1:])}
        piece_of, pieces = {}, []
        for q in parts:
            if q not in piece_of:
                pc = sorted(reach(q, cuts), key=order.get)
                for v in pc:
                    piece_of[v] = len(pieces)
                pieces.append(pc)
        sig = [signature(pc) for pc in pieces]
        link = defaultdict(set)          # piece -> same-kind pieces one cut net away
        for n in sorted(cuts):
            ends = sorted({piece_of[v] for v in nets[n]})
            if len(ends) == 2:
                a, b = ends
                if sig[a] == sig[b] and len(pieces[a]) >= 2:
                    link[a].add(b)
                    link[b].add(a)
        chains, seen = [], set()
        for start in sorted(link):
            if start in seen or len(link[start]) != 1:
                continue                 # walk each chain from one of its ends
            path, prev, cur = [start], None, start
            while True:
                nxt = [x for x in link[cur] if x != prev]
                if len(nxt) != 1 or nxt[0] in path:
                    break
                prev, cur = cur, nxt[0]
                path.append(cur)
            seen.update(path)
            if any(len(link[x]) > 2 for x in path) or len(path) < 2:
                continue
            if flows(pieces[path[1]], pieces[path[0]]):
                path.reverse()
            chains.append([pieces[x] for x in path])
        return chains

    def _stage_layers(self, layer, anchors, dag, topo, chains, comps):
        """Columns for a circuit made of stages: each stage is laid out on its
        own (the same way for every stage), then the stages and the remaining
        parts are placed as blocks along the signal flow. Returns False (and
        changes nothing) if the blocks would form a loop."""
        parts = [a for a in anchors if comps[a].type not in ("input", "output")]
        group = {}
        for ci, chain in enumerate(chains):
            for si, st in enumerate(chain):
                for q in st:
                    group[q] = ("stage", ci, si)
        for q in parts:
            group.setdefault(q, ("part", q))
        members = defaultdict(list)
        for q in parts:
            members[group[q]].append(q)
        local, width = {}, {}
        for g, ps in members.items():
            inside = set(ps)
            loc = {q: 0 for q in ps}
            for u in topo:
                if u in inside:
                    for v in dag.get(u, ()):
                        if v in inside:
                            loc[v] = max(loc[v], loc[u] + 1)
            for u in reversed(topo):
                if u in inside:
                    nxt = [loc[v] for v in dag.get(u, ()) if v in inside]
                    if nxt:
                        loc[u] = max(loc[u], min(nxt) - 1)
            local.update(loc)
            width[g] = max(loc.values()) + 1
        gsucc = defaultdict(set)
        for u in dag:
            for v in dag[u]:
                if u in group and v in group and group[u] != group[v]:
                    gsucc[group[u]].add(group[v])
        indeg = defaultdict(int)
        for g in gsucc:
            for h in gsucc[g]:
                indeg[h] += 1
        first = {}
        for q in parts:
            first.setdefault(group[q], len(first))
        todo = deque(sorted((g for g in members if not indeg[g]), key=first.get))
        gorder = []
        while todo:
            g = todo.popleft()
            gorder.append(g)
            for h in sorted(gsucc[g], key=first.get):
                indeg[h] -= 1
                if not indeg[h]:
                    todo.append(h)
        if len(gorder) != len(members):
            return False
        base = {g: 0 for g in members}
        for g in gorder:
            for h in gsucc[g]:
                base[h] = max(base[h], base[g] + width[g])
        for g in reversed(gorder):          # pull blocks toward what they drive
            if gsucc[g]:
                base[g] = max(base[g], min(base[h] for h in gsucc[g]) - width[g])
        for q in parts:
            layer[q] = base[group[q]] + local[q]
        return True

    def _spread_bridges(self, layer, anchors, root, comps):
        """A two-terminal part between two groups that ended up in the same
        column (the meter of a bridge) is a rung: put one group on each side."""
        for x in anchors:
            inst = self.insts[x]
            c = inst.comp
            if not inst.sym.two_terminal or len(c.pins) != 2 or x in self.parent:
                continue
            na, nb = c.pins["a"], c.pins["b"]
            if not (self.is_signal(na) and self.is_signal(nb)) or na == nb:
                continue
            side = lambda n: {root(o) for o, _ in self.net_pins[n]} - {x}
            A, B = side(na), side(nb)
            if not A or not B or A & B:
                continue
            ls = {layer[n] for n in A | B}
            if len(ls) != 1 or layer[x] != ls.pop() + 1:
                continue
            if any(comps[n].type in ("input", "output") or self.insts[n].sym.source for n in B):
                continue
            L = layer[x]
            for n in anchors:
                if layer[n] > L and n not in B:
                    layer[n] += 1
            for n in B:
                layer[n] = L + 1

    # series orientation --------------------------------------------------------------
    def _orient_series(self):
        root = self.root
        for cid, inst in self.insts.items():
            sym, c = inst.sym, inst.comp
            if not sym.two_terminal or inst.t.rot != 0:
                continue
            if cid in self.parent and self.sat_kind[cid] == "above":
                host = self.insts[self.parent[cid]]
                ins = {host.comp.pins[p] for p, pin in host.sym.pins.items()
                       if pin.kind == "in" and p in host.comp.pins}
                if c.pins["b"] in ins and c.pins["a"] not in ins:
                    inst.t = Transform(2)
                continue
            me = root(cid)
            my = self.layer[me]

            def side(net):
                ls = [self.layer[root(o)] for o, _ in self.net_pins[net] if root(o) != me]
                return sum(ls) / len(ls) if ls else my
            if side(c.pins["a"]) > side(c.pins["b"]):
                inst.t = Transform(2)

    # node construction ------------------------------------------------------------------
    def _build_nodes(self):
        children = defaultdict(list)
        for c in self.ckt.components:        # keep netlist order
            if c.id in self.parent:
                children[self.parent[c.id]].append(c.id)
        for i, a in enumerate(self.anchors):
            anchor = self.insts[a]
            members = [anchor]
            box = self._inst_box(anchor)
            later = []                       # above/below go outside everything else
            queue = deque([a])
            while queue:
                pid = queue.popleft()
                parent = self.insts[pid]
                for cid in children[pid]:
                    kind = self.sat_kind[cid]
                    inst = self.insts[cid]
                    if kind in ("above", "below"):
                        later.append((cid, pid))
                        continue
                    self._place_child(inst, parent, kind)
                    members.append(inst)
                    box = union([box, self._inst_box(inst)])
                    queue.append(cid)
            done = set()
            while later:
                cid, pid = later.pop(0)
                inst, parent = self.insts[cid], self.insts[pid]
                sb = self._inst_box(inst)
                if self.sat_kind[cid] == "above":
                    host = anchor
                    ax0 = min(host.pin_pos(p)[0] for p in host.sym.pins)
                    ax1 = max(host.pin_pos(p)[0] for p in host.sym.pins)
                    pa, pb = inst.pin_pos("a"), inst.pin_pos("b")
                    span = abs(pb[0] - pa[0])
                    dx = round((ax0 + ax1 - span) / 2) - min(pa[0], pb[0])
                    # go on the side of the host input this part feeds back into
                    if self._feedback_below(host, inst):
                        dy = round(box[3] + 1 - sb[1])
                    else:
                        dy = round(box[1] - 1 - sb[3])
                elif isinstance(self.sat_kind[cid], tuple) and self.sat_kind[cid][0] == "hang":
                    # signal pin right under (or over) the feedback pin's exit point
                    _, fp, sp = self.sat_kind[cid]
                    (fx, fy), d = parent.pin_pos(fp), parent.pin_dir(fp)
                    sx, sy = inst.pin_pos(sp)
                    down = inst.pin_dir(sp) == "U"
                    dx = fx + DIRS[d][0] - sx
                    dy = fy + (1 if down else -1) - sy
                else:
                    same = "a" if parent.comp.pins.get("a") == inst.comp.pins["a"] else "b"
                    ref = parent.pin_pos(same)
                    mine = inst.pin_pos("a")
                    dx = ref[0] - mine[0]
                    dy = round(box[3] + 1 - sb[1])
                inst.t = inst.moved(dx, dy)
                members.append(inst)
                box = union([box, self._inst_box(inst)])
                for gc in children[cid]:
                    later.append((gc, cid))
            node = Node(i, anchor, members, layer=self.layer[a])
            self.nodes.append(node)
            for m in members:
                self.node_of[m.comp.id] = node
        for node in self.nodes:
            for m in node.members:
                m.decor = self._decor(m)
            node.box = union([self._inst_box(m) for m in node.members])
            node.core = union([self._inst_box(m, side_stubs=False) for m in node.members])
            node.obst = [prim_box(apply(m.t, p)) for m in node.members for p in m.sym.prims] + \
                [prim_box(p) for m in node.members for p in m.decor]

    def _place_child(self, inst, parent, kind):
        if kind == ("cell",):                  # placed by _cells
            return
        if isinstance(kind, tuple):            # stacked on a vertical pin
            _, hp, sp = kind
            hx, hy = parent.pin_pos(hp)
            d = parent.pin_dir(hp)
            want = "D" if d == "U" else "U"
            for rot in (1, 3):
                t = Transform(rot)
                if t.dir(inst.sym.pins[sp].dir) == want:
                    inst.t = t
                    break
            sx, sy = inst.pin_pos(sp)
            inst.t = inst.moved(hx - sx, hy + (-2 if d == "U" else 2) - sy)
            return
        psig = self._shunt_info(parent.comp.id)[0]
        csig = self._shunt_info(inst.comp.id)[0]
        px, py = parent.pin_pos(psig)
        if kind == "under":                    # divider: signal pins face each other
            sx, sy = inst.pin_pos(csig)
            inst.t = inst.moved(px - sx, py + 2 - sy)
            return
        if kind == "beside":                   # parallel shunt to the right
            sx, sy = inst.pin_pos(csig)
            inst.t = inst.moved(px - sx, py - sy)
            pb = self._inst_box(parent)
            cb = self._inst_box(inst)
            import math
            inst.t = inst.moved(math.ceil(pb[2] + 0.6 - cb[0]), 0)
            return
        raise ValueError(kind)

    def _decor(self, inst):
        prims = []
        for p, net in inst.comp.pins.items():
            if p not in inst.sym.pins:
                continue
            kind = self.rail(net)
            if kind:
                prims += rail_symbol(kind, net, inst.pin_pos(p), inst.pin_dir(p))
        prims += make_label(inst)
        return prims

    def _inst_box(self, inst, side_stubs=True):
        boxes = [prim_box(apply(inst.t, p)) for p in inst.sym.prims]
        boxes += [(x, y, x, y) for x, y in (inst.pin_pos(p) for p in inst.sym.pins)]
        # the grid point just outside each wired pin belongs to the part too,
        # so two parts never offer different nets the same exit point
        # (column spacing handles left/right exits itself: side_stubs=False)
        for p in inst.sym.pins:
            net = inst.comp.pins.get(p)
            if net is not None and self.is_signal(net):
                d = inst.pin_dir(p)
                if not side_stubs and d in ("L", "R"):
                    continue
                (x, y), (dx, dy) = inst.pin_pos(p), DIRS[d]
                boxes.append((x + dx, y + dy, x + dx, y + dy))
        deco = inst.decor if inst.decor else self._decor(inst)
        boxes += [prim_box(p) for p in deco]
        return union(boxes)

    def _place_columns(self):
        """x positions, chosen once every y is known.

        Facing pins of consecutive columns may share their exit point when it
        is the same net (a straight wire two units long). Each net that has to
        jog up or down in a channel gets one extra column of room there.
        """
        cols = [c for c in self.cols if c]
        exits = {}                      # node id -> [(dir, dx, y, net)] of left/right exits
        ys = defaultdict(set)           # net -> exit heights (absolute)
        for n in self.nodes:
            out = []
            for m in n.members:
                for p in m.sym.pins:
                    net = m.comp.pins.get(p)
                    if net is None or not self.is_signal(net):
                        continue
                    (x, y), d = m.pin_pos(p), m.pin_dir(p)
                    dx, dy = DIRS[d]
                    ys[net].add(n.y + y + dy)
                    out.append((d, x + dx, n.y + y + dy, net, x, n.y + y))
            exits[n.id] = out
        straight = {net for net, h in ys.items() if len(h) == 1}
        placed_layers = []
        right = None                   # right edge of the previous column
        for col in cols:
            width = max(n.core[2] - n.core[0] for n in col)
            if right is None:
                left = 0
            else:
                l = col[0].layer
                spanning = {net for net in ys
                            if any(self.node_of[c].layer < l for c, _ in self.net_pins[net])
                            and any(self.node_of[c].layer >= l for c, _ in self.net_pins[net])}
                jogs = len(spanning - straight)
                prev = placed_layers[-1]
                left = math.floor(min(m.x + m.core[0] for m in prev))
                for _ in range(200):          # always terminates; 200 columns is far enough
                    if self._columns_clear(placed_layers[-1], col, left, width, exits, jogs,
                                           straight):
                        break
                    left += 1
            if right is not None:
                left += int(self.opts.get("spacing", 0) or 0)   # extra room on retry
                left += len(self.bus.get(col[0].layer, ()))    # one track per bus trunk
            for n in col:
                n.x = round(left + (width - (n.core[2] - n.core[0])) / 2 - n.core[0])
            right = max(n.x + n.core[2] for n in col)
            placed_layers.append(col)

    def _columns_clear(self, prev, col, left, width, exits, jogs, straight=()):
        """Can `col` sit at `left`?

        * every pin lies right of every pin of the previous column;
        * no pin or exit point lands on the neighbours' drawings;
        * facing exits may coincide only when they carry the same net;
        * drawings that overlap vertically keep a unit apart;
        * each net that jogs in the channel gets a column of its own.
        """
        def boxes(n, x):
            return [(b[0] + x, b[1] + n.y, b[2] + x, b[3] + n.y) for b in n.obst]

        def hits(pt, bs, eps=0.3):
            return any(b[0] - eps <= pt[0] <= b[2] + eps and b[1] - eps <= pt[1] <= b[3] + eps
                       for b in bs)
        prev_boxes = [bb for m in prev for bb in boxes(m, m.x)]
        prev_pin_x = max((m.x + e[4] for m in prev for e in exits[m.id]), default=None)
        prev_pts = {}                                   # exit/pin point -> net
        for m in prev:
            for d, ex, ey, net, px, py in exits[m.id]:
                prev_pts[(m.x + ex, ey)] = net
                prev_pts[(m.x + px, py)] = net
        r_exit_x = [m.x + e[1] for m in prev for e in exits[m.id] if e[0] == "R"]
        l_exit_x = []
        new_boxes = []
        new_pts = []
        for n in col:
            nx = round(left + (width - (n.core[2] - n.core[0])) / 2 - n.core[0])
            new_boxes += boxes(n, nx)
            for d, ex, ey, net, px, py in exits[n.id]:
                if prev_pin_x is not None and nx + px <= prev_pin_x:
                    return False
                for pt in ((nx + ex, ey), (nx + px, py)):
                    if prev_pts.get(pt, net) != net or hits(pt, prev_boxes):
                        return False
                new_pts.append((nx + ex, ey))
                new_pts.append((nx + px, py))
                if d == "L":
                    l_exit_x.append(nx + ex)
        for pt in prev_pts:
            if hits(pt, new_boxes):
                return False
        # a net that jogs climbs from its exit to where it meets the other
        # column; that climb must not run through a drawing
        def blocked(x, y0, y1, bs):
            lo, hi = min(y0, y1), max(y0, y1)
            return any(b[0] - 0.3 <= x <= b[2] + 0.3 and b[1] - 0.3 <= hi and lo <= b[3] + 0.3
                       for b in bs)
        prev_pin_xs = defaultdict(list)     # net -> pin x in the previous column
        new_pin_xs = defaultdict(list)      # net -> pin x in the new column
        for m in prev:
            for e in exits[m.id]:
                prev_pin_xs[e[3]].append(m.x + e[4])
        for n in col:
            nx = round(left + (width - (n.core[2] - n.core[0])) / 2 - n.core[0])
            for e in exits[n.id]:
                new_pin_xs[e[3]].append(nx + e[4])
        prev_ys = defaultdict(list)
        for m in prev:
            for d, ex, ey, net, px, py in exits[m.id]:
                prev_ys[net].append(ey)
        for n in col:
            nx = round(left + (width - (n.core[2] - n.core[0])) / 2 - n.core[0])
            for d, ex, ey, net, px, py in exits[n.id]:
                if d != "L" or net in straight:
                    continue
                for ty in prev_ys.get(net, ()):
                    if blocked(nx + ex, ey, ty, prev_boxes):
                        return False
                # a climbing wire needs a free column beside the pins it lands next to
                if any(px_ >= nx + ex - 1 for px_ in prev_pin_xs.get(net, ())):
                    return False
        new_ys = defaultdict(list)
        for n in col:
            for d, ex, ey, net, px, py in exits[n.id]:
                new_ys[net].append(ey)
        for m in prev:
            for d, ex, ey, net, px, py in exits[m.id]:
                if d != "R" or net in straight:
                    continue
                for ty in new_ys.get(net, ()):
                    if blocked(m.x + ex, ey, ty, new_boxes):
                        return False
                if any(px_ <= m.x + ex + 1 for px_ in new_pin_xs.get(net, ())):
                    return False
        for a in prev_boxes:
            for b in new_boxes:
                if min(a[3], b[3]) - max(a[1], b[1]) > -0.5 and b[0] < a[2] + 1 - 1e-9:
                    return False
        if r_exit_x and l_exit_x and min(l_exit_x) - max(r_exit_x) < max(0, jogs - 1):
            return False
        return True

    # 4. ordering --------------------------------------------------------------------------
    def _order(self):
        L = max(n.layer for n in self.nodes) + 1
        cols = [[] for _ in range(L)]
        for n in sorted(self.nodes, key=lambda n: (self.dist.get(n.anchor.comp.id, 0), n.id)):
            cols[n.layer].append(n)
        nbrs = defaultdict(list)
        for a, bs in self.adj.items():
            for b in bs:
                nbrs[self.node_of[a].id].append(self.node_of[b])

        # where each node touches each net, as a fraction of its height
        touch = defaultdict(dict)          # node id -> {net: fraction}
        for n in self.nodes:
            ys = {}
            for m in n.members:
                for p in m.sym.pins:
                    net = m.comp.pins.get(p)
                    if net is not None and self.is_signal(net):
                        ys.setdefault(net, []).append(m.pin_pos(p)[1])
            h = max(n.h, 1)
            for net, v in ys.items():
                touch[n.id][net] = (sum(v) / len(v) - n.box[1]) / (h + 1)
        net_nodes = defaultdict(list)
        for n in self.nodes:
            for net in touch[n.id]:
                net_nodes[net].append(n)

        def frac(n):
            """Relative height of a node within its column, 0..1."""
            col = cols[n.layer]
            return (col.index(n) + 0.5) / len(col)

        for it in range(7):
            forward = it % 2 == 0
            rng = range(1, L) if forward else range(L - 2, -1, -1)
            for l in rng:
                cur = {n.id: frac(n) for n in cols[l]}

                def key(n):
                    xs, ws = [], []
                    for net, mine in touch[n.id].items():
                        for m in net_nodes[net]:
                            if m is n or m.layer == l:
                                continue
                            span = len(cols[m.layer])
                            xs.append(frac(m) + (touch[m.id][net] - 0.3 * mine) / span)
                            side = 1.0 if (m.layer < l) == forward else 0.35
                            ws.append(side / abs(m.layer - l))
                    if not xs:
                        return cur[n.id]
                    return sum(x * w for x, w in zip(xs, ws)) / sum(ws)
                cols[l].sort(key=key)
        if self.opts.get("xmin"):
            self._reduce_crossings(cols, touch, net_nodes)
        self._copy_template(cols)
        if self.bus:                     # bus sources go on top (see _bus_sources_on_top)
            src = self._bus_sources()
            index = {c.id: i for i, c in enumerate(self.ckt.components)}
            for l in range(len(cols)):
                if l in self.bus:
                    # every gate taps the same trunks, so the order hardly
                    # matters to the wiring: keep the netlist's (y0, y1, ...)
                    cols[l].sort(key=lambda n: index[n.anchor.comp.id])
                elif l > min(self.bus):
                    # later columns follow: each node by where what feeds it sits
                    pos = {n.id: i / len(c) for c in cols[:l] for i, n in enumerate(c)}
                    cur = {n.id: i for i, n in enumerate(cols[l])}

                    def fed_from(n):
                        xs = [pos[self.node_of[c].id] for m in n.members for net in m.comp.pins.values()
                              for c, _ in self.net_pins.get(net, ()) if self.node_of[c].id in pos]
                        return (sum(xs) / len(xs) if xs else 2, cur[n.id])
                    cols[l].sort(key=fed_from)
                cols[l].sort(key=lambda n: n.id not in src)
        # swaps requested by the engine's layout search: (layer, index); a
        # swap inside a stage is made in every stage alike
        for l, i in self.opts.get("swaps", ()):
            if l < len(cols) and i + 1 < len(cols[l]):
                u, v = cols[l][i], cols[l][i + 1]
                cols[l][i], cols[l][i + 1] = v, u
                su, sv = self.node_stage.get(u.id), self.node_stage.get(v.id)
                if su is None or su != sv:
                    continue
                for (ci, si), _ in self.stage_nodes.items():
                    if ci != su[0] or si == su[1]:
                        continue
                    a = self.twin.get((ci, si, self.node_key[u.id]))
                    b = self.twin.get((ci, si, self.node_key[v.id]))
                    if a and b and a.layer == b.layer:
                        col = cols[a.layer]
                        ia, ib = col.index(a), col.index(b)
                        col[ia], col[ib] = b, a
        self.cols = cols

    def _bus_nets(self):
        """With routing=bus, a net feeding two or more gates of one column
        from the left becomes a bus: a vertical trunk in the channel just
        left of that column, which every input taps onto (textbook decoders
        and multiplexers). Each net gets one trunk, in the column where it
        has the most taps. self.bus: {layer: [nets]}."""
        self.bus = {}
        if self.opts.get("routing") != "bus":
            return
        taps = defaultdict(lambda: defaultdict(int))     # net -> layer -> taps
        for n in self.nodes:
            if _is_port(n):
                continue
            for m in n.members:
                for p, pin in m.sym.pins.items():
                    net = m.comp.pins.get(p)
                    if net is not None and self.is_signal(net) and pin.kind == "in" \
                            and m.pin_dir(p) == "L":
                        taps[net][n.layer] += 1
        for net in sorted(taps):
            l, k = max(taps[net].items(), key=lambda t: (t[1], -t[0]))
            fed = any(self.node_of[c].layer < l for c, _ in self.net_pins[net])
            if k >= 2 and fed:
                self.bus.setdefault(l, []).append(net)

    def _bus_sources(self):
        """Nodes that only drive and read bus nets (select inputs and their
        inverters): they feed the trunks from above."""
        bus = {net: l for l, nets in self.bus.items() for net in nets}
        out = set()
        for n in self.nodes:
            nets = {m.comp.pins[p] for m in n.members for p in m.sym.pins
                    if p in m.comp.pins and self.is_signal(m.comp.pins[p])}
            if nets and all(net in bus and n.layer < bus[net] for net in nets):
                out.add(n.id)
        return out

    def _bus_sources_on_top(self):
        """Textbook bus drawing: the sources sit above the gates and each
        source wire runs to the top of its trunk, so the trunks start in a
        staircase and no source wire crosses another trunk."""
        if not self.bus:
            return
        src = self._bus_sources()
        if not src:
            return
        # Each source gets rows of its own, from the top: a wire running right
        # to its trunk never meets another source. A source fed by another
        # (an inverter on a select line) goes right under it.
        by_id = {n.id: n for n in self.nodes}
        nets_of = defaultdict(set)
        for n in self.nodes:
            for m in n.members:
                for p in m.sym.pins:
                    if p in m.comp.pins:
                        nets_of[n.id].add(m.comp.pins[p])
        order = []

        def visit(n):
            if n.id in order:
                return
            order.append(n.id)
            for col in self.cols[n.layer + 1:]:
                for o in col:
                    if o.id in src and nets_of[o.id] & nets_of[n.id]:
                        visit(o)
        for col in self.cols:
            for n in col:
                if n.id in src:
                    visit(n)
        top = min(n.y + n.box[1] for n in self.nodes)
        for i in order:
            n = by_id[i]
            n.y = round(top - n.box[1])
            top = n.y + n.box[3] + 1
        # everything else goes below the sources, so no other wire has to
        # thread through the rows where source wires run to their trunks
        low_src = max(n.y + n.box[3] for n in self.nodes if n.id in src)
        top_tap = min(n.y + n.box[1] for n in self.nodes if n.id not in src)
        delta = math.ceil(low_src + GAP_Y - top_tap)
        if delta > 0:
            for n in self.nodes:
                if n.id not in src:
                    n.y += delta

    def _stage_keys(self):
        """Match up the parts of repeated stages: the k-th part of a kind in
        one stage (in netlist order) is the twin of the k-th of that kind in
        every other stage."""
        self.node_stage, self.node_key, self.twin = {}, {}, {}
        self.stage_nodes, self.template, self.peers = {}, {}, {}
        index = {c.id: i for i, c in enumerate(self.ckt.components)}
        comp_key = defaultdict(dict)        # chain -> {comp key: [comp ids by stage]}
        for ci, chain in enumerate(self.stages):
            self.template[ci] = len(chain) // 2
            for si, st in enumerate(chain):
                nodes = sorted({self.node_of[q].id: self.node_of[q] for q in st}.values(),
                               key=lambda n: index[n.anchor.comp.id])
                self.stage_nodes[(ci, si)] = nodes
                count = defaultdict(int)
                for n in nodes:
                    kind = tuple(sorted(m.comp.type for m in n.members))
                    key = (kind, count[kind])
                    count[kind] += 1
                    self.node_stage[n.id] = (ci, si)
                    self.node_key[n.id] = key
                    self.twin[(ci, si, key)] = n
                comps = sorted((m.comp for n in nodes for m in n.members), key=lambda c: index[c.id])
                count = defaultdict(int)
                for c in comps:
                    comp_key[ci].setdefault((c.type, count[c.type]), []).append(c.id)
                    count[c.type] += 1
        for ci, keys in comp_key.items():
            for ids in keys.values():
                for c in ids:
                    self.peers[c] = [o for o in ids if o != c]

    def _copy_template(self, cols):
        """Every stage takes the column order of its chain's template (the
        middle stage), so repeated stages are drawn alike."""
        for (ci, si), nodes in self.stage_nodes.items():
            t = self.template[ci]
            if si == t:
                continue
            rank = {}
            for n in self.stage_nodes[(ci, t)]:
                rank[n.id] = cols[n.layer].index(n)
            mine = set(n.id for n in nodes)
            for col in cols:
                slots = [i for i, n in enumerate(col) if n.id in mine]
                if len(slots) < 2:
                    continue
                want = sorted((col[i] for i in slots),
                              key=lambda n: rank.get(self.twin.get((ci, t, self.node_key[n.id]), n).id, 1e9))
                for i, n in zip(slots, want):
                    col[i] = n

    def _reduce_crossings(self, cols, touch, net_nodes):
        """Sugiyama crossing reduction on top of the barycentre order.

        A wire spanning several columns gets a virtual point in every column
        it passes, so it takes part in the ordering like a node (one point per
        net and column: the trunk of the net is shared). Then barycentre
        sweeps over nodes and virtual points, and transposing neighbours,
        lower the number of crossings between adjacent columns. The new order
        is kept only if it has fewer crossings than the one it started from.
        """
        L = len(cols)
        drivers = defaultdict(set)
        for n in self.nodes:
            for m in n.members:
                for p, pin in m.sym.pins.items():
                    net = m.comp.pins.get(p)
                    if net is not None and pin.kind == "out":
                        drivers[net].add(n.id)
        # an entry is a node id, or ("v", net, layer) for a virtual point
        layer_of = {n.id: n.layer for n in self.nodes}
        adj = defaultdict(list)          # entry -> [(other entry, net)]
        seen = set()
        guess = defaultdict(list)        # virtual point -> initial height guesses
        home = {n.id: (cols[n.layer].index(n) + 0.5) / len(cols[n.layer]) for n in self.nodes}

        def link(a, b, net):
            if (a, b, net) not in seen:
                seen.add((a, b, net))
                seen.add((b, a, net))
                adj[a].append((b, net))
                adj[b].append((a, net))
        for net, nodes in net_nodes.items():
            if len(nodes) < 2:
                continue
            ds = [n for n in nodes if n.id in drivers[net]]
            src = ds[0] if ds else min(nodes, key=lambda n: (n.layer, n.id))
            for v in nodes:
                if v.layer == src.layer:
                    continue
                step = 1 if v.layer > src.layer else -1
                prev = src.id
                for l in range(src.layer + step, v.layer, step):
                    k = ("v", net, l)
                    layer_of[k] = l
                    t = (l - src.layer) / (v.layer - src.layer)
                    guess[k].append(home[src.id] * (1 - t) + home[v.id] * t)
                    link(prev, k, net)
                    prev = k
                link(prev, v.id, net)
        if not guess:
            return                       # no wire spans a column: nothing to add
        seq = []
        for l in range(L):
            items = [(home[n.id], 0, n.id) for n in cols[l]]
            items += [(sum(g) / len(g), 1, k) for k, g in guess.items() if layer_of[k] == l]
            seq.append([k for _, _, k in sorted(items, key=lambda t: (t[0], t[1]))])

        def fr(k, net):
            return touch[k][net] if not isinstance(k, tuple) else 0.5

        def positions(sq):
            return {k: i for col in sq for i, k in enumerate(col)}

        def crossings(sq):
            pos = positions(sq)
            total = 0
            for l in range(L - 1):
                es = [(pos[a] + fr(a, net), pos[b] + fr(b, net), net)
                      for a in sq[l] for b, net in adj[a] if layer_of[b] == l + 1]
                for i, (x1, y1, n1) in enumerate(es):
                    for x2, y2, n2 in es[i + 1:]:
                        if n1 != n2 and (x1 - x2) * (y1 - y2) < 0:
                            total += 1
            return total

        def sweep(sq, l, side):
            pos = positions(sq)
            cur = {k: i for i, k in enumerate(sq[l])}

            def bary(k):
                xs = [pos[o] + fr(o, net) for o, net in adj[k] if layer_of[o] == l + side]
                return sum(xs) / len(xs) if xs else cur[k] + 0.5
            sq[l] = sorted(sq[l], key=lambda k: (bary(k), cur[k]))

        start = crossings(seq)
        best, best_c = [list(c) for c in seq], start
        for it in range(4):
            forward = it % 2 == 0
            for l in (range(1, L) if forward else range(L - 2, -1, -1)):
                sweep(seq, l, -1 if forward else 1)
            c = crossings(seq)
            if c < best_c:
                best, best_c = [list(col) for col in seq], c
        seq = best
        # transpose: swap neighbours while that removes crossings
        pos = positions(seq)
        for _ in range(20):
            improved = False
            for l in range(L):
                col = seq[l]
                for i in range(len(col) - 1):
                    u, v = col[i], col[i + 1]
                    delta = 0
                    for side in (-1, 1):
                        eu = [(pos[o] + fr(o, net), net) for o, net in adj[u] if layer_of[o] == l + side]
                        ev = [(pos[o] + fr(o, net), net) for o, net in adj[v] if layer_of[o] == l + side]
                        for ou, nu in eu:
                            for ov, nv in ev:
                                if nu != nv:
                                    delta += (ou < ov) - (ou > ov)
                    if delta < 0:
                        col[i], col[i + 1] = v, u
                        pos[u], pos[v] = i + 1, i
                        improved = True
            if not improved:
                break
        if crossings(seq) < start:
            by_id = {n.id: n for n in self.nodes}
            for l in range(L):
                cols[l] = [by_id[k] for k in seq[l] if not isinstance(k, tuple)]

    # 5. coordinates ----------------------------------------------------------------------------
    def _place(self):
        cols = self.cols
        # y: iterate towards straight wires
        stub = {}   # (comp, pin) -> (node, dy of the stub point relative to node.y)
        for n in self.nodes:
            for m in n.members:
                for p in m.sym.pins:
                    if p in m.comp.pins:
                        px, py = m.pin_pos(p)
                        dx, dy = DIRS[m.pin_dir(p)]
                        stub[(m.comp.id, p)] = (n, py + dy, len(m.comp.pins))
        links = defaultdict(list)   # node id -> [(my_off, other node, other_off, weight)]
        # a bus net is tapped from its trunk: lining its pins up gains nothing
        bus = {net for nets in self.bus.values() for net in nets}
        for net, pins in self.net_pins.items():
            if not self.is_signal(net) or net in bus:
                continue
            ends = []
            for k in pins:
                if k not in stub:
                    continue
                ax = self.axis_pins.get(k)
                if ax and ax[1] in pins:
                    # both gates of a complementary pair: one end, on the axis
                    if (k, ax[1]) < (ax[1], k):
                        n, _, mass = stub[k]
                        ends.append((n, ax[0], mass))
                    continue
                ends.append(stub[k])
            for n1, o1, _ in ends:
                for n2, o2, mass in ends:
                    if n1 is n2 or n1.layer == n2.layer:
                        continue
                    # heavy parts (more pins) are harder to move: align to them
                    w = mass / abs(n1.layer - n2.layer) / max(len(ends) - 1, 1)
                    links[n1.id].append((o1, n2, o2, w))
        # initial stacking
        for col in cols:
            y = 0
            for n in col:
                n.y = round(y - n.box[1])
                y += n.h + GAP_Y
        L = len(cols)
        sweeps = list(range(1, L)) + list(range(L - 2, -1, -1))
        sweeps = sweeps * 4 + list(range(0, L))
        for l in sweeps:
            col = cols[l]
            if not col:
                continue
            desired, weights = [], []
            for n in col:
                cands = [(m.y + o2 - o1, w) for o1, m, o2, w in links[n.id]]
                if cands:
                    desired.append(_wmedian(cands, n.y))
                    weights.append(sum(w for _, w in cands))
                else:
                    desired.append(n.y)
                    weights.append(0.01)
            ys = _stack(col, desired, weights)
            for n, y in zip(col, ys):
                n.y = y
        _straighten(self, links)
        self._bus_sources_on_top()
        self._place_columns()
        # normalise
        x0 = min(n.x + n.box[0] for n in self.nodes)
        y0 = min(n.y + n.box[1] for n in self.nodes)
        for n in self.nodes:
            n.x -= round(x0)
            n.y -= round(y0)


def _misalign(links, nodes):
    return sum(w * abs((m.y + o2) - (n.y + o1))
               for n in nodes for o1, m, o2, w in links[n.id])


def _fits(cols, moved):
    """Do the columns touched by `moved` still keep order and gaps?"""
    for col in cols:
        if not any(n.id in moved for n in col):
            continue
        for a, b in zip(col, col[1:]):
            if b.y + b.box[1] < a.y + a.box[3] + gap_y(a, b) - 1e-9:
                return False
    return True


def _block(start, links, exclude):
    """Nodes joined to `start` by links that are already perfectly straight."""
    seen = {start.id: start}
    todo = [start]
    while todo:
        n = todo.pop()
        for o1, m, o2, w in links[n.id]:
            if m.id not in seen and m.id not in exclude and m.y + o2 == n.y + o1:
                seen[m.id] = m
                todo.append(m)
    return seen


def _straighten(lay, links):
    """Whole-chain alignment. The per-node median sweeps can leave a chain
    split into two straight runs at different heights (each half is happy
    with its own neighbours). Here a whole straight run moves at once to
    remove a jog, as long as nothing overlaps and total misalignment drops."""
    nodes = lay.nodes
    for _ in range(4 * len(nodes)):
        improved = False
        base = _misalign(links, nodes)
        jogs = [(w, n, o1, m, o2) for n in nodes for o1, m, o2, w in links[n.id]
                if m.y + o2 != n.y + o1]
        jogs.sort(key=lambda j: -j[0])
        for w, n, o1, m, o2 in jogs:
            if m.y + o2 == n.y + o1:
                continue
            for mover, other, delta in ((n, m, (m.y + o2) - (n.y + o1)),
                                        (m, n, (n.y + o1) - (m.y + o2))):
                blk = _block(mover, links, {other.id})
                if other.id in blk:
                    continue
                for b in blk.values():
                    b.y += delta
                if _fits(lay.cols, blk) and _misalign(links, nodes) < base - 1e-9:
                    improved = True
                    break
                for b in blk.values():
                    b.y -= delta
            if improved:
                break
        if not improved:
            return


def _wmedian(cands, current):
    """Weighted median; on an exact tie keep the candidate nearest `current`
    (averaging two targets would align with neither and leave two jogs)."""
    cands = sorted(cands)
    total = sum(w for _, w in cands)
    acc = 0.0
    for i, (v, w) in enumerate(cands):
        acc += w
        if acc > total / 2 + 1e-9:
            return v
        if abs(acc - total / 2) < 1e-9 and i + 1 < len(cands):
            nxt = cands[i + 1][0]
            return v if abs(v - current) <= abs(nxt - current) else nxt
    return cands[-1][0]


def _stack(col, desired, weights):
    """Closest y positions (weighted least squares) keeping order and gaps.

    Node tops must satisfy top[i+1] >= top[i] + h[i] + gap. Subtracting the
    running offsets turns that into a monotone sequence, which is isotonic
    regression, solved exactly by pool-adjacent-violators.
    """
    import math
    offs, acc = [], 0.0
    for i, n in enumerate(col):
        offs.append(acc)
        acc += n.h + (gap_y(n, col[i + 1]) if i + 1 < len(col) else 0)
    zs = [d + n.box[1] - o for d, n, o in zip(desired, col, offs)]
    blocks = []   # [value, weight, count]
    for z, w in zip(zs, weights):
        blocks.append([z, w, 1])
        while len(blocks) > 1 and blocks[-2][0] > blocks[-1][0]:
            v2, w2, c2 = blocks.pop()
            v1, w1, c1 = blocks.pop()
            blocks.append([(v1 * w1 + v2 * w2) / (w1 + w2), w1 + w2, c1 + c2])
    fitted = []
    for v, w, c in blocks:
        fitted += [v] * c
    res = []
    prev = None
    for z, o, n in zip(fitted, offs, col):
        y = round(z + o - n.box[1])
        if prev is not None:
            pn, py = prev
            y = max(y, math.ceil(py + pn.box[3] + gap_y(pn, n) - n.box[1] - 1e-9))
        res.append(y)
        prev = (n, y)
    return res
