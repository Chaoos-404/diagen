"""The symbol library.

Every symbol is drawn in its own canonical frame (y down). Pins sit on
integer grid points and carry the direction a wire must leave in. The layout
engine only ever rotates symbols by quarter turns and mirrors them, so pins
stay on the grid.

To add a symbol, write a builder that returns a `SymbolDef` and register the
type names in `lookup()` at the bottom of this file.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

from .geom import Circle, Line, Path, Poly, Text, cubic_point


@dataclass
class Pin:
    name: str
    x: int
    y: int
    dir: str               # direction a wire leaves the pin: L R U D
    kind: str | None = None  # "in" | "out" | None (bidirectional / passive)


@dataclass
class SymbolDef:
    type: str
    pins: dict
    prims: list
    body: tuple             # obstacle rectangle for the router
    two_terminal: bool = False
    source: bool = False    # a signal source: layout starts from it
    label_spot: tuple | None = None  # (x, y, anchor) for multi-pin symbols
    label_default: str = "both"      # "both" | "name" | "none"
    polar: bool = False     # pin a is the "+"/anode side


K = 0.5523  # cubic Bezier constant for quarter ellipses


def _half_ellipse_right(cx, cy, rx, ry):
    """Right half of an ellipse, top to bottom, as two cubic segments."""
    top, bot = (cx, cy - ry), (cx, cy + ry)
    return top, [((cx + K * rx, cy - ry), (cx + rx, cy - K * ry), (cx + rx, cy)),
                 ((cx + rx, cy + K * ry), (cx + K * rx, cy + ry), bot)]


# --- two-terminal parts ------------------------------------------------------
# Canonical frame: pin a at (0,0) facing left, pin b at (4,0) facing right.

def _leads(x0, x1):
    return [Line([(0, 0), (x0, 0)]), Line([(x1, 0), (4, 0)])]


def _resistor(style):
    if style == "european":
        return _leads(1, 3) + [Poly([(1, -0.3), (3, -0.3), (3, 0.3), (1, 0.3)])], (1, -0.35, 3, 0.35)
    pts = [(1, 0)]
    for i in range(1, 12, 2):
        pts.append((1 + i / 6, -0.3 if (i // 2) % 2 == 0 else 0.3))
    pts.append((3, 0))
    return _leads(1, 3) + [Line(pts)], (1, -0.35, 3, 0.35)


def _capacitor(_):
    return _leads(1.8, 2.2) + [Line([(1.8, -0.6), (1.8, 0.6)]),
                               Line([(2.2, -0.6), (2.2, 0.6)])], (1.7, -0.65, 2.3, 0.65)


def _cpol(_):
    curve = Path((2.45, -0.6), [((2.2, -0.3), (2.2, 0.3), (2.45, 0.6))])
    return [Line([(0, 0), (1.8, 0)]), Line([(2.25, 0), (4, 0)]),
            Line([(1.8, -0.6), (1.8, 0.6)]), curve,
            Text((1.35, -0.55), "+", "c", 0.8)], (1.2, -0.9, 2.5, 0.65)


def _inductor(_):
    segs = []
    for i in range(4):
        x = 1 + 0.5 * i
        segs.append(((x, -0.33), (x + 0.5, -0.33), (x + 0.5, 0)))
    return _leads(1, 3) + [Path((1, 0), segs)], (1, -0.4, 3, 0.1)


def _diode(kind):
    prims = _leads(1.4, 2.6) + [Poly([(1.4, -0.55), (1.4, 0.55), (2.6, 0)], "black")]
    if kind == "zener":
        prims.append(Line([(2.35, -0.7), (2.6, -0.55), (2.6, 0.55), (2.85, 0.7)]))
    elif kind == "schottky":
        prims.append(Line([(2.85, -0.4), (2.85, -0.55), (2.6, -0.55), (2.6, 0.55),
                           (2.35, 0.55), (2.35, 0.4)]))
    else:
        prims.append(Line([(2.6, -0.55), (2.6, 0.55)]))
    box = (1.4, -0.7, 2.6, 0.7)
    if kind == "led":
        for dx in (0, 0.45):
            a, b = (1.9 + dx, -0.7), (2.4 + dx, -1.2)
            prims.append(Line([a, b]))
            prims.append(Poly([b, (b[0] - 0.25, b[1] + 0.05), (b[0] - 0.05, b[1] + 0.25)], "black"))
        box = (1.4, -1.25, 2.9, 0.7)
    return prims, box


def _circle_source(inner):
    def build(_):
        prims = [Line([(0, 0), (1.2, 0)]), Line([(2.8, 0), (4, 0)]), Circle((2, 0), 0.8)]
        if inner == "v":
            prims += [Text((1.55, 0), "+", "c", 0.8), Text((2.45, 0), "−", "c", 0.8)]
        elif inner == "i":
            prims += [Line([(1.45, 0), (2.35, 0)]),
                      Poly([(2.6, 0), (2.25, -0.2), (2.25, 0.2)], "black")]
        elif inner == "ac":
            prims.append(Path((1.5, 0), [((1.7, -0.6), (1.85, -0.6), (2, 0)),
                                         ((2.15, 0.6), (2.3, 0.6), (2.5, 0))]))
        else:  # meters: a letter
            prims.append(Text((2, 0), inner, "c", 1.0))
        return prims, (1.2, -0.8, 2.8, 0.8)
    return build


def _battery(_):
    return [Line([(0, 0), (1.6, 0)]), Line([(2.4, 0), (4, 0)]),
            Line([(1.6, -0.7), (1.6, 0.7)]), Line([(1.87, -0.35), (1.87, 0.35)]),
            Line([(2.13, -0.7), (2.13, 0.7)]), Line([(2.4, -0.35), (2.4, 0.35)])], (1.5, -0.75, 2.5, 0.75)


def _switch(_):
    return [Line([(0, 0), (1.2, 0)]), Line([(2.8, 0), (4, 0)]),
            Circle((1.3, 0), 0.1), Circle((2.7, 0), 0.1),
            Line([(1.38, -0.05), (2.7, -0.7)])], (1.2, -0.75, 2.8, 0.2)


def _fuse(_):
    return [Line([(0, 0), (1.2, 0)]), Line([(2.8, 0), (4, 0)]),
            Poly([(1.2, -0.25), (2.8, -0.25), (2.8, 0.25), (1.2, 0.25)]),
            Line([(1.2, 0), (2.8, 0)])], (1.2, -0.3, 2.8, 0.3)


def _lamp(_):
    d = 0.8 / math.sqrt(2)
    return [Line([(0, 0), (1.2, 0)]), Line([(2.8, 0), (4, 0)]), Circle((2, 0), 0.8),
            Line([(2 - d, -d), (2 + d, d)]), Line([(2 - d, d), (2 + d, -d)])], (1.2, -0.8, 2.8, 0.8)


TWO_TERMINAL = {
    # type: (artwork builder, pin-a aliases, pin-b aliases, is_source, is_polar)
    "resistor": (_resistor, (), (), False, False),
    "capacitor": (_capacitor, (), (), False, False),
    "cpol": (_cpol, ("+", "p", "pos"), ("-", "n", "neg"), False, True),
    "inductor": (_inductor, (), (), False, False),
    "diode": (lambda s: _diode("d"), ("a", "anode", "+"), ("k", "c", "cathode", "-"), False, True),
    "zener": (lambda s: _diode("zener"), ("a", "anode", "+"), ("k", "c", "cathode", "-"), False, True),
    "schottky": (lambda s: _diode("schottky"), ("a", "anode", "+"), ("k", "c", "cathode", "-"), False, True),
    "led": (lambda s: _diode("led"), ("a", "anode", "+"), ("k", "c", "cathode", "-"), False, True),
    "vsource": (_circle_source("v"), ("+", "p", "pos"), ("-", "n", "neg"), True, True),
    "isource": (_circle_source("i"), ("+", "p", "from"), ("-", "n", "to"), True, True),
    "vac": (_circle_source("ac"), ("+", "p"), ("-", "n"), True, True),
    "battery": (_battery, ("+", "p", "pos"), ("-", "n", "neg"), True, True),
    "switch": (_switch, (), (), False, False),
    "fuse": (_fuse, (), (), False, False),
    "lamp": (_lamp, (), (), False, False),
    "ammeter": (_circle_source("A"), ("+", "p"), ("-", "n"), False, True),
    "voltmeter": (_circle_source("V"), ("+", "p"), ("-", "n"), False, True),
}
TWO_ALIASES = {
    "r": "resistor", "res": "resistor", "c": "capacitor", "cap": "capacitor",
    "ecap": "cpol", "l": "inductor", "ind": "inductor", "d": "diode",
    "v": "vsource", "vdc": "vsource", "vsrc": "vsource", "i": "isource",
    "isrc": "isource", "idc": "isource", "ac": "vac", "vsin": "vac", "bat": "battery",
    "sw": "switch", "am": "ammeter", "vm": "voltmeter",
}


class TwoTerminal:
    def __init__(self, typ):
        self.type = typ
        self.order = ["a", "b"]
        build, a_al, b_al, self.source, self.polar = TWO_TERMINAL[typ]
        self.aliases = {"1": "a", "2": "b", "p": "a", "n": "b", "a": "a", "b": "b"}
        self.aliases.update({x: "a" for x in a_al})
        self.aliases.update({x: "b" for x in b_al})
        self._build = build

    def build(self, comp, opts):
        prims, body = self._build(comp.attrs.get("style") or opts.get("resistor", "american"))
        pins = {"a": Pin("a", 0, 0, "L"), "b": Pin("b", 4, 0, "R")}
        return SymbolDef(self.type, pins, prims, body, two_terminal=True,
                         source=self.source, polar=self.polar)


# --- op-amp ----------------------------------------------------------------

class OpAmp:
    type = "opamp"
    order = ["+", "-", "out", "v+", "v-"]
    aliases = {"+": "+", "in+": "+", "inp": "+", "p": "+", "non": "+",
               "-": "-", "in-": "-", "inn": "-", "n": "-", "inv": "-",
               "out": "out", "o": "out", "y": "out", "vout": "out",
               "v+": "v+", "vcc": "v+", "vdd": "v+", "vs+": "v+",
               "v-": "v-", "vee": "v-", "vss": "v-", "vs-": "v-"}

    def build(self, comp, opts):
        prims = [Poly([(1, -0.3), (1, 4.3), (4.3, 2)]),
                 Line([(0, 1), (1, 1)]), Line([(0, 3), (1, 3)]), Line([(4.3, 2), (5, 2)]),
                 Text((1.4, 1), "−", "c", 0.9), Text((1.4, 3), "+", "c", 0.9)]
        pins = {"-": Pin("-", 0, 1, "L", "in"), "+": Pin("+", 0, 3, "L", "in"),
                "out": Pin("out", 5, 2, "R", "out")}
        edge = 2.3 / 3.3
        if "v+" in comp.pins:
            pins["v+"] = Pin("v+", 2, -1, "U")
            prims.append(Line([(2, -1), (2, -0.3 + edge)]))
        if "v-" in comp.pins:
            pins["v-"] = Pin("v-", 2, 5, "D")
            prims.append(Line([(2, 5), (2, 4.3 - edge)]))
        return SymbolDef("opamp", pins, prims, (1, -0.3, 4.3, 4.3),
                         label_spot=(3.0, 0.6, "sw"), label_default="name")


# --- transistors -------------------------------------------------------------

def _arrow(tip, frm, size=0.3):
    dx, dy = tip[0] - frm[0], tip[1] - frm[1]
    n = math.hypot(dx, dy)
    ux, uy = dx / n, dy / n
    bx, by = tip[0] - ux * size, tip[1] - uy * size
    px, py = -uy * size * 0.5, ux * size * 0.5
    return Poly([tip, (bx + px, by + py), (bx - px, by - py)], "black")


class BJT:
    order = ["c", "b", "e"]
    aliases = {"c": "c", "collector": "c", "b": "b", "base": "b", "e": "e", "emitter": "e"}

    def __init__(self, typ):
        self.type = typ

    def build(self, comp, opts):
        # npn: collector on top, emitter below. pnp: emitter on top.
        top, bot = ("c", "e") if self.type == "npn" else ("e", "c")
        prims = [Line([(0, 2), (1.8, 2)]), Line([(1.8, 1.2), (1.8, 2.8)], width=2.0),
                 Line([(1.8, 1.6), (3, 0.9), (3, 0)]), Line([(1.8, 2.4), (3, 3.1), (3, 4)])]
        if self.type == "npn":
            prims.append(_arrow((2.85, 3.02), (1.8, 2.4)))
        else:
            prims.append(_arrow((1.95, 1.52), (3, 0.9)))
        if str(opts.get("transistor_circle", True)).lower() not in ("false", "0", "no", "off"):
            prims.append(Circle((2.25, 2), 1.3))
        pins = {"b": Pin("b", 0, 2, "L", "in"), top: Pin(top, 3, 0, "U"), bot: Pin(bot, 3, 4, "D")}
        pins["c"].kind = "out"
        return SymbolDef(self.type, pins, prims, (1.0, 0.7, 3.55, 3.3),
                         label_spot=(3.7, 2, "w"))


class MOSFET:
    order = ["d", "g", "s"]
    aliases = {"d": "d", "drain": "d", "g": "g", "gate": "g", "s": "s", "source": "s"}

    def __init__(self, typ):
        self.type = typ

    def build(self, comp, opts):
        top, bot = ("d", "s") if self.type == "nmos" else ("s", "d")
        gx = 1.5
        prims = [Line([(1.5, 1.2), (1.5, 2.8)]), Line([(1.85, 1.1), (1.85, 2.9)], width=2.0),
                 Line([(1.85, 1.4), (3, 1.4), (3, 0)]), Line([(1.85, 2.6), (3, 2.6), (3, 4)])]
        if self.type == "pmos":
            prims += [Circle((1.25, 2), 0.2), Line([(0, 2), (1.05, 2)]),
                      _arrow((2.1, 1.4), (3, 1.4))]
        else:
            prims += [Line([(0, 2), (gx, 2)]), _arrow((3, 2.6), (1.85, 2.6))]
        pins = {"g": Pin("g", 0, 2, "L", "in"), top: Pin(top, 3, 0, "U"), bot: Pin(bot, 3, 4, "D")}
        pins["d"].kind = "out"
        return SymbolDef(self.type, pins, prims, (1.0, 0.9, 3.4, 3.1),
                         label_spot=(3.5, 2, "w"))


# --- logic gates ---------------------------------------------------------------

GATE_OUT = ("y", "out", "o", "q", "z")


class Gate:
    def __init__(self, base, n):
        self.base, self.n = base, n
        self.type = base
        if base in ("not", "buf"):
            self.n = 1
        ins = [chr(ord("a") + i) for i in range(self.n)]
        self.order = ins + ["y"]
        self.aliases = {x: x for x in self.order}
        for i, x in enumerate(ins):
            self.aliases[f"in{i + 1}"] = x
            self.aliases[f"i{i}"] = x
            self.aliases[f"{i + 1}"] = x
        if self.n == 1:
            self.aliases["in"] = "a"
        for x in GATE_OUT:
            self.aliases[x] = "y"

    def build(self, comp, opts):
        n = self.n
        if self.base in ("not", "buf"):
            prims = [Line([(0, 1), (1, 1)]), Poly([(1, 0.1), (1, 1.9), (2.8, 1)])]
            if self.base == "not":
                prims += [Circle((3.0, 1), 0.2), Line([(3.2, 1), (4, 1)])]
            else:
                prims.append(Line([(2.8, 1), (4, 1)]))
            pins = {"a": Pin("a", 0, 1, "L", "in"), "y": Pin("y", 4, 1, "R", "out")}
            return SymbolDef(self.type, pins, prims, (1, 0.1, 3.2, 1.9),
                             label_spot=(1.6, 1.95, "n"), label_default="none")

        h = 2 * n
        top, bot, cy = 0.4, h - 0.4, h / 2
        bh = bot - top
        w = 3.2
        base = self.base
        neg = base in ("nand", "nor", "xnor")
        x0 = 1.0
        prims = []
        if base in ("and", "nand"):
            rx = min(bh / 2, w - 0.8)
            cx = x0 + w - rx
            start, segs = _half_ellipse_right(cx, cy, rx, bh / 2)
            prims.append(Path((x0, bot), [((x0, bot), (x0, top), (x0, top)),
                                          ((x0, top), (cx, top), start)] + segs +
                              [((cx, bot), (x0, bot), (x0, bot))], closed=True))
            back = lambda y: x0
            tip = x0 + w
        else:
            xb = x0 + (0.35 if base in ("xor", "xnor") else 0)
            depth = 0.55
            back_seg = ((xb + depth, top + bh * 0.3), (xb + depth, bot - bh * 0.3), (xb, bot))
            tip = x0 + w
            front_top = ((xb + w * 0.45, top), (tip - 0.6, cy - bh * 0.25), (tip, cy))
            front_bot = ((tip - 0.6, cy + bh * 0.25), (xb + w * 0.45, bot), (xb, bot))
            prims.append(Path((xb, top), [front_top, front_bot,
                                          ((xb + depth, bot - bh * 0.3), (xb + depth, top + bh * 0.3), (xb, top))],
                              closed=True))
            p0, c1, c2, p1 = (xb, top), *back_seg

            def curve_x(y, off=0.0):
                lo, hi = 0.0, 1.0
                for _ in range(30):
                    mid = (lo + hi) / 2
                    if cubic_point(p0, c1, c2, p1, mid)[1] < y:
                        lo = mid
                    else:
                        hi = mid
                return cubic_point(p0, c1, c2, p1, lo)[0] - off
            back = curve_x
            if base in ("xor", "xnor"):
                prims.append(Path((x0, top), [((x0 + depth, top + bh * 0.3), (x0 + depth, bot - bh * 0.3), (x0, bot))]))
                back = lambda y: curve_x(y, 0.35)
        pins = {}
        for i in range(n):
            y = 2 * i + 1
            name = chr(ord("a") + i)
            pins[name] = Pin(name, 0, y, "L", "in")
            prims.append(Line([(0, y), (back(y), y)]))
        out_x = math.ceil(tip + (0.4 if neg else 0) + 0.3)
        if neg:
            prims.append(Circle((tip + 0.2, cy), 0.2))
            prims.append(Line([(tip + 0.4, cy), (out_x, cy)]))
        else:
            prims.append(Line([(tip, cy), (out_x, cy)]))
        pins["y"] = Pin("y", out_x, int(cy), "R", "out")
        return SymbolDef(self.type, pins, prims, (x0, top, tip + (0.4 if neg else 0), bot),
                         label_spot=(x0 + w / 2, bot + 0.1, "n"), label_default="none")


# --- boxes: flip-flops and generic blocks -------------------------------------

def _pin_text(name):
    up = name.upper()
    if up in ("QN", "Q_N", "NQ", "QB", "QBAR", "~Q"):
        return "~Q"
    return name


class Box:
    """A rectangle with pins on its sides. Flip-flops are preset boxes."""
    PRESETS = {
        "dff": (["d", "clk"], ["q", "qn"], "", {"clk"}),
        "tff": (["t", "clk"], ["q", "qn"], "", {"clk"}),
        "jkff": (["j", "clk", "k"], ["q", "qn"], "", {"clk"}),
        "srlatch": (["s", "r"], ["q", "qn"], "", set()),
        "dlatch": (["d", "en"], ["q", "qn"], "", set()),
    }
    PIN_TEXT = {"d": "D", "t": "T", "j": "J", "k": "K", "s": "S", "r": "R", "en": "EN",
                "q": "Q", "qn": "~Q", "clk": ""}

    def __init__(self, typ):
        self.type = typ
        if typ in self.PRESETS:
            left, right, _, self.clocks = self.PRESETS[typ]
            self.order = left + right
            self.aliases = {x: x for x in self.order}
            self.aliases.update({"c": "clk", "ck": "clk", "clock": "clk", ">": "clk",
                                 "q_n": "qn", "nq": "qn", "qb": "qn", "qbar": "qn", "~q": "qn",
                                 "e": "en", "g": "en"})
            self.sides = {"left": left, "right": right, "top": [], "bottom": []}
        else:
            self.order = []
            self.aliases = {}
            self.clocks = set()
            self.sides = None

    def resolve_sides(self, comp):
        if self.sides:
            return self.sides
        sides = {"left": [], "right": [], "top": [], "bottom": []}
        explicit = {}
        for side in sides:
            for p in re.split(r"[,\s]+", comp.attrs.get(side, "")):
                if p:
                    explicit[p] = side
        for p in comp.pins:
            side = explicit.get(p)
            if side is None:
                side = "right" if p.lower() in GATE_OUT or p.lower().startswith("out") else "left"
            sides[side].append(p)
        # keep the order the user listed in left=/right=, else pin order
        for side in sides:
            listed = [p for p in re.split(r"[,\s]+", comp.attrs.get(side, "")) if p in comp.pins]
            if listed:
                sides[side] = listed + [p for p in sides[side] if p not in listed]
        return sides

    def build(self, comp, opts):
        sides = self.resolve_sides(comp)
        clocks = set(self.clocks) | {p for p in re.split(r"[,\s]+", comp.attrs.get("clock", "")) if p}
        text = lambda p: self.PIN_TEXT.get(p, p) if self.type in self.PRESETS else _pin_text(p)
        from .markup import to_plain
        tw = lambda s: len(to_plain(s)) * 0.34 * 0.85
        rows = max(len(sides["left"]), len(sides["right"]), 1)
        h = 2 * rows
        title = comp.attrs.get("text", "")
        inner = max([tw(text(p)) for p in sides["left"]] + [0]) + \
            max([tw(text(p)) for p in sides["right"]] + [0]) + 0.8
        if title:
            inner = max(inner, tw(title) * 1.2 + 0.8)
        cols = max(len(sides["top"]), len(sides["bottom"]))
        w = max(4, math.ceil(inner), 2 * cols)
        if w % 2:
            w += 1
        x0, x1 = 1, 1 + w
        prims = [Poly([(x0, 0), (x1, 0), (x1, h), (x0, h)])]
        pins = {}
        for i, p in enumerate(sides["left"]):
            y = 2 * i + 1
            pins[p] = Pin(p, 0, y, "L", "in")
            prims.append(Line([(0, y), (x0, y)]))
            if p in clocks:
                prims.append(Line([(x0, y - 0.35), (x0 + 0.45, y), (x0, y + 0.35)]))
            if text(p):
                prims.append(Text((x0 + 0.2, y), text(p), "w", 0.85))
        for i, p in enumerate(sides["right"]):
            y = 2 * i + 1
            pins[p] = Pin(p, x1 + 1, y, "R", "out")
            prims.append(Line([(x1, y), (x1 + 1, y)]))
            prims.append(Text((x1 - 0.2, y), text(p), "e", 0.85))
        off = (w - 2 * (len(sides["top"]) - 1)) // 2 + x0
        for i, p in enumerate(sides["top"]):
            x = off + 2 * i
            pins[p] = Pin(p, x, -1, "U")
            prims += [Line([(x, -1), (x, 0)]), Text((x, 0.15), text(p), "n", 0.8)]
        off = (w - 2 * (len(sides["bottom"]) - 1)) // 2 + x0
        for i, p in enumerate(sides["bottom"]):
            x = off + 2 * i
            pins[p] = Pin(p, x, h + 1, "D")
            prims += [Line([(x, h), (x, h + 1)]), Text((x, h - 0.15), text(p), "s", 0.8)]
        if title:
            ty = h / 2 if not sides["top"] else h / 2 + 0.4
            prims.append(Text(((x0 + x1) / 2, ty), title, "c", 1.0))
        spot = ((x0 + x1) / 2, -0.15 if not sides["top"] else -1.1, "s")
        return SymbolDef(self.type, pins, prims, (x0, 0, x1, h), label_spot=spot,
                         label_default="name")


# --- off-sheet ports ---------------------------------------------------------------

class Port:
    order = ["a"]
    aliases = {"a": "a"}

    def __init__(self, typ):
        self.type = typ

    def build(self, comp, opts):
        name = comp.attrs.get("text", comp.net_name)
        if self.type == "input":
            pin = Pin("a", 0, 0, "R", "out")
            prims = [Circle((0, 0), 0.15), Text((-0.3, 0), name, "e")]
        else:
            pin = Pin("a", 0, 0, "L", "in")
            prims = [Circle((0, 0), 0.15), Text((0.3, 0), name, "w")]
        from .geom import text_box
        body = text_box(prims[1])
        return SymbolDef(self.type, {"a": pin}, prims, body, label_default="none")


# --- registry ------------------------------------------------------------------

def lookup(typ: str):
    t = typ.lower()
    t = TWO_ALIASES.get(t, t)
    if t in TWO_TERMINAL:
        return TwoTerminal(t)
    if t in ("opamp", "op", "oa", "comparator"):
        return OpAmp()
    if t in ("npn", "pnp"):
        return BJT(t)
    if t in ("nmos", "pmos", "nfet", "pfet"):
        return MOSFET({"nfet": "nmos", "pfet": "pmos"}.get(t, t))
    if t in ("not", "inv", "inverter"):
        return Gate("not", 1)
    if t in ("buf", "buffer"):
        return Gate("buf", 1)
    m = re.fullmatch(r"(and|or|nand|nor|xor|xnor)(\d*)", t)
    if m:
        return Gate(m.group(1), int(m.group(2) or 2))
    if t in Box.PRESETS or t in ("block", "ic", "box", "chip"):
        return Box(t if t in Box.PRESETS else "block")
    if t in ("input", "output"):
        return Port(t)
    return None


KNOWN_TYPES = sorted(set(TWO_TERMINAL) | set(TWO_ALIASES) | {
    "opamp", "npn", "pnp", "nmos", "pmos", "not", "buf", "and", "or", "nand", "nor",
    "xor", "xnor", "and3", "or4", "dff", "tff", "jkff", "srlatch", "dlatch", "block"})
