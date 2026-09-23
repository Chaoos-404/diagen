"""Drawing primitives and the quarter-turn transforms used to orient symbols.

All coordinates are in grid units with y pointing down. One grid unit is
0.5 cm in TikZ output and 20 px in SVG output. Wires and pins always sit on
integer grid points; symbol artwork may use fractional coordinates.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

DIRS = {"L": (-1, 0), "R": (1, 0), "U": (0, -1), "D": (0, 1)}
OPPOSITE = {"L": "R", "R": "L", "U": "D", "D": "U"}


@dataclass
class Line:
    pts: list
    dashed: bool = False
    color: str | None = None
    width: float = 1.0


@dataclass
class Poly:
    pts: list
    fill: str = "none"  # "none" | "black" | "white"


@dataclass
class Circle:
    c: tuple
    r: float
    fill: str = "none"


@dataclass
class Path:
    """A cubic Bezier path: start point, then (c1, c2, end) segments."""
    start: tuple
    segs: list
    closed: bool = False
    fill: str = "none"


@dataclass
class Text:
    pos: tuple
    s: str
    anchor: str = "c"  # c n s e w ne nw se sw: the side of the text box at pos
    size: float = 1.0


@dataclass
class Transform:
    """Mirror (flip y), then rotate by `rot` quarter turns clockwise, then translate."""
    rot: int = 0
    mirror: bool = False
    dx: float = 0
    dy: float = 0

    def pt(self, p):
        x, y = p
        if self.mirror:
            y = -y
        for _ in range(self.rot % 4):
            x, y = -y, x
        return (x + self.dx, y + self.dy)

    def dir(self, d):
        x, y = DIRS[d]
        if self.mirror:
            y = -y
        for _ in range(self.rot % 4):
            x, y = -y, x
        for k, v in DIRS.items():
            if v == (x, y):
                return k
        raise ValueError(d)

    def anchor(self, a):
        if a == "c":
            return a
        vec = {"n": (0, -1), "s": (0, 1), "e": (1, 0), "w": (-1, 0)}
        x = y = 0
        for ch in a:
            vx, vy = vec[ch]
            x += vx
            y += vy
        if self.mirror:
            y = -y
        for _ in range(self.rot % 4):
            x, y = -y, x
        out = ("n" if y < 0 else "s" if y > 0 else "") + ("w" if x < 0 else "e" if x > 0 else "")
        return out or "c"


def apply(t: Transform, prim):
    if isinstance(prim, Line):
        return replace(prim, pts=[t.pt(p) for p in prim.pts])
    if isinstance(prim, Poly):
        return replace(prim, pts=[t.pt(p) for p in prim.pts])
    if isinstance(prim, Circle):
        return replace(prim, c=t.pt(prim.c))
    if isinstance(prim, Path):
        return replace(prim, start=t.pt(prim.start),
                       segs=[(t.pt(a), t.pt(b), t.pt(c)) for a, b, c in prim.segs])
    if isinstance(prim, Text):
        return replace(prim, pos=t.pt(prim.pos), anchor=t.anchor(prim.anchor))
    raise TypeError(prim)


# --- text metrics -----------------------------------------------------------

CHAR_W = 0.34   # average glyph advance at size 1.0, in grid units
LINE_H = 0.85   # line height at size 1.0


def visible_text(s: str) -> str:
    """Approximate what a label looks like once markup is rendered."""
    from .markup import to_plain
    return to_plain(s)


def text_box(t: Text):
    """Bounding box (x0, y0, x1, y1) of a text primitive."""
    w = max(len(visible_text(t.s)), 1) * CHAR_W * t.size + 0.2
    h = LINE_H * t.size
    x, y = t.pos
    a = t.anchor
    x0 = x - w / 2 if ("e" not in a and "w" not in a) else (x if "w" in a else x - w)
    y0 = y - h / 2 if ("n" not in a and "s" not in a) else (y if "n" in a else y - h)
    return (x0, y0, x0 + w, y0 + h)


def prim_box(prim):
    if isinstance(prim, (Line, Poly)):
        xs = [p[0] for p in prim.pts]
        ys = [p[1] for p in prim.pts]
        return (min(xs), min(ys), max(xs), max(ys))
    if isinstance(prim, Circle):
        x, y = prim.c
        return (x - prim.r, y - prim.r, x + prim.r, y + prim.r)
    if isinstance(prim, Path):
        pts = [prim.start] + [p for seg in prim.segs for p in seg]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return (min(xs), min(ys), max(xs), max(ys))
    if isinstance(prim, Text):
        return text_box(prim)
    raise TypeError(prim)


def union(boxes):
    boxes = [b for b in boxes if b]
    if not boxes:
        return None
    return (min(b[0] for b in boxes), min(b[1] for b in boxes),
            max(b[2] for b in boxes), max(b[3] for b in boxes))


def cubic_point(p0, c1, c2, p1, t):
    u = 1 - t
    return (u**3 * p0[0] + 3 * u * u * t * c1[0] + 3 * u * t * t * c2[0] + t**3 * p1[0],
            u**3 * p0[1] + 3 * u * u * t * c1[1] + 3 * u * t * t * c2[1] + t**3 * p1[1])


@dataclass
class Drawing:
    prims: list = field(default_factory=list)
    bbox: tuple = (0, 0, 1, 1)
    title: str | None = None
