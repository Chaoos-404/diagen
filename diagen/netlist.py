"""Parse a netlist (text or JSON) into a `Circuit`.

Text format, one statement per line:

    title  Inverting amplifier
    input  vin                 # off-sheet input ports, drawn on the left
    output vout                # off-sheet output ports, drawn on the right
    V1  vsource vin 0 5V       # ID TYPE pins... [value] [key=value...]
    R1  res   vin n1 10k
    U1  opamp +=0 -=n1 out=vout
    X1  and2  a b y            # positional pins use the type's pin order
    B1  block A=a B=b Y=y right=Y text=ALU
    rail VREF                  # draw net VREF as a supply symbol, not a wire
    option resistor=european

Nets named 0/gnd/ground/vss are drawn as ground symbols. Nets named like
vcc/vdd/vee/+5V/-12V are drawn as supply symbols. Everything else is wired.
"""
from __future__ import annotations

import json
import re
import shlex
from dataclasses import dataclass, field

from .symbols import KNOWN_TYPES, lookup

ATTR_KEYS = {"value", "label", "style", "left", "right", "top", "bottom", "text",
             "clock", "rank", "flip", "rot", "name"}
GROUND = {"0", "gnd", "ground", "vss", "agnd", "dgnd", "gnd!", "com"}
SUPPLY_POS = re.compile(r"^(vcc|vdd|v\+|vbat|vsup|vs\+|avdd|dvdd|vin_?supply|\+\d+(\.\d+)?v)$", re.I)
SUPPLY_NEG = re.compile(r"^(vee|v-|vs-|-\d+(\.\d+)?v)$", re.I)


class ParseError(Exception):
    def __init__(self, errors):
        super().__init__("\n".join(errors))
        self.errors = errors


@dataclass
class Component:
    id: str
    type: str
    spec: object
    pins: dict                  # canonical pin name -> net
    attrs: dict = field(default_factory=dict)
    line: int = 0
    net_name: str = ""          # ports only

    @property
    def value(self):
        return self.attrs.get("value", "")


@dataclass
class Circuit:
    components: list = field(default_factory=list)
    title: str = ""
    rails: dict = field(default_factory=dict)      # net -> "gnd" | "pos" | "neg"
    options: dict = field(default_factory=dict)
    warnings: list = field(default_factory=list)

    def rail_kind(self, net):
        if net in self.rails:
            return self.rails[net]
        if net.lower() in GROUND:
            return "gnd"
        if SUPPLY_POS.match(net):
            return "pos"
        if SUPPLY_NEG.match(net):
            return "neg"
        return None

    def nets(self):
        out = {}
        for c in self.components:
            for p, n in c.pins.items():
                out.setdefault(n, []).append((c, p))
        return out


def _make(cid, typ, positional, named, attrs, line, errors):
    spec = lookup(typ)
    if spec is None:
        errors.append(f"line {line}: unknown type '{typ}' for {cid}. Known: {', '.join(KNOWN_TYPES)}")
        return None
    pins = {}
    custom = not spec.order  # generic blocks take any pin names
    for key, net in named.items():
        k = key if custom else spec.aliases.get(key.lower())
        if k is None:
            errors.append(f"line {line}: {cid} ({typ}) has no pin '{key}'. Pins: {', '.join(spec.order)}")
            continue
        pins[k] = net
    free = [p for p in spec.order if p not in pins]
    extra = []
    for tok in positional:
        if free:
            pins[free.pop(0)] = tok
        else:
            extra.append(tok)
    if custom and positional:
        errors.append(f"line {line}: {cid} is a block; name its pins, e.g. A=net1 Y=net2")
    if extra:
        if "value" not in attrs and len(extra) == 1:
            attrs["value"] = extra[0]
        else:
            errors.append(f"line {line}: {cid} has too many positional arguments: {' '.join(extra)}")
    return Component(cid, typ.lower(), spec, pins, attrs, line)


def _split(line):
    """Whitespace split honouring quotes, but keeping backslashes (\\Omega)."""
    lex = shlex.shlex(line, posix=True)
    lex.whitespace_split = True
    lex.escape = ""
    lex.commenters = ""
    return list(lex)


def parse_text(text: str) -> Circuit:
    ckt = Circuit()
    errors = []
    inputs, outputs = [], []
    for ln, raw in enumerate(text.splitlines(), 1):
        line = re.split(r"\s(?:#|//)|^(?:#|//|\*)", raw, maxsplit=1)[0].strip()
        if not line:
            continue
        try:
            toks = _split(line)
        except ValueError as e:
            errors.append(f"line {ln}: {e}")
            continue
        head = toks[0].lower()
        if head == "title":
            ckt.title = line.split(None, 1)[1] if len(toks) > 1 else ""
            continue
        if head in ("input", "inputs"):
            inputs += toks[1:]
            continue
        if head in ("output", "outputs"):
            outputs += toks[1:]
            continue
        if head in ("rail", "supply", "ground", "wire"):
            for t in toks[1:]:
                name, _, kind = t.partition("=")
                if head == "wire":
                    ckt.rails[name] = None
                elif head == "ground":
                    ckt.rails[name] = "gnd"
                else:
                    ckt.rails[name] = kind or ("neg" if name.startswith("-") else "pos")
            continue
        if head in ("option", "options"):
            for t in toks[1:]:
                k, _, v = t.partition("=")
                ckt.options[k] = v or "true"
            continue
        if len(toks) < 2:
            errors.append(f"line {ln}: expected 'ID TYPE pins...', got '{line}'")
            continue
        cid, typ = toks[0], toks[1]
        positional, named, attrs = [], {}, {}
        for t in toks[2:]:
            if "=" in t[1:]:
                k, v = t.split("=", 1) if not t.startswith("=") else ("=", t[1:])
                if k.lower() in ATTR_KEYS:
                    attrs[k.lower()] = v
                else:
                    named[k] = v
            else:
                positional.append(t)
        c = _make(cid, typ, positional, named, attrs, ln, errors)
        if c:
            ckt.components.append(c)
    _add_ports(ckt, inputs, outputs)
    _validate(ckt, errors)
    return ckt


def parse_json(data) -> Circuit:
    if isinstance(data, str):
        data = json.loads(data)
    ckt = Circuit(title=data.get("title", ""))
    ckt.rails.update(data.get("rails", {}))
    ckt.options.update(data.get("options", {}))
    errors = []
    for i, c in enumerate(data.get("components", []), 1):
        pins = c.get("pins", {})
        positional = pins if isinstance(pins, list) else []
        named = pins if isinstance(pins, dict) else {}
        attrs = {k: str(v) for k, v in c.items() if k in ATTR_KEYS}
        comp = _make(c.get("id", f"X{i}"), c.get("type", "?"), positional, named, attrs, i, errors)
        if comp:
            ckt.components.append(comp)
    _add_ports(ckt, data.get("inputs", []), data.get("outputs", []))
    _validate(ckt, errors)
    return ckt


def parse(text: str) -> Circuit:
    s = text.lstrip()
    if s.startswith("{"):
        return parse_json(s)
    return parse_text(text)


def _add_ports(ckt, inputs, outputs):
    for kind, names in (("input", inputs), ("output", outputs)):
        for n in names:
            spec = lookup(kind)
            c = Component(f"{kind}:{n}", kind, spec, {"a": n}, {}, 0, n)
            ckt.components.append(c)


def _validate(ckt, errors):
    seen = set()
    for c in ckt.components:
        if c.id in seen:
            errors.append(f"line {c.line}: duplicate component id '{c.id}'")
        seen.add(c.id)
        if not c.pins:
            errors.append(f"line {c.line}: {c.id} has no connections")
        if c.spec.order and c.type not in ("opamp", "op", "oa", "comparator"):
            missing = [p for p in c.spec.order if p not in c.pins]
            if missing and c.spec.order != ["a"]:
                ckt.warnings.append(f"{c.id}: pins left unconnected: {', '.join(missing)}")
    for net, conns in ckt.nets().items():
        if len(conns) == 1 and ckt.rail_kind(net) is None:
            c, p = conns[0]
            if c.type not in ("input", "output"):
                ckt.warnings.append(f"net '{net}' only touches {c.id}.{p} (dangling)")
    if errors:
        raise ParseError(errors)
