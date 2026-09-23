"""Command line: python -m diagen circuit.cir -o circuit.svg -o circuit.tex"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

from .engine import build
from .netlist import ParseError, parse
from .render import to_svg, to_tikz


def svg_to_png(svg_text: str, out_path: str, size=1600) -> bool:
    """Rasterise with macOS Quick Look + sips (no extra installs). Returns success.

    Quick Look fits the image into a square and sips crops around the centre,
    so the drawing is centred on a square canvas first and cropped back after.
    """
    import re
    if not (shutil.which("qlmanage") and shutil.which("sips")):
        return False
    m = re.search(r'width="([\d.]+)" height="([\d.]+)"', svg_text)
    w, h = float(m.group(1)), float(m.group(2))
    side = max(w, h)
    inner = svg_text.split(">", 1)[1].rsplit("</svg>", 1)[0]
    head = svg_text.split(">", 1)[0]
    # render the square at the final pixel size: Quick Look will not upscale
    square = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{size}" '
              f'viewBox="0 0 {side} {side}"><rect width="100%" height="100%" fill="white"/>'
              f'{head.replace("<svg", "<svg x=\"%s\" y=\"%s\"" % ((side - w) / 2, (side - h) / 2), 1)}>'
              f'{inner}</svg></svg>')
    with tempfile.TemporaryDirectory() as d:
        src = os.path.join(d, "diagram.svg")
        with open(src, "w") as f:
            f.write(square)
        subprocess.run(["qlmanage", "-t", "-s", str(size), "-o", d, src],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        png = src + ".png"
        if not os.path.exists(png):
            return False
        scale = size / side
        subprocess.run(["sips", "-c", str(round(h * scale)), str(round(w * scale)), png],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        shutil.move(png, out_path)
    return True


def main(argv=None):
    ap = argparse.ArgumentParser(prog="diagen", description=__doc__)
    ap.add_argument("netlist", help="netlist file ('-' for stdin); text or JSON")
    ap.add_argument("-o", "--out", action="append", default=[],
                    help="output file: .svg, .tex (TikZ picture) or .png; repeatable")
    ap.add_argument("--standalone", action="store_true", help="wrap TikZ in a standalone document")
    ap.add_argument("--unit", type=float, default=0.5, help="TikZ grid unit in cm (default 0.5)")
    ap.add_argument("--json", action="store_true", help="print the report as JSON")
    ap.add_argument("--resistor", choices=["american", "european"], help="resistor style")
    ap.add_argument("--ports", choices=["auto", "edge", "near"], help="where ports go")
    ap.add_argument("--routing", choices=["direct", "bus"],
                    help="bus: shared inputs run as vertical trunks that gates tap onto")
    ap.add_argument("--stages", choices=["auto", "off"],
                    help="lay out repeated stages side by side, all alike (default auto)")
    args = ap.parse_args(argv)

    text = sys.stdin.read() if args.netlist == "-" else open(args.netlist, encoding="utf-8").read()
    opts = {}
    for key in ("resistor", "ports", "routing", "stages"):
        if getattr(args, key):
            opts[key] = getattr(args, key)
    try:
        ckt = parse(text)
    except ParseError as e:
        for msg in e.errors:
            print(f"error: {msg}", file=sys.stderr)
        return 2
    drawing, report = build(ckt, opts)
    outs = args.out or [os.path.splitext(args.netlist)[0] + ".svg" if args.netlist != "-" else "-"]
    for path in outs:
        if path == "-":
            sys.stdout.write(to_svg(drawing))
            continue
        ext = os.path.splitext(path)[1].lower()
        if ext == ".tex":
            content = to_tikz(drawing, args.unit, args.standalone)
        elif ext == ".png":
            if not svg_to_png(to_svg(drawing), path):
                print("error: PNG export needs macOS qlmanage", file=sys.stderr)
                return 1
            continue
        else:
            content = to_svg(drawing)
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
    if args.json:
        print(json.dumps(report.as_dict(), indent=2))
    else:
        print(report.text(), file=sys.stderr)
    return 0 if report.ok else 1
