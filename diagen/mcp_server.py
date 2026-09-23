"""A dependency-free MCP server (stdio, JSON-RPC 2.0) exposing diagen to AI agents.

Register it with Claude Code:

    claude mcp add diagen -- python3 -m diagen.mcp_server

Tools:
  render_circuit     netlist -> report, TikZ/SVG text, files, and a PNG preview
                     the model can look at to check its own work
  netlist_reference  the netlist language reference
"""
from __future__ import annotations

import base64
import json
import os
import sys
import tempfile

from . import __version__
from .cli import svg_to_png
from .engine import build
from .netlist import ParseError, parse
from .render import to_svg, to_tikz

HERE = os.path.dirname(os.path.abspath(__file__))
REFERENCE = os.path.join(HERE, "NETLIST.md")

TOOLS = [
    {
        "name": "render_circuit",
        "description": (
            "Lay out and route a circuit schematic (analog or digital) from a netlist, "
            "producing TikZ and SVG with every coordinate resolved, so no manual "
            "adjustment is needed. Describe only connectivity: 'ID TYPE pins... [value]' "
            "per line, e.g. 'R1 res vin out 10k', 'U1 opamp +=0 -=n1 out=vout', "
            "'X1 and2 a b y', 'Q1 npn c=c b=b e=e'. Nets named 0/gnd become ground symbols, "
            "vcc/vdd/+5V become supply symbols. Use 'input NAME' / 'output NAME' for ports. "
            "Call netlist_reference for the full list of part types. Returns a quality "
            "report (OK/INCOMPLETE, crossings, warnings about dangling nets) and a PNG "
            "preview; fix problems by editing the netlist."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "netlist": {"type": "string", "description": "The netlist text (or JSON)."},
                "formats": {
                    "type": "array", "items": {"enum": ["tikz", "svg"]},
                    "description": "Which sources to return inline. Default: ['tikz'].",
                },
                "standalone": {"type": "boolean",
                               "description": "Wrap TikZ in a compilable standalone document."},
                "out_dir": {"type": "string",
                            "description": "Optional directory to also write <name>.svg/.tex/.png into."},
                "name": {"type": "string", "description": "Base file name for out_dir. Default 'circuit'."},
                "preview": {"type": "boolean", "description": "Return a PNG preview image. Default true."},
                "resistor": {"enum": ["american", "european"]},
            },
            "required": ["netlist"],
        },
    },
    {
        "name": "netlist_reference",
        "description": "The diagen netlist language: statements, all part types and pin names, rails, options.",
        "inputSchema": {"type": "object", "properties": {}},
    },
]


def call_render(args):
    try:
        ckt = parse(args["netlist"])
    except ParseError as e:
        return [{"type": "text", "text": "Netlist errors:\n" + "\n".join(e.errors)}], True
    opts = {}
    if args.get("resistor"):
        opts["resistor"] = args["resistor"]
    drawing, report = build(ckt, opts)
    svg = to_svg(drawing)
    tikz = to_tikz(drawing, standalone=bool(args.get("standalone")))
    content = [{"type": "text", "text": report.text()}]
    formats = args.get("formats") or ["tikz"]
    if "tikz" in formats:
        content.append({"type": "text", "text": "TikZ (needs only \\usepackage{tikz}):\n" + tikz})
    if "svg" in formats:
        content.append({"type": "text", "text": svg})
    out_dir = args.get("out_dir")
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        base = os.path.join(out_dir, args.get("name") or "circuit")
        with open(base + ".svg", "w", encoding="utf-8") as f:
            f.write(svg)
        with open(base + ".tex", "w", encoding="utf-8") as f:
            f.write(tikz)
        wrote = [base + ".svg", base + ".tex"]
        if svg_to_png(svg, base + ".png"):
            wrote.append(base + ".png")
        content.append({"type": "text", "text": "Wrote " + ", ".join(wrote)})
    if args.get("preview", True):
        with tempfile.TemporaryDirectory() as d:
            png = os.path.join(d, "preview.png")
            if svg_to_png(svg, png, size=1200):
                with open(png, "rb") as f:
                    content.append({"type": "image", "mimeType": "image/png",
                                    "data": base64.b64encode(f.read()).decode()})
    return content, not report.ok


def handle(msg):
    method = msg.get("method")
    mid = msg.get("id")
    params = msg.get("params") or {}
    if mid is None:          # notification
        return None
    if method == "initialize":
        result = {
            "protocolVersion": params.get("protocolVersion", "2024-11-05"),
            "capabilities": {"tools": {}},
            "serverInfo": {"name": "diagen", "version": __version__},
            "instructions": "Use render_circuit to draw schematics from netlists; read "
                            "netlist_reference first if unsure about part types.",
        }
    elif method == "ping":
        result = {}
    elif method == "tools/list":
        result = {"tools": TOOLS}
    elif method == "tools/call":
        name = params.get("name")
        args = params.get("arguments") or {}
        try:
            if name == "render_circuit":
                content, is_error = call_render(args)
            elif name == "netlist_reference":
                with open(REFERENCE, encoding="utf-8") as f:
                    content, is_error = [{"type": "text", "text": f.read()}], False
            else:
                return {"jsonrpc": "2.0", "id": mid,
                        "error": {"code": -32602, "message": f"unknown tool {name}"}}
        except Exception as e:  # report engine bugs to the model instead of dying
            content, is_error = [{"type": "text", "text": f"diagen internal error: {e!r}"}], True
        result = {"content": content, "isError": is_error}
    else:
        return {"jsonrpc": "2.0", "id": mid,
                "error": {"code": -32601, "message": f"method not found: {method}"}}
    return {"jsonrpc": "2.0", "id": mid, "result": result}


def main():
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue
        reply = handle(msg)
        if reply is not None:
            sys.stdout.write(json.dumps(reply) + "\n")
            sys.stdout.flush()


if __name__ == "__main__":
    main()
