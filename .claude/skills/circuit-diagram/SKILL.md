---
name: circuit-diagram
description: Draw analog or digital circuit schematics (op-amps, transistor amplifiers, filters, logic gates, flip-flops, block diagrams) as TikZ or SVG from a netlist, using the diagen layout engine instead of hand-placing TikZ/circuitikz coordinates. Use whenever a circuit diagram, schematic, or logic diagram is needed for LaTeX, notes, or docs.
---

# Drawing circuits with diagen

Never hand-write TikZ or circuitikz coordinates for a schematic. Write a
**netlist** (what connects to what), let diagen place and route it, look at
the preview, and fix the netlist rather than the coordinates.

## Workflow

1. Write the netlist. The full language is in `diagen/NETLIST.md`; read it if
   you need a part type that is not shown below.
2. Render it:
   - with the MCP tool `render_circuit` (returns a report, the TikZ, and a PNG
     preview), or
   - from the shell: `python3 -m diagen circuit.cir -o circuit.tex -o circuit.png`
     (add `--standalone` for a compilable document). Then Read the PNG.
3. Check the report: it must say `OK`. Fix any `warning: net 'x' only touches ...`
   because that is almost always a misspelled net name.
4. Look at the preview. If something reads badly, change the netlist:
   declare `input`/`output` ports, give the supply its rail name (`vcc`, `vdd`,
   `+5V`) so it becomes a symbol, split a net, or add `rank=N` / `flip=1`.
   For decoders and multiplexers, `option routing=bus` draws the select lines
   as vertical buses. Repeated stages are laid out alike automatically; tag
   them with `stage=N` if they share a clock or enable.
5. Hand over the `.tex` (`\usepackage{tikz}` is the only requirement) or the `.svg`.

## Netlist in one screen

```
title Inverting amplifier
input  vin                       # ports: left edge / right edge
output vout
R1 res   vin n1 10k              # ID TYPE pins... [value]
Rf res   n1 vout 100k            # feedback across U1 is drawn above it automatically
U1 opamp +=0 -=n1 out=vout       # named pins; net 0 = ground symbol
```

```
title Full adder
input a b cin
output s cout
X1 xor2 a b p                    # gates: inputs..., output
X2 xor2 p cin s
A1 and2 a b g
A2 and2 p cin t
O1 or2 g t cout
```

Common types: `res cap ind cpol diode zener led vsource vac isource battery
switch` (two-terminal), `opamp` (`+ - out`), `npn`/`pnp` (`c b e`),
`nmos`/`pmos` (`d g s`), `and2 or3 nand2 nor2 xor2 xnor2 not buf`, `dff`
(`d clk q qn`), `jkff`, `tff`, `srlatch`, and `block` for any box
(`B1 block A=a Y=y right=Y text=ALU`).

Labels take LaTeX-style markup: `R_{f}`, `10\,k\Omega`, `~Q`.
