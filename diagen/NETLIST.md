# Diagen netlist reference

You describe **what is connected**, never where anything goes. The layout
engine places parts by signal flow (left to right), draws ground and supply
nets as local symbols, and routes every wire orthogonally. The output is SVG
and plain TikZ with every coordinate resolved.

## Statements

One statement per line. `#` or `//` starts a comment.

```
title  Common-emitter amplifier     # optional
input  vin                          # off-sheet input port(s), drawn on the left
output vout                         # off-sheet output port(s), drawn on the right
C1  cap  vin b   10u                # ID TYPE pins... [value] [key=value ...]
Q1  npn  c=c b=b e=e                # pins by name
RC  res  vcc c   4.7k
```

A part is `ID TYPE` followed by its connections:

* **Positional**: nets in the type's pin order (see the table below).
* **Named**: `pin=net`, in any order, mixed with positional.
* **Value**: one extra positional token after all pins, or `value=...`.
* **Attributes**: `label=` (replaces the ID in the drawing; `label=""` shows
  the value only), `flip=1` (mirror a multi-pin part), `rank=N` (force column N),
  `stage=N` (put the part in repeated stage N, see below).
* Quote tokens with spaces: `"1 \mu F"`.

Net names are any token without spaces. Nets connect by name.

## Rails are symbols, not wires

| net name (case-insensitive) | drawn as |
|---|---|
| `0` `gnd` `ground` `vss` `agnd` `dgnd` `com` | ground symbol at each pin |
| `vcc` `vdd` `v+` `vbat` `+5V` `+3.3V` ... | supply bar with the net name, pointing up |
| `vee` `v-` `-12V` ... | supply bar pointing down |

* `rail NAME` makes any net a positive supply symbol, `rail NAME=neg` negative,
  `ground NAME` a ground, and `wire NAME` forces a rail-looking name to be wired.
* A two-terminal part between a signal net and a rail hangs vertically.
  Between two signal nets it lies horizontally in the signal path.

## Part types

Two-terminal (pins `a b`, drawn a → b left to right, or top to bottom when hanging):

| type (aliases) | pin a | pin b |
|---|---|---|
| `resistor` (`res`, `r`) | 1 | 2 |
| `capacitor` (`cap`, `c`) | 1 | 2 |
| `cpol` (`ecap`) polarised | `+` | `-` |
| `inductor` (`ind`, `l`) | 1 | 2 |
| `diode` (`d`), `zener`, `schottky`, `led` | anode `a` | cathode `k` |
| `vsource` (`v`, `vdc`) | `+` | `-` |
| `vac` (`ac`, `vsin`) | `+` | `-` |
| `isource` (`i`) arrow points a → b | `from` | `to` |
| `battery` (`bat`) | `+` | `-` |
| `switch` (`sw`), `fuse`, `lamp` | 1 | 2 |
| `ammeter` (`am`), `voltmeter` (`vm`) | `+` | `-` |

Multi-pin:

| type | positional order | pin names (aliases) |
|---|---|---|
| `opamp` (`comparator`) | `+ - out [v+ v-]` | `+` (`in+`, `non`), `-` (`in-`, `inv`), `out`, `v+` (`vcc`), `v-` (`vee`) |
| `npn`, `pnp` | `c b e` | `c` collector, `b` base, `e` emitter |
| `nmos`, `pmos` | `d g s` | `d` drain, `g` gate, `s` source |
| `and` `or` `nand` `nor` `xor` `xnor` + input count (`and3`, default 2) | `a b ... y` | inputs `a b c d` (`in1`...), output `y` (`out`, `q`) |
| `not` (`inv`), `buf` | `a y` | `a`, `y` |
| `dff` | `d clk q qn` | `d`, `clk`, `q`, `qn` |
| `jkff` | `j clk k q qn` | |
| `tff` | `t clk q qn` | |
| `srlatch` | `s r q qn` | |
| `dlatch` | `d en q qn` | |
| `block` (`ic`, `chip`) | named pins only | any names |

`block` draws a box. Pins go on the left unless listed in `right=`, `top=` or
`bottom=`. Names like `y`, `q`, `out*` default to the right. `text=` is the
caption inside the box and `clock=CLK` draws a clock triangle:

```
U3 block A=a B=b OP=op Y=y FLAGS=f right=Y,FLAGS text=ALU
```

## Labels

Labels use LaTeX-like markup. It becomes real math in TikZ and styled text in SVG:
`R_1`, `R_{out}`, `V^2`, `\overline{Q}` or `~Q`, `10\,k\Omega`, `4.7\mu F`.
Unicode `Ω µ` also works.

## Options

`option key=value` in the netlist, or the matching CLI flag:

* `ports=auto|edge|near`: where input/output ports go. `auto` (the default)
  tries both and keeps the clearer layout.
* `routing=direct|bus`: with `bus`, a net that feeds several gates of one
  column (select lines, decoder inputs) is drawn as a vertical trunk that
  each gate taps onto, with its source above the gates, as in textbook
  decoders and multiplexers. `direct` (the default) routes every net as short
  as it can.
* `stages=auto|off`: repeated stages are laid out side by side, all alike
  (see below). `off` lays the circuit out as one piece.
* `symmetry=auto|off`: transistors are drawn as textbook cells: stacked
  through their collector/emitter (drain/source) pins, symmetric halves side
  by side as mirror images (differential pair, current mirror, SRAM,
  H-bridge) or as copies (parallel pull-ups), the tail on the axis. `off`
  places every transistor on its own. A transistor with `flip=`, `rot=` or
  `rank=` is left out of cells.
* `resistor=american|european`: zigzag or box resistors.
* `transistor_circle=false`: no envelope circle on BJTs.

## How the layout reads your circuit (so you can steer it)

* Signal flows left to right from `input` ports and sources. Directed pins
  (gate/op-amp outputs, transistor bases) define the flow in digital and
  amplifier circuits.
* A part in parallel with an op-amp or gate from an input net to its output
  net is drawn as feedback above or below it (inverting and non-inverting
  amps come out textbook-style).
* A load hanging from a transistor's collector, emitter, drain or source to a
  rail is stacked in line with it. Parallel parts to the same rail sit side by
  side. A supply-side part and a ground-side part on the same net form a
  vertical divider.
* Two gates that feed each other (latches) share a column.
* Repeated stages (a ripple adder built from gates, cascaded amplifier
  stages) are found when each stage has the same parts and is linked to the
  next by one net. They are drawn one after another, all in the same
  arrangement. When the stages share more than one net (a common clock or
  enable), tag the parts with `stage=1`, `stage=2`, ... instead.
* To get a ladder or bridge drawn with vertical dividers, name the top node
  as a rail (`vcc`, or `rail VTOP`) so that each branch becomes a divider.

## Reading the report

Every render prints a line such as
`OK: 8 parts, 5 wired nets, 35x21 grid units` followed by wire length, bends,
crossings and junctions. `INCOMPLETE` means a net could not be routed and is
drawn as a dashed red air wire. Warnings flag dangling nets (usually a typo
in a net name) and unconnected pins. Fix the netlist, not coordinates.
