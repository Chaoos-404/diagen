import glob
import json
import os
import random
import unittest

from diagen import ParseError, build, parse, to_svg, to_tikz
from diagen.markup import to_plain, to_tex
from diagen.symbols import KNOWN_TYPES

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLES = sorted(glob.glob(os.path.join(ROOT, "examples", "*.cir")))


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def render(text, **opts):
    return build(parse(text), opts)


class ExamplesTest(unittest.TestCase):
    def test_every_example_routes_completely(self):
        self.assertTrue(EXAMPLES)
        for path in EXAMPLES:
            with self.subTest(example=os.path.basename(path)):
                drawing, report = render(read(path))
                self.assertTrue(report.ok, report.text())
                self.assertIn("<svg", to_svg(drawing))
                self.assertIn(r"\begin{tikzpicture}", to_tikz(drawing))

    def test_layout_is_deterministic(self):
        text = read(EXAMPLES[0])
        self.assertEqual(to_svg(render(text)[0]), to_svg(render(text)[0]))


class ParserTest(unittest.TestCase):
    def test_positional_named_and_value(self):
        ckt = parse("R1 res a b 10k\nU1 opamp +=a -=b out=c\n")
        r1, u1 = ckt.components
        self.assertEqual(r1.pins, {"a": "a", "b": "b"})
        self.assertEqual(r1.value, "10k")
        self.assertEqual(u1.pins, {"+": "a", "-": "b", "out": "c"})

    def test_aliases(self):
        ckt = parse("D1 diode anode=x cathode=y\nQ1 npn base=b collector=c emitter=0\n")
        self.assertEqual(ckt.components[0].pins, {"a": "x", "b": "y"})
        self.assertEqual(ckt.components[1].pins, {"b": "b", "c": "c", "e": "0"})

    def test_errors_are_collected_with_line_numbers(self):
        with self.assertRaises(ParseError) as cm:
            parse("R1 res a b\nX1 frobnicator a b\nU1 opamp q=1\n")
        msgs = "\n".join(cm.exception.errors)
        self.assertIn("line 2", msgs)
        self.assertIn("unknown type", msgs)
        self.assertIn("line 3", msgs)
        self.assertIn("no pin 'q'", msgs)

    def test_rails(self):
        ckt = parse("R1 res a 0\nR2 res a VCC\nR3 res a -12V\nrail VREF\nR4 res a VREF\n")
        self.assertEqual(ckt.rail_kind("0"), "gnd")
        self.assertEqual(ckt.rail_kind("VCC"), "pos")
        self.assertEqual(ckt.rail_kind("-12V"), "neg")
        self.assertEqual(ckt.rail_kind("VREF"), "pos")
        self.assertIsNone(ckt.rail_kind("a"))

    def test_json_input(self):
        data = {"title": "t", "inputs": ["i"], "components": [
            {"id": "R1", "type": "res", "pins": ["i", "o"], "value": "1k"},
            {"id": "C1", "type": "cap", "pins": {"a": "o", "b": "0"}}]}
        drawing, report = build(parse(json.dumps(data)))
        self.assertTrue(report.ok)

    def test_backslashes_survive(self):
        ckt = parse(r"R1 res a b 10\Omega")
        self.assertEqual(ckt.components[0].value, r"10\Omega")

    def test_dangling_net_warning(self):
        ckt = parse("R1 res a b\n")
        self.assertTrue(any("dangling" in w for w in ckt.warnings))


class MarkupTest(unittest.TestCase):
    def test_plain(self):
        self.assertEqual(to_plain(r"R_{out}"), "Rout")
        self.assertEqual(to_plain(r"10\,k\Omega"), "10 kΩ")
        self.assertEqual(to_plain(r"$\overline{Q}$"), "Q")

    def test_tex(self):
        self.assertIn(r"\textsubscript{out}", to_tex("R_{out}"))
        self.assertIn(r"$\Omega$", to_tex("10Ω"))
        self.assertIn(r"\overline", to_tex("~Q"))


class LayoutRulesTest(unittest.TestCase):
    def test_shunt_hangs_vertically_and_series_lies_flat(self):
        from diagen.layout import Layout
        lay = Layout(parse("V1 vsource in 0\nR1 res in out\nC1 cap out 0\n")).run()
        c1, r1 = lay.insts["C1"], lay.insts["R1"]
        self.assertEqual(c1.pin_pos("a")[0], c1.pin_pos("b")[0])   # vertical
        self.assertEqual(r1.pin_pos("a")[1], r1.pin_pos("b")[1])   # horizontal

    def test_signal_flows_left_to_right(self):
        from diagen.layout import Layout
        lay = Layout(parse("input a\noutput y\nN1 not a b\nN2 not b c\nN3 not c y\n")).run()
        xs = [lay.node_of[c].x for c in ("input:a", "N1", "N2", "N3", "output:y")]
        self.assertEqual(xs, sorted(xs))

    def test_straight_chain_has_no_bends(self):
        _, report = render("input a\noutput y\nR1 res a b\nR2 res b c\nR3 res c y\n")
        self.assertEqual(report.bends, 0)
        self.assertEqual(report.crossings, 0)

    def test_divider_and_emitter_stack_share_columns(self):
        from diagen.layout import Layout
        lay = Layout(parse(read(os.path.join(ROOT, "examples", "ce_amp.cir")))).run()
        self.assertIs(lay.node_of["R1"], lay.node_of["R2"])      # divider
        self.assertIs(lay.node_of["RC"], lay.node_of["Q1"])      # collector load
        self.assertIs(lay.node_of["CE"], lay.node_of["Q1"])      # bypass beside RE

    def test_bridge_is_two_vertical_branches_with_a_rung(self):
        from diagen.layout import Layout
        lay = Layout(parse(read(os.path.join(ROOT, "examples", "wheatstone.cir")))).run()
        self.assertIs(lay.node_of["R1"], lay.node_of["R2"])
        self.assertIs(lay.node_of["R3"], lay.node_of["R4"])
        cols = [lay.node_of[c].layer for c in ("V1", "R1", "M1", "R3")]
        self.assertEqual(cols, sorted(set(cols)))
        _, report = render(read(os.path.join(ROOT, "examples", "wheatstone.cir")))
        self.assertEqual(report.crossings, 0)

    def test_single_branch_stays_horizontal(self):
        from diagen.layout import Layout
        lay = Layout(parse("V1 vsource in 0\nR1 res in out\nC1 cap out 0\n")).run()
        r1 = lay.insts["R1"]
        self.assertEqual(r1.pin_pos("a")[1], r1.pin_pos("b")[1])

    def test_feedback_resistor_rides_with_opamp(self):
        from diagen.layout import Layout
        lay = Layout(parse(read(os.path.join(ROOT, "examples", "inverting_amp.cir")))).run()
        self.assertIs(lay.node_of["Rf"], lay.node_of["U1"])

    def test_noninverting_gain_resistor_hangs_under_feedback(self):
        from diagen.layout import Layout
        lay = Layout(parse(read(os.path.join(ROOT, "examples", "noninverting_amp.cir")))).run()
        self.assertIs(lay.node_of["R1"], lay.node_of["U1"])
        r1, r2 = lay.insts["R1"], lay.insts["R2"]
        top, bottom = sorted((r1.pin_pos("a"), r1.pin_pos("b")), key=lambda p: p[1])
        self.assertEqual(top[0], bottom[0])                        # vertical
        fb = "a" if r2.comp.pins["a"] == "fb" else "b"
        x, y = r2.pin_pos(fb)
        self.assertEqual(top[0], x - 1 if r2.pin_dir(fb) == "L" else x + 1)
        self.assertGreater(top[1], y)                              # right below the junction

    def test_near_ports_of_adjacent_stages_get_separate_columns(self):
        from diagen.layout import Layout
        text = read(os.path.join(ROOT, "examples", "ripple_adder.cir"))
        lay = Layout(parse(text), {"ports": "near"}).run()
        self.assertNotEqual(lay.node_of["output:s0"].layer, lay.node_of["input:a1"].layer)
        _, report = render(text)
        self.assertEqual(report.bends, 0)                          # carries run straight

    def test_gate_driving_only_an_output_joins_its_siblings(self):
        from diagen.layout import Layout
        lay = Layout(parse("input a b en\noutput y0 y1 y2 y3\nN1 not a an\nN2 not b bn\n"
                           "G0 and3 an bn en y0\nG1 and3 a bn en y1\n"
                           "G2 and3 an b en y2\nG3 and3 a b en y3\n")).run()
        self.assertEqual({lay.node_of[g].layer for g in ("G0", "G1", "G2", "G3")}, {2})


DIFF_PAIR = ("input vin1 vin2\noutput vo1 vo2\nRC1 res vcc vo1 10k\nRC2 res vcc vo2 10k\n"
             "Q1 npn c=vo1 b=vin1 e=tail\nQ2 npn c=vo2 b=vin2 e=tail\nRE res tail vee 20k\n")


class SymmetryTest(unittest.TestCase):
    def layout(self, text, **opts):
        from diagen.layout import Layout
        return Layout(parse(text), opts).run()

    @staticmethod
    def hflip(inst):
        return inst.t.mirror and inst.t.rot == 2

    def test_differential_pair_is_a_mirror_image(self):
        lay = self.layout(DIFF_PAIR)
        q1, q2, re = (lay.insts[c] for c in ("Q1", "Q2", "RE"))
        self.assertIs(lay.node_of["Q1"], lay.node_of["Q2"])
        self.assertEqual(q1.pin_pos("e")[1], q2.pin_pos("e")[1])        # side by side
        self.assertFalse(self.hflip(q1))
        self.assertTrue(self.hflip(q2))                                  # bases face outwards
        self.assertEqual(re.pin_pos("a")[0], (q1.pin_pos("e")[0] + q2.pin_pos("e")[0]) / 2)
        self.assertGreater(lay.node_of["input:vin2"].layer, lay.node_of["Q2"].layer)   # from the right
        _, report = render(DIFF_PAIR)
        self.assertTrue(report.ok)
        self.assertEqual(report.crossings, 0)

    def test_nand_pull_ups_are_copies_over_the_pull_down_stack(self):
        lay = self.layout("input a b\noutput y\nP1 pmos d=y g=a s=vdd\nP2 pmos d=y g=b s=vdd\n"
                          "N1 nmos d=y g=a s=m\nN2 nmos d=m g=b s=0\n")
        p1, p2, n1, n2 = (lay.insts[c] for c in ("P1", "P2", "N1", "N2"))
        self.assertEqual(p1.pin_pos("d")[1], p2.pin_pos("d")[1])
        self.assertFalse(self.hflip(p1) or self.hflip(p2))              # same way round
        self.assertLess(p1.pin_pos("d")[0], n1.pin_pos("d")[0])
        self.assertLess(n1.pin_pos("d")[0], p2.pin_pos("d")[0])         # on the axis
        self.assertEqual(n1.pin_pos("s")[0], n2.pin_pos("d")[0])        # stacked in line
        self.assertLess(n1.pin_pos("s")[1], n2.pin_pos("d")[1])

    def test_each_level_of_a_mirror_faces_its_own_way(self):
        # current-mirror load: bases shared, facing in; input pair: facing out
        lay = self.layout("input vin1 vin2\noutput vout\n"
                          "Q3 pnp c=x b=x e=vcc\nQ4 pnp c=vout b=x e=vcc\n"
                          "Q1 npn c=x b=vin1 e=tail\nQ2 npn c=vout b=vin2 e=tail\n"
                          "Q5 npn c=tail b=vb e=0\nRB res vcc vb 20k\nQ6 npn c=vb b=vb e=0\n")
        flips = {c: self.hflip(lay.insts[c]) for c in ("Q1", "Q2", "Q3", "Q4", "Q5")}
        self.assertEqual(flips, {"Q1": False, "Q2": True, "Q3": True, "Q4": False, "Q5": False})
        self.assertIs(lay.node_of["Q5"], lay.node_of["Q1"])             # the tail on the axis

    def test_bridge_load_goes_across_the_middle(self):
        lay = self.layout("input a b\nQ1 pnp c=m1 b=a e=vcc\nQ2 pnp c=m2 b=b e=vcc\n"
                          "Q3 npn c=m1 b=b e=0\nQ4 npn c=m2 b=a e=0\nM1 lamp m1 m2\n")
        m, q1, q2 = lay.insts["M1"], lay.insts["Q1"], lay.insts["Q2"]
        self.assertIs(lay.node_of["M1"], lay.node_of["Q1"])
        self.assertEqual(m.pin_pos("a")[1], m.pin_pos("b")[1])          # lying flat
        self.assertLess(q1.pin_pos("c")[0], min(m.pin_pos("a")[0], m.pin_pos("b")[0]))
        self.assertGreater(q2.pin_pos("c")[0], max(m.pin_pos("a")[0], m.pin_pos("b")[0]))

    def test_inverter_input_is_level_with_its_output(self):
        drawing, report = render(read(os.path.join(ROOT, "examples", "cmos_inverter.cir")))
        self.assertTrue(report.ok)
        lay = self.layout(read(os.path.join(ROOT, "examples", "cmos_inverter.cir")))

        def y(c, p):
            return lay.insts[c].pin_pos(p)[1] + lay.node_of[c].y
        axis = (y("M1", "g") + y("M2", "g")) / 2                        # mirrored top to bottom
        self.assertEqual(y("input:in", "a"), axis)
        self.assertEqual(y("output:out", "a"), axis)

    def test_sram_access_transistors_lie_on_either_side(self):
        text = read(os.path.join(ROOT, "examples", "sram_6t.cir"))
        lay = self.layout(text)
        a1, a2, p1, p2 = (lay.insts[c] for c in ("A1", "A2", "P1", "P2"))
        self.assertIs(lay.node_of["A1"], lay.node_of["P1"])
        self.assertLess(a1.pin_pos("s")[0], p1.pin_pos("d")[0])        # left of the left half
        self.assertGreater(a2.pin_pos("s")[0], p2.pin_pos("d")[0])     # right of the right half
        self.assertEqual((a1.pin_dir("s"), a2.pin_dir("s")), ("R", "L"))   # pointing inwards
        self.assertEqual((a1.pin_dir("g"), a2.pin_dir("g")), ("U", "U"))   # gates up to wl
        self.assertEqual(a1.pin_pos("s")[1], a2.pin_pos("s")[1])        # level with q and qb
        _, report = render(text)
        self.assertTrue(report.ok)

    def test_symmetry_off(self):
        lay = self.layout(DIFF_PAIR, symmetry="off")
        self.assertIsNot(lay.node_of["Q1"], lay.node_of["Q2"])
        self.assertFalse(self.hflip(lay.insts["Q2"]))


class StagesTest(unittest.TestCase):
    def test_repeated_stages_are_found_and_laid_out_alike(self):
        from diagen.layout import Layout
        lay = Layout(parse(read(os.path.join(ROOT, "examples", "adder3.cir")))).run()
        self.assertEqual([[len(st) for st in ch] for ch in lay.stages], [[5, 5, 5]])
        spans = []
        for si in range(3):
            nodes = lay.stage_nodes[(0, si)]
            spans.append((min(n.layer for n in nodes), max(n.layer for n in nodes)))
            # the same parts in the same column order, stage by stage
            order = [(n.layer - spans[-1][0], lay.cols[n.layer].index(n), lay.node_key[n.id])
                     for n in nodes]
            if si:
                self.assertEqual(sorted((c, k) for c, _, k in order),
                                 sorted((c, k) for c, _, k in first))
                self.assertEqual([k for *_, k in sorted(order)], [k for *_, k in sorted(first)])
            else:
                first = order
        self.assertLess(spans[0][1], spans[1][0])                  # stages side by side
        self.assertLess(spans[1][1], spans[2][0])

    def test_stage_tags(self):
        from diagen.layout import Layout
        lay = Layout(parse("input vin\noutput vout\n"
                           "C1 cap vin b1 stage=1\nR1 res b1 0 stage=1\nQ1 npn c=c1 b=b1 e=0 stage=1\n"
                           "RC1 res vcc c1 stage=1\nC2 cap c1 b2 stage=2\nR2 res b2 0 stage=2\n"
                           "Q2 npn c=vout b=b2 e=0 stage=2\nRC2 res vcc vout stage=2\n")).run()
        self.assertEqual(lay.stages, [[["C1", "R1", "Q1"], ["C2", "R2", "Q2"]]])

    def test_stages_off(self):
        from diagen.layout import Layout
        lay = Layout(parse(read(os.path.join(ROOT, "examples", "adder3.cir"))), {"stages": "off"}).run()
        self.assertEqual(lay.stages, [])


class BusTest(unittest.TestCase):
    def test_select_lines_become_trunks_fed_from_above(self):
        from diagen.engine import _start
        ckt = parse(read(os.path.join(ROOT, "examples", "mux4_bus.cir")))
        _, report, lay = _start(ckt, {"ports": "edge"})
        self.assertTrue(report.ok, report.text())
        self.assertEqual(sorted(n for nets in lay.bus.values() for n in nets),
                         ["s0", "s0n", "s1", "s1n"])
        src = lay._bus_sources()
        low = max(n.y + n.box[3] for n in lay.nodes if n.id in src)
        gates = [lay.node_of[g] for g in ("G0", "G1", "G2", "G3")]
        self.assertLess(low, min(n.y + n.box[1] for n in gates))
        # a dot where each trunk's upper tap joins, and where s0, s1 branch to
        # their inverters; the lower tap and the source entry are corners
        self.assertEqual(report.junctions, 4 + 2)

    def test_direct_routing_has_no_trunks(self):
        from diagen.layout import Layout
        text = read(os.path.join(ROOT, "examples", "mux4_bus.cir")).replace("option routing=bus", "")
        self.assertEqual(Layout(parse(text)).run().bus, {})


def adder_chain(n):
    lines = ["input " + " ".join(f"a{i} b{i}" for i in range(n)) + " cin",
             "output " + " ".join(f"s{i}" for i in range(n)) + " cout"]
    for i in range(n):
        ci, co = ("cin" if i == 0 else f"c{i}"), ("cout" if i == n - 1 else f"c{i + 1}")
        lines += [f"X{i} xor2 a{i} b{i} p{i}", f"Y{i} xor2 p{i} {ci} s{i}",
                  f"A{i} and2 a{i} b{i} g{i}", f"B{i} and2 p{i} {ci} t{i}",
                  f"O{i} or2 g{i} t{i} {co}"]
    return "\n".join(lines)


class SearchTest(unittest.TestCase):
    def test_crossing_reduction_untangles_a_long_chain(self):
        from diagen.engine import _start
        ckt = parse(adder_chain(4))
        plain = _start(ckt, {"ports": "edge"})[1]
        reduced = _start(ckt, {"ports": "edge", "xmin": True})[1]
        self.assertLess(reduced.crossings, plain.crossings)

    def test_opamp_feedback_side_is_chosen_with_its_neighbours(self):
        # Both input op-amps have '-' on top, so the local rule puts both
        # feedback resistors above; mirroring the upper one (feedback below)
        # is what untangles the gain resistor between them.
        text = ("input v1 v2\noutput vout\n"
                "U1 opamp +=v1 -=a out=o1\nU2 opamp +=v2 -=b out=o2\n"
                "R1 res o1 a 10k\nRG res a b 1k\nR2 res b o2 10k\n"
                "R3 res o1 c 10k\nR4 res o2 d 10k\nR5 res c vout 10k\nR6 res d 0 10k\n"
                "U3 opamp +=d -=c out=vout\n")
        drawing, report = render(text)
        self.assertTrue(report.ok)
        self.assertEqual(report.crossings, 0)
        # a part the netlist flips itself is left alone
        from diagen.engine import _Search, _start
        ckt = parse(text.replace("out=o1", "out=o1 flip=1"))
        moves = _Search(ckt, _start(ckt, {"ports": "edge"}), [0, 0]).flip_moves
        self.assertEqual(sorted(m[1][0] for m in moves), ["U2", "U3"])

    def test_result_does_not_depend_on_machine_speed(self):
        import time
        import diagen.engine as engine
        text = adder_chain(3)
        fast = to_svg(render(text)[0])
        orig = engine._build_once

        def slow(*a, **k):
            time.sleep(0.003)
            return orig(*a, **k)
        engine._build_once = slow
        try:
            self.assertEqual(to_svg(render(text)[0]), fast)
        finally:
            engine._build_once = orig


class FuzzTest(unittest.TestCase):
    """Random netlists must never crash, and nets must route or be reported."""
    TYPES = ["res", "cap", "ind", "diode", "vsource", "npn", "nmos", "opamp",
             "and2", "or3", "not", "xor2", "dff", "led"]

    def test_random_circuits(self):
        rng = random.Random(1234)
        for trial in range(60):
            nets = [f"n{i}" for i in range(rng.randint(2, 8))] + ["0", "vcc"]
            lines = []
            for i in range(rng.randint(1, 10)):
                t = rng.choice(self.TYPES)
                from diagen.symbols import lookup
                spec = lookup(t)
                k = len(spec.order) if t != "opamp" else 3
                pins = [rng.choice(nets) for _ in range(k)]
                lines.append(f"X{i} {t} {' '.join(pins)}")
            if rng.random() < 0.5:
                lines.append(f"input {nets[0]}")
            if rng.random() < 0.5:
                lines.append(f"output {nets[1]}")
            text = "\n".join(lines)
            with self.subTest(trial=trial, netlist=text):
                drawing, report = render(text)
                to_svg(drawing)
                to_tikz(drawing)


class TypesTest(unittest.TestCase):
    def test_every_known_type_builds(self):
        from diagen.symbols import lookup
        for t in KNOWN_TYPES:
            spec = lookup(t)
            self.assertIsNotNone(spec, t)
            if spec.order:
                k = len(spec.order) if t not in ("opamp",) else 3
                text = f"X1 {t} " + " ".join(f"n{i}" for i in range(k))
            else:
                text = f"X1 {t} A=n1 Y=n2"
            with self.subTest(type=t):
                render(text)


if __name__ == "__main__":
    unittest.main()
