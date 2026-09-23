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
