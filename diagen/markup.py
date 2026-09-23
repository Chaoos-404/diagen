"""A tiny label markup shared by both back ends.

Labels are written the LaTeX way, because that is what models already know:

    R_1          subscript (one character), R_{out} for longer ones
    V^2          superscript
    \\overline{Q} bar over text (also ~Q as a shorthand)
    \\Omega \\mu  Greek letters; Unicode Ω µ is accepted too
    $...$        optional; dollar signs are stripped, not required

`parse` turns a label into runs of (text, style) where style is one of
"", "sub", "sup", "over". The SVG back end renders the runs as tspans and
the TikZ back end turns them back into math mode.
"""
from __future__ import annotations

SYMBOLS = {
    "Omega": "Ω", "omega": "ω", "mu": "µ", "alpha": "α", "beta": "β",
    "gamma": "γ", "delta": "δ", "Delta": "Δ", "theta": "θ", "phi": "φ",
    "pi": "π", "tau": "τ", "lambda": "λ", "sigma": "σ", "infty": "∞",
    "pm": "±", "cdot": "·", "times": "×", "deg": "°", "circ": "°",
}
TEX_OF = {v: k for k, v in SYMBOLS.items()}
TEX_OF["µ"] = "mu"
TEX_OF["μ"] = "mu"   # Greek mu, distinct code point from the micro sign


def _group(s, i):
    """Read a {...} group or a single character starting at s[i]."""
    if i < len(s) and s[i] == "{":
        depth, j = 1, i + 1
        while j < len(s) and depth:
            depth += {"{": 1, "}": -1}.get(s[j], 0)
            j += 1
        return s[i + 1:j - 1], j
    if i < len(s) and s[i] == "\\":
        j = i + 1
        while j < len(s) and s[j].isalpha():
            j += 1
        return s[i:j], j
    return s[i:i + 1], i + 1


def parse(s: str, style: str = "") -> list:
    s = s.replace("$", "")
    runs: list = []
    buf = ""
    i = 0

    def flush():
        nonlocal buf
        if buf:
            runs.append((buf, style))
            buf = ""

    while i < len(s):
        ch = s[i]
        if ch == "\\":
            j = i + 1
            while j < len(s) and s[j].isalpha():
                j += 1
            name = s[i + 1:j]
            if name == "overline":
                flush()
                inner, i = _group(s, j)
                runs += [(t, "over") for t, _ in parse(inner)]
                continue
            if name in SYMBOLS:
                buf += SYMBOLS[name]
            elif name in ("," , ";", "!", " "):
                buf += " "
            elif j == i + 1 and j < len(s):     # escaped punctuation: \, \_ \%
                buf += " " if s[j] in ",; " else s[j]
                j += 1
            else:
                buf += name
            i = j
        elif ch == "~" and i + 1 < len(s):
            flush()
            inner, i = _group(s, i + 1)
            runs += [(t, "over") for t, _ in parse(inner)]
        elif ch in "_^" and i + 1 < len(s):
            flush()
            inner, i = _group(s, i + 1)
            sub = "sub" if ch == "_" else "sup"
            runs += [(t, sub) for t, _ in parse(inner)]
        elif ch in "{}":
            i += 1
        else:
            buf += ch
            i += 1
    flush()
    return runs


def to_plain(s: str) -> str:
    return "".join(t for t, _ in parse(s))


def _tex_escape_text(t: str) -> str:
    out = ""
    for ch in t:
        if ch in TEX_OF:
            out += f"$\\{TEX_OF[ch]}$"
        elif ch in "&%#_{}":
            out += "\\" + ch
        elif ch == "~":
            out += "\\textasciitilde{}"
        else:
            out += ch
    return out.replace("$$", "")


def to_tex(s: str) -> str:
    out = ""
    for text, style in parse(s):
        body = _tex_escape_text(text)
        if style == "sub":
            out += f"\\textsubscript{{{body}}}"
        elif style == "sup":
            out += f"\\textsuperscript{{{body}}}"
        elif style == "over":
            out += f"$\\overline{{\\mbox{{{body}}}}}$"
        else:
            out += body
    return out
