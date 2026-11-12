"""Extract arm interfaces from KiCad PCB connectivity and MCU symbol pin names."""
import argparse
import csv
import re
from pathlib import Path

SIGNAL = re.compile(r"(?:PWM[1-6]|DIR[1-6]|ENC(?:[1-6]_[AB]|_[1-6])|LIM_SW[1-3]|FSR_ADC|FSR_SIG|LASER_SIG)$")
FIELDS = ["signal", "net", "connector", "connector_pin", "connector_path", "mcu_pin", "port", "passives"]


def parse(text):
    """Parse S-expressions, respecting quoted parentheses and escapes."""
    stack = [[]]
    for token in re.findall(r'"(?:\\.|[^"\\])*"|[()]|[^\s()]+', text):
        if token == "(":
            child = []
            stack[-1].append(child)
            stack.append(child)
        elif token == ")":
            stack.pop()
        else:
            stack[-1].append(re.sub(r'\\(["\\])', r'\1', token[1:-1]) if token.startswith('"') else token)
    return stack[0][0]


def children(node, tag):
    return [x for x in node if isinstance(x, list) and x and x[0] == tag]


def walk(node, tag):
    for child in node:
        if isinstance(child, list):
            if child and child[0] == tag:
                yield child
            yield from walk(child, tag)


def components(folder):
    pcb = parse((Path(folder) / "single_board.kicad_pcb").read_text(encoding="utf-8"))
    result = []
    for fp in children(pcb, "footprint"):
        props = {x[1]: x[2] for x in children(fp, "property")}
        for x in children(fp, "fp_text"):
            props.setdefault(x[1].capitalize(), x[2])
        pads = [(p[1], children(p, "net")[0][-1]) for p in children(fp, "pad") if children(p, "net")]
        result.append((props["Reference"], props.get("Value", ""), pads))
    return result


def extract(folder):
    sch = parse((Path(folder) / "single_board.kicad_sch").read_text(encoding="utf-8"))
    library = children(sch, "lib_symbols")[0]
    symbol = next(s for s in children(library, "symbol") if "STM32G474" in s[1])
    names = {children(p, "number")[0][1]: children(p, "name")[0][1] for p in walk(symbol, "pin") if children(p, "number")}
    parts = components(folder)
    mcu = next(p for p in parts if p[0] == "U5")
    nets = {net for _, _, pads in parts for _, net in pads if SIGNAL.fullmatch(net.split("/")[-1])}
    rows = []
    for net in sorted(nets):
        pins = [(num, names[num]) for num, n in mcu[2] if n == net]
        # A connector behind one series resistor still belongs to this interface.
        paths = {net: "direct"}
        passive = []
        for ref, value, pads in parts:
            if ref.startswith(("R", "C")) and any(n == net for _, n in pads):
                passive.append(f"{ref}={value} (" + "; ".join(f"{num}:{n}" for num, n in pads) + ")")
                if ref.startswith("R"):
                    for _, other in pads:
                        if other != net and other.startswith("Net-(J"):
                            paths[other] = f"via {ref}={value}"
        # Control signals can reach a switched connector through a transistor.
        # Label that relationship explicitly: these are NOT the same net.
        control_nets = {net: ""}
        for ref, value, pads in parts:
            if ref.startswith("R") and any(n == net for _, n in pads):
                control_nets.update({n: ref + "/" for _, n in pads if n.startswith("Net-(Q")})
        for ref, _, pads in parts:
            if ref.startswith("Q"):
                controls = [control_nets[n] for _, n in pads if n in control_nets]
                if controls:
                    for _, n in pads:
                        if n.startswith("Net-(J"):
                            paths[n] = f"via {controls[0]}{ref} (switched node, separate net)"
        connectors = [(ref, num, paths[n]) for ref, _, pads in parts if ref.startswith("J") for num, n in pads if n in paths]
        for ref, num, path in connectors or [("", "", "no connector on net or across one series resistor")]:
            rows.append(dict(zip(FIELDS, [net.split("/")[-1], net, ref, num, path, ";".join(p[0] for p in pins), ";".join(p[1] for p in pins), " | ".join(sorted(passive))])))
    return rows


def write_csv(rows, destination):
    with Path(destination).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, FIELDS)
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    write_csv(extract(args.folder), args.output)
