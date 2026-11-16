#!/usr/bin/env python3
"""
Конвертер конектома мухи (FlyWire FAFB v783) у connectome.h для Arduino IDE.

Джерело: data/circuit.json з репозиторію DenisSergeevitch/desktop-fly
(668 нейронів ланцюга втечі/кермування, ~19 000 знакових синапсів).

Що робить:
  1. Завантажує circuit.json (або бере локальний файл через --input).
  2. Будує CSR-граф (rowStart / col / w) — зберігаються лише реальні зв'язки.
  3. Переводить ваги у фіксовану кому Q14 (1.0 == 16384), бо RP2040 не має FPU.
     Ваги масштабуються так само, як у оригінальному sim.js:
       w = synapses * 0.0008, а LC4/LPLC2/сенсорні -> GF  ×6 (електричні синапси).
  4. Генерує базовий струм (baseline) для кожного нейрона з фіксованим seed.

Лише стандартна бібліотека Python — pandas не потрібен.

Використання:
  python3 convert_circuit.py                     # завантажити з GitHub
  python3 convert_circuit.py --input circuit.json
"""
import argparse
import json
import os
import random
import urllib.request

URL = ("https://raw.githubusercontent.com/DenisSergeevitch/desktop-fly/"
       "master/data/circuit.json")

Q = 14
ONE = 1 << Q

WEIGHT_SCALE = 0.0008      # sim.js weightScale
GAP_JUNCTION_BOOST = 6.0   # sim.js gapJunctionBoost

# Коди ролей — мають збігатися з fly_brain.h
ROLE_OTHER, ROLE_LOOM, ROLE_DNA, ROLE_MDN, ROLE_FWD, ROLE_GROOM, \
    ROLE_ESCW, ROLE_GF, ROLE_SENSORY, ROLE_ASCEND = range(10)

ROLE_MAP = {
    "lc4": ROLE_LOOM, "lplc2": ROLE_LOOM,
    "dna01": ROLE_DNA, "dna02": ROLE_DNA,
    "mdn": ROLE_MDN, "dnp09": ROLE_FWD, "dng11": ROLE_GROOM,
    "escw": ROLE_ESCW, "gf": ROLE_GF,
}


def role_code(n):
    if n["role"] == "other":
        if n["type"] == "sensory":
            return ROLE_SENSORY
        if n["type"] == "ascending":
            return ROLE_ASCEND
        return ROLE_OTHER
    return ROLE_MAP[n["role"]]


def baseline(n, rnd):
    # sim.js: гетерогенний базовий струм
    r = n["role"]
    if r == "other":
        return 0.010 + rnd.random() * 0.060
    if r in ("lc4", "lplc2"):
        return 0.004
    if r in ("dna01", "dna02", "mdn", "dng11", "escw"):
        return 0.036
    if r == "dnp09":
        return 0.038
    return 0.002  # gf: мовчить, доки його не збудять синапси


def q14(x):
    return int(round(x * ONE))


def c_array(ctype, name, values, per_line=16):
    lines = []
    for i in range(0, len(values), per_line):
        lines.append("  " + ", ".join(str(v) for v in values[i:i + per_line]))
    return f"{ctype} {name}[{len(values)}] = {{\n" + ",\n".join(lines) + "\n};\n\n"


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", help="локальний circuit.json (інакше — завантаження)")
    ap.add_argument("--out", default=os.path.join(here, "..", "FlyBrain", "connectome.h"))
    ap.add_argument("--seed", type=int, default=1234)
    args = ap.parse_args()

    if args.input:
        with open(args.input, encoding="utf-8") as f:
            circuit = json.load(f)
    else:
        print("Завантаження circuit.json з GitHub...")
        with urllib.request.urlopen(URL, timeout=60) as r:
            circuit = json.loads(r.read().decode("utf-8"))

    neurons = circuit["neurons"]
    edges = circuit["edges"]
    n, e = len(neurons), len(edges)
    if n > 65535 or e > 65535:
        raise SystemExit("Граф завеликий для uint16-індексів")

    roles = [role_code(x) for x in neurons]
    sides = [1 if x.get("side") == "right" else 0 for x in neurons]
    rnd = random.Random(args.seed)
    base = [q14(baseline(x, rnd)) for x in neurons]

    # CSR: стабільний порядок ребер усередині рядка, як у sim.js
    rows = [[] for _ in range(n)]
    for pre, post, syn in edges:
        pre, post = int(pre), int(post)
        w = syn * WEIGHT_SCALE
        electrical = roles[pre] in (ROLE_LOOM, ROLE_SENSORY)
        if electrical and roles[post] == ROLE_GF:
            w *= GAP_JUNCTION_BOOST
        wq = q14(w)
        if wq == 0:
            wq = 1 if w > 0 else -1
        if not -32768 <= wq <= 32767:
            raise SystemExit(f"Вага {w} не влазить у int16")
        rows[pre].append((post, wq))

    row_start, col, wts = [0], [], []
    for r in rows:
        for post, wq in r:
            col.append(post)
            wts.append(wq)
        row_start.append(len(col))

    counts = {}
    for r in roles:
        counts[r] = counts.get(r, 0) + 1
    names = ["other", "loom(LC4/LPLC2)", "DNa01/02", "MDN", "DNp09", "DNg11",
             "escW(DNp02/04/11)", "GF(DNp01)", "sensory", "ascending"]

    with open(args.out, "w", encoding="utf-8") as f:
        f.write("// Автоматично згенеровано tools/convert_circuit.py — не редагуйте вручну.\n")
        f.write(f"// Джерело: {circuit.get('source', 'FlyWire FAFB v783')}\n")
        f.write("// Дані FlyWire: CC BY-NC 4.0 (Dorkenwald et al. 2024, Schlegel et al. 2024)\n")
        f.write("// Похідні дані з github.com/DenisSergeevitch/desktop-fly\n")
        f.write("//\n// Склад:\n")
        for code in sorted(counts):
            f.write(f"//   {names[code]:<20} {counts[code]}\n")
        f.write("\n#pragma once\n#include <stdint.h>\n\n")
        f.write(f"#define FLY_N {n}\n#define FLY_E {e}\n#define FLY_Q {Q}\n\n")
        f.write("// Гарячі масиви (col, w) — у SRAM для швидкості (на Pico: FLY_RAM = __not_in_flash).\n")
        f.write("#ifndef FLY_RAM\n#define FLY_RAM\n#endif\n\n")
        f.write(c_array("static const uint16_t", "fly_row_start", row_start))
        f.write(c_array("static FLY_RAM uint16_t", "fly_col", col, 24))
        f.write(c_array("static FLY_RAM int16_t", "fly_w", wts, 20))
        f.write(c_array("static const uint8_t", "fly_role", roles, 32))
        f.write(c_array("static const uint8_t", "fly_side", sides, 32))
        f.write(c_array("static const int16_t", "fly_baseline", base, 24))

    print(f"Готово: {n} нейронів, {e} синапсів -> {os.path.normpath(args.out)}")
    print(f"RAM під граф: ~{(len(col) * 4) // 1024} КБ")


if __name__ == "__main__":
    main()
