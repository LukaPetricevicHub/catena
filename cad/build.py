"""`make cad`: build every part, run the checks of spec section 11, export STEP, STL and 3MF.

    python -m cad.build            checks and exports into cad/out/
    python -m cad.build --check    checks only
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from cadquery import exporters

from cad import verify

OUT = Path(__file__).resolve().parent / "out"
FORMATS = ("step", "stl", "3mf")


def export(m: verify.Model, out: Path = OUT) -> list[Path]:
    out.mkdir(parents=True, exist_ok=True)
    shapes = {p.name: p.print_shape for p in m.printable()}
    shapes["coupon_plate"] = m.coupon_plate()
    written = []
    for name, shape in shapes.items():
        for ext in FORMATS:
            path = out / f"{name}.{ext}"
            exporters.export(shape, str(path))
            written.append(path)
    return written


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="run the checks, export nothing")
    args = parser.parse_args(argv)

    m = verify.Model()
    failed = 0
    for check in verify.CHECKS:
        r = check(m)
        failed += not r.ok
        print(f"{'PASS' if r.ok else 'FAIL'}  {r.name}\n      {r.detail}")

    print("\nprinted parts (each in its print orientation, z up from the bed):")
    for p in m.printed:
        notes = "; ".join(f"{b.kind}: {b.why}" for b in p.listed) or "no support"
        print(f"  {p.count} x {p.name:17s} {p.material:5s} {p.orientation}. {notes}")
    print("\nmass if printed solid (an upper bound; nothing has been weighed):")
    budget = m.d.cfg["mass"]
    for group, printed, steel in verify.mass_report(m):
        allowed = budget[f"{group}_assembly"]
        print(f"  {group}: printed parts {printed:.0f} g, hub washers {steel:.0f} g; "
              f"the config allows {allowed:.0f} g for the whole assembly")  # fmt: skip
    wx, wy = m.d.c["edge_window_xy"]
    sx, sy = m.d.cfg["sensing"]["edge_sensor_xy"]
    if (wx, wy) != (sx, sy):
        print(f"\nNOTE  edge sensor windows are at ({wx}, +/-{wy}) mm, the simulator senses at "
              f"({sx}, +/-{sy}) mm,\n      where no sensor body fits (D49, open in STATUS.md)")

    if failed:
        print(f"\n{failed} check(s) failed; nothing exported")
        return 1
    if not args.check:
        written = export(m)
        print(f"\nwrote {len(written)} files to {OUT.relative_to(Path.cwd())}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
