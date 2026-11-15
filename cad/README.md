# cad

CadQuery models of the printed parts and coupons (`docs/SPEC.md` section 11). Nothing has
been printed yet. Hardware files are licensed under CERN-OHL-P v2.

```
make cad       build all parts, run the checks, export STEP, STL and 3MF to cad/out/
make render    draw docs/media/cad_robot.png and cad_linked.png
```

- `parts.py`: the parts of one robot in the robot frame, and the list of printed parts
  with print orientation, material and listed supports.
- `coupons.py`: the coupons and the coupon plate.
- `checks.py`: bed size, thin walls, overhangs, shared volume; each tested on shapes whose
  answer is known (`tests/test_m5_checks.py`).
- `verify.py`: the checks `make cad` runs on the robot (`tests/test_m5_cad.py`).
- `build.py`, `render.py`: the two commands above.

Every dimension comes from `config/robot.yaml`; the CAD-only ones are under `cad:`.
Exports are in print orientation, z up from the bed. `cad/out/` is not in git.
