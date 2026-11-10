# Status

Updated at the end of every session. Newest entry on top.

## 10 Nov 2026: M0 scaffold done, CI not yet run

Done
- Layout from `CLAUDE.md`, Makefile targets (`make help`), CMake build of `core/` (shared
  library for ctypes, static library for unit tests), Python package `sim`, config loader,
  generated header `build/generated/catena/params_gen.h`, licences, GitHub Actions workflow.
- Decisions D15 to D17 (parameter table, tooling and pins, licence files).

Verified how
- `make setup && make test` on a fresh `git clone` of commit 03de145 into a temporary
  directory on this Mac (Apple clang 21, CMake 4.0, Python 3.12): 1 of 1 C++ test binary
  and 6 of 6 pytest tests passed; `make lint` clean.
- Header and loader agree: `tests/test_config.py` compares the pitch, and then every
  field, between the compiled library and the Python loader.
- CI is NOT verified. The repository has no GitHub remote, so the workflow in
  `.github/workflows/ci.yml` has never run. Nothing was built on Linux.

Open problems
- M0 acceptance line "CI is green" is open until the repository is pushed to GitHub.
- This Mac had only Python 3.10; Python 3.12 was installed with `brew install python@3.12`.
- `make sim`, `make cad`, `make fw` and `make clips` exit with an error that names the
  milestone that brings them. They are placeholders, not working targets.

Next
- M1 Simulator foundation.

## 10 Nov 2026: project set up, no code yet

Done
- Design locked: see `docs/DECISIONS.md` D1 to D14.
- Spec, milestones, hardware draft, prior art and config written.
- `sim3d/concept.html`: hand-written 3D concept viewer. Scripted motion, not driven by `core/`.

Verified how
- Nothing under `core/`, `sim/`, `cad/` or `firmware/` exists yet.
- The concept viewer's logic was run headless on 10 Nov: chain forming completed in 19 of
  20 random starts within 400 simulated seconds (the 20th needed longer); lasso, ring heal
  and scoop heal each passed 10 of 10; the bridge crossed with 1.3 mm computed tip droop.
  This is ideal kinematics without physics or noise and proves the rules are sound, not
  that they survive real sensors.

Open problems
- All dimensions are from typical part sizes, not measured parts.
- IR range and bearing accuracy are assumptions until two boards exist.

Next
- M0 Scaffold, then M1 Simulator foundation.
