# Status

Updated at the end of every session. Newest entry on top.

## 11 Nov 2026: M1 simulator foundation done (simulated); CI green

Done
- core: C API (`catena_create/step/destroy`, input and output structs), `Hal` interface,
  flash ranging estimator, FLASH announcements about 10 times a second, neighbour table,
  `WANDER` (cruise, steer away from neighbours ahead, back off and turn at a table edge).
- sim: pymunk world (traction-limited wheels, ring on a pivot with the detent, links as
  groove + magnet + yaw limit), sensor and radio models, swarm runner, `wander` scenario,
  JSON runner (`python -m sim.runner wander --seed 3`), pygame viewer (`make sim S=wander`).
- Config gained radio timing, ring and edge sensor details, `behaviour.wander` and `sim`.
  Decisions D18 to D22.

Verified how (all simulated; nothing has run on hardware)
- Wander, 12 robots, 120 s: no fall in 10 of 10 seeds (`make test`) and 50 of 50
  (`make test-full`), plus regression seed 15. Closest any centre came to an edge over the
  50 seeds: 10 mm. Every robot drove at least 5.3 m per run, so nobody passes by standing still.
- Seed 15 first failed: a robot parallel to an edge reversed along it until it fell. Fixed in
  core (D22); C++ test `a robot finishes its turn in place even while a sensor still sees no table`.
- Link: a robot pushed at 20 mm/s links from 0, +/-45 and +/-90 degrees off the rear and
  not from 120, 180 or -135; the joint holds 3 s at 0.5 and 0.9 times the hold force and
  breaks at 1.1 and 1.5; a robot reversing away unlinks (D1).
- Same seed, same result: a repeat of seed 0 in a separate process gives identical final
  positions and metrics; a repeat in the same process too.
- Flash ranging over 1000 samples through radio model and core estimator: bearing error sd
  10.03 deg (configured 10), mean -0.13 deg; range error sd 9.7% (configured 10%), mean
  0.3%. Frames dropped 5.0% (configured 5%). By design (D18) this proves the pipeline
  reproduces the configured noise, not that real IR behaves this way.
- `make sim S=wander` opened a real window on this Mac (30 s at 8x; screenshot checked);
  `tests/test_m1_viewer.py` runs the same make target headless.
- Fresh `git clone`, `make setup && make test`: 14 C++ test cases, 29 pytest tests pass,
  `make lint` clean. `make test` takes about 9 s here, `make test-full` about 29 s.

Open problems
- CI is green (closes M0 "CI is green"): run 43032980045 on 11 Nov, ubuntu-latest and
  macos-latest, each 1 C++ test binary, 29 pytest tests and ruff passed. The local history
  was replayed onto GitHub's design-doc commits first, so the push was a fast-forward.
- About half of all IR samples are spoiled by flash collisions (0.46 to 0.51 with 12 robots).
  Fine for wandering; watch it in M2 docking.
- Wandering robots dock by accident (161 link events over 50 runs). M1 core ignores links;
  M2 must handle links it did not ask for.
- Not modelled: IR occlusion by other robots, the solid front sector of the ring (tongues
  collide only with hubs, as spec section 8 says), clip recording in the viewer (needed by
  `make clips` in M4).
- Edge sensor spot positions (`sensing.edge_sensor_xy`) and all wander tuning are assumed.

Next
- M2 chain forming.

## 10 Nov 2026: M0 scaffold done, CI not yet run

Done
- Layout from `CLAUDE.md`, Makefile targets (`make help`), CMake build of `core/` (shared
  library for ctypes, static library for unit tests), Python package `sim`, config loader,
  generated header `build/generated/catena/params_gen.h`, licences, GitHub Actions workflow.
- Decisions D15 to D17 (parameter table, tooling and pins, licence files).

Verified how
- `make setup && make test` on a fresh `git clone` of commit 03de145 (7354acc after the
  history was joined to GitHub's on 11 Nov; same files) into a temporary
  directory on this Mac (Apple clang 21, CMake 4.0, Python 3.12): 1 of 1 C++ test binary
  and 6 of 6 pytest tests passed; `make lint` clean.
- Header and loader agree: `tests/test_config.py` compares the pitch, and then every
  field, between the compiled library and the Python loader.
- CI is NOT verified. Nothing had been pushed, so the workflow in
  `.github/workflows/ci.yml` has never run. Nothing was built on Linux. (This entry first
  said there was no GitHub remote; the repository existed, the local folder was not linked.)

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
