# Catena

A swarm of twelve identical table robots that link into one chain, bend it into a stroke
drawn on a phone, and use the shape as a tool (ring cages a ball, scoop sweeps blocks,
straight chain bridges a gap). No central controller: every robot runs the same firmware
and talks only to robots it can see.

Owner: Luka, an Information Engineering student (C++, Python, Java). He reviews in short
evening slots, so work must be resumable and self-explaining.

## Read first, every session

1. `STATUS.md`: where the last session stopped and what is next.
2. `docs/MILESTONES.md`: the milestone you are on and its acceptance tests.
3. `docs/SPEC.md`: only the sections the milestone names.

`docs/DECISIONS.md` holds settled design decisions. Do not reopen them; add a new entry
when you make a decision of your own.

## Rules that do not bend

- **Neighbour-only.** Control code may use only what one robot can sense or what a
  confirmed neighbour sent it. No global positions, no shared state, no leader that is
  special by construction. The simulator may know the truth; `core/` may not.
- **One codebase for simulation and robot.** All behaviour lives in `core/` as portable
  C++17 with no Arduino, ESP-IDF or OS headers. The simulator and the firmware both call
  it through the `Hal` interface. If a behaviour exists only in Python, it is not done.
- **One source of numbers.** Dimensions, masses, ranges and limits come from
  `config/robot.yaml`. Generated headers and Python loaders read it. No magic numbers.
- **Deterministic tests.** Every simulation run takes a seed. A failing seed is a bug
  report: write it into the test.
- **Honest claims.** Nothing has run on hardware yet. Docs and README say "simulated"
  wherever that is the evidence. Never report a test as passing without running it.

## Layout

```
config/robot.yaml     parameters (single source of truth)
core/                 portable C++17: link layer, chain logic, shape maths, behaviours
core/include/catena/  public headers, incl. hal.h and the C API (catena_c.h)
sim/                  Python 3.11+: pymunk world, sensor models, runner, viewer, metrics
sim/scenarios/        one file per demo; each returns pass/fail plus metrics
ui/                   phone drawing page (static HTML/JS), talks to sim or robot
firmware/             PlatformIO project for ESP32-S3: Hal implementation, bring-up tests
cad/                  CadQuery models, checks, exports to cad/out/
sim3d/                concept viewer (concept.html), later the MuJoCo bridge model
docs/                 SPEC, MILESTONES, DECISIONS, HARDWARE, PRIOR_ART
tests/                pytest scenario tests and C++ unit tests
```

## Commands

Create these in M0 and keep them working. `make help` lists them.

```
make setup        create .venv, install Python deps, build core
make core         build core as a shared library plus unit tests (CMake)
make test         C++ unit tests, then fast pytest scenarios (under 3 minutes)
make test-full    all scenarios across the full seed sets
make sim S=lasso  run a scenario with the viewer
make cad          build all parts, run checks, export STEP/STL/3MF
make fw           build the firmware with PlatformIO (no upload)
make clips        render README clips from scenarios
```

## How to work

- One milestone at a time, in order. Finish its acceptance tests before starting the next.
- Start each milestone by writing its tests from `docs/MILESTONES.md`, then make them pass.
- Commit small and often with messages that say what changed and why. Never commit
  failing `make test`.
- End every session by updating `STATUS.md`: done, verified how, open problems, next step.
- When a spec detail is missing, choose the simplest option that keeps the rules above,
  record it in `docs/DECISIONS.md` in three lines, and continue. Stop and ask only if the
  choice would change the hardware or break an acceptance test.
- When an acceptance threshold cannot be met after real effort, do not lower it silently.
  Record the measured value and the suspected cause in `STATUS.md` and move on.

## Budget

Sessions run on a limited allowance. Spend it on behaviour and tests, not on polish.

- Do not refactor working code, add abstractions for later, or write long comments.
- Do not add dependencies beyond the ones in `docs/SPEC.md` section 9 without a decision entry.
- Keep generated files (`cad/out/`, clips, build output) out of git except where
  `docs/MILESTONES.md` says to commit them.
- Prefer running a focused test over the full suite while iterating.

## Conventions

- C++: C++17, no exceptions or RTTI in `core/`, no heap allocation after start-up, fixed-size
  containers, `snake_case` functions, `PascalCase` types, units in names (`range_mm`,
  `heading_rad`). Angles in radians inside code, degrees only in config and logs.
- Python: 3.11+, type hints, `ruff` clean, no notebooks.
- Frames: robot base frame is x forward, y left, angles counter-clockwise. The ring angle
  is the direction of the tongue in the base frame.
- Tests name the behaviour, not the function: `test_ring_recloses_after_removal`.

## Safety for the later hardware phase

Lithium cells, magnets and a table edge are involved. Firmware must stop the motors on low
battery, on loss of both edge sensors outside bridge mode, and on a watchdog timeout.
Do not write instructions that have the user charge cells unattended.
