# Catena

Twelve identical table robots that link into a chain, bend it into a shape you draw on a
phone, and use the shape as a tool: a ring that cages and carries a ball, a scoop that
sweeps blocks, a straight chain that bridges a gap. No central controller. Every robot
runs the same firmware and talks only to robots it can see.

**Status: under development, simulated only. In a 2D physics simulation that runs the real
control code the robots form a chain, take drawn shapes, carry a ball in a ring, heal after
a robot is removed and sweep blocks into a corner. Nothing has run on hardware; no part
has been printed.**

## Look at it

Open `sim3d/concept.html` in a browser (it needs an internet connection to load one
library). It is a concept viewer with scripted motion, there to show how the robots link
and move. The physics simulation (`make sim`) runs the real control code.

## Build and test

Needs Python 3.11 or newer, CMake 3.20 or newer and a C++17 compiler.

```
make setup        # creates .venv, installs the Python packages, builds core
make test         # C++ unit tests, then the Python tests
make sim S=sweep  # watch a scenario: wander, chain, shape:ring, twist, lasso, heal, sweep
                  # (space pauses, + and - change speed)
make help         # every target
```

## Read more

- `docs/SPEC.md`: what is being built
- `docs/DECISIONS.md`: why it looks the way it does
- `docs/HARDWARE.md`: parts, pins, printing, bring-up
- `docs/PRIOR_ART.md`: related work

## Licences

- Code (`core/`, `sim/`, `firmware/`, `ui/`, `tests/`, build files): MIT, see `LICENSE`.
- Hardware files (`cad/`): CERN-OHL-P v2, see `LICENSES/CERN-OHL-P-2.0.txt`.
- Documentation (`docs/`, this README, media): CC BY 4.0, see `LICENSES/CC-BY-4.0.txt`.
- `tests/cpp/third_party/doctest.h` is third-party code under MIT; its licence sits next to it.
