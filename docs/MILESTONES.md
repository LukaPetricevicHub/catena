# Milestones

Work in order. A milestone is done when every acceptance line holds and `STATUS.md` says
how it was verified. "Model" is a suggestion for which model to run the session on: Fable
for the hard parts, a cheaper model where the work is routine.

Cut lines: M0 to M5 are the core of v0.1. M6 to M8 complete it. M9 and M10 are stretch.

| # | Milestone | Model | Spec sections |
| --- | --- | --- | --- |
| M0 | Scaffold | cheaper | 6, 9 |
| M1 | Simulator foundation | Fable | 3, 5, 6, 8 |
| M2 | Chain forming | Fable | 7.1 to 7.3 |
| M3 | Shapes, moving, phone page | Fable | 7.4, 7.5, 10 |
| M4 | Lasso and heal | Fable | 7.6, 7.8 |
| M5 | CAD v0 and coupons | Fable | 3, 11 |
| M6 | Sweep | cheaper | 7.7 |
| M7 | Firmware builds | cheaper | 4, 12 |
| M8 | Docs and release v0.1.0 | cheaper | all |
| M9 | Bridge in 3D | Fable | 7.9 |
| M10 | Cube stacking in simulation | Fable | decision D2 |

## M0 Scaffold

Repository layout from `CLAUDE.md`, Makefile targets, CMake for `core/`, Python package for
`sim/`, config loader plus generated C header, licences, GitHub Actions running `make test`.

- `make setup && make test` passes on a clean clone (with placeholder tests).
- A unit test proves the generated header and the Python loader read the same pitch.
- CI is green.

## M1 Simulator foundation

Physics world, robot bodies, ring on its pivot, link joints, sensor and radio models,
C API, `Hal`, viewer. `core` contains only `WANDER` with edge avoidance.

- 12 robots wander for 120 simulated seconds and none leaves the table, in 10 of 10 seeds.
- Two robots pushed together by a test script link; the joint breaks above the hold force
  and not below it.
- Same seed, same result: two runs produce identical final positions.
- Flash ranging in simulation: bearing error and range error match the configured noise
  within 20% over 1,000 samples.
- `make sim S=wander` opens the viewer.

## M2 Chain forming

Rank, seek, dock, link handshake, index, count, `FOLLOW`.

- From random starts, one chain of 12 within 180 s in at least 90% of 50 seeds.
- Index runs 0..11 without gaps and every robot reports count 12 within 5 s of the last link.
- No run ends with two chains chasing each other for more than 60 s.
- Unit tests for rank ordering and index propagation, including dropped frames.

## M3 Shapes, moving, phone page

Stroke to turning angles, local shape rule, ring closure, twist in the shape frame, phone
page over websocket.

- Line, ring, U and S: mean vertex error under 12 mm after 60 s, 5 seeds each.
- Ring: the head docks onto its own tail without a script telling it to.
- Twist: a ring commanded 400 mm along shape x arrives within 40 mm and keeps shape error
  under 15 mm on the way.
- Drawing on the phone page changes the shape in the running simulator within 2 s.
- Report shape error with IR bearing noise at 5, 10 and 20 degrees in `STATUS.md`.

## M4 Lasso and heal

Object detection, boundary following, closing round an object, recovery after removal.

- Lasso: ball inside the closed ring and carried 400 mm in at least 80% of 50 seeds.
- Heal: a random linked robot is deleted at a random time during `SHAPE` or `MOVE`; the
  chain re-links, recounts and returns under 15 mm shape error within 60 s in 80% of 50 seeds.
- Heal also passes for a ring holding the ball.
- `make clips` produces `docs/media/lasso.gif` and `docs/media/heal.gif` (commit these).

## M5 CAD v0 and coupons

All parts and coupons from spec section 11, checks, exports.

- `make cad` passes every check and writes STEP, STL and 3MF.
- A rendered image of one robot and of two linked robots is committed to `docs/media/`.
- `docs/HARDWARE.md` lists every printed part with print orientation and material.
- Open questions that need a measured part are listed in `STATUS.md`, not guessed.

## M6 Sweep

- Eight blocks: the scoop ends parked against both edges of the corner with at least 80% of
  the blocks inside it, no falls, in 80% of 20 seeds.
- `docs/media/sweep.gif` committed.

## M7 Firmware builds

- `make fw` builds for the ESP32-S3 target in CI.
- `Hal` implementation complete; bring-up sketches build.
- `core` unit tests still pass unchanged: no behaviour was added in `firmware/`.
- Pin map in `docs/HARDWARE.md` matches the code.

## M8 Docs and release

- README: what it is, clips, results table from `make test-full`, how to run, honest status
  ("simulated; hardware bring-up next"), prior art, licences.
- `docs/HARDWARE.md`: priced parts list with links, wiring diagram, assembly guide v0,
  bring-up order.
- Tag `v0.1.0`.

## M9 Bridge in 3D (stretch)

MuJoCo model of the chain with joint play from the config, two tables, an 80 mm gap.

- All 12 robots cross; report tip droop against joint play of 0.5, 0.9 and 1.5 degrees.
- A 160 mm gap is attempted and the result reported either way.
- `docs/media/bridge.gif` committed.

## M10 Cube stacking in simulation (stretch)

Abstract lattice model: unit cubes that move by pivoting about a shared edge, as in
pivoting-cube models of modular robots. No physics, no hardware claim.

- 12 cubes go from a 1x1x12 line to a 2x2x3 block using only moves each cube can decide
  from its occupied neighbours.
- The structure stays connected after every move.
- A short clip and a paragraph in the README explain why this is simulation only.
