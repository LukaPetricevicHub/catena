# Status

Updated at the end of every session. Newest entry on top.

## 15 Nov 2026: M4 lasso and heal: acceptance lines met in simulation

Branch `m4-lasso-heal`, merged into `main` on 15 Nov 2026; CI green on Ubuntu and macOS
(`make test`, `make lint`; CI does not run the 50-seed sets).

Done
- core: recovery after a lost link (D42, D43); object sensing from the reflection of a
  robot's own pulse (`Hal::reflection`, `estimate_object`, D44); lasso (state `LASSO`, D45,
  D46, D48); a towing head's memory of the edge it met (D47). The debug output carries the
  object a robot believes it sees.
- sim: a ball (`World.add_ball`), robot removal (`World.remove`), the reflection model,
  scenarios `heal`, `heal:ball`, `lasso`; `sim/clips.py` and `make clips`.
- Clips committed: `docs/media/lasso.gif`, `docs/media/heal.gif` (seed 0 of each scenario).
- Try it: `make sim S=lasso SEED=0`, `make sim S=heal SEED=4`, `make sim S=heal:ball`.

Verified how (all simulated; nothing has run on hardware)
- `make test`: 39 C++ test cases, 68 pytest tests pass (88 s). `make test-full`: 68 pass
  (300 s; it was 113 s before the lasso runs). `make lint` clean.
- Line 1, lasso, ball inside the closed ring and carried 400 mm in 80% of 50 seeds:
  **45 of 50 (90%)**. The ball is ringed after a median of 116 s (longest 352 s; 360 s are
  allowed, the milestone names no limit). The ring's centre arrives 1 to 20 mm from where
  the twist should bring it (40 allowed); the ball itself travels 245 to 506 mm, since it
  rolls inside the ring. No robot and no ball left the table. In all 5 failures a robot
  knocked the ball away before a ring was round it. As in M3, the script turns the ring
  toward free table before the carry.
- Line 2, heal, 80% of 50 seeds: **44 of 50 (88%)**: one chain again, index and count
  right, under 15 mm of the shape for eleven within 60 s. Re-linked after a median of 4.9 s.
  Shapes by seed (line, ring, U, S); 30% of the removals happen during `MOVE`.
- Line 3, heal for a ring holding the ball: **50 of 50**, ring closed again, ball inside.
- Line 4: `make clips` writes both GIFs; I looked at frames of the lasso clip.
- Earlier milestones after these changes: M2 holds its 90% of 50 in `make test-full`
  (49 of 50 when last counted, median 79 s); shapes 30 of 30 for all four; twist 20 of 20.

Open problems
- **Results differ between machines.** The same seeds play out differently on Ubuntu and
  macOS (a chaotic system, different floating point). On CI ten lasso seeds once gave 7 on
  Ubuntu where macOS had 9. So `make test` only asks 60% of its short lasso and heal sets;
  the 80% of 50 is held by `make test-full`, which CI does not run (D48). All percentages
  in this file are from this laptop.
- **A robot dragged sideways or backward over an edge does not see it** (sensors at the
  front only). Every fall in M3 and M4 work was of this kind; each trigger found was fixed
  (lasso centre, tongue offset sign, a forced ring unfolding across the table), the cause
  was not. A third edge sensor at the rear would close it. That is a hardware change, so it
  is yours to decide.
- **The lasso search is a random walk** and needs up to 6 minutes. The head only sees an
  object in front of it (the other receivers face its chain), and the chain behind it
  sometimes knocks the ball away first.
- The lasso's circle is open loop: nothing checks that the ball is inside before the ring
  closes. A ring drawn beside the ball stays there.
- The 6 heal failures are removals during `MOVE` early in the run; not analysed further.
- Reflection sensing is a model of my own (D44): strengths, the 2.0 threshold and the
  mask are untested against real receivers. `sensing.object_range` is still "assumed".
- No C++ unit tests for recovery, the closing link's conversion or the lasso; scenario
  tests cover them. Object estimation has three.
- `make test-full` now takes five minutes.
- Still true from below: the phone page has not been opened in a browser; shapes are slow
  (20 mm/s); the gyro model (D37); parameters tuned in simulation only.

Next
- M5 CAD v0 and coupons (spec sections 3 and 11). M4 is closed.

## 14 Nov 2026: M3 shapes, moving and phone page: acceptance lines met in simulation

Branch `m3-shapes`, merged into `main` on 15 Nov 2026; CI green on Ubuntu and macOS (`make test`,
`make lint`).

Done
- core: `core/src/shape.{h,cpp}` (stroke to polygon and turning angles); in the robot the
  stroke gossip, the local rule of spec 7.4, ring closing and opening, twist with ramp and
  catch-up, edge push; states `SHAPE` and `MOVE`; C API `catena_set_shape`,
  `catena_set_twist`. Decisions D33 to D41.
- sim: `sim/shapes.py` (strokes, target polygon, fit), scenarios `shape:line|ring|u|s`,
  `twist`, `play`; `sim/bridge.py` (websocket and web server); `--serve` in the runner; the
  viewer draws the target shape. The simulated gyro now reports the mean rate per tick (D37).
- ui: `ui/index.html` (pad, Line/Ring/U/Release, joystick, rotate slider, status line).
- Try it: `make sim S=shape:ring`, `make sim S=twist`, `make sim S=play ARGS=--serve` and
  open `http://localhost:8000/`.

Verified how (all simulated; nothing has run on hardware)
- `make test`: 36 C++ test cases, 61 pytest tests pass (32 s). `make test-full`: 61 pass
  (113 s). `make lint` clean.
- Line 1, mean vertex error under 12 mm after 60 s, 5 seeds each: line 1.3 to 3.9 mm,
  ring 1.1 to 2.1, U 2.4 to 4.7, S 2.4 to 4.9. Under 12 mm from about 13 s (line), 14 s (S),
  31 s (U), 40 s (ring). Over 30 seeds each: 30 of 30 for all four, no fall, no broken link.
- Line 2, the ring closes itself: in all 5 seeds, 35 to 42 s after the stroke; the scenario
  only hands over the stroke. Open shapes stay open.
- Line 3, twist: a ring told to go 400 mm along shape x arrives 0.5 to 17.5 mm off (40
  allowed), worst shape error on the way 3.6 to 10.0 mm (15 allowed), 5 seeds; 20 of 20
  over 20 seeds. The script first turns the ring with the rotate command until shape x lies
  along the table, as a person at the phone would: 400 mm does not fit across it.
- Line 4, phone: a stroke sent over a real websocket is held by all twelve robots within
  2 s of simulated time (0.5 s measured), a second stroke replaces it, the joystick message
  sets the chain moving, malformed messages change nothing (`tests/test_m3_phone.py`).
- Line 5, shape error against IR bearing noise, mean (worst) of 5 seeds in mm:

  | sigma | line | ring | U | S |
  | --- | --- | --- | --- | --- |
  | 5 deg | 1.8 (3.0) | 1.0 (1.6) | 2.3 (4.1) | 1.6 (2.9) |
  | 10 deg | 2.0 (3.9) | 1.6 (2.1) | 3.4 (4.7) | 3.5 (4.9) |
  | 20 deg | 5.7 (9.4) | 1.6 (2.1) | 5.7 (9.6) | 3.9 (6.6) |

- M2 after these changes: 48 of 50 seeds, no falls.

Open problems
- **The page has not been opened in a browser.** Its protocol is tested, its stroke maths
  was checked once by hand in Node; drawing, touch and layout are untried. Please try it.
- **A robot needs about 2 s of flash bearings before it can hold a shape** (D34). Tests
  hand the stroke to a chain that has been linked for 3 s. Sooner, the shape waits.
- **Edges.** A robot pushed sideways toward an edge by its chain does not see the edge
  (sensors at the front only). Two falls of this kind were seen and their triggers fixed
  (blind reverse, push pointing along the edge); the gap itself remains. Shape scenarios
  start at least 150 mm from the edges; 4 of 90 runs that reached an edge anyway failed
  before the last fix and pass after it. No acceptance test covers shapes at an edge.
- **Slow.** Corrections are capped at 20 mm/s (D36), so a ring takes about 40 s. Bases
  turn in place toward every small velocity; a real gearmotor will not drive 6 mm/s smoothly.
- The shape rule waits on one gyro model (D37): the firmware must hand core the mean rate.
- The rotate command has no catch-up and no test of its own beyond its use in `twist`.
- The new frames (SHAPE, CLOSE, the bytes added to LINK_ACK and LINK_UP) and the traced
  bit from M2 have no C++ unit tests; only the shape maths has. Scenario tests cover them.
- An open shape after a ring (the head pulling out of its tail) is implemented but has no test.
- Still true from below: lead speed 70 mm/s, loosened M1 movement test, flash collisions,
  things not modelled, all parameters tuned in simulation only.

Next
- M4 Lasso and heal (spec 7.6, 7.8). M3 is closed.

## 12 Nov 2026 (later): M2 acceptance lines all met in simulation (48 of 50 seeds)

Branch `m2-chain-forming`, merged into `main`; CI green on Ubuntu and macOS (`make test`,
`make lint`; CI does not run the 50-seed set). This entry replaces the open problem "why 13 of 50 fail" below.

Done
- Jammed tongue (D30): a robot towed round the standoff circle trails inside it, about
  60 degrees off the tail, and the joint on the head's hub needs 80. A head with a chain now
  walks round 60 mm further out, runs straight in, and docks only when the robot behind it
  is clear of the joint limit.
- Edge trap (D31): a towing head between its own curled chain and an edge turned back and
  forth for a minute or more. The edge turn now clears every nearby robot.
- Fall (D31): a towing head reversed blind off the table after its own tail refused its
  tongue (seed 4, during this work). That reverse is now a quarter pitch.
- Tracing (D32): a chain leads at 30% speed while a lower-ranked head works along it.
- Tests: the two 90% tests are no longer marked `xfail`; the 40% floor test is removed.

Verified how (all simulated; nothing has run on hardware)
- `make test`: 31 C++ test cases, 38 pytest tests pass. `make lint` clean.
  `make test-full` (50 seeds): 38 pass, 68 s.
- M2 line 1: **48 of 50 seeds (96%)**, 90% asked. Median forming time 74 s. Seed 11 ends
  with two chains; seed 17 forms at 180.4 s, 0.4 s late. Over seeds 0 to 149: 140 of 150 (93%).
- M2 lines 2 to 4: hold as before; slowest count 0.28 s, no mutual chase, no falls in the
  50 seeds or in the 150. 6 links broke over the 50 runs and re-formed.
- Each step measured over 150 seeds: before 74% (of 50); outer circle, joint check and
  edge turn 84%; with the slow-down 93%. A full stop instead of the slow-down: 90%, one fall.

Open problems
- The new LINK_UP bit (traced) has no C++ unit test of its own; it is covered only by the
  scenario tests.
- Tried and removed: keeping the turn direction in corners, and driving on for 2 s after an
  edge turn before seeking (no gain over 150 seeds; the second produced a fall).
- The 10 failures in 150 seeds were not analysed one by one. Before the slow-down the time
  of failing runs went to searching without contact (31%), tracing (23%) and edge turns (31%).
- The outer circle sits at 185 mm, close to the 220 mm IR range assumed in the config. If
  the real range is shorter, `seek.tow_standoff_extra` must shrink and the jam may return.
- Still true from the entry below: lead speed 70 mm/s, loosened M1 movement test, flash
  collisions, things not modelled, all parameters tuned in simulation only.

Next
- M3 Shapes, moving, phone page. M2 is closed.

## 12 Nov 2026: M2 chain forming works, but the 90% line is NOT met (74%, simulated)

Branch `m2-chain-forming` (not merged, not pushed; CI has not run on it).

Done
- core: link layer `core/src/chain.{h,cpp}` (rank, `ahead`/`behind`, index, count, chain id
  by gossip), handshake `LINK_REQ`/`LINK_ACK`, states `SEEK`, `DOCK`, `FOLLOW`, a chain
  standing still while a robot docks onto its tail, adoption of links nobody asked for,
  seekers working along a chain to its tail. Decisions D23 to D29.
- sim: scenario `chain` (`python -m sim.runner chain --seed 3`, `make sim S=chain`),
  viewer shows state and position in the chain.
- Config: `behaviour.chain/seek/dock/follow`, `sensing.flash_filter_gain`;
  `radio.flash_jitter` 0.1 -> 0.5 (D29).

Verified how (all simulated; nothing has run on hardware)
- `make test`: 31 C++ test cases, 37 pytest tests pass, 2 expected failures (below).
  `make lint` clean. `make test-full` (50 seeds): same counts, 69 s.
- M2 line 1, one chain of 12 within 180 s in 90% of 50 seeds: **NOT MET. 37 of 50 (74%).**
  Median forming time of the runs that made it: 83 s. The two tests holding the 90% line
  are marked `xfail` in `tests/test_m2_chain.py`; `test_chain_forming_has_not_got_worse`
  holds a floor of 40%.
- M2 line 2, index 0..11 and count 12 within 5 s of the last link: holds in every run
  that formed a chain; slowest 0.24 s.
- M2 line 3, no run ends with two chains chasing each other for more than 60 s: holds,
  longest mutual chase in any of the 50 runs 0.0 s.
- M2 line 4, unit tests: rank is a strict order (all 144 x 144 pairs); index, count and
  chain id settle with 5%, 30% and 60% of frames dropped, and again after a chain is cut
  and rejoined; handshake, link timeout, follower control.
- No robot fell in any of the 50 runs. 12 links broke over the 50 runs and re-formed.
- With core in the loop: two robots dock by rank; pre-linked robots adopt their links; a
  chain of 5 follows its wandering head for 90 s without a break; a cut chain recounts.

Open problems
- **Why 13 of 50 fail.** In 7 of them (seeds 3, 12, 25, 28, 35, 37, 45) a head with a
  chain behind it sits in `DOCK` align for most of the last minute: its base points at
  the tail but its tongue cannot, because the joint on its own hub lets the tongue swing
  only 100 degrees off the line of the robot behind it. The other 6 are still searching
  when time runs out. Two cures were tried and made it worse, so they are not in the code:
  backing off 150 mm (the head loses sight of the tail: 33 of 50) and walking round the
  tail until the chain trails (it fights the tail's guidance: 6 to 11 of 50, more broken
  links). Untried: approach the tail head-on from further out, so the chain is already
  straight; or let the tail's chain turn towards the seeker.
- Lead speed 70 mm/s breaks more links than 40 (12 against 3 in 50 runs) and forms faster
  (median 83 s against 134 s); pass rate is the same within noise (37 against 35).
- M1 test `test_wandering_robots_keep_moving` was loosened (average path, low floor per
  robot): wanderers now link up and stand still while others dock.
- Flash collisions still spoil about half of all IR samples.
- Not modelled, as before: IR occlusion by other robots, the solid front of the ring.
- All M2 parameters are tuned in simulation only.

Next
- Close the 90% line (start with the jammed tongue above), then merge and let CI run.
- Then M3 Shapes, moving, phone page.

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
