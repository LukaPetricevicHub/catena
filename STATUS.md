# Status

Updated at the end of every session. Newest entry on top.

## 12 Nov 2026 (later): M2 acceptance lines all met in simulation (48 of 50 seeds)

Branch `m2-chain-forming`. This entry replaces the open problem "why 13 of 50 fail" below.

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
- Merge `m2-chain-forming` and let CI run.
- Then M3 Shapes, moving, phone page.

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
