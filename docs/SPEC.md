# Catena specification, v0.1

Status: design locked on 10 Nov 2026. Nothing has been built or run on hardware.
Numbers quoted here are copies for readability; `config/robot.yaml` is authoritative.

## 1. Goal and scope

Twelve identical robots on a table link into one chain, bend it into a stroke drawn on a
phone and use the shape as a tool. Control is decentralised: same firmware everywhere,
neighbour-only sensing and messaging.

v0.1 (target 16 Nov 2026) is a simulation-validated repository: behaviour code proven in a
2D physics simulator, printable CAD, firmware that compiles, parts list and build guide.
Hardware bring-up follows after v0.1.

Out of scope for v0.1: filled 2D shapes, branching structures, hardware tests, climbing on
hardware, a custom PCB.

## 2. Demos and what counts as success

| Demo | What happens | Passes when (simulation) |
| --- | --- | --- |
| Assemble | Robots start scattered and form one chain | one chain of 12 within 180 s in 90% of 50 seeds |
| Draw | A stroke from the phone becomes the chain's shape | mean vertex error under 12 mm for line, ring, U and S |
| Lasso | Chain finds a ball, closes a ring round it, carries it 400 mm | ball inside ring at the end in 80% of 50 seeds |
| Heal | One robot is removed mid-task | chain re-links and re-forms the shape within 60 s in 80% of 50 seeds |
| Sweep | A U-shaped scoop pushes eight blocks into a corner | scoop parked against both edges of the corner with 80% of blocks inside it, no robot off the table |
| Bridge | A straight chain crosses an 80 mm gap between tables | 3D simulation: all robots cross, tip droop reported (stretch) |
| Stack | Twelve cubes go from 1x1x12 to 2x2x3 with local rules | lattice simulation only (stretch) |

A robot leaving the table, or a chain splitting when nobody removed a robot, fails the run.

## 3. Robot: mechanics

Two assemblies, joined by a bearing on the vertical centre axis.

**Base (drive unit).** Round, 72 mm envelope, 27 mm tall, 3 mm ground clearance. Two plain
N20 gearmotors end to end on one axis, 30 mm wheels with soft tyres, track 58 mm, two
skids front and rear. Carries battery, controller, all sensors. Everything on the base stays
inside the 72 mm circle so a base can spin in place while its neighbours stand 80 mm away.

**Ring (turret).** Passive, no electronics. Two 78 mm plates 3 mm thick with a 5.4 mm
groove between them, a 28 mm steel hub in the middle (two M14 washers) and one flat tongue,
5 x 22 mm, reaching 66 mm from the centre. The front sector between the plates is solid and
carries the tongue. The ring turns freely on the base and must not lift off: the bearing
holds at least 3 N axially because a robot over a gap hangs from its ring.

**Coupling.** A robot links by driving its tongue into another ring's groove until the tip
magnet lands on the steel hub. Linked centres are 80 mm apart.

- Vertical loads and bending go through the tongue pressing on the two plates over 25 mm.
  This is what carries a robot over a gap. Magnets carry none of it.
- Along the tongue the joint is held only by the tip magnet: 0.4 N target, below the 0.7 N
  a robot can pull. A robot unlinks by driving away.
- The joint is free in yaw. The tip can sit anywhere on the hub within 100 degrees of the
  rear, and the chain can bend up to 100 degrees at each robot.
- A weak magnetic detent between base and ring points a free robot's tongue along its
  heading. When linked, the chain overrides it.

**Why the ring turns.** Wheels cannot roll sideways. If the coupling were fixed to the
wheels, robots in a ring would point in different directions and the ring could not
translate. With a free ring, each base steers under the locked shape.

**Bridge load case.** With two robots unsupported the worst joint carries
0.095 kg x 9.81 x (0.08 + 0.16) m = 0.22 N m. Over 25 mm engagement that is roughly 13 N on
each plate edge. Vertical play of about 0.9 degrees per joint gives about 5 mm droop at the
tip, so the base has a sloped nose and the first target gap is one pitch.

## 4. Robot: electronics (v0)

| Function | Part | Notes |
| --- | --- | --- |
| Controller | ESP32-S3 mini board | needs 10 ADC1 inputs; ADC2 is unusable while the radio runs |
| Motor driver | DRV8833 module | motors run from the cell directly |
| Motors | 2 x N20 plain, 6 V class | no shaft encoder: encoder versions are too long for the base |
| Wheel ticks | 2 x reflective sensor | reads a 12-stripe pattern printed on the wheel |
| Neighbour sensing | 6 x IR LED + 6 x phototransistor | 60 degrees apart, one LED drive pin |
| Table edge | 2 x reflective sensor | front-left and front-right, pointing down |
| Heading rate | 6-axis IMU on I2C | gyro only is required |
| Ring angle | AS5600 on I2C | diametric magnet in the ring's centre boss |
| Dock detect | analogue hall sensor | senses a tip magnet arriving at the hub |
| Power | protected 1S LiPo, about 500 mAh, charger board, slide switch | |

Pin map, sourcing and prices: `docs/HARDWARE.md`.

## 5. Sensing and communication

**Flash ranging.** Each robot announces itself about ten times a second: it sends a short
radio frame (`FLASH`, its id and status) and 2 ms later pulses all six IR LEDs. A listener
that hears the frame samples its six receivers before and during the pulse. The difference
gives bearing (weighted vector sum of the six) and range (calibrated inverse-square on the
total). If two `FLASH` frames arrive within 3 ms the sample is discarded.

**Neighbour rule.** The radio reaches every robot, so locality is enforced in software:
the link layer accepts frames only from senders it has seen flash in the last 2 seconds.
Everything above the link layer sees only neighbours.

**Objects.** During its own pulse a robot also samples its receivers. A reflection from a
direction where no known neighbour stands is an object. The usable range is assumed to be
about 110 mm for a white ball and has to be measured.

**Linked neighbours.** `ahead` is the robot my tongue sits on; its direction is the ring
angle, which is precise. `behind` is the robot whose tongue sits on my hub; its direction
comes from flash bearings, filtered over time. The hall sensor confirms that a tip has
arrived.

**Messages.** `FLASH` (broadcast): id, chain id, chain size, index, role flags, hub free,
shape sequence number, twist sequence number. Unicast to linked neighbours: `LINK_REQ`,
`LINK_ACK`, `UNLINK`, `INDEX`, `COUNT`, `SHAPE` (chunked), `TWIST`. Shape and twist spread
hop by hop along the chain; a higher sequence number wins.

## 6. Software architecture

```
        phone page (ui/)                 scenario tests
              |                                |
        websocket bridge                  sim runner
              |                                |
   +----------+---------- sim (Python, pymunk) +-----------+
   |  world, physics, sensor models, radio model, viewer   |
   +---------------------- C API --------------------------+
                           |
        core (C++17): link layer, chain, shape, behaviours
                           |
                     Hal interface
                 /                      \
        sim Hal (via C API)        firmware Hal (ESP32-S3)
```

`core/` is a library with one object per robot and one entry point:

```c
catena_t* catena_create(uint8_t robot_id, const catena_params_t* params);
void      catena_step(catena_t*, const catena_inputs_t* in, catena_outputs_t* out);
void      catena_destroy(catena_t*);
```

`catena_inputs_t` carries elapsed time, sensor readings and received frames.
`catena_outputs_t` carries wheel speeds, frames to send, a flash request and a small debug
record (state, index, chain size) that only the viewer and tests read. The simulator loads
the shared library with `ctypes`; the firmware links it statically. Parameters are
generated from `config/robot.yaml` into a header and passed in at creation.

## 7. Behaviour

### 7.1 States

`WANDER` (free, exploring) -> `SEEK` (approaching a tail) -> `DOCK` (final approach,
tongue first) -> `LINKED`. A linked robot runs the active mode: `FOLLOW` (train behind the
head), `SHAPE` (hold the drawn shape), `MOVE` (shape plus twist), `LASSO` (head only),
`BRIDGE`. `RECOVER` handles a lost link. Edge avoidance overrides everything except
`BRIDGE`.

### 7.2 Forming and merging chains

Every robot starts as a chain of one: its tongue is free (it is a head) and its hub is free
(it is a tail). Chains have a rank: larger size wins, lower chain id breaks ties; chain id
is the lowest robot id in the chain, spread by gossip. A head seeks the nearest visible
tail of a higher-ranked chain and docks onto it. Because rank is a strict order, two chains
never chase each other's tails. A chain following its seeking head runs `FOLLOW`.

Docking: approach until range under 100 mm, align heading to the bearing, send `LINK_REQ`,
drive straight at dock speed; the target answers `LINK_ACK` when its hall sensor fires.
Three failed attempts: back off 150 mm and re-approach from a new angle.

### 7.3 Index and count

The head has index 0. Each linked robot sets index = index of `ahead` + 1 and tells
`behind`. The tail learns the count and sends it back toward the head. A ring has no head:
the robot with the lowest id takes index 0.

### 7.4 Shape from a stroke

The phone resamples the stroke to 32 points by arc length and marks it closed if its ends
are within 10% of its length. Each robot places N vertices evenly along the stroke (N edges
if closed, N - 1 if open), scales the result so the mean edge is one pitch, and reads off
the turning angle at its own vertex. Turning angles are clamped to the joint limit.

Local rule. Every robot with two linked neighbours measures the turning angle at itself,
subtracts its target and sends the error e to both neighbours a few times a second. Every
linked robot, the two ends included, then moves sideways to its links:

    v = -g * pitch * [ (e_ahead - e_self) * n(link to ahead) + (e_behind - e_self) * n(link to behind) ]

Conventions: u is the link from `ahead` to me and w the link from me to `behind`; the
turning angle is atan2(u x w, u . w); n(v) = (-v.y, v.x) / |v|, taken of u and of w; a
missing neighbour or a robot without a turning angle contributes e = 0. This is gradient
descent on the sum of squared angle errors, written so that each robot needs only its own
two links and its neighbours' errors. The physical links keep the edge lengths, so holding
every turning angle holds the shape.

Do not replace this with "move toward the point that fixes my own angle". That simpler rule
was tried in the concept viewer: with fixed link lengths a uniform error cannot be removed
by every robot moving at once, and the chain stalls a few degrees off or drifts open. The
ends must swing, which is what the neighbour-error terms do.

The rule fixes the shape, not where it sits. Robots that sense a table edge add a push
away from it and gossip it, so the whole shape shifts and turns away from edges.

A closed shape closes itself: when the head sees its own chain's tail within docking range
it docks onto it.

### 7.5 Moving as one body

A twist (vx, vy, wz) is given in the shape frame, whose origin is the centroid of the
target polygon. Each robot knows the polygon, so it knows the direction of its own edge in
the shape frame; it measures the same edge in its base frame with the ring angle sensor.
The difference rotates the twist into its base frame. Its velocity is the twist's linear
part plus wz times its lever arm from the centroid, added to the shape-holding correction.
The base turns toward that velocity and drives, slower while misaligned.

In v0.1 the twist comes from the phone joystick or from a scenario script.

### 7.6 Lasso

The chain is in `FOLLOW` and its head wanders. When the head detects an object it stops
approaching at the radius of a ring of N robots (155 mm for twelve) and drives a circle of
that radius round the object by gyro. The shape becomes a regular N-gon, but a robot takes
up its ring angle only once it has reached the circle; until then it follows the robot
ahead. Curling the whole chain at once swings the tail through the object. When the head
sees its own tail it docks, and the ring waits for a twist.

### 7.7 Sweep

Shape U with its opening toward the travel direction, then a twist toward a table corner.
When robots report the first edge, the scoop slides along that edge until robots report the
second one, then stops. Gaps between linked bases are 8 mm, so 20 mm blocks cannot slip
through. The U of twelve robots is about 370 mm deep, so the blocks end inside the parked
scoop roughly half a metre from the corner, not in the corner itself.

### 7.8 Recover

A robot that loses `ahead` (hall or link timeout) becomes the head of what is behind it; a
robot that loses `behind` becomes a tail. The orphaned head seeks the tail of its former
chain first. After re-linking, index and count are recomputed and every robot re-derives
its turning angle for the new N, which closes the gap. A ring first becomes an open chain,
then closes again.

### 7.9 Bridge

Shape: straight line along the crossing direction. In `BRIDGE` a robot that loses the table
under both edge sensors stops its wheels and lets the chain carry it; robots on the table
keep driving. If more than two robots report no table at once, all reverse. The head
resumes driving when it finds the table again.

## 8. Simulator

Top-down 2D with pymunk. Physics at 240 Hz, `core` ticks at 50 Hz.

- Base: dynamic circle, 72 mm. Wheel forces limited by traction; sideways motion resisted
  up to the traction limit, then it slips.
- Ring: second body on a pivot at the base centre, free to turn, with the detent torque.
- Link: pivot joint between the point one pitch along one ring's tongue and the other
  ring's centre. It is created when a tip comes within capture range of a free hub inside
  the acceptance angle, and removed when tension exceeds the hold force.
- Collisions: plates with plates, bases with bases and objects. Tongues collide only with hubs.
- Sensors and radio follow section 5 with the noise values from the config, including
  dropped frames (default 5%).
- A robot whose centre passes a table edge has fallen.
- Every run takes a seed, a scenario and overrides, and returns pass/fail plus metrics as JSON.

The viewer draws the table, robots, tongues, links, each robot's state and index, and can
record clips. It also serves the websocket the phone page talks to.

`sim3d/concept.html` is a separate hand-written concept viewer. It works on ideal
positions with no physics, noise or wheel slip, and it does not use `core/`. Its JavaScript
holds working reference versions of the shape rule (7.4), ring closing, the lasso (7.6),
the sweep (7.7) and re-linking after a removal (7.8); read them before writing the C++.
Keep it as an explainer; do not extend it.

## 9. Dependencies

- C++: CMake 3.20+, a C++17 compiler, doctest (vendored single header).
- Python: pymunk, numpy, pyyaml, pytest, pygame-ce, websockets, imageio with ffmpeg.
- CAD: cadquery.
- Firmware: PlatformIO with the Arduino core for ESP32-S3; small in-repo drivers.
- M9 only: mujoco. M10 only: no new dependency.

## 10. Phone page

One static page, usable with a thumb. A square pad to draw one stroke; buttons for line,
ring and U; a joystick for the twist with a rotate slider; a status line (robots linked,
chain count, shape error). It sends JSON over a websocket to the simulator. On hardware the
same page is served by whichever robot the phone joins over Wi-Fi, and that robot injects
the shape into the gossip.

## 11. CAD

CadQuery, parameters from the config. Parts: base chassis, base lid and board carrier,
wheel hub and tyre, ring lower with hub core and tongue, ring upper plate, bearing
retainer. Coupons: groove fit at three clearances with a tongue stub, magnet pocket, ring
bearing, wheel press fit on a 3 mm D-shaft.

Checks run by `make cad`: every part fits a 180 x 180 mm bed; minimum wall 1.2 mm; no
overhang above 50 degrees without a listed support; two linked robots do not intersect at
any yaw from -100 to +100 degrees in 10 degree steps; a base turns a full circle inside its
ring; the linked centre distance equals the pitch.

Exports: STEP, STL and 3MF per part into `cad/out/`, plus one plate of coupons. Ring parts
are meant for PETG with the tongue printed flat; base parts may be PLA.

## 12. Firmware

PlatformIO project targeting an ESP32-S3 mini board. `Hal` implemented with hardware
timers for flash timing, ESP-NOW for frames, ADC1 for the analogue sensors. Safety: motors
off on low battery, on watchdog timeout, and when both edge sensors lose the table outside
`BRIDGE`. Bring-up sketches, one per subsystem: motors, wheel ticks, IR ranging between two
boards, ring angle, hall, edge sensors. Over-the-air update so twelve robots do not need
twelve cables. v0.1 requires only that it builds.

## 13. Tests and metrics

- C++ unit tests: shape maths, index and count, rank ordering, link-layer filtering.
- Scenario tests: the table in section 2. `make test` runs 5 seeds per scenario;
  `make test-full` runs all 50.
- Metrics per run: time to one chain, shape error, heal time, frames sent, falls, splits.
  `make clips` renders the README clips and a results table from the full run.

## 14. Licences

Code under MIT, hardware files under CERN-OHL-P v2, documentation under CC BY 4.0.
