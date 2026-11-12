# Decisions

Settled choices and why. Add new entries at the bottom: date, decision, reason, what would
change it. Do not rewrite old entries; supersede them.

## D1 A printed tongue carries load; the magnet only holds position (10 Nov 2026)

Holding two 95 g robots over a table edge needs several newtons at a magnet's lever arm,
while a robot that light can pull about 0.7 N to undock. A magnet cannot be both. The flat
tongue in a groove carries vertical load and bending without any force along the tongue,
so a 0.4 N tip magnet is enough and a robot unlinks by driving away.
Would change it: a switchable magnet or latch per robot.

## D2 Round disc with a turning ring, not a dodecahedron or cube (10 Nov 2026)

Asked: would a dodecahedron let robots stack 2x2x2, 2x2x3, 1x1x12? No, for three reasons.
A regular dodecahedron does not fill space; the rhombic one does but not in rows and
columns; those stacks are cube stacks. Stacking needs a robot to lift its own weight, which
wheels cannot do; the known hardware answer is a flywheel-driven cube (M-Blocks), a
different robot. A polyhedron with docking faces all round has no room for wheels that stay
on the table or for the turning ring.
3D stacking is kept as simulation only: milestone M10, twelve pivoting cubes.
Would change it: budget and time for a lifting actuator.

## D3 The coupling sits on a free-turning ring (10 Nov 2026)

Wheels cannot roll sideways. With couplings fixed to the wheels, a closed ring of robots
cannot translate. A free ring lets each base steer under the locked shape, so the shape
moves in any direction.

## D4 Version 1 shapes are single chains (10 Nov 2026)

Ring, chain and scoop are all one chain, open or closed. That gives one local rule (hold a
turning angle between two neighbours) and makes healing a re-link plus a recount. Filled
shapes and branches are later work; the hub geometry does not prevent them.

## D5 Tongue lands on the neighbour's hub, so joints sit at robot centres (10 Nov 2026)

A tongue can enter the groove from a wide range of directions, which makes docking
tolerant, and the chain pivots about robot centres, so a base can spin without disturbing
the chain. The alternative, a slot at the rear of each ring, needs approach from behind.

## D6 80 mm pitch and plain N20 motors (10 Nov 2026)

Two N20 gearmotors end to end are about 51 mm with terminals; wheels on the 10 mm shafts
bring the width to about 71 mm. That fits a 72 mm base envelope and an 80 mm pitch, not the
75 mm first sketched. Versions with shaft encoders are about 8 mm longer each and do not
fit, so wheel motion is read from a stripe pattern on the wheel.
Would change it: measured parts that differ from these typical dimensions.

## D7 ESP32-S3 instead of ESP32-C3 (10 Nov 2026)

Ten analogue inputs are needed (six IR, two edge, hall, battery) plus motor, tick, LED and
I2C pins. The C3 has too few pins and too few usable ADC channels.

## D8 Infrared for who and where, radio for what (10 Nov 2026)

A radio frame announces a flash; the IR pulse that follows gives bearing and range. No data
is modulated onto IR, which keeps the sensing analogue and simple. Locality is enforced by
dropping radio frames from senders that were not seen flashing.

## D9 No overhead camera in the control loop (10 Nov 2026)

A camera that tells robots where things are is a central controller. Objects are found
on board: something that reflects IR but never announces a flash. A camera may record runs.

## D10 One C++ core, called through a C API (10 Nov 2026)

Behaviour is written once and runs in the simulator and on the robot. A plain C interface
loaded with ctypes avoids binding generators and builds anywhere.

## D11 Carry direction comes from a twist command in v0.1 (10 Nov 2026)

The swarm assembles, finds the ball and closes the ring by itself. Where to carry it comes
from the phone joystick or a test script. Homing on a beacon is later work.

## D12 Design for twelve, build six first (10 Nov 2026)

Six robots do every table demo at small scale and cost roughly half. Simulation and tests
use twelve.

## D13 Shape rule uses the neighbours' angle errors (10 Nov 2026)

First attempt in the concept viewer: each robot moves to the point that would give it its
own turning angle. With fixed link lengths that stalls: a ring of twelve asked to become a
ring of eleven sat 3 degrees off and drifted open. Replacement: gradient descent on the sum
of squared angle errors, where each robot uses its own error and those of its two linked
neighbours. In the viewer this takes line, ring and scoop to under 0.2 degrees and re-closes
a ring after a removal in 10 of 10 runs. Those runs are ideal kinematics, not physics.

## D14 The lasso curls progressively (10 Nov 2026)

If every robot takes its ring angle the moment the head finds the object, the tail swings
through the object. Robots take the ring angle only once they are on the circle.

## D15 One parameter table feeds the C header and the Python binding (10 Nov 2026)

`CORE_FIELDS` in `sim/config.py` lists every core parameter with its YAML key and unit
conversion. The CMake build runs `python -m sim.config --header` to write `params_gen.h`
and `sim/corelib.py` builds its ctypes struct from the same table, so the two cannot drift.
Cost: building `core/` needs Python with pyyaml. Would change it: a firmware build that
cannot run Python.

## D16 Tooling and pins beyond spec section 9 (10 Nov 2026)

`ruff` is added as a development-only tool because `CLAUDE.md` asks for ruff-clean Python.
`pymunk` is pinned to one exact version because the physics engine decides the outcome of
every seed. `cadquery` sits in the optional extra `cad` so `make setup` stays small until M5.

## D17 Licence files (10 Nov 2026)

`LICENSE` is MIT and covers all code. `LICENSES/` holds CERN-OHL-P v2 for `cad/` and
CC BY 4.0 for `docs/`, the README and media. Texts are the SPDX reference copies.

## D18 Flash ranging: noise on bearing and range, rendered into six readings (11 Nov 2026)

The simulator draws bearing and range noise with the configured sigmas, then renders six
cosine-lobe receiver readings (receiver k points at k x 60 degrees) by the inverse-square
law. Core computes bearing from the vector sum and range from its length, so the estimator
runs in C++ for sim and robot alike. A physical receiver noise model waits for two boards.

## D19 Randomness comes from the platform through `catena_inputs_t.entropy` (11 Nov 2026)

Core needs random numbers (wander, flash timing). Each step the platform supplies 32 fresh
bits (the ESP32 hardware RNG; a seeded generator in the simulator), mixed into a PRNG in
core. The spec's `catena_create` signature stays unchanged and every run follows its seed.

## D20 Simulated link: groove, magnet, yaw limit (11 Nov 2026)

A link is a groove joint (hub stays on the tongue axis, carries compression), a slide joint
whose force is capped at the hold force (the magnet), and a +/-100 degree yaw limit. It breaks
when the tip is pulled more than the capture range off the hub. The magnet cannot pull a
robot whose wheels hold, so the docking robot drives the tip in; the hall reading is
analogue, 1 when seated and 0 at the edge of capture range.

## D21 Flash timing and collisions in the simulator (11 Nov 2026)

Core requests a flash with a delay inside the current 20 ms step. The simulator resolves
collisions (3 ms window) once the following step's flashes are known and delivers samples
one step later. With twelve robots at 10 Hz about half the IR samples are spoiled (measured
0.46 to 0.51); smarter scheduling can wait until it limits chain forming.

## D22 Wander backs off a fixed distance and turns in place (11 Nov 2026)

After a front edge sensor loses the table the robot reverses 40 mm, then turns in place
90 to 180 degrees, without waiting for the sensors to see table again. Waiting let a robot
parallel to an edge reverse along it until it fell (seed 15). Neighbours ahead, measured by
flash, make it steer away. IR occlusion by other robots is not modelled yet.

## D23 Link frames: LINK_ACK repeats every step and carries the gossip (12 Nov 2026)

The hub robot sends LINK_ACK to the robot on its hub every step while its hall sensor holds:
index, lowest id from the head down, its hall reading and its speed along the link. The
linked robot sends LINK_UP to the robot ahead every step: robots from itself to the tail,
lowest id among them, hold and sight flags, lowest hall reading behind it. Index, count and
chain id follow from these (spec 7.3); a lost frame costs one step and a silent link is
dropped after `chain.link_timeout`. Would change it: a radio that cannot carry 100 frames/s.

## D24 A follower steers by its ring angle and keeps the gap by the hall reading ahead (12 Nov 2026)

Flash range (10%) is too coarse for a 6 mm capture range. The tongue points at the hub
ahead, so the ring angle is the bearing, and the hub robot reports its analogue hall reading.
A head with robots behind it drives at `follow.lead_speed`, slowed by the lowest hall
reading along its chain, never towards the robot on its hub, and at an edge turns in place
instead of reversing.

## D25 Edge rule for a linked follower (12 Nov 2026)

A follower stops driving forward when both front sensors see no table, and keeps following
on one: the hub it follows stands on the table and the straight line to it stays on it. It
takes no backward feed-forward, because nothing senses the edge behind it.

## D26 The tail guides the seeker; a tail alone turns its back (12 Nov 2026)

A seeker cannot see which way a tail's ring points. The tail measures the seeker's bearing
against its own ring angle and sends the offset from the middle of its acceptance zone
(GUIDE). The seeker walks round the standoff circle until the offset is inside
`dock.window`. A tail that is a chain of one turns its back to the seeker instead. While a
lower-ranked head within `seek.hold_range` announces a tail as its target, that tail's
chain stands still (flag passed up in LINK_UP).

## D27 Unrequested links are adopted; rings are refused (12 Nov 2026)

Robots bump into each other and tongues land without a LINK_REQ (M1 saw it). After
`chain.adopt_delay` the hub robot offers the link to the nearest free tongue inside its
acceptance zone; that robot accepts if its tongue points at the sender. If it is the
sender's own head the hub robot sends LINK_NAK and the head backs out: rings arrive in M3.
A robot whose `ahead` is measured out of link range for `chain.link_timeout` drops the link.

## D28 Seekers see more than tails (12 Nov 2026)

A head goes for the nearest robot of any higher-ranked chain and moves along that chain's
`behind` ids, as far as they are in sight, to its tail. A follower that sees a higher-ranked
chain sets a flag in LINK_UP; a head with nothing in sight then turns back along its own
chain. Both use only flash sightings and frames from confirmed neighbours.

## D29 Flash period jitter 0.5 instead of 0.1 (12 Nov 2026)

With 10% jitter two robots whose flashes collided kept colliding: one robot got no clean
sample of a linked neighbour for 2.5 s and the link layer dropped it. With 50% jitter
successive collisions are nearly independent. A linked neighbour also stays an accepted
sender while its link frames arrive. Would change it: a slotted flash schedule.

## D30 A head with a chain walks round a tail further out, then runs straight in (12 Nov 2026)

A robot towed round the standoff circle trails inside it, about 60 deg off the tail's
direction; the joint on the head's hub needs 80, so the head's tongue never reached the tail.
With a chain the head circles `seek.tow_standoff_extra` further out, runs in, and docks only
while the robot behind it is `dock.tow_margin` clear of the joint limit (its own flash bearing).

## D31 Edge turns of a towing head clear every nearby robot; its blind reverse is short (12 Nov 2026)

A head whose own chain lay curled beside it turned away from the edge, avoided the chain and
swung back to the edge, for a minute or more. The turn now goes on until no robot within
avoiding range is ahead. After a refused link (LINK_NAK) a towing head reverses a quarter
pitch only: it has no edge sensor at the rear and fell off the table once in simulation.

## D32 A chain slows down while a lower-ranked head works along it (12 Nov 2026)

A seeker with a chain is no faster than the chain it follows, so it rarely reached the tail.
Any member that sees a lower-ranked head with a target sets bit 4 in LINK_UP; the head then
leads at `seek.traced_speed` (0.3) of its speed. Slowing, not stopping: a full stop passed
fewer seeds (135 against 140 of 150) and D28's deadlock of a held chain stays impossible.
