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
