# Catena

Twelve identical table robots that link into a chain, bend it into a shape you draw on a
phone, and use the shape as a tool: a ring that cages and carries a ball, a scoop that
sweeps blocks, a straight chain that bridges a gap. No central controller. Every robot
runs the same firmware and talks only to robots it can see.

**Status: design and plan. Nothing is built or tested yet.** This README is rewritten in
milestone M8 with clips and measured results from simulation.

## Look at it

Open `sim3d/concept.html` in a browser (it needs an internet connection to load one
library). It is a concept viewer with scripted motion, there to show how the robots link
and move. The physics simulation that runs the real control code is built in milestones
M1 to M4.

## Read more

- `docs/SPEC.md`: what is being built
- `docs/MILESTONES.md`: in which order, and how each step is checked
- `docs/DECISIONS.md`: why it looks the way it does
- `docs/HARDWARE.md`: parts, pins, printing, bring-up
- `docs/PRIOR_ART.md`: related work
- `CLAUDE.md`: working instructions for Claude Code

## Licences

Code MIT, hardware files CERN-OHL-P v2, documentation CC BY 4.0 (files added in M0).
