# Prior art

What exists, what each project shows, and what Catena takes from it. Collected by a quick
search on 10 Nov 2026; it is not a literature review. Check each source before citing it
in the README.

| Project | What it shows | What it lacks for our purpose | Source |
| --- | --- | --- | --- |
| Kilobots (Harvard, 2014) | 1,024 robots form 2D shapes using only neighbour-to-neighbour information | Robots do not attach; a shape takes hours | [Science summary](https://architexturez.net/node/166244), [National Geographic](https://www.nationalgeographic.com/science/article/a-swarm-of-a-thousand-cooperative-self-organising-robots) |
| PuzzleBots (CMU, 2021-22) | Low-cost robots couple passively with knobs and holes and cross gaps with up to nine robots; hardware and software are open | Coupled shapes are not used as tools; later work added special pilot robots to get onto the far platform | [paper](https://arxiv.org/pdf/2202.02686), [follow-up](https://ar5iv.labs.arxiv.org/html/2202.13461) |
| Material-like robot collective (UCSB and TU Dresden, 2025) | Disc robots with motorised rim gears and perimeter magnets flow, then stiffen into load-bearing tools | Lab hardware; coordination uses an external light signal | [UCSB](https://news.ucsb.edu/2025/021769/how-get-robot-collective-act-smart-material), [SingularityHub](https://singularityhub.com/2025/02/24/this-robot-swarm-can-flow-like-liquid-and-support-a-humans-weight/) |
| Self-Assembling Robots (UPenn course project, 2026) | Four wheeled robots assemble a 2D shape | Central controller; magnetic latching was dropped | [project page](https://upenn-embedded.github.io/final-project-s26-t9/) |
| M-Blocks (MIT) | Cubes that pivot over each other using an internal flywheel and edge magnets | Reference for 3D stacking; far beyond this budget. Details from memory, verify before citing | search "M-Blocks MIT" |

## Lessons taken

- From Kilobots: gradient-style hop counts for position, and the discipline of one program
  on every robot.
- From PuzzleBots: passive mechanical coupling is enough to cross gaps, and getting onto
  the far side is the hard part. Hence the sloped nose and the one-pitch first gap.
- From the UCSB collective: magnets for adhesion, something else for strength.

## Claim to check before publishing

"No open project combines neighbour-only control, physical locking and shapes used as
tools at this cost." This rests on a quick search. Search again during M8 and soften the
sentence if anything close turns up.
