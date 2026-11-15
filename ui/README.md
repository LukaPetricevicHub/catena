# ui

The phone drawing page (spec section 10): one static file, `index.html`.

- A square pad to draw one stroke; end where you started for a closed shape.
- Buttons for line, ring and U, and Release (the chain goes back to following its head).
- A joystick that pushes the shape and a slider that turns it.
- A status line: robots linked, chain count, shape error.

## Use it with the simulator

```
make sim S=play ARGS=--serve
```

starts a linked chain of twelve, the viewer, a web server for this folder on port 8000 and
a websocket on port 8765. On the same computer open `http://localhost:8000/`. On a phone in
the same Wi-Fi open `http://<address of the computer>:8000/`. `S=chain` works too: the
robots first form their chain by themselves, then take the shape.

The page sends JSON over the websocket; `sim/bridge.py` lists the messages. It resamples
the stroke to 32 points and marks it closed if its ends are within 10% of its length.

## What is tested

`tests/test_phone.py` speaks the page's protocol to a running simulator over a real
websocket. The JavaScript in the page itself has no automated test and has not been
opened in a browser yet: its stroke maths (32 points, closed or open) was checked once by
hand in Node, its drawing and touch handling not at all (see `STATUS.md`).

## On hardware (not built yet)

The same page is to be served by whichever robot the phone joins over Wi-Fi; that robot
hands the stroke to its neighbours (`catena_set_shape`, `catena_set_twist` in the C API).
