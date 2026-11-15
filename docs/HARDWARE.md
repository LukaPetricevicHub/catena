# Hardware

Status: draft for planning. No part has been bought or measured. Prices are rough
estimates from memory, not checked against shops; a priced list with links
replaces them before the first release. The printed-parts table is CAD v0; nothing has been printed.

## Parts per robot

| Part | Qty | Est. Germany | Est. AliExpress | Notes |
| --- | --- | --- | --- | --- |
| ESP32-S3 mini board | 1 | 8 EUR | 4 EUR | must expose 19 GPIO incl. GPIO1-10 |
| N20 gearmotor, plain, 6 V, about 100 rpm | 2 | 10 EUR | 4 EUR | 3 mm D-shaft, no encoder |
| DRV8833 motor driver module | 1 | 2.50 EUR | 0.80 EUR | |
| 6-axis IMU module (I2C) | 1 | 3.50 EUR | 1.50 EUR | gyro is what matters |
| AS5600 module with diametric magnet | 1 | 3.50 EUR | 1.20 EUR | ring angle |
| IR LED 940 nm, 5 mm or 3 mm | 6 | 1 EUR | 0.30 EUR | wide beam |
| IR phototransistor 940 nm | 6 | 1.50 EUR | 0.40 EUR | daylight-filter type |
| Reflective sensor (TCRT5000 class) | 4 | 2 EUR | 0.60 EUR | 2 edge, 2 wheel ticks |
| Analogue hall sensor (SS49E class) | 1 | 0.50 EUR | 0.20 EUR | dock detect |
| Protected 1S LiPo, about 500 mAh, JST-PH | 1 | 6 EUR | 3 EUR | must fit the tray: measure first |
| LiPo charger board, USB-C, with protection | 1 | 1.50 EUR | 0.40 EUR | |
| Slide switch, NPN transistor, resistors | 1 set | 1 EUR | 0.30 EUR | |
| Neodymium disc 6 x 3 mm N42 | 1 | 0.40 EUR | 0.15 EUR | tongue tip |
| Neodymium disc 3 x 2 mm | 2 | 0.30 EUR | 0.10 EUR | centring detent |
| Steel washer DIN 125 M14 (28 x 15 x 2.5) | 2 | 0.30 EUR | | hub ring; must be plain steel |
| O-ring about 24 x 3 mm | 2 | 0.60 EUR | 0.20 EUR | tyres, unless TPU is printed |
| Screws M2 and M3, wire, perfboard | 1 set | 3 EUR | 1.50 EUR | countersunk: 4 x M2 x 6, 1 x M3 x 12 |
| **Per robot** | | **about 46 EUR** | **about 20 EUR** | |

Plan: three robots from German shops for bring-up, the rest from AliExpress.
Six robots come to roughly 200 EUR, twelve to roughly 320 EUR, plus filament.

## Pin map (draft, check against the pinout of the board you buy)

| Signal | GPIO | Kind |
| --- | --- | --- |
| IR receiver 0 to 5 | 1 to 6 | ADC1 |
| Edge sensor left, right | 7, 8 | ADC1 |
| Hall dock sensor | 9 | ADC1 |
| Battery voltage (divider) | 10 | ADC1 |
| Motor left IN1, IN2 | 11, 12 | PWM |
| Motor right IN1, IN2 | 13, 14 | PWM |
| Wheel tick left, right | 15, 16 | digital in |
| IR LED drive (all six, via transistor) | 17 | digital out |
| I2C SDA, SCL | 38, 39 | IMU and AS5600 |
| Status LED | on board | |

Avoid strapping pins (0, 3, 45, 46) and the USB pins (19, 20). ADC2 pins cannot be read
while the radio is active.

## Printed parts

CAD v0 (`make cad` writes STEP, STL and 3MF per part to `cad/out/`, each
in its print orientation). Nothing has been printed. The masses are the model's volume
times the filament density, as if printed solid: an upper bound, not a weight.

| Part | Qty | Material | Print orientation | Support | Solid mass |
| --- | --- | --- | --- | --- | --- |
| `chassis` | 1 | PLA | as it sits in the robot, floor on the bed | none; bridges over the 3.2 mm infrared holes and the two 6.2 mm tick sensor windows | 29 g |
| `lid` | 1 | PLA | as it sits in the robot, underside on the bed | none | 10 g |
| `board_carrier` | 1 | PLA | as it sits in the robot, plate on the bed | none | 6 g |
| `skid` | 2 | PETG | on its side, so the ramp is drawn in the layer | none | 0.2 g |
| `wheel_hub` | 2 | PETG | axle upright, striped face up | none | 4 g |
| `tyre` | 2 | TPU | axle upright | none | 1 g |
| `ring_lower` | 1 | PETG | as it sits in the robot, plate on the bed, tongue flat | under the tongue outside the plate, 3.2 mm high, 22 x 29 mm; bridge over the 3.1 mm detent pocket | 31 g |
| `ring_upper` | 1 | PETG | as it sits in the robot, underside on the bed | none | 18 g |
| `bearing_retainer` | 1 | PETG | flange on the bed, pin upright | none; bridge over the 6.1 mm magnet pocket | 1 g |

Pictures: `docs/media/cad_robot.png`, `docs/media/cad_linked.png` (`make render`).

How the parts go together
- The two motors lie end to end in the cradle of the chassis; the board carrier sits on
  the cradle rails and on the motors, and its two posts reach up to the lid. Two M2
  countersunk screws through the lid into the front and rear pillars close the base.
- The skids press into two slots in the floor, front and rear. Their feet stand 0.3 mm
  above the table. The base is chamfered front and rear from its floor up 3 mm, so skid
  and nose together climb 5.7 mm (the bridge expects 5 mm of droop).
- Ring bearing: the retainer is a pin. Its flange lies under the lid, its 10 mm journal
  turns in the lid's bore, its flatted 8 mm spigot goes up through the lower ring. One M3
  x 12 countersunk screw through the upper plate into the pin clamps upper plate, lower
  ring and pin together; the lid is caught between flange and ring with 0.2 mm of play.
  The ring rides on a 26 mm land on the lid. The diametric magnet of the AS5600 sits in
  the pin's lower end, above the sensor on the board carrier.
- Hub: two M14 washers drop over the hub core before the upper plate goes on. Two M2
  countersunk screws hold the upper plate on the solid front sector.
- Magnets: 6 x 3 mm in the tongue tip, pocket open to the top, 1.6 mm of plastic toward
  the hub. 3 x 2 mm detent magnets in the lid and under the lower ring, both 24 mm in
  front of the axis, 3 mm apart.
- Tyre: the hub has a V groove that takes either the printed tyre or a 24 x 3 mm O-ring.
- Free room for the cell: behind the motors, about 21 x 30 mm of floor, 15 mm high.

Not designed yet, because they depend on parts nobody has measured: mounts for the
controller board, AS5600 module, IMU and charger; a USB opening; how the two edge sensors
and the two wheel tick sensors are held in their windows; the battery tray.

Printers available: Prusa CORE One L (300 x 300 x 330 mm), Prusa XL with five tool heads,
Prusa Pro HT90. No laser cutter. The XL can print a TPU tyre onto a rigid wheel hub in one
job; if that works on the coupon, the O-rings drop out of the parts list.

Starting materials: PETG for the ring parts (tongue printed flat, 4 perimeters), PLA or
PETG for the base. 0.4 mm nozzle, 0.2 mm layers.

## Coupons to print before any robot

| Coupon | Files in `cad/out/` | Question it answers |
| --- | --- | --- |
| Groove at 5.3, 5.4, 5.6 mm with a tongue stub | `coupon_groove_5.3/5.4/5.6` (1, 2, 3 notches), `coupon_groove_top` x 3, any `coupon_tongue_wall_*` | Which clearance slides freely with the least vertical play? |
| Tip magnet pocket at three wall thicknesses, against two M14 washers | `coupon_tongue_wall_1.2/1.6/2.0` (1, 2, 3 notches), `coupon_washer_post` | Which wall gives a 0.25 to 0.55 N hold? Measure with a kitchen scale. |
| Ring bearing pair | `coupon_bearing_lid`, `coupon_bearing_ring`, `coupon_bearing_cap`, `bearing_retainer` | Does the ring spin freely, stay on when pulled with 3 N, and not wobble? |
| Wheel hub on a 3 mm D-shaft | `coupon_shaft_bar` (bores -0.1, 0, +0.1 mm; 1, 2, 3 notches), `wheel_hub`, `tyre` | Press fit without cracking; tyre grip on the table surface |
| Detent magnet spacing | none yet: use the real `lid` and `ring_lower` | Does a free ring follow the base, and does a held ring let the base turn? |

`coupon_plate` holds all PETG coupons plus one retainer and one wheel hub on one
180 x 180 mm bed (165 x 132 mm used). The tyre is TPU and prints on its own. The tongue
stubs print flat on the bed; the real tongue prints on support, so its underside will be
rougher than the stub's.

## Bring-up order after v0.1

1. Print coupons, measure, update `config/robot.yaml`.
2. Build one base without a ring: motors, ticks, gyro, edge sensors. Drive a square, stop at the table edge.
3. Add the ring: angle sensor, detent, free rotation while driving.
4. Build a second robot: flash ranging between two boards, bearing and range calibration.
5. Dock and undock twenty times in a row. Tune magnet wall and dock speed.
6. Third robot: chain of three, index and count, pull the middle one out.
7. Six robots: ring, lasso, heal, sweep. Then the bridge over a one-pitch gap.

After step 5 works, replace hand wiring with a carrier board: twelve hand-wired robots
would take 30 to 40 hours of soldering.

## Safety

- Use protected cells and a charger board with protection. Do not charge unattended.
- Cells must not be squeezed by the chassis; the tray has clearance on every side.
- Small magnets are a swallowing hazard and can pinch; keep them away from children and cards.
