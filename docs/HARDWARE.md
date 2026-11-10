# Hardware

Status: draft for planning. No part has been bought or measured. Prices are rough
estimates from memory, not checked against shops; milestone M8 replaces them with a priced
list with links. Milestone M5 fills in the printed-parts table.

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
| Screws M2 and M3, wire, perfboard | 1 set | 3 EUR | 1.50 EUR | |
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

Filled in by M5: part, quantity per robot, material, orientation, supports, print time.

Printers available: Prusa CORE One L (300 x 300 x 330 mm), Prusa XL with five tool heads,
Prusa Pro HT90. No laser cutter. The XL can print a TPU tyre onto a rigid wheel hub in one
job; if that works on the coupon, the O-rings drop out of the parts list.

Starting materials: PETG for the ring parts (tongue printed flat, 4 perimeters), PLA or
PETG for the base. 0.4 mm nozzle, 0.2 mm layers.

## Coupons to print before any robot

| Coupon | Question it answers |
| --- | --- |
| Groove at 5.3, 5.4, 5.6 mm with a tongue stub | Which clearance slides freely with the least vertical play? |
| Tip magnet pocket at three wall thicknesses, against two M14 washers | Which wall gives a 0.25 to 0.55 N hold? Measure with a kitchen scale. |
| Ring bearing pair | Does the ring spin freely, stay on when pulled with 3 N, and not wobble? |
| Wheel hub on a 3 mm D-shaft | Press fit without cracking; tyre grip on the table surface |
| Detent magnet spacing | Does a free ring follow the base, and does a held ring let the base turn? |

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
