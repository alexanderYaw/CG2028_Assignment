# Part 2 – Wearable application: design notes

## Files changed
| File | What it does |
|---|---|
| `Core/Src/main.c` | 50 Hz sampling loop, 6 × `ewma_filter` calls (one state per axis), ±8 g accel range, non-blocking LED, button, UART |
| `Core/Src/fall_detector.c` + `Core/Inc/fall_detector.h` | Feature extraction and the fall state machine; all thresholds are `#define`s in the header |
| `Part2_Simulation/fall_sim.c` | PC simulation of 16 motion scenarios through the same EWMA + detector code |

## Data flow
LSM6DSL accel (mg) + gyro (mdps) → **assembly EWMA** (6 axes, separate states) → features `|a|` (mg), `|ω|` (dps), tilt (deg) → state machine → LED2 + UART

## State machine
```
CALIBRATING (1 s) → NORMAL ──free fall |a|<0.6 g ≥60 ms──→ FREE-FALL? ──landing ≥1.3 g within 1 s──┐
                      │ ──direct impact |a| ≥ 1.7 g──────────────────────────────────────────────→ CONFIRMING (2 s)
                      │                                                                               │
                      ↑←── near-fall rejected (reason printed) ←── any check fails ────────────────────┤
                      │                                                                all pass ↓
                      ↑←── button "I'm OK" / back upright 3 s ←──────────────────────── FALL DETECTED (LED 150 ms)
                      ↑←── button ←── LONG LIE (30 s, LED 50 ms)                            │ 30 s
   hold button 2 s → SOS (LED 50 ms) → short press clears
```
Checks in CONFIRMING (all required, each uses a different sensor or quantity):
1. **Rotation (gyro):** peak |ω| ≥ 150 dps in the 1 s before and 0.5 s after impact
2. **Posture (accel direction):** gravity direction changed ≥ 45° vs pre-event posture
3. **Stillness (both):** ≥ 70 % of samples in 0.5–2.0 s after impact have |a| = 1 g ± 0.15 g and |ω| < 25 dps

## Parameter justification
| Parameter | Value | Reason |
|---|---|---|
| Sample period | 20 ms | Sensor ODR is 52 Hz; the starter's 1 s `HAL_Delay` loop would miss a 0.5 s fall entirely |
| Accel range | ±8 g | BSP default ±2 g clips impacts (2–6 g) |
| α accel | 50 % (τ ≈ 29 ms) | Keeps short impact spikes; α = 25 % cuts a one-sample spike to ¼ |
| α gyro | 30 % (τ ≈ 56 ms) | Fall rotation lasts 300–800 ms, so extra smoothing is harmless and damps shake jitter |
| Free fall | < 600 mg for 3 samples | Normal activity never drops below ~0.7 g |
| Impact | 1.7 g direct / 1.3 g after free fall | Sitting ≈ 1.2–1.5 g; soft landings (bed, cushion) need the lower tier |
| Rotation | 150 dps | Falls rotate ~90° in < 0.6 s; sitting and medium-speed bending stay below ~100 dps |
| Re-arm delay | 2 s | Picking the board up after an alert does not re-trigger |

**Tune these on the real board:** watch the UART `|a|`, `|w|` and `tilt` values, and the per-event lines. Every rejected near-fall prints its rotation, posture change and stillness %, which gives you measured evidence for the slides.

## Simulation results (20 noisy repeats each)
Falls detected: forward with free fall, sideways slump without free fall, backward onto cushion, very soft fall onto bed – 80/80.
No false alarm: still, walking, carrying, sitting down, plopping into chair (2 g), medium-speed bend, slow lowering to lying, light shaking, vigorous shaking, hop, stumble-and-catch, placing on table – 0 false alarms.
Also verified: fall latched while lying, long-lie at 30 s, button acknowledgement, self-recovery, SOS.

## Demo tips
- Hold or strap the board **upright** (like a chest-worn pendant) before the "fall"; posture change is measured relative to that.
- Drop it in a rotating motion onto a cushion, then leave it still for 2 s. The LED switches to fast blink.
- Press the blue USER button to acknowledge, or stand the board back upright for 3 s.
- For the long-lie demo you may want to shorten `FD_LONG_LIE_MS` (e.g. 15000).
