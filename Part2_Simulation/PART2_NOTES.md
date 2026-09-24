# Part 2 – Wearable application: design notes

This builds on the `fall-detection` branch: the `FallPhase` enum, the
`FallDetector` struct, the squared-magnitude helpers and the thresholds are
kept, the phase transitions are implemented, and four checks are added
(posture change, inactivity ratio, long-lie escalation, SOS).

## Files
| File | What it does |
|---|---|
| `Core/Src/main.c` | 50 Hz loop, 6 × `ewma_filter` (one state per axis), ±8 g range, non-blocking LED, button, UART |
| `Core/Src/fall_detector.c` + `Core/Inc/fall_detector.h` | Features and the phase machine. No HAL calls, so the PC simulation compiles the same code that runs on the board |
| `Part2_Simulation/fall_sim.c` | 16 motion scenarios × 20 noisy repeats |

## Data flow
LSM6DSL accel (mg) + gyro (mdps) → **assembly EWMA** (6 axes, separate states) → `FallDetector_Update` → LED2 + UART

## Phase machine
```
MONITORING ──free fall <0.65 g ≥60 ms──→ AWAIT_IMPACT ──impact──→ POST_IMPACT ──┐
     │  ──impact ≥1.8 g──────────────────────────────────→ POST_IMPACT          │
     │  ──jerk ≥600 mg AND rotation ≥150 dps──→ AWAIT_IMPACT                    │
     ↑←── near fall, reason on UART ←── any check fails ←──────────────────────┤
     │                                                          all pass ↓
     ↑←── button / upright 3 s ←────────────────── ALARM (LED 150 ms) ←─────────┘
     ↑←── button ←── LONG LIE (30 s, LED 50 ms)          │ 30 s
  hold button 2 s → SOS (LED 50 ms) → press clears
```

Confirmation in POST_IMPACT needs all three, each a different quantity:

| Check | Threshold | Rejects |
|---|---|---|
| Rapid rotation (gyro) | peak ≥ 150 dps, incl. the second before the trigger | sitting, bending (< ~100 dps) |
| Posture change (accel direction) | ≥ 45° vs last stationary posture | shaking, hopping, placing the board down |
| Inactivity | stationary ≥ 750 ms continuously and ≥ 70 % of the window | carrying, walking on after a stumble |

## Parameter justification
| Parameter | Value | Reason |
|---|---|---|
| Sample period | 20 ms | Sensor ODR is 52 Hz; the starter's 1 s `HAL_Delay` loop missed falls entirely |
| Accel range | ±8 g | BSP default ±2 g clips impacts (2–6 g) |
| α accel | 50 % (τ ≈ 29 ms) | Keeps short impact spikes; 25 % cuts a one-sample spike to ¼ |
| α gyro | 30 % (τ ≈ 56 ms) | Fall rotation lasts 300–800 ms, so smoothing is free and damps shake |
| Free fall | < 650 mg for 3 samples | Normal activity never drops below ~0.7 g |
| Impact | 1800 mg, or 1300 mg after a free fall | Sitting ≈ 1.2–1.5 g; soft landings (bed, cushion) need the lower tier |
| Warm-up | 500 ms | EWMA outputs start at 0 and would look like free fall |
| Re-arm delay | 2 s | Picking the board up after an alarm does not re-trigger |

**Tune on the board:** every rejected near-fall prints its rotation, posture
change and inactivity %, so the UART log is the evidence for these numbers.

## Simulation results (20 noisy repeats each)
Detected: forward fall with free fall, sideways slump without free fall,
backward onto cushion, soft fall onto bed — 80/80.
No false alarm: still, walking, carrying, sitting, plopping into a chair (2 g),
medium bend, slow lowering to lying, light and vigorous shaking, hop,
stumble-and-catch, placing on table.
Also checked: alarm latched while lying, long lie at 30 s, button
acknowledgement, self-recovery, SOS.

Run it from the repository root:
```
gcc -O2 -IAssignment/CG2028_Assignment/Core/Inc Part2_Simulation/fall_sim.c \
    Assignment/CG2028_Assignment/Core/Src/fall_detector.c -lm -o fall_sim && ./fall_sim
```

## Demo tips
- Hold or strap the board **upright** (chest-worn pendant) before the "fall";
  posture change is measured against the last stationary posture.
- Drop it in a rotating motion onto a cushion, then leave it still for 2 s.
- Press the blue USER button to acknowledge, or stand it upright for 3 s.
- For the long-lie demo, shorten `LONG_LIE_MS` (e.g. 15000).
