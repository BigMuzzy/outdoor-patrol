# Field validation: teach-and-repeat in a 4 m × 30 m alley

**Site:** back alley, ~4 m wide, ~30 m long, walls both sides.
**Goal:** prove the teach-and-repeat stack works outdoors, on real RTK.
**Total time:** about 3 hours including setup. Two people is much easier than one.

Simulation results and design rationale:
[issue-8-teach-and-repeat.md](./issue-8-teach-and-repeat.md).

---

## Do this first

Before you go outside, from the dev box:

```bash
scp src/outdoor_patrol_route/config/route_alley.yaml \
    robot:~/code/outdoor-patrol/deploy/data/route_alley.yaml
```

That file is the whole reason this is safe. **Do not use the default
`route.yaml` in the alley** — it allows a 2.4 m lateral offset, which puts the
robot's outer edge 2.70 m from the centerline. Your walls are at 2.00 m. It
would drive into one.

`route_alley.yaml` caps the offset at 1.2 m, leaving 0.50 m of wall gap.

---

## The alley, in numbers

Everything below assumes you drive down the **middle** of the alley. Measure
your alley before trusting these.

```
   wall                    centerline                     wall
  -2.00 ─────────────────────── 0.00 ─────────────────────── +2.00
          ◄── retreat to -1.2 ──┤
        robot edge at -1.50, so 0.50 m of wall left
```

| Quantity | Value | Where from |
|---|---|---|
| Alley half-width | 2.00 m | your site |
| Robot width | 0.605 m | `chassis.yaml` |
| `corridor_half_width_m` | **1.80 m** | `route_alley.yaml` |
| Reachable offsets | 0, −0.6, −1.2 m | derived at start-up |
| Wall gap at full retreat | 0.50 m | 2.00 − 1.20 − 0.30 |
| Retreat needs | 2 m ramp + ~3 m lag = 5 m | 8 m trigger, 3 m spare |

Both numbers were checked against the real node, not just arithmetic: with
`route_alley.yaml` the follower reports `1.20 m of offset ... 6.00 m margin`,
where the sim default reports `2.40 m`.

If your alley is not 4.0 m, re-derive before driving:

```
corridor_half_width_m  ≤  (width / 2) − 0.30 − 0.45
```

The follower prints what it worked out at start-up. Read that line every time:

```
retreat geometry: 1.20 m of offset needs 2.00 m of travel,
                  seen 8.00 m ahead (6.00 m margin)
```

A negative margin means it will stall against the first obstacle it meets. It
logs an error in that case, but check anyway.

---

## Before you drive

1. **Kill switch works.** Test it with the wheels off the ground. Non-negotiable.
2. **Alley is clear** — no people, pets, cars, bins you did not put there.
3. **Phone has the robot's IP**, and you can `ssh robot` from wherever you'll stand.
4. **Battery above 50%.** The full procedure is ~40 minutes of driving.
5. **You can reach the robot** at every point of the route. 30 m is a jog.

---

## Phase 0 — Deploy and bring up (20 min)

```bash
# Dev box: push the code the robot will run
git push origin main

# Robot: pull, build, start
ssh robot 'cd ~/code/outdoor-patrol && git pull --ff-only && \
  git submodule update --init --recursive && \
  docker compose -f deploy/docker-compose.yaml build && \
  NTRIP_PARAMS=/data/ntrip.yaml docker compose -f deploy/docker-compose.yaml up -d'

ssh robot 'docker ps'
```

**Gate:** container `Up`, no restart loop.

---

## GNSS accuracy budget

**Check your correction service's spec against this before you drive.** The
stack has a hard threshold at 5 cm, and a provider quoting "3-7 cm" straddles
it.

`confidence_gate` compares the fix's horizontal 1-sigma against
`max_horizontal_sigma_m` (0.05 m in
[confidence_gate.yaml](../../../src/outdoor_patrol_loc/config/confidence_gate.yaml)).
Anything above it is republished with covariance multiplied by
`covariance_inflation` (1000).

| Reported sigma | Gate | Follower sees | Behaviour |
|---|---|---|---|
| 3 cm | pass | 0.03 m | full speed |
| 4 cm | pass | 0.04 m | full speed |
| 5 cm | pass | 0.05 m | full speed |
| **6 cm** | degraded | **1.90 m** | **dead stop** |
| 7 cm | degraded | 2.21 m | dead stop |

**There is no middle.** The x1000 inflation jumps clean over the follower's
own slow band (`sigma_slow_m` 0.10 -> `sigma_stop_m` 0.50): 0.06 x sqrt(1000)
= 1.90 m, nearly 4x past the stop threshold. So the intent -- degrade
gracefully as the fix worsens -- and the implementation -- binary cliff -- do
not currently agree. A 4 cm fix drives at full speed; a 6 cm fix does not move.

### Before changing anything, settle two questions

1. **Is the provider quoting 1-sigma, or CEP/95%?** This matters more than the
   numbers do. A 95% figure is roughly 2-sigma, so "3-7 cm at 95%" is about
   1.5-3.5 cm 1-sigma and passes comfortably. Quoted as 1-sigma, the top half
   of that range parks the robot. The stack compares against 1-sigma.
2. **Is the correction source a physical base or a VRS?** This decides
   whether baseline distance means anything at all. With a *physical* base,
   RTK error grows roughly 1 cm per 10 km of baseline, so a "3-7 cm" spec is
   usually 3 cm near the base and 7 cm at the edge of coverage. With a
   **virtual reference station** the baseline is ~0 by construction -- the
   caster synthesises a base beside you from your uploaded GGA -- so that
   rule does not apply and the error is set by the density and quality of the
   provider's physical network instead.

   This site uses a VRS (Point One `AUTO` mountpoint), measured at a 99 m
   baseline. See [Which base station, and how far?](#which-base-station-and-how-far)
   for how that was determined and what it implies.

### Which base station, and how far?

**This site: a virtual base 99 m away, station ID 4053.** Which means the
baseline question has a different answer than it would for a physical base --
see below.

Two independent sources, neither needing extra tooling.

**GGA already carries the station ID and correction age.** The driver parses
both, and the raw sentence is on `/um982_driver/nmea_sentence`:

```bash
ros2 topic echo /um982_driver/nmea_sentence --field sentence | grep -m3 GGA
```

```
$GNGGA,025453.40,4732.58913274,N,12152.82201833,W,5,30,0.5,250.6193,M,-21.0746,M,1.4,4053*6A
                                                        ^  ^   ^                    ^   ^
                                                  quality  |  HDOP        corr. age  station
                                                        sats
```

Quality `5` is RTK float, `4` is RTK fixed, `1` is standalone. Correction age
should stay a few seconds; tens of seconds means the stream has stalled even
though the socket is still open.

**RTCM 1005/1006 carries the base position.** That is the only way to get an
actual distance. Decode it off `/rtcm`:

```bash
python3 src/outdoor_patrol_loc/scripts/rtcm_base_info.py
```

Measured here:

| | |
|---|---|
| Station ID | 4053 |
| Base LLA | 47.5440000, -121.8800000, 250.00 m |
| Rover | 47.5431488, -121.8803670, 250.6 m |
| **Baseline** | **0.099 km** |
| Antenna descriptor (1033) | `ADVNULLANTENNA NONE` |
| Message types | 1005, 1033, 1074, 1084, 1094, 1124, 1230 |

**That base is virtual, and three things say so.** The coordinates are exactly
round to every decimal place -- a surveyed monument has arbitrary decimals.
`ADVNULLANTENNA` is the conventional "no physical antenna" descriptor. And the
position sits ~99 m from the rover, which is where a caster puts a synthesised
base: right next to you.

### What a VRS changes

**The 1 cm per 10 km baseline rule does not apply.** The 99 m figure is an
artefact of how the VRS is generated, not a measure of correction quality. Do
not read it as "we are 99 m from the reference network and therefore have
excellent corrections" -- the real physical stations are tens of kilometres
away, and the interpolation between them is what sets the error.

Practical consequences:

- **`send_gga: true` is load-bearing, not optional.** The caster needs your
  position to synthesise the base. Stop the GGA upload and corrections either
  stop or silently go stale. This is why `mountpoint: "AUTO"` and
  `send_gga: true` are paired in
  [`ntrip.yaml.example`](../../../src/ntrip_client/config/ntrip.yaml.example).
- **The base position moves when you do**, in jumps, as the caster
  re-synthesises. Irrelevant over a 30 m alley; worth knowing before a long
  traverse.
- **Provider accuracy specs describe network quality**, not your baseline. So
  the "3-7 cm" range is about where you sit in their coverage, and cannot be
  narrowed by measuring distance to the virtual base.
- **A VRS is normally *better* than a distant physical base**, because the
  network models the atmosphere across several stations rather than
  extrapolating from one. Expect the good end of the spec, not the bad end --
  but confirm it with a soak rather than assuming it.

### Measured at this site, 2026-09-03

A 20.5-minute soak at RTK-fixed (GGA quality 4, 26-30 satellites, HDOP 0.6,
corrections 0.6-1.4 s old). Raw report:
[runs/gnss/soak_day1_report.txt](../../../runs/gnss/soak_day1_report.txt).

| | |
|---|---|
| Receiver-reported sigma | **0.017 m** median, 0.021 m worst |
| Actual scatter | 0.006 m east, **0.017 m** north, 0.019 m up |
| Radial | CEP 0.014 m, R95 0.033 m, max 0.051 m |
| Reported vs actual | ratio **0.97** |
| Decorrelation time | 239 s |

**The go/no-go: PASSES.** Worst reported sigma 0.021 m against a 0.05 m gate,
so the robot drives with better than 2x margin. Nothing in
`confidence_gate` or `route_alley.yaml` needs changing.

**The receiver is honest.** It claims 0.017 m and scatters 0.017 m -- ratio
0.97. That matters because the gate trusts the claim: a receiver that
under-reported would pass fixes the gate should stop, and this one does not.

**The provider's "3-7 cm" is a 95%-class figure, not 1-sigma.** A 1-sigma
reading would imply a 0.017 m spec, well under the quoted 3 cm floor; R95
(0.041 m) and 2DRMS (0.047 m) both land inside 3-7 cm. So the honest 1-sigma
accuracy here is roughly 1.5-3 cm -- the good end, which is what a VRS on a
dense network should give. **The earlier worry that 6-7 cm would park the
robot does not apply at this site.** Leave `covariance_inflation` at 1000 and
`max_horizontal_sigma_m` at 0.05.

**Two caveats, both real:**

- **Only ~5 independent samples.** 6155 fixes at 5 Hz sounds like a lot, but
  the error decorrelates over 239 s, so 20 minutes buys about five. Hence the
  wide interval on that sigma (95% CI 0.005-0.028 m). It is enough to settle
  1-sigma vs 95%, which is a 2.4x question; it is not a precise sigma.
- **This is precision, not accuracy.** One session cannot see a slowly-varying
  bias -- and with a 239 s decorrelation time there is clearly one present.
  Occupy the same mark on another day to measure it:

  ```bash
  analyse_gnss_soak.py --bag runs/gnss/soak_day1 --bag runs/gnss/soak_day2
  ```

Note the 3:1 north/east anisotropy (0.017 vs 0.006 m). Normal for
mid-latitude sites -- satellite geometry is poorer north-south -- and the
reason `confidence_gate` tests `max(cov[0], cov[4])` rather than averaging
the two.

### Measuring it, and what the measurement can settle

**Short answer: a measurement settles the question that matters, and only
partly settles the one you asked.**

The gate never sees the provider's datasheet. It tests the sigma the
*receiver* reports, so "will it drive here" is directly measurable and is the
real go/no-go. Whether the provider meant 1-sigma or 95% is a secondary
curiosity.

Record a static soak -- robot parked, not moving, on a mark you can find
again:

```bash
ssh robot 'ros2 bag record -o /data/soak_day1 /um982_driver/fix'   # 15+ min
scp -r robot:~/code/outdoor-patrol/deploy/data/soak_day1 /tmp/
python3 src/outdoor_patrol_loc/scripts/analyse_gnss_soak.py --bag /tmp/soak_day1
```

Record the **raw** `/um982_driver/fix`, not `/gnss/fix_gated`: the gate
inflates covariance x1000, which would corrupt the very number being measured.

What it can and cannot conclude:

| Question | Answerable? |
|---|---|
| Will the stack drive here? | **Yes** — directly, from the reported sigma |
| Is the receiver's sigma honest? | **Yes** — reported vs actual scatter |
| Is the spec 1-sigma or 95%? | **Probably** — they differ by 2.4x |
| Is it 1-sigma or CEP? | **No** — only 1.18x apart, inside site variation |
| What is the true accuracy? | **Not from one session** — see below |

Two traps the script is built to avoid, both of which make a naive soak look
better than it is:

**Samples are not independent.** At 5 Hz a 30-minute soak is 9000 samples, but
GNSS error decorrelates over roughly a minute, so it is worth about 30
independent ones. The script estimates the autocorrelation time and widens its
confidence interval to match. A tight sigma from a short soak is an
impression, not a measurement.

**One session measures precision, not accuracy.** Scatter is computed about
that session's own mean, so a slowly-varying bias -- for RTK, several cm from
multipath geometry and baseline drift -- is invisible: it looks like a
constant, not like noise. Providers normally quote accuracy against truth, so
comparing your session scatter to their number flatters them.

The fix costs nothing but a second trip. Mark the spot, occupy it again on
another day at a different time so the satellite geometry differs, and pass
both:

```bash
analyse_gnss_soak.py --bag /tmp/soak_day1 --bag /tmp/soak_day2
```

The spread of the session *means* is the part a single session cannot show
you, and for RTK it is usually the larger number. If you have a surveyed
benchmark, `--truth EAST NORTH` gives real accuracy instead of a proxy.

The script self-verifies against synthetic data with known sigma, known
correlation time and a known injected bias:

```bash
python3 src/outdoor_patrol_loc/scripts/analyse_gnss_soak.py --self-test
```

Confirm what the receiver actually reports, once, before trusting either
answer:

```bash
ssh robot 'ros2 topic echo /um982_driver/fix --once' | tail -12
```

`position_covariance_type: 2` means the numbers came from the receiver's own
GST-measured sigma -- the honest value. Type 1 means the driver fell back to
`quality_to_sigma_m(fix) x HDOP` (0.02 m per HDOP when RTK-fixed,
[um982_driver_node.cpp](../../../src/um982_driver/src/um982_driver_node.cpp)),
which is an estimate, and on that path HDOP >= 3.0 alone trips the gate.

### If the site really does deliver 6-7 cm

Two honest options. **Neither is "raise `sigma_stop_m`"** -- that threshold is
what stops the robot driving on a fix it cannot trust, and moving it removes
the protection rather than fixing the problem.

- **Reduce `covariance_inflation` from 1000 to ~25.** This is the better fix:
  it makes a degraded fix land *inside* the follower's slow band instead of
  leaping over it, which is what the design intended. 0.06 x sqrt(25) = 0.30 m
  -> the follower slows to about 50% rather than stopping.
- **Raise `max_horizontal_sigma_m` from 0.05 to 0.08.** Accepts the provider's
  full range at face value, at the cost of feeding up to 8 cm of position
  error to the EKF. Affordable in the alley specifically -- there is 0.50 m of
  wall clearance at full retreat -- but it widens the corridor error budget
  everywhere else too.

Whichever you pick, re-run the sim at that accuracy before going out. The
harness takes the sigma as a knob, so this is a measurement rather than an
argument:

```bash
GNSS_SIGMA=0.07 ./src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/val r3
```


## Phase 1 — GNSS soak, the go/no-go (15 min)

**This is the phase most likely to end the day, so do it before anything else.**

An alley is the worst case for RTK. Walls cut sky view and reflect signals
(multipath). The stack is built to refuse to drive on a bad fix, so if the
alley cannot hold an RTK fix, the robot will simply stop and there is nothing
to tune. Better to learn that in 15 minutes than after an hour of setup.

Park the robot in the **middle of the alley, halfway along**, and watch:

```bash
ssh robot 'ros2 topic echo /gnss/fix_gated --once'
```

Then leave this running for a full 10 minutes:

```bash
ssh robot "ros2 topic echo /gnss/fix_gated --field position_covariance" \
  | awk 'NR%20==1'
```

| `position_covariance[0]` | σ | What the stack does |
|---|---|---|
| ≤ 0.0025 | ≤ 5 cm | Passes the gate — **full speed** |
| > 0.0025 | > 5 cm | Gate inflates ×1000 → follower sees ~2 m → **dead stop** |

There is no middle. The gate is a hard cliff at 5 cm, and the ×1000 inflation
jumps clean over the follower's entire slow band (10–50 cm). A 4 cm fix drives
at full speed; a 6 cm fix does not move at all. Read [GNSS accuracy
budget](#gnss-accuracy-budget) before you go out — if your correction service
delivers 6–7 cm, this phase fails and nothing later runs.

**Gate: σ ≤ 0.05 m, held for 10 minutes, with no dropouts.**

If it fails: the alley is not a viable site for GNSS-only navigation. That is a
real finding, not a setback — record it and pick a more open site. Do not raise
`sigma_stop_m` to make the robot move. That threshold is what stops it driving
on a fix it cannot trust.

---

## Phase 2 — Verify heading, the silent killer (20 min)

`yaw_offset` in `heading_to_imu.yaml` is set to **−π/2**, and its own comment
says *"FINALIZE empirically at first bringup"*. It is a reasoned guess, not a
measurement.

If it is 180° wrong — ANT1 and ANT2 wired opposite to what was assumed — the
robot believes it is facing backwards. It will not fail loudly. It will drive
away from the route.

**Test, wheels ON the ground, hand on the kill switch:**

1. Point the robot's nose **along the alley**, in the direction you'll teach.
2. Read the heading:
   ```bash
   ssh robot 'ros2 topic echo /gnss/heading --once'
   ```
3. Convert `orientation.z`/`.w` to a yaw:
   ```bash
   python3 -c "import math,sys; z,w=map(float,sys.argv[1:3]); \
     print('yaw = %.1f deg' % math.degrees(2*math.atan2(z,w)))" <z> <w>
   ```
4. Compare with reality. In ENU: **0° = East, 90° = North.** Use a phone
   compass, and remember phone compasses read magnetic — subtract your local
   declination to get true.
5. Drive forward 2 m on teleop and confirm `/odometry/global` x/y moves in the
   direction the yaw says it should.

**Gate: reported yaw within ~10° of the true nose direction, and the robot
moves the way it claims.**

If it is 180° out, flip the sign in `heading_to_imu.yaml`
(`+1.5707963267948966`), redeploy, and repeat. Do not continue until this
passes — every later phase depends on it.

---

## Phase 3 — Teach pass (20 min)

Record all three sources at once. The third one is uncorrected on purpose: it
is the control for Phase 4.

```bash
ssh robot 'ros2 run outdoor_patrol_route route_recorder --ros-args \
  --params-file /data/route_alley.yaml \
  -r __node:=rec_corrected \
  -p source:=odometry_global \
  -p output_path:=/data/alley_corrected.yaml' &

ssh robot 'ros2 run outdoor_patrol_route route_recorder --ros-args \
  --params-file /data/route_alley.yaml \
  -r __node:=rec_control \
  -p source:=raw_antenna \
  -p output_path:=/data/alley_control.yaml' &
```

`--params-file` must come **before** the `-p` overrides, or the file wins and
you get the wrong source. Confirm the settings actually landed before you
drive — this is silent when it goes wrong:

```bash
ssh robot 'ros2 param get /rec_corrected lane_half_width_m'   # 1.0, not 2.0
ssh robot 'ros2 param get /rec_corrected loop'                # False
```

Then **drive the alley on teleop**, down the middle, one end to the other:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_raw
```

Drive it the way you want it repeated:

- Walking pace, ~0.4 m/s. Smooth. No stopping, no reversing.
- Straight down the middle — this line becomes the centerline.
- Go the full 30 m, then **stop and leave the robot there.**

Save:

```bash
ssh robot 'ros2 service call /rec_corrected/save std_srvs/srv/Trigger'
ssh robot 'ros2 service call /rec_control/save std_srvs/srv/Trigger'
```

**Gate:** ~30 samples, `loop: false`, `worst fix: fixed`.

The recorder measures loop closure and writes `loop: false` itself for a
there-and-back route, which is correct here — do not override it.

---

## Phase 4 — Prove the base_link correction, outdoors (10 min)

In simulation this is scored against ground truth. There is no ground truth in
an alley, so the two recordings are compared **against each other** instead.
The uncorrected one should sit 0.42 m to the *right* of the corrected one,
because that is where the antenna is bolted.

```bash
scp robot:~/code/outdoor-patrol/deploy/data/alley_*.yaml /tmp/
python3 src/outdoor_patrol_route/scripts/score_route.py \
    --compare /tmp/alley_corrected.yaml /tmp/alley_control.yaml
```

Expect:

```
the control track sits -0.417 m from the corrected one
PASS: the two recordings differ by the antenna offset, in the right
      direction, so the base_link correction is live.
```

The sign is the real content. A separation of the right size but the wrong
sign means the lever arm is being added instead of subtracted — the check
catches that, and a magnitude-only comparison would not.

**Gate: PASS.** If it fails, stop. Every later phase is built on this
recording being where the robot actually was.

---

## Phase 5 — First autonomous run (20 min)

**Clear alley. No obstacle yet.** Carry the kill switch and walk alongside.

Drive the robot back to the start on teleop, then:

```bash
ssh robot 'ros2 launch outdoor_patrol_route route_follow.launch.py \
  params_file:=/data/route_alley.yaml \
  route_path:=/data/alley_corrected.yaml \
  nominal_speed_ms:=0.4'
```

Watch from the dev box:

```bash
ros2 topic echo /route_follower/status
```

**Abort immediately if any of these:**

- The robot heads for a wall
- `cross_track` exceeds 0.5 m
- `state` sticks on `blocked` with nothing in front of it
- It moves off before you have read one status message

Expect `state: driving`, `d_cmd: 0.0`, `cross_track` within ±0.2 m.

**Gate: drives the alley end to end, stops at the far end, never closer than
1 m to either wall.**

Optional, from the dev box, if you want to see it:

```bash
ros2 launch outdoor_patrol_bringup rviz.launch.py \
    rviz_config:=$PWD/src/outdoor_patrol_route/config/route.rviz
```

Green line is the route, orange lines are the corridor edges, the orange
sphere is the live look-ahead point.

---

## Phase 6 — The obstacle (25 min)

**Obstacle spec — this size is not arbitrary.** It has to block the lane *and*
the left side so that only a full retreat gets past. Anything narrower and the
robot squeaks by at −0.6 m and the test proves less than you think.

| Property | Value |
|---|---|
| Width across the alley | **2.4 m** |
| Placed against | the **left** wall (as the robot drives) |
| Leaves clear | 1.6 m on the right |
| Height | above 0.20 m — the lidar sits low |
| Made of | something soft. Cardboard box, laundry basket. **Not** concrete. |
| Position along alley | ~18 m from the start |

At 18 m the robot gets a 10 m run-up (it needs 5 m) and 12 m afterwards to
prove it returns to the lane.

What the follower will do, and why:

| Offset | Verdict | Clearance |
|---|---|---|
| 0.0 m | blocked | — |
| −0.6 m | blocked | — |
| **−1.2 m** | **clear** | 0.50 m to obstacle, 0.50 m to wall |

Run it exactly as Phase 5. Expected sequence:

1. `driving`, `d_cmd: 0.0` — approaching
2. `retreating`, `d_cmd` ramping 0 → −1.2 — should begin ~10 m out
3. `retreating`, `d_cmd: -1.2` — passing the obstacle
4. `resuming`, `d_cmd` returning to 0 — within ~10 m past
5. `driving` — done

**Gate:**

- Never touches the obstacle or a wall
- `d_cmd` **never goes positive** — it must pass on the right
- Back to `|d_cmd| < 0.2` within 10 m of clearing
- Never stationary more than 3 s

**If it stops and stays stopped** — that is the designed fallback when nothing
is clear, not a crash. Check `blocked` in the status message. Most likely
cause in a narrow alley is both walls reading as blocked; note the numbers and
stop for the day.

---

## Phase 7 — GNSS fault path (10 min, optional)

Only if 1–6 all passed and you have battery left.

Start a run as in Phase 5, and halfway along, block sky view — a metal bucket
over the antenna, or walk it under a fire escape.

**Expect: the robot slows, then stops.** It should not carry on at speed on a
degraded fix.

---

## Bring home

Copy everything off the robot:

```bash
scp robot:~/code/outdoor-patrol/deploy/data/alley_*.yaml ./runs/
```

Record for each phase: pass/fail, the σ you saw, anything surprising. A phase
that failed for an understood reason is a result. A phase that passed and you
do not know why is not.

---

## Quick reference

| Situation | Do this |
|---|---|
| Robot moving unexpectedly | **Kill switch.** Ask afterwards. |
| Won't start moving | Check σ — it is probably refusing a bad fix |
| Drives away from the route | `yaw_offset` is wrong. Back to Phase 2. |
| Stops in front of the obstacle | Read `blocked` in the status message |
| Hugs one wall | Check `cross_track`; the teach pass may not have been central |
| Everything looks wrong | `docker compose ... logs --tail=40` |

**Never** raise `sigma_stop_m`, `corridor_half_width_m`, or
`nominal_speed_ms` to make a phase pass. Each one is a wall the robot is not
supposed to drive through.
