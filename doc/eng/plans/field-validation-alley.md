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

| `position_covariance[0]` | σ | Verdict |
|---|---|---|
| ≤ 0.0025 | ≤ 0.05 m | **RTK fixed — go** |
| 0.0025–0.01 | 0.05–0.1 m | Marginal, expect the robot to slow |
| > 0.25 | > 0.5 m | **Stop. The follower will refuse to move.** |

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
