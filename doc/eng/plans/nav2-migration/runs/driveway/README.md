# Driveway world — an 18 ft square

A scaled-down loop for driving the stack somewhere real: 18 ft (5.4864 m) on
a side, which is a domestic driveway. **Not a validated configuration and not
part of any phase gate.** The Phase 0 baseline and the Phase 1 parity numbers
are all measured on the 100 m `patrol_road` world with the default configs,
and nothing here changes those.

Its value is as a second data point. Every scale-dependent parameter in the
stack had only ever been exercised at one scale, so "does it work on a road
5.6× smaller" is a real test of which numbers are physics and which were
fitted to the 100 m road.

## Geometry

| | 100 m road | driveway |
|---|---|---|
| lap length | 100 m | 17.7488 m |
| corner radius | 5.0 m | 1.0 m |
| straight | 17.146 m | 2.8664 m |
| corridor half-width | 3.0 m | 0.31 m |

The corridor is set to 0.31 m — exactly the robot's half-width — so the
painted road spans 4.8664 + 2×0.31 = **5.4864 m**, the 18 ft square to the
millimetre. The robot body fills the corridor completely, which is the point:
there is no shoulder, so "inside the corridor" means "on the driveway".

There is no obstacles variant. The stock barrier stations are at s = 8, 46 and
77 m, which do not exist on a 17.7 m lap, and a barrier sized to block a 0.4 m
lane is not a meaningful test.

## Results

`teach` + `r3n`, GUI on, 2026-09-06. Artefacts in this directory.

| | teach route vs truth | Nav2 lap vs truth |
|---|---|---|
| lateral RMS | 0.031 m | **0.057 m** |
| peak | 0.096 m | 0.150 m |
| lap | — | 0.97 |
| longest stop | — | 4.0 s |

0.057 m RMS is **better** than the 100 m road's R3-N (0.0883 m), on corners
five times tighter. The scorer prints FAIL, for two reasons that are both
about the gates rather than the driving:

- **`laps` 0.97 < 0.98.** 0.98 of a 17.7 m lap leaves 0.35 m of slack. The
  final station sits one `station_spacing_m` (0.6 m) before the start point
  and `xy_goal_tolerance` is 0.15 m, so ~0.97 is the arithmetic maximum. The
  threshold is a fraction; the geometry it has to cover is absolute.
- **`longest_stop_s` 4.0 > 3.0.** One chunk-boundary replan plus the trailing
  zero-`/cmd_vel` tail that `score_run.py` counts unwindowed (open risk 2 in
  [progress.md](../../progress.md)). On a 186 s run that is one pause.

Neither is worth "fixing" by raising a limit — see the plan's rule about walls
versus knobs. They are recorded here as evidence that R3's pass criteria are
written for a 100 m lap and do not transfer unmodified.

## What this found

**The teach driver's `lookahead_m` was the dominant error, not the follower.**
The first two runs scored 0.145 m RMS under Nav2 — and 0.147 m for the
recorded route itself, measured against ground truth. Nav2 was adding
essentially nothing; it was faithfully driving a bad route.

Pure pursuit with lookahead `L` on a corner of radius `R` cuts the corner by
about `L²/(8R)`. The default `lookahead_m` is 1.5 m:

| | L | R | predicted cut | measured |
|---|---|---|---|---|
| 100 m road | 1.5 | 5.0 | 0.056 m | 0.028–0.034 m |
| driveway, default | 1.5 | 1.0 | 0.281 m | 0.271 m |
| driveway, `L=0.45` | 0.45 | 1.0 | 0.025 m | 0.031 m |

Re-teaching at 0.15 m/s instead of 0.35 changed nothing (0.147 → 0.146 m),
which is the confirmation that it is geometric and not dynamic. Dropping the
lookahead to 0.45 m took the route to 0.031 m and the Nav2 lap to 0.057 m.

This is the same class of defect as finding 7 in
[progress.md](../../progress.md) — a length that is fine at one corner radius
and silently wrong at another — in a different component. `TEACH_LOOKAHEAD` is
now an environment override in `run_validation.sh`, defaulting to the
unchanged 1.5 m.

**`minimum_turning_radius: 1.5` is not a chassis limit.**
`config/nav2_params.yaml` says it is "the 1.5 m chassis minimum from issue
#8", but issue-8 states no chassis turning limit; its only 5 m radius is the
*world's* corner. `chassis.yaml` describes a 0.545 m-track differential drive
with a caster, which pivots in place. The driveway config uses 0.4 m, and
Smac plans the 1.0 m corners without complaint. Worth revisiting on the 100 m
road: 1.5 m is larger than needed there too, and finding 7 showed Smac
cornering tighter than the road as a result.

## Running it

```bash
WS=$PWD
NAV=$WS/install/share/outdoor_patrol_nav
SIM=$WS/install/share/outdoor_patrol_sim

WORLD=driveway.sdf \
CENTERLINE=$SIM/worlds/driveway_centerline.yaml \
NAV_PARAMS=$NAV/config/nav2_params_driveway.yaml \
MISSION_PARAMS=$NAV/config/patrol_mission_driveway.yaml \
BT_XML=$NAV/bt/patrol_driveway.xml \
START_X=-1.4332 START_Y=-2.4332 START_YAW=0.0 \
TEACH_SPEED=0.35 TEACH_LOOKAHEAD=0.45 \
GUI=1 src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/driveway teach r3n
```

`GUI=1` gives Gazebo and RViz; `GUI=rviz` is much lighter and shows the
corridor markers, which is usually what you want to watch. Headless is
`GUI=0`, and the numbers above are from a GUI run, so treat them as a look
rather than a measurement.

Regenerate the world with:

```bash
cd src/outdoor_patrol_sim && python3 scripts/gen_patrol_road.py \
  --name-prefix driveway --length 17.7488 --corner-radius 1.0 \
  --lane-half-width 0.20 --shoulder-width 0.11 --sample-spacing 0.10
```

then delete `worlds/driveway_obstacles.sdf`, which the generator writes
unconditionally and which is meaningless at this scale.

## Files

```
outdoor_patrol_sim/worlds/driveway.sdf              generated
outdoor_patrol_sim/worlds/driveway_centerline.yaml  generated
outdoor_patrol_nav/config/nav2_params_driveway.yaml
outdoor_patrol_nav/config/patrol_mission_driveway.yaml
outdoor_patrol_nav/bt/patrol_driveway.xml
```

Every one of them is a copy-and-rescale of its 100 m counterpart, with each
changed value carrying the derivation in a comment. When a Phase 2+ change
edits the original, the driveway copy does not follow automatically.
