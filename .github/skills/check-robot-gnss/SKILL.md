---
name: check-robot-gnss
description: "Use when the user asks to check / show GNSS data quality, RTK fix status, correction (RTCM) flow / age, satellite count, HDOP, or dual-antenna heading quality on the robot — a quick liveness + quality readout from the dev box (or in-container). Parses raw $GNGGA / $KSXT. DO NOT USE for bringing up the GNSS stack (gnss-bringup), full NTRIP/RTK failure debugging, deploying the image (deploy-to-robot), or VESC tuning."
---

# Check robot GNSS data quality

Quick read of the live GNSS quality: **position fix level, satellites, HDOP,
RTCM correction age, and dual-antenna heading quality**. Works from the dev box
(Cyclone DDS, domain 0, host net → discovers the robot's topics) or, if host
discovery is lost, in-container over `ssh robot`.

Prereq: deploy stack running (see **deploy-to-robot**). Source first:
`cd /workspaces/outdoor-patrol && source install/setup.bash`.

## Quick liveness + quality (run this first)

Captures a few seconds of NMEA to a file, then reads the **last** `$GNGGA`
(position) and `$KSXT` (heading) sentence. Always prints something — a `0` line
count means the topic is silent (driver down or host lost DDS discovery).

```bash
source install/setup.bash 2>/dev/null
timeout 5 ros2 topic echo /um982_driver/nmea_sentence > /tmp/nmea.txt 2>/dev/null
n=$(wc -l < /tmp/nmea.txt); echo "nmea lines in 5s: $n"
[ "$n" -eq 0 ] && echo "  -> topic SILENT (driver down or host lost DDS discovery)"
g=$(grep -oE 'GGA[^*]*' /tmp/nmea.txt | tail -1)
k=$(grep -oE 'KSXT[^*]*' /tmp/nmea.txt | tail -1)
echo "GGA : ${g:-<none>}"
echo "KSXT: ${k:-<none>}"
```

### Labeled version (human-readable)

```bash
source install/setup.bash 2>/dev/null
timeout 6 ros2 topic echo /um982_driver/nmea_sentence > /tmp/nmea.txt 2>/dev/null
echo "== GGA  quality,sats,HDOP,corrAge(s) — last 4 =="
grep -oE 'GGA[^*]*' /tmp/nmea.txt | tail -4 | \
  awk -F, '{printf "  q=%s sats=%s hdop=%s corrAge=%ss\n",$7,$8,$9,$14}'
echo "== KSXT  HdgQual,ANT2,ANT1,hdg =="
grep -oE 'KSXT[^*]*' /tmp/nmea.txt | tail -2 | \
  awk -F, '{printf "  hdgQ=%s ANT2=%s ANT1=%s hdg=%s\n",$12,$13,$14,$6}'
echo "== /rtcm corrections rate =="
timeout 6 ros2 topic hz /rtcm 2>/dev/null | grep -m1 "average rate" || echo "  SILENT (no corrections)"
echo "== /um982_driver/fix status + covariance(x,y) =="
timeout 6 ros2 topic echo --once /um982_driver/fix 2>/dev/null | grep -E 'status:|^- ' | head -3
```

## How to read it

**GGA quality** (field 7) — the position fix level:

| q | Meaning | `/um982_driver/fix` cov (x) |
|---|---|---|
| 0 | invalid | — |
| 1 | single-point (no corrections) | ~1–2 m² (std ~1.3 m) |
| 2 | DGPS/SBAS | sub-m |
| 4 | **RTK FIXED** (cm) | **~0.0004 m² (std 0.02 m)** |
| 5 | RTK float | ~0.04–0.25 m² |

**KSXT HdgQual** (field 12): `0` invalid · `1` single · `2` float · `3` **RTK
fixed heading**. `ANT2`=field 13 (slave sats), `ANT1`=field 14 (master sats);
both should be similar & non-zero, and the heading value should **jitter**
(a frozen exact value = placeholder / dead ANT2).

**Correction age** (`corrAge`, GGA field 14): seconds since the last applied
RTCM. **< ~2 s = healthy.** Growing/large or `/rtcm` SILENT ⇒ corrections aren't
reaching the receiver (NTRIP stalled) → fix decays RTK→float→single.

**Good looks like:** `q=4 sats≈24 hdop<1 corrAge~1s`, `hdgQ=3`, `/rtcm` ~5–7 Hz,
fix `status: 2` with a tiny first covariance term.

## In-container variant (host lost discovery)

Same data straight from the receiver inside the container. **Use `cut`, not
`awk`, inside nested `ssh`→`docker exec`** — `awk`'s `$n` escaping breaks through
the nested quoting.

```bash
ssh robot 'docker exec outdoor-patrol bash -lc "source /opt/outdoor-patrol/install/setup.bash; \
  k=\$(timeout 5 ros2 topic echo /um982_driver/nmea_sentence 2>/dev/null | grep -m1 -oE \"KSXT[^*]*\"); \
  echo HdgQual=\$(echo \$k|cut -d, -f12) ANT2=\$(echo \$k|cut -d, -f13) ANT1=\$(echo \$k|cut -d, -f14); \
  timeout 5 ros2 topic echo /um982_driver/nmea_sentence 2>/dev/null | grep -m1 -oE \"GGA[^*]*\" | cut -d, -f7,8,9,14"'
```

## Gotchas (learned the hard way)

- **Capture to a file with `timeout N`, don't loop `ros2 topic echo --once` with
  a short timeout.** Each fresh `echo` pays a DDS discovery cost (~1–3 s); a
  short per-sample timeout gets killed before it receives a message and prints
  nothing — making a healthy stream look dead. One `timeout 5–8 … > file` is
  reliable.
- A per-cycle monitor printing `no-GGA/no-KSXT` intermittently is a **sampling
  artifact**, not a dropout — confirm with a single file capture (count types:
  `grep -oE '(GGA|KSXT)' /tmp/nmea.txt | sort | uniq -c`).
- `timeout` killing a piped `ros2 topic echo | grep` can **lose buffered output**
  (looks empty). Redirect to a file first, then grep the file.
- Field indexes are 1-based from the sentence tag: GGA `q=7 sats=8 hdop=9
  corrAge=14`; KSXT `hdg=6 PosQual=11 HdgQual=12 ANT2=13 ANT1=14`.

## Related

- Bring up / RTK enable: **gnss-bringup**.
- Heading wrong / ANT2 dead: [doc/eng/wiki/gnss/heading-wrong-ant2-no-signal.md](../../../doc/eng/wiki/gnss/heading-wrong-ant2-no-signal.md).
- RTK won't hold while moving: [doc/eng/wiki/gnss/rtk-fix-hard-to-hold-while-moving.md](../../../doc/eng/wiki/gnss/rtk-fix-hard-to-hold-while-moving.md).
