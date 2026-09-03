# NTRIP "handshake rejected (unauthorized)" with correct credentials — disabled device profile

- **Date:** 2026-09-03
- **Affects:** `ntrip_client` (all versions), Point One Navigation
  (`truertk.pointonenav.com`) and any caster that returns HTTP 401; the
  deployed `outdoor-patrol` container
- **Severity:** gotcha (the error names the wrong cause, and two plausible
  debugging paths both dead-end)

## Symptom

The NTRIP client cannot authenticate, retrying forever:

```
[ntrip_client]: NTRIP v2 connecting to truertk.pointonenav.com:2101/AUTO
[ntrip_client]: NTRIP handshake rejected (unauthorized).
[ntrip_client]: Falling back to NTRIP v1.
[ntrip_client]: NTRIP v1 connecting to truertk.pointonenav.com:2101/AUTO
[ntrip_client]: NTRIP handshake rejected (unauthorized).
[ntrip_client]: Reconnecting in 30.0s
```

No RTCM, so the fix stays standalone. `/um982_driver/fix` shows
`status: 0` (`STATUS_FIX`, not GBAS) and metre-class covariance:

```
position_covariance: [0.998001, 0, 0,  0, 2.039184, 0,  0, 0, 4.6656]
                      ^ sigma_east 1.00 m   ^ sigma_north 1.43 m
```

`confidence_gate` needs <= 0.05 m, so the robot will not move at all.

Meanwhile the credentials are demonstrably correct — same username as the
provider's dashboard, and the file on the robot is byte-identical to the one
on the dev box.

## Root cause

**The account existed and the subscription was paid, but the device profile
was disabled in the provider's dashboard.** The caster rejects with HTTP 401,
which is indistinguishable from a wrong password on the wire.

`classify_response()` in
[`ntrip_client_node.cpp`](../../../../src/ntrip_client/src/ntrip_client_node.cpp)
maps any `401` to `"unauthorized"`:

```cpp
if (header.find(" 401 ") != std::string::npos) {return "unauthorized";}
```

That is correct behaviour — 401 genuinely means "not authorized" — but the
word steers you toward credentials, and the credentials are fine. A caster has
several reasons to answer 401:

| Cause | Fixable where |
|---|---|
| Wrong username or password | `ntrip.yaml` |
| Subscription lapsed / unpaid | provider billing |
| **Device profile disabled** | **provider dashboard** |
| Mountpoint not entitled for this account | provider dashboard |
| Too many concurrent connections for the plan | disconnect other clients |

Only the first is in our config, and it is the one that was already correct.

## Two debugging traps on the way there

**1. `NTRIP_PARAMS` is not in the container's environment, and that is
normal.** The obvious check looks damning:

```console
$ docker inspect outdoor-patrol --format '{{range .Config.Env}}{{println .}}{{end}}' | grep -i ntrip
                                                        # ← nothing
```

It is a red herring. Compose substitutes `${NTRIP_PARAMS}` into the
`command:` at container-creation time
([`deploy/docker-compose.yaml`](../../../../deploy/docker-compose.yaml)), so
the value ends up baked into the argv, not exported as a variable. Check the
command instead:

```console
$ docker inspect outdoor-patrol --format '{{join .Config.Cmd " "}}' \
    | tr ' ' '\n' | grep ntrip
ntrip_params_file:=/data/ntrip.yaml       # ← the real answer
```

If that shows `.../ntrip.yaml.example`, the container was started without
`NTRIP_PARAMS` and *is* running the placeholder config. That is a different
bug, and it produces the same 401.

**2. Restarting does not help, and a bare `up -d` can silently make it
worse.** The client already retries every 30 s, so a restart buys nothing a
30-second wait would not. Worse, `docker compose up -d` without the env var
set will recreate the container against the in-image example and *replace*
working credentials with empty ones. If you restart at all:

```bash
# Safe: preserves the existing command, including ntrip_params_file.
ssh robot 'cd ~/code/outdoor-patrol && \
  docker compose -f deploy/docker-compose.yaml restart'

# Also safe: re-states the env var explicitly.
ssh robot 'cd ~/code/outdoor-patrol && NTRIP_PARAMS=/data/ntrip.yaml \
  docker compose -f deploy/docker-compose.yaml up -d'
```

## Fix

Enable the device profile in the provider's dashboard. No restart, no config
change, no redeploy — the client's own 30-second retry picks it up:

```
[ntrip_client]: NTRIP v2 connecting to truertk.pointonenav.com:2101/AUTO
[ntrip_client]: NTRIP stream open (v2_ok, chunked=yes).
```

## How to verify

Three checks, cheapest first. Each one is decisive on its own.

```bash
# 1. Did the handshake actually succeed? Two things make this awkward:
#    navsat_transform logs at ~5 Hz and buries everything, and the client is
#    quiet once connected -- so do NOT use --since, or a healthy stream that
#    connected an hour ago prints nothing and reads like a failure.
ssh robot 'docker compose -f ~/code/outdoor-patrol/deploy/docker-compose.yaml \
  logs 2>/dev/null | grep -i "ntrip_client\]" \
  | grep -viE "type hash|USER_DATA" | tail -3'
# want, as the LAST line: NTRIP stream open (v2_ok, chunked=yes)
# Silence here means no NTRIP client at all, not a healthy one.

# 2. Are corrections reaching the receiver?
ros2 topic hz /rtcm --window 10
# want: a few Hz, steady

# 3. Is the receiver applying them?
ros2 topic echo /um982_driver/fix --once | grep -A2 '^status:'
# want: status: 2 (STATUS_GBAS_FIX). 0 = STATUS_FIX = no corrections.
```

`status: 2` with metre-class covariance is the expected state **indoors**:
corrections are being applied but the sky view is blocked, so the fix cannot
converge. Take the robot outside and watch sigma fall:

```bash
ros2 topic echo /um982_driver/fix --field position_covariance \
  | awk 'NR%9==1{print sqrt($2)" m"}'
```

| Stage | sigma |
|---|---|
| Standalone, no corrections | 1–3 m |
| Corrections applied, poor sky view | 0.3–1 m |
| RTK float | 0.1–0.5 m |
| RTK fixed | 0.01–0.03 m |

`confidence_gate` passes at <= 0.05 m, so only the last row drives.

## Ruling out our side in one pass

If it happens again, this sequence separates "our config" from "their
account" in about a minute:

```bash
# Same file on both ends?
md5sum ntrip.yaml
ssh robot 'md5sum ~/code/outdoor-patrol/deploy/data/ntrip.yaml'

# Container reading the real one, not the example?
ssh robot 'docker inspect outdoor-patrol --format "{{join .Config.Cmd \" \"}}" \
  | tr " " "\n" | grep ntrip'

# Mounted where the command expects?
ssh robot 'docker inspect outdoor-patrol \
  --format "{{range .Mounts}}{{.Source}} -> {{.Destination}}{{println}}{{end}}" \
  | grep data'
```

Hashes equal, command pointing at `/data/ntrip.yaml`, mount present, and
still 401 -> **the problem is on the provider's side**. Stop editing config
and go look at the dashboard: profile enabled, subscription active,
mountpoint entitled, no other client holding the connection slot.

## Related

- [rtk-fix-hard-to-hold-while-moving.md](rtk-fix-hard-to-hold-while-moving.md)
  — the next thing to go wrong once corrections do flow
- [`ntrip.yaml.example`](../../../../src/ntrip_client/config/ntrip.yaml.example)
  — field reference
- [GNSS accuracy budget](../../plans/field-validation-alley.md#gnss-accuracy-budget)
  — why 0.05 m is the threshold that matters
- `.github/skills/gnss-bringup` — full bringup procedure
