# Pi container races WiFi at boot → DDS discovery silently fails

- **Date:** 2026-05-16
- **Affects:** Orange Pi 5 (Ubuntu 24.04 + NetworkManager), Docker
  Compose stack `deploy/docker-compose.yaml` with `network_mode: host`
- **Severity:** annoyance (works after manual `docker restart`, but
  surprising on every reboot)

## Symptom

After a fresh `reboot` on the Pi, SSH to the Pi works fine but
`ros2 topic list` on the dev box returns only `/parameter_events` and
`/rosout`. Inside the container, the bringup nodes look healthy.
Running `docker restart outdoor-patrol` makes everything work
immediately and predictably.

## Root cause

Two-layer race:

1. **Docker starts before WiFi has an IP.** On Ubuntu with
   NetworkManager, `network-online.target` does not block on NM by
   default. `docker.service` (`After=network-online.target`) therefore
   starts as soon as the kernel says "any" link exists.
2. **`network_mode: host` + `restart: unless-stopped` then binds the
   ROS process to a half-initialized stack.** DDS/Cyclone picks an
   interface without an IP, discovery never converges, and the
   container does not crash — so the restart policy doesn't help.

## Fix

Two complementary guards. Apply both.

### Host-side: make Docker wait for WiFi

```bash
sudo systemctl enable --now NetworkManager-wait-online.service
```

This causes `network-online.target` to actually block until
NetworkManager reports its active connections are up, so `docker.service`
won't start the engine until the WiFi has an IP.

### Container-side: pre-flight wait in `deploy/entrypoint.sh`

`entrypoint.sh` now waits up to `WAIT_FOR_NETWORK_SECS` (default 30s)
for a default route before `exec`ing the ROS command. If the route
never appears, the entrypoint exits and Compose's
`restart: unless-stopped` retries — better than a stuck process bound
to a dead interface. The runtime image installs `iproute2` so the
`ip route` check is available.

Snippet:

```bash
WAIT_FOR_NETWORK_SECS="${WAIT_FOR_NETWORK_SECS:-30}"
for _ in $(seq 1 "${WAIT_FOR_NETWORK_SECS}"); do
    ip route show default 2>/dev/null | grep -q '^default ' && break
    sleep 1
done
```

## How to verify

After applying both fixes:

```bash
sudo reboot
# wait, SSH back in
docker compose -f ~/code/outdoor-patrol/deploy/docker-compose.yaml logs --tail=20
# Expect: no "no default route" message; agent + RSP up; on dev box:
ros2 topic list   # full list
```

A useful failure injection to confirm the guard works:

```bash
# Temporarily kill WiFi just before container start to trigger the wait
sudo nmcli radio wifi off
docker restart outdoor-patrol
docker logs -f outdoor-patrol   # should show wait, then exit-for-restart
sudo nmcli radio wifi on
# Next restart attempt should succeed.
```

## Related

- [networking/dds-discovery-fails-on-wifi-with-multi-nic-dev-box.md](../networking/dds-discovery-fails-on-wifi-with-multi-nic-dev-box.md)
- `deploy/entrypoint.sh`, `deploy/Dockerfile`, `deploy/docker-compose.yaml`
- Upstream: <https://bugs.launchpad.net/ubuntu/+source/network-manager/+bug/1879338>
  (NM-wait-online behavior discussion)
