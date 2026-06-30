---
name: connect-to-robot
description: "Use when the user asks to connect to / log into / SSH into the robot (Orange Pi 5), open a shell on the robot, or run a one-off command on the robot. Primary entry point is `ssh robot` (a passwordless key-auth alias in ~/.ssh/config). Also covers first-time SSH key setup and the persistent ~/.ssh dev-container mount. DO NOT USE for ROS 2 topic/DDS introspection from the dev box, or for building/pushing the deploy container image."
---

# Connect to the robot

The robot is an **Orange Pi 5** on the LAN. From the dev container (or the host),
the one command you need is:

```bash
ssh robot
```

`robot` is an alias defined in `~/.ssh/config` that maps to the robot's address,
login user, and key. It uses **SSH public-key authentication**, so there is no
password prompt once setup is done.

Run a single command without an interactive shell:

```bash
ssh robot 'docker ps'
ssh robot 'docker compose -f ~/code/outdoor-patrol/deploy/docker-compose.yaml logs --tail=20'
```

Copy files to/from the robot:

```bash
scp ./file.txt robot:~/            # to the robot
scp robot:~/file.txt ./           # from the robot
```

## How persistence works (dev container)

`~/.ssh` inside the dev container is **bind-mounted from the host** (see the
`mounts` entry in [.devcontainer/devcontainer.json](../../../.devcontainer/devcontainer.json)).
The key and `~/.ssh/config` live on the host, so they **survive a
"Dev Containers: Rebuild Container"**. Do not regenerate the key on every
rebuild — it is already there.

## First-time setup (only if `ssh robot` fails with "Could not resolve hostname robot" or asks for a password)

Run these in the dev-container terminal. Replace `<user>` and `<robot-host>`
(e.g. `ros` and `192.168.1.50` or `orangepi.local`):

```bash
# 1) Generate a dedicated key once (no passphrase = non-interactive)
test -f ~/.ssh/id_ed25519 || ssh-keygen -t ed25519 -C "outdoor-patrol-devcontainer" -f ~/.ssh/id_ed25519 -N ""

# 2) Install the PUBLIC key on the robot — type the robot password ONCE
ssh-copy-id -i ~/.ssh/id_ed25519.pub <user>@<robot-host>

# 3) Add the `robot` alias (skip if it already exists in ~/.ssh/config)
cat >> ~/.ssh/config <<'EOF'

Host robot
    HostName <robot-host>
    User <user>
    IdentityFile ~/.ssh/id_ed25519
EOF
chmod 600 ~/.ssh/config

# 4) Verify
ssh robot
```

If the dev container was just configured for the persistent mount and the bind
mount is not active yet, **Rebuild Container first**, then do the steps above so
the key lands in the host-backed `~/.ssh`.

## Troubleshooting

- **Still prompted for a password** — the public key isn't installed on the
  robot. Re-run step 2 (`ssh-copy-id`). Confirm `~/.ssh/authorized_keys` on the
  robot contains the line from `~/.ssh/id_ed25519.pub`.
- **`Could not resolve hostname robot`** — the `~/.ssh/config` alias is missing
  (e.g. fresh container before rebuild, or `.ssh` not mounted). Re-add it
  (step 3) or `ssh <user>@<robot-host>` directly.
- **`Permissions ... are too open` / `bad ownership`** — host-side `~/.ssh`
  perms/UID don't match the container `ros` user (UID 1000). On the host:
  `chmod 700 ~/.ssh && chmod 600 ~/.ssh/id_ed25519 ~/.ssh/config`.
- **Connection times out** — robot off the network. The Pi races WiFi at boot;
  see [doc/eng/wiki/deployment/pi-container-races-wifi-at-boot.md](../../../doc/eng/wiki/deployment/pi-container-races-wifi-at-boot.md).
- **Host key changed warning** — only if the Pi was re-imaged. Remove the stale
  entry: `ssh-keygen -R <robot-host>` (and `ssh-keygen -R robot`).
