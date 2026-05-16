# Engineering Wiki

Field notes from building the outdoor-patrol robot. One article per
problem we hit and how we fixed it — so the next person (often
future-us) doesn't burn an evening re-deriving the same workaround.

## Scope

- **Lessons learned**, not tutorials. Assume the reader already knows
  ROS 2 basics; explain only the non-obvious bits.
- **Concrete and dated.** Mention the exact tool versions, hardware,
  and surrounding context so we can tell when an article goes stale.
- **Decisions go in `recipe/`, not here.** Architecture/ADR-style
  records live with the research notes under
  [`src/robot-research/notes/outdoor-patrol/recipe/`](../../../src/robot-research/notes/outdoor-patrol/recipe/).
  This wiki is for *operational* knowledge: "thing broke, here's why,
  here's the fix."

## Structure

```
doc/eng/wiki/
├── README.md                 ← this file (index)
├── networking/               ← DDS, discovery, multi-NIC, WiFi
├── ros2/                     ← launch, RMW, rclpy/rclcpp gotchas
├── micro_ros/                ← agent, transports, firmware bridge
├── hardware/                 ← ESP32, motors, sensors, USB enumeration
├── deployment/               ← Docker, compose, Pi bringup, systemd
├── devcontainer/             ← VS Code dev container quirks
└── build/                    ← colcon, rosdep, CMake, submodules
```

Create a category folder the first time you have an article for it.
Don't pre-create empty folders.

## Article template

Each article is a single Markdown file, kebab-case filename, with this
shape:

```markdown
# <Short, specific title>

- **Date:** YYYY-MM-DD
- **Affects:** <component(s), version(s)>
- **Severity:** blocker | annoyance | gotcha

## Symptom
What you saw (error message, observed behavior). Paste exact log lines.

## Root cause
Why it happens. Link upstream issues / docs if relevant.

## Fix
The minimum change that made it work. Code/diff/commands.

## How to verify
A repeatable check that proves the fix held.

## Related
Other articles, ADRs, commits.
```

## Index

### networking
- [dds-discovery-fails-on-wifi-with-multi-nic-dev-box.md](networking/dds-discovery-fails-on-wifi-with-multi-nic-dev-box.md)
