# AGENTS.md - SICK_3D_LIDAR

> **Documentation Version**: 1.0  
> **Last Updated**: 2026-11-06
> **Project**: SICK_3D_LIDAR  
> **Description**: Receive and analyze SICK 3D LiDAR raw data, then transform it into vehicle shapes.

This file provides guidance to AI agents working in this repository.

## Critical Rules

### Absolute prohibitions

- Never create files in the repository root unless the project structure requires it to be flat; use the appropriate module structure.
- Never write generated output directly to the repository root; use designated output folders.
- Never create documentation files unless the user explicitly requests them.
- Never use interactive Git commands.
- Never use shell `grep`, `find`, or `cat` to search or read code. Use code search, file listing, and file reading tools instead. One-off inspection with `wc`, `head`, `diff`, or pipelines is permitted.
- Never create duplicate files or multiple implementations of the same concept; extend the existing source of truth.
- Never copy-paste code blocks; extract shared behavior into utilities or functions.
- Never hardcode values that should be configurable; use configuration files or environment variables.
- Do not use names such as `enhanced_`, `improved_`, `new_`, or `v2_`; extend the original implementation.

### Mandatory requirements

- Commit each completed task or stage. Do not push unless requested; auto-push is disabled.
- For operations expected to take longer than 30 seconds, use a task agent when available. Use parallel task agents for independent long-running operations.
- For tasks with three or more steps, track the work, use Git checkpoints, and validate the result.
- Read a file before editing it.
- Before creating a file, search for similar existing functionality and extend it when appropriate.
- Maintain one authoritative implementation for each feature or concept.

## Pre-task compliance checklist

Before starting a task, verify each item:

1. Acknowledge and follow all rules in this file.
2. Check whether the task would add files to the root; use the proper module structure unless a root-level project/config file is necessary.
3. Use a task agent for operations expected to exceed 30 seconds, and track tasks involving three or more steps.
4. Search for existing implementations and read them before creating or editing files; avoid duplicates and multiple sources of truth.
5. Plan context checkpoints for long tasks; consider a break or context compaction after an hour.
6. For text-only changes, review the result. For code or behavior changes, execute the relevant behavior once after completing the task and run available tests.

Do not proceed until these checks have been considered.

## GitHub and backup workflow

Current status: the user created https://github.com/RogerLo-52693448/RogerLo-52693448-SICK_3D_LIDAR. This is the origin remote; auto-push is disabled.

- Commit as usual and push only when the user asks.
- If the user explicitly enables auto-push in a future setup, configure and verify the agreed post-commit behavior before relying on it.
- Useful commands:
  - Check auth: `gh auth status`
  - Check remotes: `git remote -v`
  - Check repository: `gh repo view`
  - Push only when requested: `git push origin main`

## Multi-device workflow

Initialize Git on one device only. After the first push, clone the repository on other devices instead of initializing independent histories. Confirm network and authentication early on work devices, and agree on Git identity before committing.

Before work on a single-user, multi-device project:

```bash
git status
git pull --rebase
```

Resolve conflicts before continuing. For a shared branch with collaborators, do not rebase commits that others may already have pulled; use a coordinated merge workflow instead.

At the end of work, stage only intended files, commit, push only when authorized, and check `git status`.

## Project overview

- **Name**: SICK_3D_LIDAR
- **Purpose**: Receive and analyze SICK 3D LiDAR raw data, then transform the data into vehicle shapes.
- **Type**: AI/ML and data-science project.
- **Primary language**: Python.
- **Source package**: `src/main/python/sick_3d_lidar/`
- **Tests**: `src/test/`
- **Data**: `data/`; raw and processed datasets are ignored by Git.
- **Models and experiments**: `models/` and `experiments/`; generated artifacts are ignored.
- The supplied manual describes multiScan270 and the supplied SDK includes multiScan270 test data. Actual firmware compatibility remains unverified.
- The requested receiver passively captures TCP bytes from `192.168.0.1:2112` by default. This is a CoLa B port, not a verified point-cloud stream. Do not send device commands or change configuration without authorization.
- The user's Windows connection tests show TCP 2112 unavailable and TCP 2115 reachable. Use `--port 2115` for real-device capture. The supplied capture contains CRC-validated Compact type 6/version 1 telegrams.

## Development status

- **Setup**: Project structure and Git history created; origin points to the user-created GitHub repository.
- **Core features**: Passive TCP capture, offline Compact type 6/version 1 conversion, and fixed-sensor live foreground clustering/tracking implemented. Detection assembles complete frames, warms all interlace phases, and saves each confirmed track's highest-point observation after departure. Shutdown exports are explicitly marked partial. No vehicle classification or full shape reconstruction.
- **Dependencies**: NumPy accelerates shared decoding. Install `.[detect]` for SciPy-based detection. Keep raw capture and offline conversion usable without the detection extra.
- **Testing**: CLI, TCP capture, Compact validation, XYZ conversion, packed intensity, PCD, background subtraction, tracking, and local TCP streaming tests included.
- **Documentation**: Project README and agent guidance included.

## Technical debt prevention

Before creating a file:

1. Search for related code and read the matching implementation.
2. Determine whether the change can extend an existing module.
3. If a new file is necessary, explain why and follow the established package layout.
4. Reuse shared utilities rather than duplicating code.
5. Validate that the change adds no duplicate implementation or source of truth.

## Verification

For this AI/ML project, run the training or inference pipeline once with sample data from `data/` after it exists; confirm there are no errors, outputs are reasonable, and expected metrics appear under `experiments/results/`. Do not invent or commit sensor data. Run tests with:

```bash
python -m pip install -e ".[detect]"
PYTHONPATH=src/main/python python -m unittest discover -s src/test
```

For web or frontend functionality added later, start the development server and verify it in a browser.

## Need help?

See README.md for TCP capture usage and limitations. Confirm the actual firmware, scan output configuration, and desired vehicle-shape output before implementing decoding or reconstruction.

## Common commands

```bash
python -m pip install -e .
PYTHONPATH=src/main/python python -m sick_3d_lidar
PYTHONPATH=src/main/python python -m sick_3d_lidar convert output/capture.bin --output-dir output/pcd --allow-truncated-tail
PYTHONPATH=src/main/python python -m sick_3d_lidar detect --host 192.168.0.1 --port 2115
PYTHONPATH=src/main/python python -m unittest discover -s src/test
git status
```
