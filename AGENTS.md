# AGENTS.md - SICK_3D_LIDAR

> **Documentation Version**: 1.0  
> **Last Updated**: 2026-11-05  
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

- Commit each completed task or stage. Do not push unless requested; this project currently has no GitHub remote and auto-push is disabled.
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

Current status: this project is local-only; repository creation was denied by the current GitHub App permissions. No remote or auto-push hook is configured.

- If a GitHub remote is connected later and auto-push remains disabled, commit as usual and push only when the user asks.
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
- The exact sensor model, raw-data format, and vehicle-shape representation have not yet been specified. Do not assume formats or fabricate sample measurements.

## Development status

- **Setup**: Local project structure and initial Git commit created; GitHub remote unavailable in this session.
- **Core features**: Not implemented; input format and shape representation need to be established first.
- **Testing**: Unit-test skeleton included.
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
PYTHONPATH=src/main/python python -m unittest discover -s src/test
```

For web or frontend functionality added later, start the development server and verify it in a browser.

## Need help?

Project-specific documentation links have not been added yet. Ask the user to confirm the SICK sensor model, raw-data format, and desired vehicle-shape output before implementing ingestion or reconstruction.

## Common commands

```bash
python -m pip install -e .
PYTHONPATH=src/main/python python -m sick_3d_lidar
PYTHONPATH=src/main/python python -m unittest discover -s src/test
git status
```
