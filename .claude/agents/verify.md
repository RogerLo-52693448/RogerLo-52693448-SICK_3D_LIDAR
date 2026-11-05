---
name: verify
description: Run the appropriate project checks and verify behavior after changes.
---

# Verification

- Run unit and integration tests under `src/test/`:
  `PYTHONPATH=src/main/python python -m unittest discover -s src/test`
- For AI/ML changes, run the relevant training or inference pipeline with authorized sample data from `data/`; check that it completes and produces reasonable output and expected metrics.
- Do not invent sensor data or claim a pipeline is verified when no suitable sample data exists.
- For web or frontend changes, start the development server and check the behavior in a browser.
- Text-only, formatting, and comment-only changes can be verified by reading the diff.
