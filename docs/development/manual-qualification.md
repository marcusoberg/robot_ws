# Qualification status and acceptance

> Historical qualification checklist. References to `archive/`, `artifacts/` and local build reports require the separately preserved original workspace; they are not downloaded by a fresh clone. The main README provides the public ACT walkthrough.

Implementation performs compilation, import/provenance and static validation. The user runs operational qualification. No unit tests are added.

Run each tutorial separately and retain its reports under a new `runs/<name>` directory:

- View/move: disabled startup, visible joints/TF/cameras, explicit enable/source choice, accepted motion and stop.
- Ownership: deadman/clutch HOLD, takeover, producer death/lease expiry, stale/duplicate requests and reset-generation rejection.
- Timing/capture: bounded native fitting compared to the existing SciPy fitter, replacement continuity, late/infeasible rejection, explicit steps and original RGB/depth stamps.
- Record/export: writer acknowledgement and episode boundaries, replayed lifecycle requests, manifest completeness, legacy 8/2 export.
- Inference: finite correctly shaped ACT/SmolVLA predictions, checkpoint identity and measured local round-trip timing; command admission observed separately.
- Optional training/resume: changed parameters beyond dtype conversion, saved optimizer/scheduler, continued step count and new output directory.

The preserved historical report is `artifacts/legacy-20261011/robot-experiments/runs/qualification-20261011/report.json`. Its paths remain historical; use the relocation manifest to find preserved files.

Checkpoint hashes establish preserved identity. Historical 100-step runs establish learning-pipeline execution, not learned task success or language grounding. The original SmolVLA base weights are missing locally. MPS training is unqualified; its resume check establishes optimizer/scheduler continuation, not exact RNG replay.

Physical hardware, physical camera transport, irregular live resampling and the last Linux recording repeat remain unqualified. No migration command connects to Vast or changes instance lifecycle.
