# CLAUDE.md — data_generation

## What This Repo Does

This repo generates the training dataset for a MASc thesis investigating [ADD: thesis goal — e.g., disturbance estimation / motion error prediction] on a Hurco CNC machine using LSTM neural networks. It is a C++ application that runs batch experiments: for each experiment it generates a G-code program with parameterized geometry, synthesizes realistic kinematic noise and vibration feedforward (VFF) signals, injects those signals into the machine's shared-memory ring (SMR) buffer, executes the program on the physical CNC via a Python REST API wrapper, and simultaneously extracts the resulting real-time motion data (commanded position, tracking error, encoder error) into CSV files. The output — one session folder per experiment, each containing several 8000-point CSV slices — is the raw dataset consumed by the LSTM training pipeline elsewhere in the thesis hub.

---

## Repo Layout

```
data_generation/
├── src/core/          # All C++ implementations
├── include/core/      # All C++ headers (mirrors src/core/ structure)
│   └── archive/       # Deprecated G-code parser & kinematics (do not use)
├── python/            # Python subprocess that talks to the CNC REST API
├── main_g_code_generation.cpp   # Standalone G-code batch utility (no CNC)
├── data_generation.vcxproj      # Visual Studio project
└── data_generation_clean.sln    # Visual Studio solution
```

### Key Files

| File | Role |
|---|---|
| `src/core/main.cpp` | CLI entry point; argument parsing → CNCOverseer |
| `src/core/CNCOverseer.cpp` | Batch loop: load config → connect → iterate experiments |
| `src/core/CNCExperimentRunner.cpp` | Single-experiment lifecycle; 100 Hz main loop |
| `src/core/GenerationPipeline.cpp` | Infinite noise generator (8000-point chunks) |
| `src/core/InjectionPipeline.cpp` | Bridges generator → SMR ring buffer |
| `src/core/ExtractionPipeline.cpp` | SMR → CSV writer; motion-filters output |
| `src/core/gcode_generator.cpp` | Synthesizes valid `.fnc` G-code (linear, arc, mixed) |
| `src/core/KinematicNoiseGenerator.cpp` | Gaussian bandpass / sum-of-sinusoids / sparse noise |
| `src/core/VffGenerator.cpp` | VFF signals (Gaussian / DC-shifted / sparse) |
| `src/core/HurcoConnection.cpp` | Manages Python subprocess; JSON stdin/stdout protocol |
| `src/core/MotionService.cpp` | Windows SMR ring buffer (semaphore-based IPC) |
| `src/core/CSVParser.cpp` | Parses experiment config CSV into `ExperimentConfig` structs |
| `include/core/system_constants.hpp` | Central config: timings, buffer sizes, machine limits |
| `python/persistent_cnc_wrapper.py` | Long-lived CNC REST client; reads commands from C++ stdin |
| `python/RestfulAPIBase.py` | Hurco REST API client (auth, subscriptions, file load/run) |
| `main_g_code_generation.cpp` | Generates 40 sample `.fnc` files for offline testing |

---

## Entry Points & How to Run

### 1. Main batch data collection (typical use)
```
CNCDataGenerator.exe experiments.csv --config config/system_config.json --output D:\output_dir
```
- `experiments.csv` — one row per experiment (see CSV format below)
- `--config` — JSON with machine constraints, buffer sizes, timing
- `--output` — root directory for session folders

The process: load config → start Python wrapper → iterate CSV rows → for each: generate G-code, run pipelines, wait for CNC completion, write CSVs → print batch summary.

### 2. Standalone G-code generation (no CNC)
Build and run `main_g_code_generation.cpp`. Produces 40 `.fnc` files in `generated_gcode/`. Useful for validating G-code logic without a live machine.

### 3. Python wrapper (debug/standalone)
```
python python/persistent_cnc_wrapper.py
```
Reads JSON commands from stdin, writes JSON responses to stdout. Useful for manually testing REST API connectivity.

### 4. REST API test suite
```
python python/api_test_code.py
```
Hits all major CNC REST endpoints. Run against the live machine to validate connectivity before batch collection.

### Building
Open `data_generation_clean.sln` in Visual Studio. [ADD: target platform — x64 Release? required SDK versions?] The project depends on Eigen (headers in `include/Eigen/`) and the Windows SDK for SMR/semaphore APIs.

---

## Experiment CSV Format

Each row defines one experiment. Required columns:

```
experimentId, familyId,
trajectory_type     (LINEAR | CIRCULAR | MIXED),
noise_type          (SMOOTH_GAUSSIAN_BANDPASS | SUM_OF_SINUSOIDS | SPARSE_INJECTION | NO_NOISE),
vff_type            (SMOOTH_GAUSSIAN | SMOOTH_GAUSSIAN_DC_SHIFT | SPARSE_VFF | NO_VFF),
noise_min_amplitude, noise_max_amplitude,
noise_min_freq, noise_max_freq,
noise_min_sines, noise_max_sines,
noise_sparse_prob,
vff_min_dc, vff_max_dc, vff_max_amplitude, vff_max_freq, vff_sparse_prob,
master_seed         (OR: gcode_seed + noise_seed + vff_seed)
```

---

## Output Structure

```
<output_dir>/
├── session_<timestamp>_seed<N>/
│   ├── config_gcode.json
│   ├── config_noise.json
│   ├── config_machine.json
│   ├── config_seeds.json
│   ├── generated_gcode.fnc
│   └── results/
│       ├── 0001_dataset.csv   # 8000 motion points, columns below
│       ├── 0002_dataset.csv
│       └── ...
├── error_log.txt
└── batch_summary.txt
```

CSV columns: `pos_x/y/z`, `dev_x/y/z`, `vff_x/y/z`, `e_enc_x/y/z`, `e_scale_x/y/z`, `line_number`, `has_motion`

---

## Key Conventions & Patterns

**Three-stage pipeline**: Generation (CPU, non-real-time) → Injection (16k-point deque bridge) → Extraction (8k-point SMR read). Each stage is independent; the main loop calls `processOneCycle()` on each at 100 Hz.

**Chunk size = 8000 points**: Matches the SMR buffer size and the CNC controller tick rate (4000 Hz × 2s max trajectory). Injection refills when the deque drops below 8000 (50% threshold).

**Seed-based reproducibility**: `master_seed` derives three child seeds via `(master_seed + offset) % 2^31`. Alternatively, set `gcode_seed`, `noise_seed`, `vff_seed` explicitly in the CSV for fine-grained control.

**Motion filtering**: Extraction buffers all SMR reads but only writes a CSV file when the 8000-point buffer contains at least one point with velocity > 1e-6 mm/s. Static hold periods produce no output files.

**Python subprocess protocol**: C++ sends one-line JSON to the wrapper's stdin; wrapper responds with one-line JSON on stdout. A background thread in `HurcoConnection` continuously reads stdout and updates `lastKnownStatus_` atomically.

**Non-blocking SMR access**: All semaphore waits use 0 ms timeout. If the buffer is full/empty the cycle is skipped silently — no blocking, no data loss recovery.

---

## Things to Know

**Noise clips at ±0.9mm hard limit.** The machine fault threshold is ±1.0mm following error; the 0.1mm margin is intentional. If your experiment config sets amplitudes above this, the generator will silently clip.

**Each 8000-point chunk is statistically independent.** Butterworth filter state and sinusoid phase are reset between chunks. There is no temporal continuity across chunk boundaries. [ADD: is this intentional for the LSTM dataset, or a known limitation?]

**`system("pause")` is in `main.cpp`** — intentional for interactive Windows debugging. Remove or gate it behind a flag before running headless/automated batch jobs.

**The Python wrapper is a subprocess, not a thread.** If `persistent_cnc_wrapper.py` crashes, `HurcoConnection` will fail on the next status poll but won't automatically restart it. Check `error_log.txt` if experiments abort silently.

**CNC REST API uses three ports**: 4503 (HTTP auth), 4504 (HTTPS data service), 4505 (subscription socket). All three must be reachable on the machine network. [ADD: static IP or hostname of the CNC controller?]

**No experiment timeout.** `CNCExperimentRunner` loops until `HurcoConnection` reports status ≥ 2 (COMPLETED). If the CNC hangs mid-program, the process waits forever. [ADD: is there a watchdog planned?]

**Archive folder is dead code.** `include/core/archive/` and `src/core/archive/` contain old G-code parser and kinematics code that predate the current generator. They are not compiled or called.

**`api_command_wrapper.py` is obsolete.** It was replaced by `persistent_cnc_wrapper.py`. Ignore it.

**`NonConsumingBufferMonitor.hpp` is an empty stub.** Not wired into anything.

**CSV file numbering is per-session, not per-batch.** The extraction pipeline counter starts at 0001 for every new experiment session. Cross-experiment file collisions are prevented by the unique session folder name.
