# CLAUDE.md — data_generation

## What This Repo Does

This repo generates the training dataset for a MASc thesis investigating CNC motion error prediction and compensation on a Hurco CNC machine using neural networks. It is a C++ application that runs batch experiments: for each experiment it generates G-code with parameterized geometry, synthesizes realistic kinematic noise and VFF signals, injects those signals into the machine's shared-memory ring (SMR) buffer, executes the program on the physical CNC via a Python wrapper, and extracts the resulting real-time motion data into CSV files. The output is the raw dataset consumed by the LSTM training pipeline elsewhere in the thesis hub.

## Repo Layout

```
data_generation/
├── src/core/                # C++ implementations
├── include/core/            # C++ headers (mirrors src/core/)
│   └── archive/             # Deprecated G-code parser and kinematics; do not use
├── python/                  # Python subprocess that talks to the CNC REST API
├── main_g_code_generation.cpp
├── data_generation.vcxproj
└── data_generation_clean.sln
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
| `src/core/gcode_generator.cpp` | Synthesizes valid `.fnc` G-code |
| `src/core/KinematicNoiseGenerator.cpp` | Gaussian bandpass / sum-of-sinusoids / sparse noise |
| `src/core/VffGenerator.cpp` | VFF signal generation |
| `src/core/HurcoConnection.cpp` | Manages Python subprocess; JSON protocol |
| `src/core/MotionService.cpp` | Windows SMR ring buffer |
| `src/core/CSVParser.cpp` | Parses experiment config CSV |
| `include/core/system_constants.hpp` | Central config: timings, buffer sizes, machine limits |
| `python/persistent_cnc_wrapper.py` | Long-lived CNC REST client |
| `python/RestfulAPIBase.py` | Hurco REST API client |
| `main_g_code_generation.cpp` | Generates sample `.fnc` files for offline testing |

## Entry Points & How to Run

### 1. Main batch data collection
```
CNCDataGenerator.exe experiments.csv --config config/system_config.json --output D:\output_dir
```
- `experiments.csv` — one row per experiment
- `--config` — JSON with machine constraints and timing
- `--output` — root directory for session folders

### 2. Standalone G-code generation
Build and run `main_g_code_generation.cpp`. It produces sample `.fnc` files for offline testing.

### 3. Python wrapper
```
python python/persistent_cnc_wrapper.py
```
Useful for manually testing the CNC REST API wrapper.

### Building
Open `data_generation_clean.sln` in Visual Studio. The project depends on the Windows SDK and the bundled Eigen headers.

## Experiment CSV Format

Each row defines one experiment. Required columns include:

```
experimentId, familyId,
trajectory_type,
noise_type,
vff_type,
noise_min_amplitude, noise_max_amplitude,
noise_min_freq, noise_max_freq,
noise_min_sines, noise_max_sines,
noise_sparse_prob,
vff_min_dc, vff_max_dc, vff_max_amplitude, vff_max_freq, vff_sparse_prob,
master_seed
```

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
│       ├── 0001_dataset.csv
│       ├── 0002_dataset.csv
│       └── ...
├── error_log.txt
└── batch_summary.txt
```

CSV columns include: `pos_x/y/z`, `dev_x/y/z`, `vff_x/y/z`, `e_enc_x/y/z`, `e_scale_x/y/z`, `line_number`, `has_motion`.

## Key Conventions & Patterns

- Three-stage pipeline: Generation → Injection → Extraction.
- Chunk size = 8000 points.
- Sample rate = 4000 Hz.
- Units stay in mm.
- Output arrays use shape `(N, 3)` with columns `[x, y, z]`.
- Noise clips at ±0.9 mm hard limit.
- Each 8000-point chunk is effectively independent for the current dataset design.
- `system("pause")` in `main.cpp` is intentional for interactive debugging; gate it before headless jobs.
- `persistent_cnc_wrapper.py` is the active Python bridge; `api_command_wrapper.py` is obsolete.
- The archive folder is legacy/dead code and should not be used.

## Working Notes

- Keep the data contract stable unless downstream loaders and analysis code are updated in the same change.
- Preserve no-motion handling so playback remains stable.
- If a change affects generated data format, update the downstream readers and the thesis pipeline accordingly.
