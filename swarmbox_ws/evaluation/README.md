# SwarmBox Evaluation

Packages, run configurations and automation used to produce the results reported in
the SwarmBox paper. **Nothing here is required to use the SwarmBox framework** — it is
kept for reproducibility only.

For the archived, citable snapshot of these experiments, use release `v0.1.0` (also
deposited on Zenodo). This directory tracks `main` and may drift from the paper.

## Building

`swarmbox_ws/evaluation/COLCON_IGNORE` keeps these packages out of the default build.
To build them:

```bash
cd swarmbox_ws
colcon build --base-paths src evaluation/src
source install/setup.bash
```

## Running

All experiments are launched through the framework runner in `swarmbox_ws/run/execute.py`:

```bash
cd swarmbox_ws
python3 ./run/execute.py --config ./evaluation/run/RQ4/config/rq4_1.yaml
```

## RQ → package map

| RQ | Package | Config |
|----|---------|--------|
| RQ1 | `rq1_adaptive`, `rq1_optimized`, `rq1_socratic` | `evaluation/run/RQ1/config/` |
| RQ2 | `sim2real` — shipped as a core example, see [`../src/examples/sim2real`](../src/examples/sim2real) | [`../run/example/sim2real.yaml`](../run/example/sim2real.yaml) |
| RQ3 | `rq3_faulty` | `evaluation/run/RQ3/config/` |
| RQ4 | `rq4_delivery` | `evaluation/run/RQ4/config/` |
| RQ5 | `airview`, `delivery`, `formation`, `network`, `tracking` | `evaluation/run/RQ5/config/` |

RQ2 lives outside this directory on purpose: it doubles as the smoke test run by
`scripts/functionality_check.sh`, so it must be buildable without the evaluation tier.

## Analysis

Log post-processing and figure generation live in `swarmbox_ws/analysis/`.
