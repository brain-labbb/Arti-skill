# Articraft evaluation pilot

This is a deliberately small end-to-end smoke test for Infinite-Mobility-style
evaluation. It keeps evaluation code and API calls outside the template runtime.

The checked-in manifest contains three already-exported Articraft assets:

- `Door_Double_Door`: 3 seeds
These are existing coverage-picked exports, not an unbiased paper sample. The
pilot is for validating the workflow and quickly seeing visual/structural trends.

## Run locally

Use the `arti-template` environment so the renderer can import the SDK:

```bash
cd /mnt/zsn/lyb/arti-skill/arti-template

# URDF joint counts and pilot tree-edit distance
uv run python ../eval_pilot/pilot.py metrics

# Four fixed Blender Cycles RGB and world-space-normal views per asset
uv run python ../eval_pilot/pilot.py render --renderer blender

# Build three within-category pairs
uv run python ../eval_pilot/pilot.py pairs

# Build an HTML report without making API calls
uv run python ../eval_pilot/pilot.py report
```

Artifacts are written to `eval_pilot/artifacts/` and ignored by Git.

## Discover and probe the compatible API

Never put a key in this repository or in a command-line argument. Inject it as
`OPENAI_API_KEY`; the base URL defaults to the configured compatible gateway.

```bash
export OPENAI_BASE_URL=https://codex.ai02.cn/v1
export OPENAI_API_KEY=...

cd /mnt/zsn/lyb/arti-skill/arti-template
uv run python ../eval_pilot/pilot.py discover-models
uv run python ../eval_pilot/pilot.py probe-model
```

`discover-models` only proves that a model ID is advertised. `probe-model`
sends a tiny image and proves that the selected route actually accepts vision
input.

## Run the VLM judge

The smoke manifest makes three within-category Articraft-vs-Articraft pairs only.
This validates image transport, structured output, caching, AB/BA order swaps,
and positional-bias accounting. It is not an Ours-vs-PartNet result.

```bash
uv run python ../eval_pilot/pilot.py judge
uv run python ../eval_pilot/pilot.py report
```

Each pair is judged twice with A/B swapped. Responses are cached by endpoint,
model, prompt, image content, and order. The report counts consistent wins,
ties, and position-sensitive outcomes.

## Promote to the real pilot

For a publishable comparison, replace `manifest.jsonl` with a frozen random
sample and add baseline rows whose `rgb_sheet` and `normal_sheet` point to
standardized PartNet-Mobility renders. Do not replace failed seeds after the
manifest is frozen. Keep random and coverage-selected cohorts separate.
