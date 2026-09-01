# Generator-picture index

`generator_picture_index.csv` indexes every current generator in
`arti-template/agent/templates`. A generator is a top-level `*.py` file whose name does not start
with `_`; this excludes `__init__.py`, `_mechanisms.py`, and `_modular.py`.

## Current inventory

- 531 current generators.
- 99 current generators are listed in `articraft_builtin_100_map.csv` and are marked
  `articraft_builtin_dataset_no_picture`.
- 432 current generators are picture-backed and resolve to 431 unique picture directories.
- 451 picture directories exist under `articraft_data/picture`; 20 have no current generator and
  are listed in `picture_without_generator.csv`.
- 54 of those picture directories are also copied under `arti-template/picture`; 53 have a current
  generator. `Textiles_Fabric/Snap_button fastener` is the local directory without one.

The apparent 100/431 split does not exactly describe the current filesystem for two reasons:

- `articraft_builtin_100_map.csv` still contains
  `lighthouse_with_rotating_beacon_assembly`, but that generator file is absent. Therefore only 99
  of the current 531 generators belong to the builtin table.
- `Urban_Environment_Public_toilet` and `Urban_Environment_Public_toilet1` intentionally share the
  same `Urban Environment/Public toilet` picture directory. Therefore 432 picture-backed
  generators use 431 unique picture indices.

## Columns

- `generator_index`: stable sorted index over the current generator names.
- `source_type`: distinguishes builtin dataset generators from picture-backed generators.
- `picture_index`: stable sorted index over the 451 directories in `articraft_data/picture`.
- `picture_source_path`: canonical existing picture directory for picture-backed generators.
- `arti_template_picture_path`: populated only when the same directory is present in the smaller
  `arti-template/picture` subset.
- `picture_storage`: states whether the directory exists only in `articraft_data` or in both
  locations.
- `mapping_source`: records whether the match came from an existing map or an exact normalized
  path/0611 label match.

Regenerate both CSV files from the repository root with:

```bash
python template_maps/build_generator_picture_index.py
```
