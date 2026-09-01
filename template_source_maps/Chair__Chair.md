# Chair / Chair — template source map

pattern: mixed

parents:
- rec_round-rolling-task-stool-with-a-circular-seat-an_20260606_120306_344111_ab6f0d5f <- picture/Chair/Chair/001.png
- rec_swivel-bar-stool-a-tan-leather-seat-with-a-curve_20260606_120302_689069_9bb33a8d <- picture/Chair/Chair/002.png

Identity:
- single-seat chair / stool
- optional low back only
- articulated support motion stays in swivel / caster system
- NOT office high-back, NOT adjustable-arm chair, NOT armchair

## Slot Coverage

### Slot A: base / support topology
| candidate | record_id | status |
|---|---|---|
| rolling_stool_base | parent 001 | converged |
| swivel_pedestal_base | parent 002 | converged |
| four_leg_dining_base | rec_chair_var_four_leg_dining_base | converged |
| sled_base | rec_chair_var_sled_base | converged |
| tripod_pedestal_base | rec_chair_var_tripod_pedestal_base | converged |
| four_casters | rec_chair_var_four_casters | converged |

### Slot B: backrest
| candidate | record_id | status |
|---|---|---|
| none | caster/task-stool lineage | converged |
| low_or_curved_back | parents / sled lineage | converged |

### Slot C: seat plan
| candidate | record_id | status |
|---|---|---|
| round_or_curved_seat | parents | converged |
| square_seat | rec_chair_var_square_seat | converged |

## Multiplicity
- `caster_count`: caster bases only
- `radial_support_count`: tripod only

## Migration / Exclusions
- 原高背网背办公椅迁移资产已转入 `Other / armchair`。
- Chair 小类不再接受高背网背 / 独立 recline / 升降扶手 fork。
