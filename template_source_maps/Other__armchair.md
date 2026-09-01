# Other / armchair — template source map

pattern: mixed（固定 named slots: `chair_form` + `base_support` + `recline_mechanism` + `armrest`，外加 `caster_count`）

parents:
- P_winged rec_model-a-modern-winged-swivel-lounge-armchair-wit_20260610_084953_512320_edd1a742
- P_egg rec_model-a-plush-egg-pod-style-swivel-lounge-chair-_20260610_085003_300896_f33ff811
- P_office rec_model-a-tall-ergonomic-office-chair-ikea-markus-_20260610_085012_906733_cb37c340
- P_gaming rec_model-a-racing-style-gaming-chair-in-matte-black_20260610_085024_287868_290ca081

Migration note:
- `rec_chair_var_blue_mesh_prismatic_armrests_codex_redo` 已从 `Chair / Chair` 迁入本小类。
- 它现在作为 `office_mesh` 语义的迁移资产存在，补强 blue-mesh + recline + height-adjust armrest 这一条 office 主表达。
- 它的 hydrated record 目录、`picture.json`、workbench sidecar、records index、subcat shard 已同步到 `Other/armchair`；lineage 已改为 root，不再指向 Chair stool parent。

## Slot Coverage

### Slot A: chair_form
| candidate | anchor | status |
|---|---|---|
| winged_lounge | P_winged | parent |
| egg_pod | P_egg | parent |
| office_mesh | P_office + rec_chair_var_blue_mesh_prismatic_armrests_codex_redo | parent+migrated-root |
| racing_bucket | P_gaming | parent |

### Slot B: base_support
| candidate | record_id | status |
|---|---|---|
| five_star_caster | P_office | parent |
| four_wood_legs | rec_variant-base-support-four-wood-legs-replace-the-_20260618_032924_946393_78b0bb00 | converged |
| cantilever_sled | rec_variant-base-support-cantilever-sled-replace-the_20260618_032924_946307_549d8992 | converged |

### Slot C: recline_mechanism
| candidate | record_id | status |
|---|---|---|
| swivel_tilt | P_winged / P_office | parent |
| rocker_glider | rec_variant-recline-mechanism-rocker-glider-mount-th_20260618_032924_949307_84c9e022 | converged |
| full_recliner_footrest | rec_variant-recline-mechanism-full-recliner-footrest_20260618_033321_088352_ad4e3477 | converged |

### Slot D: armrest
| candidate | record_id | status |
|---|---|---|
| fixed_arms | parents | parent |
| flip_up | rec_variant-armrest-flip-up-make-both-armrests-flip-_20260618_033528_379232_68a45f62 | converged |
| height_adjust | rec_variant-armrest-height-adjust-make-both-armrests_20260618_034005_449370_75232ed3 + rec_chair_var_blue_mesh_prismatic_armrests_codex_redo | converged+migrated-root |

## Multiplicity
- `caster_count` on `five_star_caster`: N in {3,5,6}

## Exclusions
- cabinet sweep 的误落 `rec_variant-base-support-*` reskin 仍不纳入本池。
