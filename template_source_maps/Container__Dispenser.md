# Container / Dispenser — template source map

pattern: parallel_children

parents:
- rec_container_dispenser_v01 ← picture/Container/Dispenser/001.png (clear hand-wash / lotion dispenser bottle; rounded clear body with front label, ribbed threaded collar, exposed press-down pump, long horizontal swiveling spout, visible straight dip tube). Occupies Slot A `rounded_clear_bottle` · Slot B `standard_press_swivel_spout` · Slot C `standard_threaded_collar` · Slot D `straight_visible_dip_tube`.

This source image is an uncapped pump dispenser. The current parent is already rebuilt from that source; the stale clear-over-cap parent and the old non-qwen cap variants have been removed from the active record set. Variants are therefore forked from the current parent and describe visible structural alternatives of the body, pump head/spout, collar, and dip tube. There is no closure/cap slot for this small class.

## Slot 候选覆盖

### Slot A:body footprint / bottle silhouette
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rounded_clear_bottle | rec_container_dispenser_v01 | `body`/`bottle_shell`, rounded transparent reservoir, label panel | rounded rectangular clear bottle with soft shoulders and centered neck | converged(parent) |
| square_body | rec_container_dispenser_var_square_body_qwen | `body`/`bottle_shell`, squarer rounded-rect footprint | chunkier square dispenser body with flatter faces and sharper corner radius | converged(qwen) |
| oval_body | rec_container_dispenser_var_oval_body_qwen | `body`/`bottle_shell`, elliptical/oval footprint | flattened oval flask body, broad front face and shallow side depth | converged(qwen) |

### Slot B:pump head / spout mechanism
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| standard_press_swivel_spout | rec_container_dispenser_v01 | `head_carrier`/`pump_head`, `swivel` REVOLUTE +Z, `pump_press` PRISMATIC +Z | exposed press-down pump with horizontal swivel spout | converged(parent) |
| long_spout_lotion_pump | rec_container_dispenser_var_tall_plunger_long_spout_qwen | `pump_head` plus elongated `spout`, press-down plunger travel | taller actuator with a longer, clearly projecting lotion-pump spout | converged(qwen) |
| detached_pump_insert | rec_container_dispenser_var_detached_pump_insert_qwen | removable pump insert assembly with visible stem/tube | replacement pump insert shown as a separable serviceable pump module | converged(qwen) |
| twist_lock_pump | rec_container_dispenser_var_twist_lock_pump_qwen | low-profile pump head with lock/unlock twist collar | compact twist-lock pump head that rotates to lock before pressing | converged(qwen) |

### Slot C:neck collar / threaded ring detail
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| standard_threaded_collar | rec_container_dispenser_v01 | `collar`, ribbed screw ring fused to neck | ordinary ribbed collar under the pump head | converged(parent) |
| oversized_ribbed_collar | rec_container_dispenser_var_ribbed_collar_qwen | enlarged two-tier ribbed collar ring | heavier ribbed pump collar, visually wider and more mechanically dominant | converged(qwen) |

### Slot D:dip tube path / visibility
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| straight_visible_dip_tube | rec_container_dispenser_v01 | `dip_tube` visual descending near bottle center | straight tube visible through the transparent reservoir | converged(parent) |
| curved_dip_tube | rec_container_dispenser_var_curved_dip_tube_qwen | curved `dip_tube` visual inside the bottle | S-curved / swept dip tube path instead of a straight vertical tube | converged(qwen) |

## Multiplicity / Copy Logic
- count_param: 无；dispenser 的核心结构是固定 named slots(body / pump head / collar / dip tube)，不是同构子件复制。
- N 样本已覆盖: 无。
- 模板建议 N_range: 无。
- copied object / naming / placement / joint policy: 无。

## 组合数预审
组合数预审: Π(Slot A 3 × Slot B 4 × Slot C 2 × Slot D 2) × N(无) = 48 ≥ 10 ✓
Every slot has at least two candidates, and all records referenced here exist in the current active record directory.

## 排除项(未来 compatibility matrix 素材)
- closure / cap slot 排除: source image has no bottle cap or clear over-cap; adding flip caps, screw domes, or transparent sleeves would contradict the current parent asset.
- old non-qwen dispenser variants are excluded because their record directories were deleted and replaced by the current qwen fork batch.
- pure color/liquid/label graphics are visual parameters only, not structural slots.
