# Urban Environment / Trashcan1 — template source map

slug: trashcan1
shard: Trashcan1
picdir: picture/Urban Environment/Trashcan1 (001.png, 002.png)

pattern: parallel_children (root can_body + lid_assembly 主机构 [REVOLUTE/PRISMATIC] + 两个 side handle children + inlined floor/rim/lug 装饰 visuals; corrugation 多重度 N 走单参数 trig→需提升为 rib_{i} loop)

identity: classic galvanized steel garbage can = tapered cylindrical sheet-metal body + vertical corrugation ribbing + rolled top rim + two riveted side handles + removable domed lid with a top loop handle. The real joint is the lid lift (REVOLUTE flip / PRISMATIC lift-off) or a handle swing. Variants stay galvanized-style trash cans.

## parents (2)

- rec_galvanized-steel-trash-can-with-a-slightly-taper_20260608_164526_754874_e62b5cca ← picture/Urban Environment/Trashcan1/002.png (single bright galvanized can; body slightly TAPERED wider-at-top; fine vertical ribbing; rolled rim; tall DOMED round lid + top loop handle; two riveted D-RING side handles)
  - 基线: body:tapered × lid:tall_domed_revolute × surface:vertical_flutes_fine × handle:two_ring_ears
- rec_old-galvanized-steel-garbage-can-with-a-vertical_20260608_164505_174293_15c2b69d ← picture/Urban Environment/Trashcan1/001.png (weathered can; vertically corrugated body; rolled rim; two WIRE-BAIL side handles; separate shallow domed lid + loop handle)
  - 基线: body:straight-ish × lid:shallow_domed_revolute × surface:vertical_flutes_coarse × handle:two_bail_ears

## loop-emission notes (readability audit)

- 腐蚀 ribs (corrugation): 两 parent 都把 rib 数烤进 trig 频率 `amp*cos(n_ribs*theta)`，ang_segs=n_ribs*4，单 `RIB_COUNT` 常数驱动。属可接受的"单参数多重度"，但 **rib-count 变体 (ribcount) 必须 request 改写为显式 `for i in range(n)` 的 rib_{i} 命名特征 + 共享 rib helper + 角度均布**（map 已在该变体 prompt 内点名）。其余继承 trig 形式的变体（smooth/horizontal/body-profile）不强制 rewrite。
- side handles: 两 parent 已 loop 化 `for side_name, ang in (("right",0.0),("left",math.pi))` → 干净 2× FIXED。handle 变体改单 bail 时保留 2-side loop（两 pivot lug + 两臂）。
- lugs: parent-2 手写 4 个 lug pad（lug_right_top/bot/left_top/bot）= inlined 非动 body visuals，OK；但 handle 变体的 pivot lug 应随 handle 走 loop。
- lid dome: parent-2 用 `for k in range(levels)` disc-stack 堆 dome，已 loop 化。

## 组合数预审 (HARD GATE)

slots × candidates:
- body_profile: {tapered(基线), straight, barrel-bulge} = 3
- lid_type: {tall_domed_revolute(基线), flat_liftoff_prismatic, no_lid_open} = 3
- surface: {vertical_flutes(基线), horizontal_rings, smooth} = 3
- handle: {two_side_ears(基线), single_overhead_bail} = 2
- rib_count multiplicity N: {coarse ~8, medium ~16, fine ~28-40} = 3 distinct N (rib_{i} loop)

product(structural candidates) = 3 × 3 × 3 × 2 = 54；× distinct-N (3) = 162 ≫ 10. **GATE PASS** (even body(3)×lid(3)=9, ×handle(2)=18 ≥ 10 alone).

## variant 覆盖 (8 NEW, cap ~8-10)

### Slot A: body_profile (体形 silhouette)
| 候选 | variant | 结构 | 状态 |
|---|---|---|---|
| tapered (基线) | parent-2 | wider-at-top 锥体 | parent |
| barrel_bulge | body_profile | mid-height 最大 girth，两端收 | converged |
| straight_cylinder | straightbody | 恒定半径直筒 | converged |

### Slot B: lid_type (主机构槽 — 开合动作)
| 候选 | variant | joint | 状态 |
|---|---|---|---|
| tall_domed (基线) | parent | REVOLUTE 后铰翻盖 | parent |
| flat_liftoff_cap | lidtype | **PRISMATIC** 垂直升降 lift-off 平盖 | converged |
| no_lid_open | openrim | 无盖；joint 转移到 REVOLUTE 摆动 side handle | converged |

### Slot C: surface (壁面纹理)
| 候选 | variant | 结构 | 状态 |
|---|---|---|---|
| vertical_flutes (基线) | parent | 竖向 trig 波纹 | parent |
| horizontal_rings | surface_horizontal | 水平环肋 stack（loop-emitted bands） | converged |
| smooth | surface_smooth | 无波纹光壁 | converged |

### Slot D: rib_count multiplicity N (rib_{i} loop rewrite)
| 候选 | variant | 结构 | 状态 |
|---|---|---|---|
| fine ~28-40 (基线) | parent | trig-baked 密 flutes | parent |
| coarse small-N bold ribs | ribcount | 显式 rib_{i} loop，N 为单 config 参 | converged |

### Slot E: handle_style
| 候选 | variant | joint | 状态 |
|---|---|---|---|
| two_side_ears (基线) | parent | 两 FIXED ring/bail ears | parent |
| single_overhead_bail | handle | 单 REVOLUTE 过顶提梁 bail（两 pivot lug loop） | converged |

## variant 清单 (record_id / label / prompt)

| record_id | label | prompt | axis |
|---|---|---|---|
| rec_trashcan1_var_body_profile | trashcan1-body_profile | /tmp/urb_trashcan1_var_body_profile.txt | body=barrel bulge |
| rec_trashcan1_var_straightbody | trashcan1-straightbody | /tmp/urb_trashcan1_var_straightbody.txt | body=straight cylinder |
| rec_trashcan1_var_lidtype | trashcan1-lidtype | /tmp/urb_trashcan1_var_lidtype.txt | lid=flat lift-off PRISMATIC |
| rec_trashcan1_var_openrim | trashcan1-openrim | /tmp/urb_trashcan1_var_openrim.txt | lid=none + REVOLUTE handle swing |
| rec_trashcan1_var_surface_horizontal | trashcan1-surface_horizontal | /tmp/urb_trashcan1_var_surface_horizontal.txt | surface=horizontal rings (loop) |
| rec_trashcan1_var_surface_smooth | trashcan1-surface_smooth | /tmp/urb_trashcan1_var_surface_smooth.txt | surface=smooth |
| rec_trashcan1_var_ribcount | trashcan1-ribcount | /tmp/urb_trashcan1_var_ribcount.txt | rib N multiplicity (rib_{i} loop rewrite) |
| rec_trashcan1_var_handle | trashcan1-handle | /tmp/urb_trashcan1_var_handle.txt | handle=single overhead bail (REVOLUTE) |

manifest: /tmp/manifest_urb_trashcan1.tsv
suffix (verbatim appended to every prompt): /tmp/urb_suffix_trashcan1.txt

## dropped / merged axes

- pure color/material galvanized weathering → 明确禁止（suffix 规则），never the change.
- pure scale (taller/shorter, fatter diameter) → 禁止 pure-scale；体形改动以 silhouette 结构（barrel/straight）承载，非尺寸缩放。
- footed base / wheeled base → out of identity（reference 无脚无轮），dropped。
- spout/step-pedal/swing-flap → not galvanized-can identity (那是塑料 wheelie/step bin)，dropped。
- 每个 slot 至少 2 候选 ✓；multiplicity 2-3 N ✓ (rib_count 3 distinct N)。
