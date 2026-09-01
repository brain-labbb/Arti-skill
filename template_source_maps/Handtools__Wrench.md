# Handtools / Wrench — template source map

pattern: parallel_children
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-wren_20260609_163952_100145_6f106d91 ← picture/Handtools/Wrench/001.png (adjustable "crescent" wrench, worm-rack drive, flat steel handle)
- rec_build-a-realistic-articulated-3d-model-of-a-wren_20260609_163955_102190_4c74c601 ← picture/Handtools/Wrench/002.png (Stillson pipe wrench, screw-nut drive, tapered wooden handle)

Adjustable wrench family. Every candidate keeps ≥1 non-fixed joint via the head mechanism
(a movable jaw on a PRISMATIC slide, usually plus a CONTINUOUS/REVOLUTE adjuster). The head
mechanism and the handle/shank are the two independent structural slots. Rigid open-end /
ring wrenches are intentionally excluded (0 moving joints).

## Slot 候选覆盖

### Slot A:head / adjust mechanism (movable jaw + adjuster)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| worm_rack_crescent | rec_..._wren_..._6f106d91 (parent A) | `movable_jaw` + `worm_screw`; `jaw_slide` (PRISMATIC), `worm_turn` (CONTINUOUS); rack teeth via `for i in range(5)` | crescent head, knurled worm meshes rack on jaw shank | converged |
| screw_nut_pipe | rec_..._wren_..._4c74c601 (parent B) | `movable_jaw` (screw bar) + `adjust_nut` (KnobGeometry); `frame_to_jaw` (PRISMATIC), `frame_to_nut` (CONTINUOUS); hook+jaw teeth via `_add_teeth_x()` | pipe-wrench hook jaw, knurled nut on screw bar | converged |
| monkey_head | rec_wrench_var_monkeyhead | monkey-wrench flat parallel jaws on screw drive; PRISMATIC slide + CONTINUOUS adjuster | flat-faced parallel jaws (vs angled crescent) | converged |
| thumb_slide | rec_wrench_var_thumbslide | thumb-slide quick adjuster; PRISMATIC jaw + REVOLUTE thumb lever | slide button / lever quick-set jaw | converged |

### Slot B:handle / shank
**handle 与 head-spine 现已解耦**:原先 handle 只随各自 spine 出现(crescent-spine 只有 flat_steel,
pipe-spine 只有 wood/tubular),合法组合仅 5 个 < 10 拓扑门槛。补造 3 个 cross-spine handle 变体后,
crescent-spine 与 pipe-spine 都覆盖全部 3 种 handle,合法组合升至 ~12。

| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_steel | rec_..._wren_..._6f106d91 (parent A, crescent) | `wrench_body` flat forged bar + ring butt | flat stamped steel handle | converged |
| tapered_wood | rec_..._wren_..._4c74c601 (parent B, pipe) | lathe `build_handle_geometry` + `ferrule` + `butt_cap` + worn band | red-painted tapered wooden handle w/ ferrule | converged |
| tubular | rec_wrench_var_tubularshank (pipe) | round tubular shank/handle (lathe revolve) | hollow tubular steel grip | converged |
| flat_steel @ pipe-spine | rec_wrench_var_pipe_flatsteel | flat forged steel bar handle on pipe head | cross-spine: pipe head + flat steel | converged |
| tapered_wood @ crescent-spine | rec_wrench_var_crescent_wood | lathe tapered wood + ferrule + butt cap on crescent head | cross-spine: crescent head + wood | converged |
| tubular @ crescent-spine | rec_wrench_var_crescent_tubular | round tubular steel shank on crescent head | cross-spine: crescent head + tubular | converged |

## Multiplicity / Copy Logic
- count_param: 无 slot 级 multiplicity 轴。
- 部件内复制(非轴): jaw 齿用循环发射——parent A `for i in range(5)` rack teeth;parent B
  `_add_teeth_x(n_teeth=6)` hook + jaw 齿。齿数是 module 内部参数,不作为独立 slot 轴。
- copied object / naming / placement / joint policy: 齿等距沿滑动轴发射,共享一个三角齿 helper,全部
  FIXED 在各自 jaw part 上。

## 组合数预审
Slot A(4 head-mech) × Slot B(3 handle types) 名义 12。受 spine 约束:crescent-spine heads
{worm_rack, monkey, thumb_slide}=3 × handle{flat/wood/tubular}=3 = 9,pipe-spine head{screw_nut}=1
× handle 3 = 3,合法组合 = **12 ≥ 10 ✓**(补造 cross-spine handle 前仅 5,不达标)。
pattern = parallel_children,无 multiplicity。

## 排除项(未来 compatibility matrix 素材)
- 刚性 open-end / box-end / combination 扳手不收(0 活动关节,违反 ≥1 non-fixed joint)。
- spine 约束:handle module 的锚定坐标系随 head-spine(crescent `wrench_body` 原位 vs pipe
  `head_frame` 经 `_lay()` 躺平)。cross-spine handle 已补造样本证明可移植,模板侧需按 head-spine
  rebase handle 锚点。
- 纯尺寸(扳手长度/开口大小)是模板连续参数,不入 slot。
