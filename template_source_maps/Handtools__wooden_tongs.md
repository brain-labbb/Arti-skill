# Handtools / wooden tongs — template source map

pattern: parallel_children
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-wood_20260609_164010_800484_c58cac31 ← picture/Handtools/wooden tongs/001.png (one-piece sprung wooden serving tongs: two flat tapered arms joined at the top by a small metal spring clip, splay into a "V", squeeze closes the paddle tips)

Wooden tongs. Core kinematics: two long flat tapered wooden **arms** that close
toward each other to grip food. The defining motion is the **arm-close
mechanism** — either a top spring-clip pivot (parent, the arm pair is hinged at
one end by a metal clip), a mid-length scissor pin (arms cross over a round
pin), or the same sprung pivot **plus** a sliding locking collar that pins the
arms shut. The three independent structural slots are: the close mechanism, the
arm/end shape, and the grip detail.

> **Coverage status (UNDER-COVERED):** this pool currently has only the parent +
> **2** converged variants. The combo product on existing samples is **< 10**
> Slots B/C originally had <2 non-parent candidates; the 6 gap variants below have since landed (converged). The rows
> below are the specific single-axis gap variants the main loop must fork to
> reach combo ≥ 10 and every slot ≥ 2. (Record-id stubs match the prior
> interrupted batch's intended names.)

## Slot 候选覆盖

### Slot A:close mechanism / pivot (how the two arms hinge & close — the `pivot` joint family)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| onepiece_spring_clip | rec_build-...-wood_...c58cac31 (parent) | `fixed_arm` + `moving_arm`, `spring_clip` visual on root, `pivot` (REVOLUTE z) at the TOP end (x≈0) | arms joined only at the narrow top by a metal spring clip; pivot at the very end of both arms; relaxed "V" at q=0, squeeze closes tips | converged |
| scissor_pin | rec_wooden_tongs_var_scissor_pin | `arm_0`/`arm_1` (loop i in range(2)), `pivot_pin` cylinder visual on `arm_0`, `pivot` (REVOLUTE z) PARTWAY along the arms | arms cross over a round pin mid-length; short handle side + long tip side either side of the pin; pin passes through both arms (allow_overlap) | converged |
| spring_clip_with_lock_ring | rec_wooden_tongs_var_lock_ring | parent pivot + `slide_ring` part, `slide` (PRISMATIC x) collar that slides toward the tips to lock | top spring-clip pivot PLUS a sliding wooden locking collar (2 non-fixed joints: revolute pivot + prismatic slide) — adds the locking-collar sub-mechanism | converged |
| onepiece_bend | rec_wooden_tongs_var_onepiece_bend | one-piece springy U/hairpin bend at the top; revolute-ish flex at the bend, NO separate metal clip | true one-piece bent-wood tong (no metal joint); the bend itself is the spring; arms are two halves of one continuous strip | converged |

> Note: the parent is **not** a true one-piece bend — its top is joined by a
> discrete `spring_clip` metal part bridging two separate arms. `onepiece_bend`
> fills the genuinely distinct "single continuous bent strip, no metal" form
> that the reference family also shows, so Slot A reaches **3 non-clip
> mechanisms** (sprung-clip / scissor-pin / one-piece-bend) + the lock-ring
> compound.

### Slot B:arm / end shape (cross-section of the arm + paddle-tip geometry)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_paddle (parent baseline) | rec_build-...-wood_...c58cac31 (parent) | `_arm_solid()` tapered flat strip, polyline outline flaring to a wide rounded paddle `TIP_WIDTH=0.030` | flat tapered slat ending in a wide rounded paddle; the default | converged |
| round_dowel | rec_wooden_tongs_var_round_dowel | `_arm_solid()` → round dowel cross-section (cylinder/lathe) instead of flat strip | arms are round wooden dowels rather than flat slats; ends rounded | converged |
| straight_slat | rec_wooden_tongs_var_straight_slat | arm outline with no paddle flare — constant-width straight flat slat to the end | flat slat that stays the same width (no paddle), squared-off-but-flat shaft full length | converged |
| spoon_ends | rec_wooden_tongs_var_spoon_ends | tip section becomes a concave scoop/spoon bowl (CadQuery cut/lathe dish at the paddle) | salad-server style concave spoon/scoop tips instead of flat paddles | converged |
| square_ends | rec_wooden_tongs_var_square_ends | tip outline squared off (rectangular blunt end, no rounding fillet) | blunt square-cut paddle ends instead of rounded ones | converged |

### Slot C:grip detail (handle/grip texture along the arm — decorative-but-structural surface layer)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| plain (parent baseline) | rec_build-...-wood_...c58cac31 (parent) | smooth `_arm_solid()`, only perimeter fillet | smooth plain wooden arm, no grip features | converged |
| scalloped_grip | rec_wooden_tongs_var_scalloped_grip | scalloped / finger-groove ridges cut along the handle section of each arm (loop-emitted grooves via shared helper, inlined as parent visuals or boolean cuts) | row of scalloped finger grooves / ridges along the grip region for purchase | converged |

> Slot C currently has only the parent (plain) → **1** candidate. `scalloped_grip`
> is the minimum to lift it to 2. The lock-ring variant's `grip_{i}` ridge loop
> (4 cardinal nubs via a shared `_grip_nub()` helper) is a reusable copy-logic
> reference for authoring this groove loop, even though those nubs live on the
> ring, not the arm.

## Multiplicity / Copy Logic
- count_param: 无核心 N 复制轴(主结构是固定的 2 个 arm + 可选 ring/clip 命名 slot)。
  - 唯一的循环发射样本是 lock_ring 变体的 `grip_{i}` 脊 (`for i in range(N_GRIPS)`,
    `N_GRIPS=4`, 共享 `_grip_nub()` helper, 4 个 cardinal 面规则放置, 全部 FIXED 在
    `slide_ring` 上)—— 这是 Slot C scalloped 槽脊的 copy-logic 范例,但不是产品的真 N 轴。
  - scissor_pin 变体把两条 arm 用 `for i in range(2)` + `f"arm_{i}"` 循环发射(对称 Z
    偏移 + 反向 splay),是把"对称双臂"写成循环的可读样本;parent 则手写
    `fixed_arm`/`moving_arm` 两份(下游模板若要对称发射双臂,折成此循环)。
- N 样本: 无 multiplicity 轴(arm 数恒为 2;grip 脊数是装饰,不作为产品 N 域)。
- 模板建议 N_range: 无(若把 scalloped grip 脊数参数化,可 N_grooves ∈ [3, 10],仅装饰层)。
- copied object / naming / placement / joint policy: 见上 —— grip 脊 = `grip_{i}`,
  cardinal/等距放置,统一 FIXED 到承载 part;双臂 = `arm_{i}`,对称 Z + 反向 splay,
  其中一臂为 root、另一臂经 `pivot` REVOLUTE 铰接。

## 组合数预审
现状(仅 parent + 2 变体):
- Slot A 已覆盖 3 候选(onepiece_spring_clip / scissor_pin / spring_clip_with_lock_ring)。
- Slot B 仅 parent 1 候选(flat_paddle)—— **< 2,不达标**。
- Slot C 仅 parent 1 候选(plain)—— **< 2,不达标**。
- **现状组合积 = 3 × 1 × 1 = 3 < 10 ✗**,且 Slot B、Slot C 各只有 1 个候选(违反"每槽 ≥2")。
  无 multiplicity 轴可放大(arm 数恒为 2),所以必须靠加候选把乘积顶上去。

补 gap 之后(下列 6 个变体已全部 fork 落地、compile-success、workbench-only、≥1 non-fixed joint):
- Slot A: 4 候选(+onepiece_bend)。
- Slot B: 5 候选(flat_paddle + round_dowel + straight_slat + spoon_ends + square_ends)。
- Slot C: 2 候选(plain + scalloped_grip)。
- **补后组合积 = 4 × 5 × 2 = 40 ≥ 10 ✓**,且每个 slot ≥ 2 候选。

> 注:即便只补 Slot B 到 2 候选、Slot C 到 2 候选(最小修复),积 = 3×2×2 = 12 已 ≥10;
> 但 §5 鼓励把候选堆厚(B 是真实结构词汇最丰富的轴),故规划 B 满配 5 个。每个 gap
> 变体都是单轴改动,从语义最近的 parent fork,diff 干净。

## gap 变体(已 fork 落地,本批补造)
每个 = 一条单轴改动;`reference image` 用 picture/Handtools/wooden tongs/001.png;
prompt 追加 FORK_VARIANTS.md §4 固定后缀;workbench-only。

| record_id stub | fork from (parent) | 目标 slot | 一句话轴描述 |
|---|---|---|---|
| rec_wooden_tongs_var_onepiece_bend | rec_build-...-wood_...c58cac31 (parent) | Slot A | 把顶端金属 spring_clip 换成真正的一体弯木 U/发夹弯(无金属件):两臂是同一连续木条的两半,弯折处即弹簧;保留 REVOLUTE-ish 闭合运动(其余功能层照 parent)。 |
| rec_wooden_tongs_var_round_dowel | rec_build-...-wood_...c58cac31 (parent) | Slot B | 把扁平 tapered 木条臂换成圆木 dowel 截面(cylinder/lathe),端头圆头;机构与抓取运动不变。 |
| rec_wooden_tongs_var_straight_slat | rec_build-...-wood_...c58cac31 (parent) | Slot B | 取消 paddle 外扩,改成全长等宽的直扁条(无加宽 paddle 头);机构不变。 |
| rec_wooden_tongs_var_spoon_ends | rec_build-...-wood_...c58cac31 (parent) | Slot B | paddle 头改成凹勺/铲斗(CadQuery 挖凹或 lathe 碟形),沙拉夹式 scoop 端;机构不变。 |
| rec_wooden_tongs_var_square_ends | rec_build-...-wood_...c58cac31 (parent) | Slot B | paddle 端改成方头平切(矩形钝端,去掉圆角收尾);机构不变。 |
| rec_wooden_tongs_var_scalloped_grip | rec_build-...-wood_...c58cac31 (parent) | Slot C | 在每条臂的握持段刻一排 scalloped 手指凹槽/脊(共享 helper + `for i in range(n)` 循环发射或布尔挖槽,inlined 为 parent visual);机构与臂形不变。 |

全部从 **parent**(onepiece_spring_clip)fork:它是除 lock_ring 外最简单的基线,
单轴 diff 最干净(scissor_pin 的中段 pin 几何会污染 B/C 轴的 diff,故不从它 fork)。
补齐这 6 个后:Slot A=4、Slot B=5、Slot C=2,组合积 40 ≥ 10,达到 §8 完成定义 1。

## 排除项(未来 compatibility matrix 素材)
- 现状(本次落盘时)Slot B / Slot C 各仅 1 候选,属"待图/待 fork 阻塞",已用上表
  6 个单轴 gap 变体规划补齐,非真排除项。
- 真实潜在干涉(留给 compatibility matrix 抽检,不预防性造变体):
  - `spring_clip_with_lock_ring`(Slot A 的滑环)× `round_dowel`(Slot B):圆截面臂
    与矩形 ring bore 的贴合/锁紧接触面与扁条不同,锁紧 overlap 容差需重算 —— 组合
    由模板采样器产出,接口风险在 spec compatibility matrix 处置。
  - `scissor_pin`(中段 pin)× `onepiece_bend`(Slot A 互斥):两者都是顶/中机构,
    属同一 Slot A 的不同候选,天然不共存(Slot A 单选),不构成组合。
- 纯尺寸(臂更长/更宽/更扁、splay 角、tip 宽度)不作候选 —— 属模板连续参数
  (controlled local parameterization),不入 slot。
