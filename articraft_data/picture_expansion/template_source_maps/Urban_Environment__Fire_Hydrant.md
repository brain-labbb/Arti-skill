# Urban Environment / Fire Hydrant — template source map

object: cast-iron street PILLAR fire hydrant — vertical +Z barrel body: bolted ground flange base, ribbed barrel, widened upper valve chamber carrying side hose outlets + a larger front pumper outlet, a bolted bonnet flange, a domed cast bonnet, and a square vertical operating nut on top. Each outlet wears a brass lift-off cap tethered by a short articulated round-link chain. Real non-fixed joints: top operating nut (REVOLUTE about Z) + each outlet cap (PRISMATIC, lifts straight off its own axis) + each cap's serial round-link chain (REVOLUTE links).

pattern: mixed (固定 named slots: bonnet_shape + outlet_cap_style + base_form；外加 outlet_{i} 的 multiplicity 复制轴)

## HETEROGENEITY NOTE + CHOSEN IDENTITY
The subcat has TWO parents that are NOT the same object:
- **001.png = cast-iron PILLAR fire hydrant** → CHOSEN CANONICAL IDENTITY. All variants are pillar hydrants; fork from this parent.
- **002.png = orange handheld pistol-grip hose NOZZLE** (horizontal barrel, bell inlet, pistol grip, bail shut-off lever). NOT the identity. It contributes only an outlet-cap / diffuser-coupling candidate idea (e.g. Storz-style quarter-turn coupling), never a whole-body candidate. Do not fork pillar variants from the nozzle parent.

parents:
- rec_cast-iron-pillar-fire-hydrant-bright-red-body-wi_20260608_164513_388731_9a6935e0 ← picture/Urban Environment/Fire Hydrant/001.png（**CHOSEN IDENTITY / fork source**。`build_object_model` name="pillar_fire_hydrant"。helper: `_round_chain_link_mesh` / `_add_round_link_chain` / `_rpy_aim_negz` / `_world_vec_to_outlet_local` / inline `add_outlet(...)`。loop 发射 OK: base bolts `for i in range(n_base_bolts)`、bonnet bolts `for i in range(n_bonnet_bolts)`、ribs `for rz in rib_zs`、cap lugs `for j in range(6)`、chain links `for i in range(n_links)`。**READABILITY GAP: 三个 outlet 由 `add_outlet(...)` helper 手写调用 3 次（left_hose / right_hose / front_pumper），positional args，NOT 一个 list-driven `for outlet in outlet_specs` 循环。模板化时应改为 outlet 配置列表 + `outlet_{i}` 循环——这正是 multiplicity 轴。** 基线：domed bonnet / 6-lug 螺旋黄铜 lift-off cap / bolted ground flange / N=3 outlets(2 side + 1 front pumper)）
- rec_orange-handheld-firefighting-hose-nozzle-pistol-_20260608_165737_705219_5af9d100 ← picture/Urban Environment/Fire Hydrant/002.png（**OFF-IDENTITY handheld nozzle**。横向 +X barrel + flared bell inlet + 旋转 fog/spray diffuser collar + pistol grip + bail shut-off lever（REVOLUTE 横 Y 销）。仅供 outlet-cap/coupling 灵感（Storz quarter-turn）。不作为车身候选。）

## 组合数预审（硬门槛 ≥10）
bonnet_shape 3 × outlet_cap_style 2 × base_form 2 × multiplicity N{1,2,3} 3 个不同 N = **3×2×2×3 = 36 ≥ 10 ✓ PASS**
（即便只取 bonnet 3 × N 3 = 9 仍接近；加任一 slot 即过门槛。）

## Slot 候选覆盖

### Slot A: bonnet_shape（顶部 bonnet/cap 形状 —— 主结构轴；operating nut REVOLUTE 保持）
| 候选(future module) | record_id | 关键 part/helper | 结构特征 | 状态 |
|---|---|---|---|---|
| domed（基线） | P_9a6935e0 | DomeGeometry(0.105) bonnet_dome + nut_boss | 低圆顶铸 bonnet | converged(parent) |
| flat_bolted_disc | rec_fire_hydrant_var_bonnet_flat | 平 LatheGeometry 盖板 + bolted flange ring；nut_boss 在平顶中心 | 扁平螺栓圆盖 | converged (to fork) |
| pointed_cone | rec_fire_hydrant_var_bonnet_pointed | 锥形 LatheGeometry（witch-hat spire）；nut_boss 在锥尖 | 高尖锥 bonnet | converged (to fork) |

### Slot B: outlet_cap_style（出水口盖样式 —— PRISMATIC lift-off 保持；chain 保持）
| 候选 | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| knurled_screw_cap（基线） | P_9a6935e0 | cap_geom LatheGeometry + `for j in range(6)` lug Box | 6 棱滚花黄铜螺帽 | converged(parent) |
| storz_lever_cap | rec_fire_hydrant_var_cap_storz | cap + 2 raised lug ear tabs（借自 002 nozzle coupling 灵感） | Storz 1/4-turn 双耳盖 | converged (to fork) |
| plain_dome_bail_cap | rec_fire_hydrant_var_cap_plain | 光滑 DomeGeometry 盖 + 单 bail loop（链穿过 loop） | 光面圆顶单提环盖 | converged (to fork) |

### Slot C: base_form（底部结构）
| 候选 | record_id | 关键 part/helper | 结构特征 | 状态 |
|---|---|---|---|---|
| bolted_flange（基线） | P_9a6935e0 | base_flange LatheGeometry + `for i in range(n_base_bolts)` 黄铜六角栓环 | 宽螺栓地面法兰 | converged(parent) |
| straight_sleeve | rec_fire_hydrant_var_base_sleeve | 直圆柱 sleeve/break-flange skirt，无栓环，落地微 flare | 光面铸套裙座 | converged (to fork) |

## Multiplicity / Copy Logic
- count_param: outlet_count（upper valve chamber 上 outlet+cap+chain 的数量）
- N 样本已覆盖: parent 基线 N=3 (2 side + 1 front pumper)。变体: N=1（单前 pumper, rec_fire_hydrant_var_outlets1）、N=2（双侧对称, rec_fire_hydrant_var_outlets2）、N=3 重排（三等分 120° 同径, rec_fire_hydrant_var_outlets3stack）
- 模板建议 N_range: [1, 4]（1=单 pumper；2=双侧；3=2侧+1前 或 三等分；4=罕见四向）
- copied object: 单个 outlet 子装配 = stub + collar + lift-off cap(PRISMATIC) + body eye(FIXED tether) + 串联 round-link chain(REVOLUTE links)
- naming: outlet_{i}（替换当前手写 left_hose/right_hose/front_pumper）；其下 outlet_{i}_cap / outlet_{i}_tether / outlet_{i}_chain_{j}
- placement: 沿 chamber 周向按 yaw 角配置（对称或 120° 等分）；每个 outlet 可有独立 outlet_r / center_z（pumper 更大更低）
- joint policy: 每个 cap = PRISMATIC 沿自身 outlet 轴向外；每个 chain = serial REVOLUTE links；tether eye = FIXED。所有 outlet 共享同一 add_outlet 逻辑（统一策略）。
- **KEY 模板化动作**: 把 3 个手写 `add_outlet(...)` 调用收成一个 `outlet_specs` 列表 + `for i, spec in enumerate(outlet_specs)` 循环，配置驱动 N。

## 排除项（未来 compatibility matrix 素材 / 已丢弃轴）
- **纯颜色/材质**（红车身、黄铜 vs 铬盖）— 规则禁止，不作为轴。
- **纯缩放**（barrel 高矮粗细）— 规则禁止。
- **rib band 数量**（barrel 上 raised ribs）— 太装饰性/低识别度，作为 inline parent visual，不升为独立轴（可作模板内随机 greeble，不计入组合门槛）。
- **bolt 环数量**（flange 螺栓数）— 同上，纯装饰 multiplicity，不升为轴。
- handheld nozzle 整机拓扑（pistol grip / bail lever / fog diffuser）— 出 pillar-hydrant 识别身份，已排除；仅借 coupling 灵感。
- 待 fork 后回填任何不收敛的轴值组合（漂浮 / 穿插 / joint origin / 出类目）。
