# 视觉多样性模型(Visual Diversity Model)— 总纲

模板多样性的**唯一权威定义**与改造规划。SPEC_TEMPLATE / FORK_VARIANTS / DESIGN_RULES /
SKILL 的多样性相关条款都以本文为准。

## 一句话总纲

把"**只奖励一根拓扑轴 + 数 distinct**"的旧机制,换成"**6 轴视觉多样性模型 + 逐轴强制考察
其余都是可选硬化。

---

## 1. 模型:一个目标,6 根轴(嵌套字段),3 把尺

**目标只有一个:视觉多样性**(肉眼看见的一切)。一个铰链物体是一棵嵌套结构,每根轴 = 树上一个字段:

```
物体(视觉多样性)
├─ 图/骨架   : 哪些 part、谁连谁(边存在)、结构复制数 N      ← ① 骨架图
│     └─ 每条边: type / 轴                                  ← ② 关节类型(嵌在"边"下)
├─ 每个 part :
│     ├─ 主体形态原型(平面边界/体量包络/宏观表面构成)       ← ③ 主体形态家族
│     ├─ 表面叠加细节 + 装饰数 N                            ← ④ 表面装饰
│     └─ 连续尺寸                                          ← ⑤ 尺寸/行程
├─ 每条边   : 行程/极限(连续)                              ← ⑤ 尺寸/行程(同一根)
└─ 每个面   : 材质 / 颜色                                   ← ⑥ 涂装
```

| # | 轴(字段) | 完整定义 | 尺子 | 进 slot_choices | 计入"结构 distinct" |
|---|---|---|---|---|---|
| **①** | **骨架图** | part-joint **运动学图**:会动的 part、谁连谁(**边存在**)、结构复制 N。**非数学亏格**(洞归③)。realized part 图是产物,slot 是生成它的旋钮 | **计数**(distinct 图,**去 N**)+ **N 覆盖** | 是 | 见 §2(已删) |
| **②** | **关节类型** | 每条边的**标签**:fixed/revolute/prismatic/continuous + 轴。边**存在**=①,边**是什么**=② | **覆盖**(支持的每种类型都出现) | 是(随 module) | — |
| **③** | **主体形态家族 / Primary Form Family** | 在 part tree / joint graph / interface 基本不变时,核心 part 的**离散几何形态原型**变化。包含 Planar Boundary Form(核心面/截面/开口/投影轮廓)、Volumetric Envelope Form(三维包络/厚薄分布/扫掠/旋转/loft 母线)、Macro Surface Construction(足以改变类别内读法的大尺度表面构成)。**结构↔外观的桥**;mesh 的洞/亏格在这 | **计数/覆盖**(声明原型都出现) | 是 | — |
| **④** | **表面装饰** | **叠加表面、不改主体形态原型**的细节:标签/印刷/条纹/肋/压缝/滚花/饰带/铆钉,常带装饰 N。比涂装多(有几何/摆放),比主体形态少(不改核心 envelope / construction) | **覆盖**(各 style + 各装饰 N 出现) | 否/不计 | 否 |
| **⑤** | **尺寸/行程** | **连续**:per-part 尺寸比例 + per-edge 关节行程/极限。喂 mesh **+ 关节原点 + 关节极限 + 锚定**(非只 mesh)。含约束/派生层 | **范围**(比例+行程跨度够宽)**+ 约束层**(可行性守卫) | 否(连续) | 否 |
| **⑥** | **涂装** | **材质大类**(glass/plastic/metal/ceramic/painted)+ 颜色/palette。纯外观,零几何 | **覆盖**(材质大类 ≥ ceil(0.5×声明)) | 否 | 否 |

---


夹带进 tuple 才普遍成立;③ 一旦拆出外观通道 + N 不计数,纯结构 distinct 会塌、≥10 反而误杀。

**替换 = 逐轴用各自的尺子查覆盖率/范围,无任何 distinct 门槛:**

```
视觉多样性裁决 = 对每根"声明为有"的轴:
   ① 骨架图    : 声明的结构都被采到 + 结构 N 覆盖(≥2-3)   ── N 不计数,只覆盖
   ② 关节类型  : 声明的类型都出现
   ③ 主体形态家族:声明的原型都出现
   ④ 表面装饰  : 声明的 style/装饰 N 都出现
   ⑤ 尺寸/行程 : 比例+行程范围拉开 + 约束层全程可行
   ⑥ 涂装      : 材质大类覆盖
"声明为无+理由"的轴 → 跳过(spec review 背书)
```

而且覆盖隐含了 distinct 下界(声明 ≥2 形状 → 采样器必须产 ≥2 distinct)。防"声明太薄"靠 **§8 +
人工 spec review**,不靠数字 gate(那会误杀井盖这种真就少的)。

**N 永不计数(实证):** 83 个模板把 N 放进 slot_choices,N 把 distinct 膨胀 3-24×,**6 个模板完全靠 N 撑过旧 gate**(去掉 N 跌破 10:Gutter_downchain=5 / Fence_Cascade=6 / Curtain_blind=6 / Container_Cosmetic=9)。所以 N 只覆盖、不计数。

---

## 3. §8 纪律:逐轴强制考察,上下游各一道

multiplicity 现成的榜样(SPEC_TEMPLATE §8:"每个 spec 都必须写本节,无也要明确说明没有")要复制给
**每一根轴**:**强制考察 → 声明"有/无"+理由,空格即不合格。**

- **上游**(source map / FORK_VARIANTS):规划阶段 6 轴逐根考察。
- **下游**(SPEC_TEMPLATE):6 轴逐根声明小节。

**它不是"豁免/逃生口",是一次主动考察**:结论可以是"无",但必须考察过、写明理由(微波炉声明"形状内在单一"
即合格)。

---

## 4. 判定树:任何一个变化,落到唯一主字段

```
1. 加/减会动的 part 或一条边、改 N ?           → ① 骨架图(边的存在)
2. 图不变,某条边换 type/轴 ?                  → ② 关节类型(边的标签)
3. 图&关节不变,核心 part 的可识别几何形态原型变了 ? → ③ 主体形态家族
4. 原型不变,叠加表面细节/改装饰数 ?           → ④ 表面装饰
5. 离散全不变,只连续改尺寸/比例/行程 ?        → ⑤ 尺寸/行程
6. 几何全不变,只改材质/颜色 ?                 → ⑥ 涂装
```
一个 module 可同时命中多条(螺帽=①+②+③)→ 各记一笔。**轴的定义干净 ≠ 每个 module 只碰一根。**

③ 的快速三问法:

- **边界变了吗?** 核心面、截面、开口或投影轮廓发生离散变化 → Planar Boundary Form。
- **包络变了吗?** 三维 envelope、厚薄分布、扫掠/旋转/loft 母线发生离散变化 → Volumetric Envelope Form。
- **宏观表面构成变了吗?** 主体表面的网格、穿孔、大尺度分块、faceting/corrugation 等构成方式足以改变类别内读法 → Macro Surface Construction。

如果答案只是"贴了标签/色带/小铆钉/浅纹理"→④;只是"变大/变小/行程变长"→⑤;新增/删除核心活动 part 或结构模块→①。

---

## 5. 两道几何正确性守卫(治真实缺陷;属约束层,非新轴)

**⑤ 扫掠碰撞**:关节全程(continuous 则整圈)不得穿模。sweep 的 compiler baseline 现已默认运行
`harness_motion_qc`(采样姿态重叠门,`{0,lower,upper,mid}`+组合姿态),"微波炉门朝内开穿本体"
这类缺陷会被默认拦截;模板自己的 `fail_if_parts_overlap_in_sampled_poses`
(TestContext 方法,model_checks.py) 仍推荐作为 belt-and-suspenders(供 record 级 run_tests 使用),
或写明 `sampled-pose exemption` 并用 targeted `ctx.pose(...)` 覆盖合法姿态。每个
非-continuous 关节在 spec ⑤ 写一条**运动包络**(轴/开启方向/[闭合,可行上界])和
`motion_test_plan`。

**④ 共形嵌入**:装饰几何须**由宿主表面逐-z profile 派生**、随 ③⑤ 共形、嵌入全足迹。反例:Container_Tube
`label_band` 用**常数 belly 半径**套在收锥体外,生硬不贴合;固定直 `Box` 平贴曲/斜面同病。派生顺序 **③→⑤→④**
(装饰最后生成,读最终表面)。

---

## 6. 上游(造变体):考察 6 轴,但区分 source-backed 与受控外推

**6 轴 ≠ fork 6 轴。** 判据:这根轴模板作者是"必须有几何源样本才能保证 part/joint/interface 正确",还是"有 anchors 后可由世界知识参数化且可被 sweep/reviewer 守住"?

| 轴 | 上游 | |
|---|---|---|
| ① 骨架图 / ② 关节类型 | **forked_anchor / source-backed** | 世界知识可辅助命名和归纳,但不得直接新增未被原始资产或 fork anchor 支撑的 skeleton/joint candidate。 |
| ③ 主体形态家族 / Primary Form Family | **fork 足够 anchors + 可 world_knowledge_extrapolation** | ③ fork 的是**锚**(证 SDK 可造 + 立接口 + 钉类别),不是全集;锚数量按类别复杂度决定(常见 2–5,复杂类更多)。模板侧世界知识扇出其余 primary-form candidate 时必须同 part tree / primitive / interface,只改变 Planar Boundary / Volumetric Envelope / Macro Surface Construction 中的离散形态参数,Rule 3 内,两道闸 sweep 必过 + reviewer。 |
| ④ 表面装饰 | **record_only + 可 world_knowledge_extrapolation** | 记录真实样本里的装饰 style / 装饰 N;模板侧可扩展 host-conformal、非结构、非关节、非新功能模块的 ribs / panel seams / rivets / labels / bands。 |
| ⑤ 尺寸/行程 / ⑥ 涂装 | **只记录**(范围 / 材质大类),**不 fork** | 采样器免费生成 / palette 直接实现 |

source map 必须标注来源类型:`forked_anchor`、`world_knowledge_extrapolation`、`record_only`、`compatibility_probe`。fork 变体采用 primary-axis 控制:普通变体只有一个主结构轴,④/⑤/⑥ 可作为低风险 companion variations,高风险跨轴组合只可作为 `compatibility_probe`。

---

## 7. 五条硬原则

1. **轴 ≠ slot**:轴=输出物体的字段(干净);slot=实现旋钮(可捆绑多字段)。spec 标注每 module 动了哪几个字段。
2. **N 永不计入结构 distinct**,只覆盖——否则膨胀挤占真结构多样性。
3. **骨架图 = 运动学 part-joint 图,不是数学拓扑/亏格**;洞、不动的把手、主体 envelope / construction 变化都归 ③ 主体形态家族。
4. **耦合 = 嵌套**,不是定义重叠;顺着树逐层考察,别压成平铺独立维度。
5. **sweep 40-50 即够**(代表性抽样),**别和训练产量混**——500 是训练生成量、你定、和 sweep 解耦;
   多样性靠模板 6 轴而非 seed 数;偶发坏 seed 生成时跳过即可。

---

## 8. 80/20 核心:6 轴 checklist(spec 必填,极简版)

> 规则:6 根轴每根都必须考察,要么"有→列出来",要么"无→写理由"。不准留空。

| 轴 | 怎么判断 | 有/无 | 若【有】列取值/范围 + source_type/来源 · 若【无】写理由 |
|---|---|---|---|
| ① 骨架图 | 加/减会动 part 或边 | | 结构形态有哪几种；必须 source-backed |
| └ multiplicity | 同构件 ×N | | 有→N 域;无→"无"+理由 |
| ② 关节类型 | 同连接关节怎么动不同 | | revolute/prismatic/continuous…；必须 source-backed |
| ③ 主体形态家族 / Primary Form Family | 换可识别几何形态原型(非缩放/换色) | | source-backed anchors + 可外推 candidate;标明 Planar Boundary / Volumetric Envelope / Macro Surface Construction;无→单一主体形态+理由 |
| ④ 表面装饰 | 不改轮廓的表面叠加细节(+装饰数) | | 真实装饰记录 + 可 host-conformal 世界知识外推 |
| ⑤ 尺寸/行程 | 只连续改大小/比例/关节行程 | | 关键比例 [min,max] + 关节行程范围 + **每非-continuous 关节运动包络** |
| ⑥ 涂装 | 只改材质/颜色 | | 材质大类 + 配色 ≥3-6 |

> 收尾:做完后确认——每个"有"里列的取值,在 batch 0-9 的 seed 里都肉眼出现了(形状拉开、材质大类出现、装饰贴合、关节不穿模)。

**最小可行 = 只做这张表 + 人工 review**,不写一行代码就堵住"漏想外观/形状/装饰"。其余全是可选硬化。

---

## 9. 分阶段规划(定稿·精简版)

> 核心价值 = **A 一张 §8.5 checklist(声明)+ C 把 gate 原地改成逐轴覆盖(核对)**。其余为可选硬化/缓办。
> 不做 report→warn→gate 长坡道自动化(已被"原地改判据"取代);⑤⑥ 全库已健康,不加负担。

| 阶段 | 做什么 | 类型 | 状态 |
|---|---|---|---|
| **P0 先量** | 全库审计(248 模板:~22 孤儿③ / 6 全靠N撑 / ⑤⑥已健康)+ 3 代表模板端到端试点 | 只读 | ✅ |
| **A SPEC** | SPEC_TEMPLATE §8.5 6 轴必填考察 + §4 主体形态家族 slot + SPEC_REVIEW 逐轴审查点 | 文档 | ✅ |
| **B 一致性** | 路由到 §8.5(entrypoint/workflow/skill)+ 修 3 句反向措辞 + FORK source-map 加 6 轴节 + AUTHORING §A 加 Rule4(④共形)/Rule5(⑤swept) | 文档 | 进行中 |
| **C′ swept 约定** | 新模版 `run_tests` 必调 `fail_if_parts_overlap_in_sampled_poses`(每关节 min/max/mid 三态,`max_pose_samples` = min(128, max(32, 1+4·关节数)))并补 targeted `ctx.pose(...)`；高自由度可写 `sampled-pose exemption`；**不碰 compiler baseline、零旧库波及** | 文档/约定 | — |
| **C gate 原地改判据** | :整条元组 distinct→ **逐 registered-slot-key 覆盖**(每登记 key,N 不计数);gate 名/verdict/JSON/CLI 不动。覆盖 ①②③(登记的 slot);④⑤⑥ 由 §8.5+review 兜 | 代码 | ✅ 已上线 |
| **缓办 retrofit** | 孤儿③登记 / N-撑补结构 / 旧库回灌 swept | 改模板 | 缓办 |

**gate 原地改造关键事实(C):** 逐 key 无 product 可膨胀 → **N-撑问题自然消解,不需 N 检测**;唯一边角是
conditional/gated slot 在小 sweep 下欠采 → 用 per-key `reachable` 豁免(`≥ min(2, reachable)`);全库 dry-run 校准。

**三句反向措辞(B 已修):**
1. FORK_VARIANTS §3.1「颜色/材质/比例不是轴」→「不是 fork 轴,但必须考察并记录」。✅
2. AUTHORING §B ✅-加一条:换 ③ 主体形态原型(同 part tree、不同 planar boundary / volumetric envelope / macro surface construction)本身即合法 module。✅
3. SPEC_TEMPLATE §9「主多样性必须来自 slot/module」→ category-relative(形态主导类 = ③ slot)。✅

---

## 10. 诚实的边界(没解决的)

- 覆盖只验"每值出现",不验"值之间肉眼够不同"——"声明5个但长一样"只有人工 viewer(GATE P4)兜得住。
- 质量是长尾:守卫只覆盖已识别的几类缺陷(穿模、装饰脱节),其余靠人审。
- 三目标有张力:**多样性↑ 会顶坏稳定/质量**;守卫+约束层的作用是让"更多样"与"不崩、不丑"**兼容**,不是额外白赚。
- 这套是**设计,不是结果**——价值未经 P0 验证前不要全铺;先在 2-3 个模板上证出来。
