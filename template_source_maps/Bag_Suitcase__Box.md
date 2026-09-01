# Bag_Suitcase / Box — template source map

pattern: parallel_children(固定 named slots:lid/closure 主机构 + body + walls + hardware + interior;无 multiplicity)

parents:
- rec_rustic-wooden-storage-box-with-plank-sides-metal_20260605_133631_387134_1a9c91ba ← picture/Bag_Suitcase/Box/001.png（rustic 矩形木箱:plank 侧壁 + 金属角件 + 后铰平盖 + 前金属 hasp 扣 + 绳侧把;基线 = lid:hinged_flat_top × body:rectangular × walls:plank × hardware:rope+hasp × interior:plain;**全批 fork 基线**）

20 个变体来自 `bag_suitcase_box_gpt55_20260611` 批次(provider openai / gpt-5.5 / med),全部 compile rc=0、均有 URDF、非 fixed joint 1–4 个;完整 record id 见 `picture_expansion/generated_assets.jsonl`(此处用 variant_index 标注)。

## 组合数预审


## Slot 候选覆盖

### Slot A:lid_closure（主机构槽——盒体的开合动作）
| 候选(未来 module) | variant | 关键 joint / 结构 | 状态 |
|---|---|---|---|
| hinged_flat_top（基线） | parent | 后沿平盖 REVOLUTE ×1 | parent(现成) |
| front_drop_panel | v02 | 前壁下翻 drop panel REVOLUTE,固定顶 rim | converged |
| split_double_leaf_top | v03 | 顶盖两叶,沿中缝各 REVOLUTE | converged |
| sliding_top_panel | v04 | 顶盖沿侧槽 **PRISMATIC** 滑出 | converged |
| hinged_front_door | v17 | 前面板侧铰开门(立轴 REVOLUTE),非顶盖 | converged |
| arched_curved_top | v19 | 后铰 **拱形** 顶盖(helper `_arched_lid_mesh`)REVOLUTE | converged |

### Slot B:body_form（体形/比例;连续尺寸由模板侧缩放,这里只列结构不同的形态）
| 候选 | variant | 结构特征 | 状态 |
|---|---|---|---|
| rectangular_standard（基线） | parent | 标准矩形箱体 | parent |
| low_wide | v01 | 低矮加宽,厚 plank 带 + 大角帽 | converged |
| long_narrow_rounded_ends | v08 | 长窄箱 + 金属圆端帽 | converged |
| shallow_tray | v09 | 浅托盘式,低侧壁 + 透明/展示盖 | converged |
| beveled_corner_posts | v15 | 切角 chamfer 角柱 | converged |
| （tall_upright / square_squat 见 v02 / v03,与 lid 变体同体,作比例样本） | v02/v03 | — | converged |

### Slot C:wall_style（壁面/表面样式）
| 候选 | variant | 结构特征 | 状态 |
|---|---|---|---|
| plank_sides（基线） | parent | 横/竖木板侧壁 | parent |
| ribbed_vertical_slats | v10 | 竖向板条/棱 + 加强件 | converged |
| weathered_crate + corner_blocks | v11 | 风化板条箱 + 凸起角块 | converged |
| reinforced_metal_straps | v16 | 深色加固木箱 + 金属包带 | converged |

### Slot D:hardware（把手 + 闩/扣 两类配件)
| 候选 | variant | 关键 joint / 结构 | 状态 |
|---|---|---|---|
| rope_side_handles（基线） | parent | 绳侧把(无 joint，parent visual) | parent |
| swing_bail_handle | v07 | 侧支架上 **可转** 提梁把 REVOLUTE | converged |
| rotating_side_rings | v16 | 两侧 **可转** 环把 REVOLUTE ×2 | converged |
| hasp_latch（基线） | parent | 前金属 hasp 扣 | parent |
| rotating_clasp_latch | v05 | 深箱 + 前 **旋转** 卡扣 REVOLUTE | converged |
| rotating_hasp_plate | v12 | lockbox + 前 **旋转** hasp 板 REVOLUTE | converged |

### Slot E:interior_base（内部机构 / 底座附加)
| 候选 | variant | 关键 joint / 结构 | 状态 |
|---|---|---|---|
| plain（基线） | parent | 无内部机构 | parent |
| raised_feet | v06 | jewelry-box 抬高脚 + 浅内托 | converged |
| slide_out_front_drawer | v13 | 前抽屉 **PRISMATIC** 拉出 + 固定铰盖 | converged |
| lift_out_inner_tray | v14 | 双深箱内托盘 **PRISMATIC** 垂直升降 | converged |
| sliding_internal_divider | v18 | 内隔板 **PRISMATIC** 左右滑动 + 铰盖 | converged |
| nesting_stackable_feet | v20 | 四个嵌套脚 + 凹底轮廓(可叠) | converged |

## Multiplicity / Copy Logic

- count_param: **无**,核心结构为固定 named slots(box 是单体)。v20 的"嵌套脚 + 凹底"只是可叠特征,不构成 N-复制轴。
- 模板建议:无 N_range;按 SPEC §8 写"无复制逻辑"。

## 格子覆盖(全部 converged;parent 基线计入)

lid 5 空格(v02/v03/v04/v17/v19) + body 4(v01/v08/v09/v15) + walls 3(v10/v11/v16) + hardware 4(v05/v07/v12/v16) + interior 5(v06/v13/v14/v18/v20)。20 个变体已填满规划格子,**Box 小类样本池就绪**。

## 排除项(未来 compatibility matrix 素材)

- (暂无;全 20 收敛。)注意 v02/v03/v16 各带次级轴(体形/壁面)与主轴同体,写 spec 时按 headline 轴归 module,次级特征作比例/装饰参数,勿当独立候选(避免连续尺寸虚胖,FORK_VARIANTS §2 line 65)。
