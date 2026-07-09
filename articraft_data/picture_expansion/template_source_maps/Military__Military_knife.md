# Military / Military knife — template source map

pattern: mixed
parents: rec_model-a-futuristic-military-otf-out-the-front-kn_20260610_080339_548849_fae188ee ← picture/Military/Military knife/001.png (futuristic OTF knife; fills Slot A=otf_prismatic_slide, Slot B=tanto, Slot D=plain; multiplicity baseline N=6 spine serrations)

## Slot 候选覆盖

### Slot A:deployment(刀身展开机构)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| otf_prismatic_slide | rec_model-a-futuristic-military-otf-out-the-front-kn_20260610_080339_548849_fae188ee | parts handle / blade / thumb_slider; joints handle_to_blade(prismatic +X), handle_to_thumb_slider(prismatic +X, Mimic of handle_to_blade); helpers _build_shell_shape / _channel_cut / _build_blade_shape / _build_slider_shape | 刀身沿前槽 +X 滑出(0→0.115 m),藏于中空 channel;拇指滑块在凹陷 track 内同轴随动(reduced mimic ratio) | converged (parent) |
| side_folding_pivot | rec_military_knife_var_foldpivot | parts handle / blade; joint handle_to_blade(revolute Z, 0→π);helpers _build_shell_shape / _build_blade_shape;handle visual pivot_pin(Cylinder),blade visual thumb_stud(Cylinder) | 侧开折刀:刀身绕近前端 Z 轴枢轴翻转 ~180°,合(藏入 slot)↔开(与柄共线);唯一非固定关节 | converged (workbench, rating pending sync) |
| sliding_sheath_fixedblade | rec_military_knife_var_sheathslide | parts handle / sheath; joint handle_to_sheath(prismatic +X);helpers _build_blade_shape(fixed visual tanto_blade on handle) / _build_guard_shape(guard) / _build_sheath_shape(sheath_tube) | 全龙骨固定刀身(handle 上的 visual,非独立 part);管状护鞘 collar 沿刀轴前滑罩住刀尖,guard 板宽于鞘作为后退止挡 | converged (workbench, rating pending sync) |

### Slot B:blade-profile(刀刃轮廓)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| tanto | rec_model-a-futuristic-military-otf-out-the-front-kn_20260610_080339_548849_fae188ee | _build_blade_shape → tanto_blade | 直主刃 + 角折式 tanto 次刃斜上到尖,clipped tip 折回脊;polyline 折线轮廓 | converged (parent) |
| drop_point | rec_military_knife_var_droppoint | _build_blade_shape → tanto_blade | 凸脊平滑下弯与缓上翘 belly 在居中点相交(无角折);用 spline 双曲线收敛到尖 | converged (workbench, rating pending sync) |
| dagger | rec_military_knife_var_dagger | _build_blade_shape → tanto_blade(含 top_ridge union) | 对称双刃匕首,y=0 两侧均为切刃收敛到居中矛尖;顶面中央 spine ridge 凸条;无锯齿 | converged (workbench, rating pending sync) |
| clip_point | rec_military_knife_var_clippoint | _build_blade_shape → tanto_blade | 直主刃近尖微上翘,脊直行 ~2/3 后凹弧 clip 下削到细前尖;n_clip=8 折线近似凹弧 | converged (workbench, rating pending sync) |

### Slot D:pommel/clip(尾端附件)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| plain | rec_model-a-futuristic-military-otf-out-the-front-kn_20260610_080339_548849_fae188ee | handle visuals grip_block / grip_ridge_i / accent_top_j / accent_{side}_j | 朴素尾端:仅 tan grip block + 防滑 grip ridge + 前部橙色 accent;无尾椎/夹 | converged (parent) |
| glassbreak | rec_military_knife_var_glassbreak | helper _build_glass_breaker → handle visual glass_breaker_spike | grip 后端 makeCone 短锥破窗椎(base_r 0.004→tip 0.0004,旋 -90° 指向 -X)+ 安装 collar 圈 | converged (workbench, rating pending sync) |
| pocketclip | rec_military_knife_var_pocketclip | helper _build_pocket_clip → handle visual pocket_clip | +Y 侧面薄簧片口袋夹:后端 anchor + bend + arm + 自由端 return lip;以 CLIP_STANDOFF 离面立起,向前延伸 | converged (workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: 脊背锯齿数(serration notch count;parent 内联 `for i in range(6)`,serr10 提为 `SERRATION_COUNT=10`)
- N 样本已覆盖: {3, 6, 10} → rec_military_knife_var_serr3 / parent / rec_military_knife_var_serr10
- 模板建议 N_range: [2, 20]
- copied object / naming / placement / joint policy: 复制对象=刀脊矩形锯齿缺口(box);for-loop 沿刀脊 +y 等距排列(equidistant pitch — parent 0.012 m;serr3 0.030 m 大齿;serr10 0.010 m 细齿);用 `blade.cut(notch)` 切入刀身,纯几何 cut 不引入关节;命名无单独 visual(齿是刀身减材,不计 part);grip-ridge 多重性归并入本锯齿轴,不单列

## 排除项(未来 compatibility matrix 素材)
- 纯固定刀(fixed-blade,零关节):无任何活动结构,不满足 articulated 类目要求 → 出类目(排除;sliding_sheath_fixedblade 通过外加滑鞘 prismatic 关节才纳入)
- 蝴蝶刀 balisong:双柄绕双枢轴翻转、刃居中夹持,拓扑超出本 named-slot 模板 → 排除
- 弹簧辅助/弹簧自动(spring-assist):需弹簧蓄能-触发联动,无法用纯静态 prismatic/revolute 表达 → 排除
- 握柄防滑棱(grip-ridge)单独作为 multiplicity 轴:与刀脊锯齿轴重复,已折叠进 serration 轴 → 排除(避免双 count_param)
