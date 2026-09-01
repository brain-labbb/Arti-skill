# PV-A P0 Blender 碰撞诊断图

> 每类选择 corrected v2 中最早的 completed strict-fail 代表；优先 seed_0000。
> formal state 只保存碰撞计数和深度，link pair 由同一 URDF、同一 joint-vector hash 独立重放取得。

## 总览

![P0 总览](contact_sheet_01.png)

## 逐类

### P0-01 圆规

- 类别：`pictureX_0611_drawing_compass_with_adjustable_legs`
- 资产：`PV-A/pictureX_0611_drawing_compass_with_adjustable_legs/seed_0000`，ordinal `204531`
- 姿态：静止位 q=0，sample `0`
- 问题 pair：`leg_style__sheet_taper__needle_leg` ↔ `leg_style__sheet_taper__lead_leg`
- 本次重放深度：`1.000 mm`
- 说明：两腿真实间隙约 1 mm；v2 的两侧 Bullet margin 把间隙吃掉。同一 seed 在 numerical-zero-margin 的 v3 已通过，属于 v2 误伤。

![圆规](cards/01_pictureX_0611_drawing_compass_with_adjustable_legs.png)
