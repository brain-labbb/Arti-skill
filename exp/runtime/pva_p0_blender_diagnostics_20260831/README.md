# PV-A P0 Blender 碰撞诊断图

> 阅读顺序：先看“问题部件”和“发生了什么”，再看“怎么处理”。
> 图中的红色、橙色是发生接触的两个部件，洋红点是评测找到的接触位置。
> “简化外壳”是物理评测使用的不可见形状；它可能比画面里的真实零件更粗或缺少孔洞。
> 每类选择 corrected v2 中最早完成但未通过的代表物体；精确技术信息保留在每项最后一行。

## 总览

![P0 总览](contact_sheet_01.png)

![P0 总览](contact_sheet_02.png)

![P0 总览](contact_sheet_03.png)

![P0 总览](contact_sheet_04.png)

## 逐类

### P0-01 圆规

- **问题部件**：针尖腿（`leg_style__sheet_taper__needle_leg`）和铅芯腿（`leg_style__sheet_taper__lead_leg`）
- **发生了什么**：针尖腿和铅芯腿实际上留有约 1 mm 缝隙，但评测把两条腿的简化外壳各向外加厚，结果把缝隙算成了 1 mm 重叠。
- **怎么处理**：这是评测误伤。同一物体在不额外加厚外壳的 v3 中已经通过，不需要修改圆规模型。
- **技术记录**：静止位 q=0，重叠深度 `1.000 mm`；资产 `PV-A/pictureX_0611_drawing_compass_with_adjustable_legs/seed_0000`，ordinal `204531`，sample `0`

![圆规](cards/01_pictureX_0611_drawing_compass_with_adjustable_legs.png)

### P0-02 齿轮齿条滑台

- **问题部件**：齿条滑架（`rack_carriage`）和小齿轮（`pinion`）
- **发生了什么**：图中齿条滑架和小齿轮的齿面已经重叠约 1.4 mm；运动时它们本应按固定比例一起走，旧评测却分别摆动，齿位还会错得更严重。
- **怎么处理**：应校准初始齿位，补上齿轮传动比例，并只采样一个主动关节；这样处理后同一物体已经通过。
- **技术记录**：静止位 q=0，重叠深度 `1.427 mm`；资产 `PV-A/rack_and_pinion_slider/seed_0000`，ordinal `225047`，sample `0`

![齿轮齿条滑台](cards/02_rack_and_pinion_slider.png)

### P0-03 同步升降桌

- **问题部件**：0号外腿（`leg_0_outer`）和桌面（`desktop`）
- **发生了什么**：0 号外腿有 18 mm 伸进桌面的安装位置。这里看起来像桌腿插入桌面底部的固定槽，不是桌腿运动错了。
- **怎么处理**：应把桌面的安装槽做成空腔，或只允许这对部件在安装区域内接触，不能放行整条桌腿。
- **技术记录**：静止位 q=0，重叠深度 `18.000 mm`；资产 `PV-A/standing_desk_with_synchronous_telescoping_legs_and_articulated_controls/seed_0000`，ordinal `266373`，sample `0`

![同步升降桌](cards/03_standing_desk_with_synchronous_telescoping_legs_and_articulated_controls.png)

### P0-04 手表

- **问题部件**：表圈（`bezel`）和分针（`minute_hand`）
- **发生了什么**：表圈压到了分针；其他指针之间也很近。评测外壳额外加厚解释了其中约 2 mm，但去掉加厚后仍有真实重叠。
- **怎么处理**：需要把表圈和三根指针按高度重新分层，并把每根指针的碰撞厚度做薄。
- **技术记录**：静止位 q=0，重叠深度 `3.701 mm`；资产 `PV-A/watch/seed_0000`，ordinal `291449`，sample `0`

![手表](cards/04_watch.png)

### P0-05 音频设备

- **问题部件**：柜体（`cabinet`）和0号旋钮（`control_deck__rotary_knob_bank__knob_00`）
- **发生了什么**：0 号旋钮与柜体只重叠约 0.5 mm，位置就在旋钮插入面板的安装处。
- **怎么处理**：这是毫米级评测误差；同一物体在 v3 已通过，不需要改可见模型。
- **技术记录**：静止位 q=0，重叠深度 `0.500 mm`；资产 `PV-A/Technology_Audio_Device/seed_0000`，ordinal `90457`，sample `0`

![音频设备](cards/05_Technology_Audio_Device.png)

### P0-06 钟楼

- **问题部件**：时针（`hour_hand_0`）和分针（`minute_hand_0`）
- **发生了什么**：时针和分针在中心轴位置叠在一起。两根指针本应一上一下安装，但简化外壳太厚，把中心叠装算成了 32 mm 穿透。
- **怎么处理**：应减薄两根指针的碰撞厚度并错开高度；中心轴附近也可单独定义为正常安装接触。
- **技术记录**：静止位 q=0，重叠深度 `32.000 mm`；资产 `PV-A/clock_tower_with_rotating_hour_and_minute_hands/seed_0000`，ordinal `139085`，sample `0`

![钟楼](cards/06_clock_tower_with_rotating_hour_and_minute_hands.png)

### P0-07 压蒜器

- **问题部件**：压柄（`lever_arm`）和枢轴销（`pivot_pin`）
- **发生了什么**：压柄包住枢轴销约 14.5 mm，这正是压柄绕轴转动的铰链位置。
- **怎么处理**：这是正常的轴销连接。应只允许压柄与枢轴销在轴孔附近接触，而不是把整对部件全部放行。
- **技术记录**：静止位 q=0，重叠深度 `14.478 mm`；资产 `PV-A/pictureX_0611_garlic_press/seed_0000`，ordinal `212879`，sample `0`

![压蒜器](cards/07_pictureX_0611_garlic_press.png)

### P0-08 步枪

- **问题部件**：枪管（`barrel`）和护木（`handguard`）
- **发生了什么**：护木本来套在枪管外面，但护木的简化外壳没有中间的孔，所以把里面的枪管算成了 41.8 mm 重叠。
- **怎么处理**：应把护木改成带内孔的碰撞形状，保留枪管穿过的空间。
- **技术记录**：静止位 q=0，重叠深度 `41.755 mm`；资产 `PV-A/Military_Rifle/seed_0000`，ordinal `56828`，sample `0`

![步枪](cards/08_Military_Rifle.png)

### P0-09 锥齿轮副

- **问题部件**：水平锥齿轮轴（`horizontal_drive`）和垂直锥齿轮轴（`vertical_drive`）
- **发生了什么**：水平和垂直锥齿轮的齿面正在啮合。齿轮工作时本来就要接触，齿位不同步时还会互相插得更深。
- **怎么处理**：应固定两只齿轮的传动比例和初始齿位，并允许齿面有很小的正常接触。
- **技术记录**：静止位 q=0，重叠深度 `10.237 mm`；资产 `PV-A/pictureX_0611_bevel_gear_pair_with_perpendicular_shafts/seed_0000`，ordinal `196918`，sample `0`

![锥齿轮副](cards/09_pictureX_0611_bevel_gear_pair_with_perpendicular_shafts.png)

### P0-10 帆船绞盘

- **问题部件**：曲柄（`crank`）和绞盘鼓（`drum`）
- **发生了什么**：曲柄有 22.4 mm 插进绞盘鼓的中心，这里是曲柄轴安装到鼓上的传动位置。
- **怎么处理**：应在绞盘鼓中心保留轴孔，或只允许曲柄轴与轴孔局部接触。
- **技术记录**：静止位 q=0，重叠深度 `22.368 mm`；资产 `PV-A/sailboat_winch_with_pawl_and_handle/seed_0000`，ordinal `250154`，sample `0`

![帆船绞盘](cards/10_sailboat_winch_with_pawl_and_handle.png)

### P0-11 木制秋千

- **问题部件**：座椅（`bench`）和右吊架（`hanger_right`）
- **发生了什么**：座椅与右吊架在第二个连接点重合约 50 mm。秋千需要左右两侧都吊住，但树形关节只把其中一侧记成直接连接。
- **怎么处理**：更像缺少第二侧闭环连接说明，不是座椅运动错误；应只登记右侧吊点这一小块正常连接区域。
- **技术记录**：静止位 q=0，重叠深度 `49.965 mm`；资产 `PV-A/Bench_Wood_Swing/seed_0000`，ordinal `9148`，sample `0`

![木制秋千](cards/11_Bench_Wood_Swing.png)

### P0-12 伸缩拉杆工具箱

- **问题部件**：箱盖（`lid`）和内拉杆（`handle_inner`）
- **发生了什么**：箱盖与内拉杆重叠约 5.3 mm，位置在拉杆穿过箱盖或导向口的地方。
- **怎么处理**：应给箱盖留出拉杆通孔；若这里本来就是导向接触，只允许通孔边缘附近接触。
- **技术记录**：静止位 q=0，重叠深度 `5.287 mm`；资产 `PV-A/rolling_toolbox_with_telescoping_handle/seed_0000`，ordinal `247402`，sample `0`

![伸缩拉杆工具箱](cards/12_rolling_toolbox_with_telescoping_handle.png)

### P0-13 斜切锯臂

- **问题部件**：锯台（`miter_table`）和锯片（`blade`）
- **发生了什么**：锯片穿过锯台约 37 mm。斜切锯本来就需要让锯片进入台面的刀槽，但简化锯台是实心的。
- **怎么处理**：应在锯台碰撞形状中挖出刀槽，不应把整块锯台与锯片的重叠都算成故障。
- **技术记录**：静止位 q=0，重叠深度 `37.000 mm`；资产 `PV-A/miter_saw_arm_assembly/seed_0000`，ordinal `174919`，sample `0`

![斜切锯臂](cards/13_miter_saw_arm_assembly.png)

### P0-14 厨师机

- **问题部件**：搅拌头（`tool_attachment`）和搅拌碗（`mixing_bowl`）
- **发生了什么**：搅拌头伸进搅拌碗约 75.7 mm，这是厨师机的正常工作位置；问题是碗的简化外壳把内部也填实了。
- **怎么处理**：应把搅拌碗做成中空碰撞形状，并只检查搅拌头是否碰到碗壁和碗底。
- **技术记录**：静止位 q=0，重叠深度 `75.654 mm`；资产 `PV-A/stand_mixer/seed_0000`，ordinal `264873`，sample `0`

![厨师机](cards/14_stand_mixer.png)

### P0-15 折叠门

- **问题部件**：机架/门框（`frame`）和第二扇门叶（`leaf_1`）
- **发生了什么**：第二扇门叶在完全不动时就插进门框约 52 mm，不只是边缘轻微接触。
- **怎么处理**：这是需要修的装配问题：重点检查第二扇门叶的安装位置、转轴原点和门框碰撞形状。
- **技术记录**：静止位 q=0，重叠深度 `52.000 mm`；资产 `PV-A/Door_folding_door/seed_0000`，ordinal `29221`，sample `0`

![折叠门](cards/15_Door_folding_door.png)

### P0-16 倾斜式搬运车

- **问题部件**：锁扣（`latch`）和料斗（`hopper`）
- **发生了什么**：锁扣与料斗在扣合位置重叠约 0.53 mm，属于锁扣贴住料斗边缘的毫米级接触。
- **怎么处理**：先去掉评测外壳的额外加厚再判断；若仍接触，只允许锁扣扣合位置这一小块区域。
- **技术记录**：静止位 q=0，重叠深度 `0.531 mm`；资产 `PV-A/Urban_Environment_Tilt_Truck2/seed_0000`，ordinal `105258`，sample `0`

![倾斜式搬运车](cards/16_Urban_Environment_Tilt_Truck2.png)

### P0-17 螺口灯泡与灯座

- **问题部件**：灯泡组件（`bulb_assembly`）和螺纹轴（`thread_axis`）
- **发生了什么**：灯泡的螺纹轴有 9 mm 旋进灯座，这是正常安装；灯座的简化外壳没有螺纹孔，所以把插入部分算成重叠。
- **怎么处理**：应把灯座做成带内孔的形状，或专门允许螺纹啮合区域接触。
- **技术记录**：静止位 q=0，重叠深度 `9.038 mm`；资产 `PV-A/screwin_light_bulb_with_socket/seed_0000`，ordinal `252375`，sample `0`

![螺口灯泡与灯座](cards/17_screwin_light_bulb_with_socket.png)

### P0-18 矿车

- **问题部件**：车斗（`tub`）和锁钩（`latch_hook`）
- **发生了什么**：锁钩伸进车斗边沿约 12 mm，位置像是锁钩扣住车斗的固定点。
- **怎么处理**：先确认锁钩是否设计为扣住车斗；若是，只允许钩口附近接触，否则应调整锁钩安装位置。
- **技术记录**：静止位 q=0，重叠深度 `12.000 mm`；资产 `PV-A/Industrial_Mine_cart/seed_0000`，ordinal `47865`，sample `0`

![矿车](cards/18_Industrial_Mine_cart.png)

### P0-19 弹簧滚珠输送单元

- **问题部件**：外壳（`housing`）和主滚珠（`load_ball`）
- **发生了什么**：主滚珠有 15.9 mm 嵌在外壳里，这是外壳托住滚珠、防止它掉出的正常结构。
- **怎么处理**：应把外壳做成带球窝的中空形状，只检查滚珠是否穿出球窝边界。
- **技术记录**：静止位 q=0，重叠深度 `15.887 mm`；资产 `PV-A/pictureX_0611_ball_transfer_unit_with_spring_loaded_ball/seed_0000`，ordinal `195307`，sample `0`

![弹簧滚珠输送单元](cards/19_pictureX_0611_ball_transfer_unit_with_spring_loaded_ball.png)

### P0-20 伸缩臂

- **问题部件**：外套筒（`outer_stage`）和内套筒（`inner_stage`）
- **发生了什么**：内套筒在组合运动姿态下有 78.8 mm 位于外套筒内部。伸缩臂本来就要这样套叠，但外套筒被当成了实心。
- **怎么处理**：应把外套筒做成空心管，并检查内套筒是否越过管壁或超过伸缩限位。
- **技术记录**：多关节 Sobol 姿态，重叠深度 `78.764 mm`；资产 `PV-A/telescoping_boom/seed_0000`，ordinal `275346`，sample `27`

![伸缩臂](cards/20_telescoping_boom.png)

### P0-21 折叠椅

- **问题部件**：前腿框（`front_leg_frame`）和座板（`seat_pan`）
- **发生了什么**：前腿框在静止位插进座板约 32.6 mm，重叠范围明显大于一个小铰链接触点。
- **怎么处理**：需要检查座板和前腿框的安装位置；转轴附近可以局部接触，但大面积相交仍应修正。
- **技术记录**：静止位 q=0，重叠深度 `32.649 mm`；资产 `PV-A/Chair_Folding_chair/seed_0000`，ordinal `11004`，sample `0`

![折叠椅](cards/21_Chair_Folding_chair.png)

### P0-22 航天飞机

- **问题部件**：左方向舵板（`left_rudder_panel`）和右方向舵板（`right_rudder_panel`）
- **发生了什么**：左右方向舵板在合拢的中缝处重叠约 2 mm，看起来是两块舵面贴得过紧。
- **怎么处理**：先去掉评测外壳的额外加厚；若仍重叠，应在中缝留出小间隙或减薄舵板碰撞形状。
- **技术记录**：静止位 q=0，重叠深度 `2.000 mm`；资产 `PV-A/Astronomy_Space_shuttle/seed_0000`，ordinal `3122`，sample `0`

![航天飞机](cards/22_Astronomy_Space_shuttle.png)

### P0-23 颚式破碎机

- **问题部件**：偏心轴（`eccentric_shaft`）和动颚（`swing_jaw`）
- **发生了什么**：偏心轴穿进动颚约 316 mm，另外机架和肘板也相交。这是大范围实体重叠，不是毫米级误差。
- **怎么处理**：需要重新检查偏心轴、动颚和肘板的安装位置及碰撞形状，不能用接触白名单掩盖。
- **技术记录**：静止位 q=0，重叠深度 `316.112 mm`；资产 `PV-A/Industrial_Ore_crusher_jaw/seed_0000`，ordinal `48800`，sample `0`

![颚式破碎机](cards/23_Industrial_Ore_crusher_jaw.png)

### P0-24 鼓踏板

- **问题部件**：踏板机架（`frame_style__cast_post__frame`）和打击杆调节螺钉（`beater_style__fixed_head__angle_screw`）
- **发生了什么**：打击杆调节螺钉有约 7 mm 穿进踏板机架，位置像螺钉穿过机架上的调节孔。
- **怎么处理**：若这是正常螺纹安装，应给机架留孔并只允许孔内接触；否则调整螺钉安装位置。
- **技术记录**：静止位 q=0，重叠深度 `7.000 mm`；资产 `PV-A/pictureX_0611_drum_pedal_with_beater_and_spring_return/seed_0000`，ordinal `207731`，sample `0`

![鼓踏板](cards/24_pictureX_0611_drum_pedal_with_beater_and_spring_return.png)

### P0-25 跑车

- **问题部件**：车身（`body`）和左前轮（`wheel_front_left`）
- **发生了什么**：左前轮被算进车身约 340 mm。可见模型有轮拱，但车身的简化外壳把轮拱封住了。
- **怎么处理**：应重做车身碰撞形状，明确挖出四个轮拱，不能把车身做成覆盖车轮的整块实体。
- **技术记录**：静止位 q=0，重叠深度 `340.055 mm`；资产 `PV-A/Vehicle_Sports_car/seed_0000`，ordinal `110713`，sample `0`

![跑车](cards/25_Vehicle_Sports_car.png)

### P0-26 陀螺仪

- **问题部件**：外万向环（`outer_gimbal_ring`）和转子（`rotor`）
- **发生了什么**：转子位于外万向环内部，这是陀螺仪的正常结构；外环的简化外壳没有保留中间开口，因而重叠约 60.9 mm。
- **怎么处理**：应把万向环做成真正的环形碰撞形状，保留转子的旋转空间。
- **技术记录**：静止位 q=0，重叠深度 `60.857 mm`；资产 `PV-A/pictureX_0611_gyroscope_with_spinning_wheel_and_gimbal_rings/seed_0000`，ordinal `215845`，sample `0`

![陀螺仪](cards/26_pictureX_0611_gyroscope_with_spinning_wheel_and_gimbal_rings.png)

### P0-27 伸缩遮阳篷

- **问题部件**：前横梁（`head`）和0号前臂杆（`forearm_0`）
- **发生了什么**：0 号前臂杆插入前横梁的铰接安装座约 157.8 mm，位置是伸缩臂与横梁的连接处。
- **怎么处理**：应给前横梁保留安装槽和轴孔，并只允许铰接座内部的局部接触。
- **技术记录**：静止位 q=0，重叠深度 `157.814 mm`；资产 `PV-A/retractable_patio_awning/seed_0000`，ordinal `232734`，sample `0`

![伸缩遮阳篷](cards/27_retractable_patio_awning.png)

### P0-28 地球仪

- **问题部件**：底座（`base`）和球体（`globe`）
- **发生了什么**：球体位于底座支架的开口内，但底座的简化外壳封住了开口，于是把半个球都算进底座，深度达到 514.9 mm。
- **怎么处理**：应把底座改成有开口的支架形状，保留球体旋转所需的内部空间。
- **技术记录**：静止位 q=0，重叠深度 `514.875 mm`；资产 `PV-A/globe/seed_0000`，ordinal `154545`，sample `0`

![地球仪](cards/28_globe.png)

### P0-29 节拍器

- **问题部件**：外壳（`housing`）和滑动配重（`sliding_weight`）
- **发生了什么**：滑动配重在静止位已经伸进外壳约 15 mm，摆杆运动后还会更深。
- **怎么处理**：需要检查配重的滑动轨迹和外壳开口；配重只能沿摆杆移动，不能进入封闭外壳。
- **技术记录**：静止位 q=0，重叠深度 `15.000 mm`；资产 `PV-A/metronome/seed_0000`，ordinal `168519`，sample `0`

![节拍器](cards/29_metronome.png)

### P0-30 压线钳

- **问题部件**：活动手柄（`moving_handle`）和驱动连杆（`drive_link`）
- **发生了什么**：活动手柄与驱动连杆在转轴位置重叠约 3.1 mm，看起来是连杆装进手柄铰链的连接处。
- **怎么处理**：应把转轴孔做出来，并只允许铰链中心附近接触；其余位置仍需保持分离。
- **技术记录**：静止位 q=0，重叠深度 `3.100 mm`；资产 `PV-A/pictureX_0611_crimping_tool/seed_0000`，ordinal `203231`，sample `0`

![压线钳](cards/30_pictureX_0611_crimping_tool.png)

### P0-31 铆钉挤压钳

- **问题部件**：上压模（`head_module__stepped_cylinder_dies__upper_die`）和下压模（`head_module__stepped_cylinder_dies__lower_die`）
- **发生了什么**：上压模与下压模在工具闭合姿态接触约 1 mm，这是夹紧铆钉时应有的闭合位置。
- **怎么处理**：应结合工具是否处于闭合终点来判断；闭合时允许压模端面接触，其他姿态仍不能互穿。
- **技术记录**：多关节 Sobol 姿态，重叠深度 `1.003 mm`；资产 `PV-A/rivet_squeeze/seed_0000`，ordinal `235620`，sample `17`

![铆钉挤压钳](cards/31_rivet_squeeze.png)

### P0-32 风力机

- **问题部件**：转子（`rotor`）和锁止销（`lock_pin`）
- **发生了什么**：锁止销插进转子约 68.9 mm，这是风机停机时用销锁住转子的结构。
- **怎么处理**：应在转子上保留锁孔，并根据锁止销是否插入来判断；不能把锁定状态直接算成故障。
- **技术记录**：静止位 q=0，重叠深度 `68.909 mm`；资产 `PV-A/wind_turbine/seed_0001`，ordinal `297744`，sample `0`

![风力机](cards/32_wind_turbine.png)

### P0-33 带箍桶

- **问题部件**：桶体（`barrel_body`）和锁紧环（`clasp_ring`）
- **发生了什么**：锁紧环压住桶体约 4.1 mm，这是锁紧环夹住桶口或桶身的安装区域。
- **怎么处理**：应让锁紧环贴合桶体表面，并只允许夹持带这一圈发生小范围接触。
- **技术记录**：静止位 q=0，重叠深度 `4.056 mm`；资产 `PV-A/Container_Barrel/seed_0000`，ordinal `11657`，sample `0`

![带箍桶](cards/33_Container_Barrel.png)

### P0-34 双滑动关节链

- **问题部件**：末端滑块（`end_effector`）和第一级滑块（`first_stage`）
- **发生了什么**：末端滑块在静止位顶进第一级滑块约 2.5 mm，位置在两级滑块的行程起点。
- **怎么处理**：需要检查零位偏移和机械止挡；若这是收到底的止挡接触，只允许端面接触，不能继续向里穿。
- **技术记录**：静止位 q=0，重叠深度 `2.491 mm`；资产 `PV-A/twojoint_prismatic_chain/seed_0001`，ordinal `282737`，sample `0`

![双滑动关节链](cards/34_twojoint_prismatic_chain.png)

### P0-35 百叶窗组件

- **问题部件**：第6片百叶片（`louver_slat_0_5`）和0号联动杆（`tilt_rod_0`）
- **发生了什么**：第 6 片百叶片与 0 号联动杆在连接点重叠约 5 mm，这是联动杆带动叶片转动的铰接位置。
- **怎么处理**：应把这个跨支路连接登记为铰链，只允许销轴和孔附近接触。
- **技术记录**：静止位 q=0，重叠深度 `5.000 mm`；资产 `PV-A/louvered_shutter_assembly/seed_0001`，ordinal `163781`，sample `0`

![百叶窗组件](cards/35_louvered_shutter_assembly.png)

### P0-36 转台

- **问题部件**：转盘（`platter`）和唱臂（`tonearm`）
- **发生了什么**：唱臂在多个关节一起运动的姿态下碰到转盘约 3.7 mm，说明唱臂高度、转动范围或停靠路线没有配合好。
- **怎么处理**：应联合限制唱臂的升降和旋转：落针时只让针尖接近唱片，移动和停靠时唱臂不能扫进转盘。
- **技术记录**：多关节 Sobol 姿态，重叠深度 `3.724 mm`；资产 `PV-A/turntable/seed_0000`，ordinal `280269`，sample `54`

![转台](cards/36_turntable.png)
