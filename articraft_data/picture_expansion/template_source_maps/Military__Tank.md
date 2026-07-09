# Military / Tank — template source map

pattern: mixed

> ⚠️ **Tank-vs-APC label note**: the 小类 is labeled **Tank**, but the parent
> (and every variant) is a **wheeled armored personnel carrier (APC)**, not a
> main battle tank. The reference asset `picture/Military/Tank/001.png` is an
> 8x8 BTR-style APC. Keep this family inside the **wheeled-APC** vocabulary:
> the `tracked` candidate is a deliberate category-risk probe (BMP-style) and
> must NOT be allowed to drift into MBT territory — see 排除项.

parents: rec_model-an-eight-wheeled-armored-personnel-carrier_20260610_081616_840659_407e2520 ← picture/Military/Tank/001.png
  (8x8 APC: 8 CONTINUOUS road wheels + turret yaw + autocannon elevation = 10 nonfixed joints; wheels driven by `AXLE_X` 4-tuple. Fills Slot A `wheeled_8x8`, Slot B `autocannon_turret`, Slot C `sloped_welded`.)

## Slot 候选覆盖

### Slot A:running-gear
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| wheeled_8x8 | rec_model-an-eight-wheeled-armored-personnel-carrier_20260610_081616_840659_407e2520 | `{side}_wheel_{idx}` part; `{side}_wheel_{idx}_spin` (CONTINUOUS, axis Y); `{side}_axle_stub_{idx}`; `AXLE_X=(2.60,1.25,-1.10,-2.45)` | 4 road-wheel pairs/side, ~7.6 m hull; TireGeometry+WheelGeometry meshes mirrored per side, off-axis `valve_stem` proves spin | converged (parent) |
| wheeled_6x6 | rec_armored_vehicle_var_6x6 | same `{side}_wheel_{idx}` / `_spin`; `AXLE_X=(2.50,0.00,-2.50)` | 3 evenly-spaced axle pairs/side (6 wheels); adds "three axle pairs evenly spaced" span check | converged (workbench, rating pending sync) |
| tracked | rec_armored_vehicle_var_tracked | `{side}_bogie_{i}` part; `{side}_bogie_{i}_spin` (CONTINUOUS, axis Y); `{side}_track_band` / `{side}_drive_sprocket` / `{side}_idler_disc` (fixed hull visuals); `BOGEY_XS=(2.20,1.10,0,-1.10,-2.20)`, `N_BOGEYS=5` | BMP-style tracked gear: 5 bogie pairs ride inside a continuous track loop, rear sprocket + front idler + suspension arms/bump stops; **category-risk** | converged (workbench, rating pending sync) |

### Slot B:weapon-station
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| autocannon_turret | rec_model-an-eight-wheeled-armored-personnel-carrier_20260610_081616_840659_407e2520 | `turret` + `autocannon` parts; `turret_traverse` (CONTINUOUS, axis +Z); `gun_elevation` (REVOLUTE, axis −Y, −5..+35°); `trunnion_housing`, `barrel_tube`, `muzzle_device`, coax MG (`coax_barrel`/`coax_flash_hider`), `aa_mg_*` | conic-frustum turret w/ camo patches, mantlet stack, smoke launchers, bustle basket; long thin autocannon (`GUN_TUBE_LEN=3.35`) on trunnion `GUN_TRUNNION=(0.88,0,0.23)` | converged (parent) |
| remote_weapon_station | rec_armored_vehicle_var_rws | `rws_pod` + `rws_gun` parts; `rws_traverse` (CONTINUOUS, axis +Z, parent=hull); `rws_gun_elevation` (REVOLUTE, axis −Y, parent=rws_pod); `rws_pedestal`, `sensor_window`/`day_camera`/`thermal_camera`/`lrf_window`, `barrel_tube`/`flash_hider` | compact unmanned RWS cupola on roof: sensor-and-gun pod, one slim MG; replaces full turret with low pod | converged (workbench, rating pending sync) |
| open_pintle | rec_armored_vehicle_var_pintle | `pintle_mount` part; `pintle_traverse` (CONTINUOUS, axis +Z, parent=hull); `pintle_ring_race` (fixed on hull) + `pintle_rotating_ring`/`pintle_spider_plate`/`pintle_post`/`yoke`, `mg_barrel`/`mg_receiver` | open troop-deck roof, pintle-mounted MG on circular ring race over forward hatch; yaw only (no powered elevation joint) | converged (workbench, rating pending sync) |

### Slot C:hull-armor
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| sloped_welded | rec_model-an-eight-wheeled-armored-personnel-carrier_20260610_081616_840659_407e2520 | `hull` part, `hull_shell` (`_build_hull_solid`); lower+upper tumblehome `yz_prism` cuts; wedge `nose_cut`; `trim_vane`, `{side}_fender_strip` | faceted olive-drab welded hull, boat-hull lower glacis + inward-leaning upper sides + wedge bow | converged (parent) |
| slab_mrap | rec_armored_vehicle_var_slab | `hull` / `hull_shell` (MRAP `_build_hull_solid`: near-vertical sides, no tumblehome); checks "Z extent ≥ 1.60 m" + "full Y width at roof" | tall upright slab-sided MRAP hull, high boxy roofline, modest approach/departure angles | converged (workbench, rating pending sync) |
| applique_panels | rec_armored_vehicle_var_applique | `hull` + `panel_{i}` visuals (bolt-on tiles w/ 4 corner standoff bosses each); 6 side panels/side + 8 glacis panels (2×4); `_AP_W/_AP_H/_EMBED` consts | sloped_welded hull base + bolt-on standoff appliqué armor tile array (sides + forward glacis) | converged (workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: **road-wheel pairs per side** (encoded by length of `AXLE_X` tuple; tracked uses `BOGEY_XS` / `N_BOGEYS`)
- N 样本已覆盖: {3, 4, 5} → rec_armored_vehicle_var_6x6 (3 pairs / 6x6) / parent rec_model-an-eight-wheeled-armored-personnel-carrier_... (4 pairs / 8x8) / rec_armored_vehicle_var_10x10 (5 pairs / 10x10)
- 模板建议 N_range: **[3, 5]** (heavy-wheeled-APC vocabulary; do NOT exceed 5 — beyond it leaves the wheeled-APC class)
- copied object / naming / placement / joint policy:
  - copied object: one road **wheel** = `tire` (TireGeometry mesh) + `rim` (WheelGeometry mesh) + off-axis `valve_stem`; per-wheel fixed `{side}_axle_stub_{idx}` lives on the hull.
  - naming: `{side}_wheel_{idx}` parts and `{side}_wheel_{idx}_spin` joints, side ∈ {left, right}, idx 0..N−1 fore→aft (tracked: `{side}_bogie_{i}` / `_spin`).
  - placement: fore/aft x taken from the `AXLE_X` tuple (even spacing), lateral y = ±`WHEEL_Y` (1.22), z = `AXLE_Z` (=`WHEEL_RADIUS`); arches cut from hull at the same `AXLE_X` points.
  - joint policy: each wheel = one **CONTINUOUS** joint, axis `(0,1,0)` (CONTINUOUS lateral axle, one continuous lateral axle per wheel), parent=hull, effort 900 / velocity 30; meshes mirrored per side so the one-sided hub cap faces outboard.

## 排除项(未来 compatibility matrix 素材)
- **tracked running gear** = category-risk: it is the BMP-style probe and must stay an APC, never an MBT — flag/gate so it does not pull turret armor/gun proportions toward a main battle tank.
- **N > 5** road-wheel pairs/side: leaves the wheeled-APC vocabulary (becomes an unrealistic many-axle layout); cap multiplicity at 5.
- **mirrored L/R wheels** are NOT a multiplicity axis — left/right is a fixed pair per index, only the per-side count (`AXLE_X` length) varies.
- **color / scale** (olive vs camo, hull length tweaks) are styling, not structural slot axes.
