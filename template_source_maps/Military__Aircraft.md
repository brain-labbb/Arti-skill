# Military / Aircraft — template source map

pattern: mixed

parents:
- rec_model-a-wwii-era-single-engine-fighter-airplane-_20260610_080057_602122_05771485 ← picture/Military/Aircraft/001.png (FIGHTER: fills Slot A=`hinged_rudder_plus_elevator`, Slot B=`none` (gear-up), multiplicity N=1 nose prop)
- rec_model-a-wwii-twin-engine-attack-bomber-douglas-a_20260610_080147_111738_a4aa2b11 ← picture/Military/Aircraft/002.png (BOMBER: fills Slot A=`fixed_tall_fin`, Slot B=`none` (gear-up), multiplicity N=2 wing props via 2 hand-written `add_nacelle_and_prop` calls; adds dorsal `gun_turret`)

## Slot 候选覆盖

### Slot A:empennage / control-surface
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| hinged_rudder_plus_elevator | rec_model-a-wwii-era-single-engine-fighter-airplane-_20260610_080057_602122_05771485 | parts `tail_fin`/`rudder` + `horizontal_stabilizer`/`elevator`; joints `fin_to_rudder`(revolute, RUDDER_AXIS near-vertical raked, ±30°), `stabilizer_to_elevator`(revolute +Y, ±25°) | Single fin with a hinged rudder on a raked vertical axis plus a one-piece elevator hinged on the lateral axis; both control surfaces are separate movable links. | converged (parent) |
| fixed_tall_fin | rec_model-a-wwii-twin-engine-attack-bomber-douglas-a_20260610_080147_111738_a4aa2b11 | part `tail_fin` (`fin_lower`/`fin_band`/`fin_upper`) + `horizontal_stabilizer`; joints `fuselage_to_tail_fin`(FIXED), `fuselage_to_stabilizer`(FIXED) | Tall fixed fin with a yellow band, no movable rudder/elevator; whole empennage is rigid (control surfaces baked into the loft). | converged (parent) |
| all_moving_stabilator | rec_military_aircraft_var_stabilator | part `stabilator` (`stabilator_loft`, `pivot_shaft`); joint `fuselage_to_stabilator`(REVOLUTE +Y, ±20°); retains `tail_fin`/`rudder` with `fin_to_rudder` | Replaces the fixed stabilizer + elevator pair with one all-moving stabilator pitching on a fuselage-mounted lateral pivot shaft. | converged (workbench, rating pending sync) |
| twin_fin_rudders | rec_military_aircraft_var_twintail | parts `fin_{i}`/`rudder_{i}` (i=0,1) on `horizontal_stabilizer`; joints `stabilizer_to_fin_{i}`(FIXED, mirrored ±TWIN_FIN_Y), `fin_{i}_to_rudder_{i}`(REVOLUTE TWIN_RUDDER_AXIS, ±28°); retains one-piece `elevator`/`stabilizer_to_elevator` | Two endplate fins at the stabilizer tips, each carrying its own hinged rudder; fins+rudders emitted in a `for i in range(2)` loop, rudders mirrored spanwise. | converged (workbench, rating pending sync) |
| split_rudder_off_bomber_fin | rec_military_aircraft_var_rudderhinge | fixed `tail_fin` (`fin_fixed_lower`/`fin_fixed_band`/`fin_fixed_upper`) + movable `rudder` (`rudder_lower`/`rudder_band`/`rudder_upper`/`hinge_barrels`); joint `fin_to_rudder`(REVOLUTE HINGE_AXIS, ±30°) | Splits the bomber's tall fixed fin into a fixed forward fin + a hinged rudder trailing surface (adds rudder articulation to the bomber empennage). | converged (workbench, rating pending sync) |
| split_elevator_off_bomber | rec_military_aircraft_var_elevatorsplit | fixed `horizontal_stabilizer` (`stabilizer_fixed_loft`) + movable `elevator` (`elevator_surface`/`torque_tube`); joint `stabilizer_to_elevator`(REVOLUTE +Y, ±ELEVATOR_LIMIT_RAD) | Splits the bomber's one-piece stabilizer into a fixed stabilizer + a hinged elevator on a lateral torque tube. | converged (workbench, rating pending sync) |

### Slot B:landing gear
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| none (gear-up) | rec_model-a-wwii-era-single-engine-fighter-airplane-_20260610_080057_602122_05771485 | no gear parts/joints; grounding via blade tips grazing z≈0 | Clean gear-up airframe; lowest geometry is the resting propeller disc. Both parents share this state. | converged (parent) |
| none (gear-up) | rec_model-a-wwii-twin-engine-attack-bomber-douglas-a_20260610_080147_111738_a4aa2b11 | no gear parts/joints; grounding via blade tips grazing z≈0 | Twin-engine gear-up airframe; prop discs define ground contact. | converged (parent) |
| fixed_taildragger | rec_military_aircraft_var_taildragger | fuselage-mounted `tail_gear_strut`/`tail_gear_wheel` (visuals on fuselage); wing main gear loop `gear_strut_{i}`/`gear_wheel_{i}` (i=0,1, `for gear_i in range(2)`, y_sign mirrored) | Two main wheels under the wing + a tail wheel inline on the aft fuselage (taildragger stance); main legs are mirrored visuals on the rigid wing (no extra joints — fixed struts). | converged (workbench, rating pending sync) |
| fixed_tricycle | rec_military_aircraft_var_tricycle | main `gear_{i}` parts (i=0,1) with joints `nacelle_to_gear_{i}`(FIXED, parent=nacelle); `nose_gear` part with joint `fuselage_to_nose_gear`(FIXED) | Two main legs hung off the engine nacelles + a nose leg on the forward fuselage (tricycle stance); each gear is its own FIXED-jointed part, mains parented to their nacelles. | converged (workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: engine/propeller count `N_ENGINE` (number of nacelle + propeller pairs); also implies prop placement (nose centerline vs wing-mounted symmetric pairs).
- N 样本已覆盖: {1 → rec_model-a-wwii-era-single-engine-fighter-airplane-... (nose prop) and rec_military_aircraft_var_singleengine (loop-rewrite, centerline nose nacelle); 2 → rec_model-a-wwii-twin-engine-attack-bomber-douglas-... (hand-written 2× `add_nacelle_and_prop`); 4 → rec_military_aircraft_var_fourengine (`for i in range(N_ENGINE)` over `NAC_Y_POSITIONS=[3.5,7.8,-3.5,-7.8]`)}
- 模板建议 N_range: [1, 4] (realistic piston-era multiplicity; sweep may sample the full domain, large-N downweighted)
- copied object / naming / placement / joint policy: copied unit = nacelle (`engine_i`: `nacelle_hull`+`cowl_ring`+`engine_face`+`antiglare_panel`) + CONTINUOUS propeller (`propeller_i`: `spinner`+`prop_shaft`+`blades`). Naming `engine_{i}`/`propeller_{i}` in the loop variants (parents use bespoke names: fighter `propeller`/`nose_prop_spin`; bomber `{tag}_nacelle`/`{tag}_propeller` with `left_prop_spin`/`right_prop_spin`). Placement mirrored spanwise from `NAC_Y_POSITIONS` (single-engine = centerline y=0, nose station). Joint policy: one independent CONTINUOUS spin joint per prop about its nacelle +X axis (`propeller_{i}_spin` / `wing_to_engine_{i}` FIXED nacelle mount), each prop a separate child link — no mimic, props are independently spun. NOTE the bomber HAND-WRITES its 2 engines (two `add_nacelle_and_prop(±1,...)` calls, not a loop); the engine-count variants (`var_fourengine`, `var_singleengine`) required rewriting that into a `for i in range(N_ENGINE)` loop driven by a station list, which is the template's copy primitive.

## 排除项(未来 compatibility matrix 素材)
- canopy articulation / sliding-canopy: blocked — no source images isolating a movable canopy; canopy stays a fixed fuselage visual.
- underwing stores / bombs / drop-tanks: blocked — no source images; would float without a structural mount reference.
- color / national markings / scale: not axes — finishes (gloss blue fighter vs bare-metal bomber), insignia (402/B vs star-and-bar/322369/STINKY nose art) and overall size are cosmetic/parametric, not module slots.
- blade-count: folded into engine-count multiplicity — 4-blade fighter vs 3-blade bomber prop is a per-prop detail of the copied propeller unit, not a separate slot.
