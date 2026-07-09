# Urban Environment / Caster Trolley2 source map

Status: variant-review gate. Do not proceed to proto-spec/template extraction until the five REDO forks are visually reviewed in workbench.

Source picture: `picture/Urban Environment/Caster Trolley2/001.png`

Parent baseline:
- `rec_chrome-wire-basket-shopping-cart-with-a-tapered-_20260608_164507_493965_58ed850d`
- Chrome supermarket shopping cart: tapered wire basket, red push handle and bumpers, fold-down child seat flap, underframe/lower tray, four swiveling caster assemblies.

Accepted REDO fork candidates:
- `rec_caster_trolley2_redo_deep_family_basket` — visibly deeper/taller family-size basket form while keeping the original moving joints.
- `rec_caster_trolley2_redo_lower_basket_visible` — larger and more visible lower cargo basket/shelf module.
- `rec_caster_trolley2_redo_front_ad_panel` — front fixed plastic advertising/nameplate panel on the basket face.
- `rec_caster_trolley2_redo_handle_sleeves` — ergonomic molded handle sleeve/end-cap/bracket visual treatment.
- `rec_caster_trolley2_redo_rim_guard_bumpers` — bright molded plastic upper-rim guard sleeves and corner caps.

Rejected/deleted earlier forks:
- `rec_caster_trolley2_var_rear_nesting_gate`
- `rec_caster_trolley2_var_child_seat_leg_holes`
- `rec_caster_trolley2_var_lower_rack_deep`
- `rec_caster_trolley2_var_front_swivel_rear_fixed`
- `rec_caster_trolley2_var_wire_density_high`
- Reason: external appearance changes were too weak and/or risked motion/joint issues.

Rejected/deleted redo attempts:
- `rec_caster_trolley2_redo_plastic_basket_panels` — rejected for floating/disconnected geometry during QC.
- `rec_caster_trolley2_redo_side_liner_panels` — rejected because the fork run stalled after disconnected front-liner warning and did not land a clean record.

Six-axis audit:
- 1. Skeleton/layout: parent cart skeleton retained. Visible fixed modules vary around basket depth, lower shelf, front face, handle, and rim protection.
- 2. Joint type: record-only/unchanged. All accepted forks preserve the parent non-fixed joint inventory: `basket_to_seat_flap`, four `frame_to_caster_yoke_*`, and four `caster_spin_*`.
- 3. Primary form family: deep basket, visible lower shelf, front panel, handle sleeve, rim guards.
- 4. Surface/detail: front plate, colored molded guards/sleeves, handle treatment.
- 5. Dimensions/travel: joint travel should remain parent-equivalent. Dimension edits are fixed-geometry only.
- 6. Materials: chrome wire/metal, red/orange/colored molded plastic, black rubber wheels.

Motion-safety notes:
- Fork prompts required no new joints and no edits to existing joint origins, axes, or limits.
- Final URDF comparison confirmed each accepted REDO fork has the same 9 non-fixed joints as the parent.
- The rim-guard fork specifically failed once on child-seat clearance and was repaired until compile passed with 0 failures and 0 warnings.

Workbench review focus:
- Confirm the five accepted forks are visually distinct enough at default viewer scale.
- Confirm no visible basket/seat/wheel collision during interaction.
- If approved, use only the five REDO fork records above for the next proto-spec/template pass.
