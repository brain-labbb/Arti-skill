from __future__ import annotations

"""M4-style military carbine rifle, all matte black.

Layout convention:
- Bore axis is world +X (muzzle toward +X, buttstock toward -X).
- World +Z is up; the rifle rests with the magazine floorplate near z=0.
- Shooter's left side is +Y (safety selector lever lives on +Y).

Articulations (all at rest/closed pose at q=0, matching the reference photo):
- trigger_pull        REVOLUTE  about +Y, ~25 deg rearward pull
- stock_slide         PRISMATIC along +X (collapse forward), 0.09 m travel
- charging_handle_slide PRISMATIC along -X (pull rearward), 0.07 m travel
- magazine_release    PRISMATIC along the canted magwell axis (down/forward), 0.06 m
- safety_selector_rotate REVOLUTE about -Y, 90 deg safe-to-fire throw
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- constants
BORE_Z = 0.175  # bore axis height so the seated magazine just reaches the floor

MAG_CANT_DEG = 10.0  # magazine / magwell cant off vertical (bottom forward)
MAG_TRAVEL = 0.06
STOCK_TRAVEL = 0.09
CHARGING_TRAVEL = 0.07
TRIGGER_PULL_RAD = math.radians(25.0)
SELECTOR_THROW_RAD = math.pi / 2.0
SIGHT_FLIP_RAD = math.pi / 2.0  # flip-up BUIS fold 90 deg flat along the rail

_D10 = math.radians(MAG_CANT_DEG)
_D22 = math.radians(22.0)


def _box_compound(boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]]):
    """Compound of axis-aligned boxes: [(size, center), ...]."""
    solids = []
    for size, center in boxes:
        solids.append(cq.Workplane("XY").box(*size).translate(center).val())
    return cq.Compound.makeCompound(solids)


def _sight_base_clamp(length: float, width: float, height: float):
    """Rail clamp base for a flip-up BUIS sight (filleted block)."""
    return (
        cq.Workplane("XY")
        .box(length, width, height)
        .edges("|Z")
        .fillet(min(0.002, 0.4 * min(length, width)))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="m4_style_carbine")

    receiver_black = model.material("receiver_black", rgba=(0.10, 0.10, 0.105, 1.0))
    polymer_black = model.material("polymer_black", rgba=(0.07, 0.07, 0.075, 1.0))
    rail_black = model.material("rail_black", rgba=(0.135, 0.135, 0.14, 1.0))
    barrel_steel = model.material("barrel_steel", rgba=(0.16, 0.16, 0.17, 1.0))
    sight_black = model.material("sight_black", rgba=(0.055, 0.055, 0.06, 1.0))
    control_steel = model.material("control_steel", rgba=(0.20, 0.20, 0.21, 1.0))

    # ============================================================ receiver
    receiver = model.part("receiver")

    receiver.visual(
        Box((0.200, 0.034, 0.044)),
        origin=Origin(xyz=(-0.020, 0.0, 0.176)),
        material=receiver_black,
        name="upper_receiver",
    )
    receiver.visual(
        Box((0.200, 0.030, 0.008)),
        origin=Origin(xyz=(-0.020, 0.0, 0.2015)),
        material=rail_black,
        name="rail_base",
    )
    # flat-top picatinny ribs (one compound mesh, embedded 0.0037 into the rail base)
    rib_xs = [-0.106 + i * 0.0186 for i in range(10)]
    receiver.visual(
        mesh_from_cadquery(
            _box_compound([((0.0075, 0.032, 0.0075), (x, 0.0, 0.2055)) for x in rib_xs]),
            "rail_ribs",
        ),
        material=rail_black,
        name="rail_ribs",
    )
    # lower receiver with a canted pocket so the seated magazine continues up inside it
    lower_shape = (
        cq.Workplane("XY")
        .box(0.160, 0.032, 0.043)
        .translate((-0.030, 0.0, 0.133))
        .cut(
            cq.Workplane("XY")
            .box(0.0675, 0.0235, 0.034)
            .rotate((0, 0, 0), (0, 1, 0), -MAG_CANT_DEG)
            .translate((0.025, 0.0, 0.107))
        )
    )
    receiver.visual(
        mesh_from_cadquery(lower_shape, "lower_receiver"),
        material=receiver_black,
        name="lower_receiver",
    )

    # magwell: solid flare with a canted through-cavity that the magazine seats into
    magwell_shape = (
        cq.Workplane("XY")
        .box(0.076, 0.034, 0.030)
        .cut(
            cq.Workplane("XY")
            .box(0.0675, 0.0235, 0.070)
            .rotate((0, 0, 0), (0, 1, 0), -MAG_CANT_DEG)
        )
        .translate((0.025, 0.0, 0.112))
    )
    receiver.visual(
        mesh_from_cadquery(magwell_shape, "magwell"),
        material=receiver_black,
        name="magwell",
    )

    # trigger guard loop
    receiver.visual(
        Box((0.060, 0.010, 0.007)),
        origin=Origin(xyz=(-0.0425, 0.0, 0.0885)),
        material=receiver_black,
        name="guard_bar",
    )
    receiver.visual(
        Box((0.006, 0.010, 0.0245)),
        origin=Origin(xyz=(-0.016, 0.0, 0.0998)),
        material=receiver_black,
        name="guard_front_post",
    )
    receiver.visual(
        Box((0.006, 0.010, 0.0245)),
        origin=Origin(xyz=(-0.0685, 0.0, 0.0998)),
        material=receiver_black,
        name="guard_rear_post",
    )

    # buffer tube + castle nut (stock rides this tube)
    receiver.visual(
        Cylinder(radius=0.015, length=0.200),
        origin=Origin(xyz=(-0.2095, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=receiver_black,
        name="buffer_tube",
    )
    receiver.visual(
        Cylinder(radius=0.0175, length=0.012),
        origin=Origin(xyz=(-0.121, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=receiver_black,
        name="castle_nut",
    )

    # angled pistol grip (static, part of the lower receiver assembly)
    receiver.visual(
        Box((0.032, 0.030, 0.008)),
        origin=Origin(xyz=(-0.076, 0.0, 0.1085)),
        material=polymer_black,
        name="grip_plate",
    )
    grip_shape = (
        cq.Workplane("XY")
        .box(0.030, 0.026, 0.105)
        .edges("|Z")
        .fillet(0.005)
        .translate((0.0, 0.0, -0.0525))
        .rotate((0, 0, 0), (0, 1, 0), 20.0)
        .translate((-0.076, 0.0, 0.108))
    )
    receiver.visual(
        mesh_from_cadquery(grip_shape, "grip_body"),
        material=polymer_black,
        name="grip_body",
    )

    # delta ring / barrel nut (annulus the barrel passes through)
    delta_ring = (
        cq.Workplane("YZ")
        .circle(0.019)
        .circle(0.0135)
        .extrude(0.0265)
        .translate((0.0795, 0.0, BORE_Z))
    )
    receiver.visual(
        mesh_from_cadquery(delta_ring, "delta_ring"),
        material=receiver_black,
        name="delta_ring",
    )

    # ============================================================ barrel
    barrel = model.part("barrel")
    barrel_blank = (
        cq.Solid.makeCone(0.013, 0.010, 0.3255)
        .rotate(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90)
        .translate(cq.Vector(0.0795, 0.0, BORE_Z))
    )
    barrel.visual(
        mesh_from_cadquery(barrel_blank, "barrel_blank"),
        material=barrel_steel,
        name="barrel_blank",
    )
    # slotted birdcage flash hider
    hider = (
        cq.Workplane("XY")
        .circle(0.0115)
        .extrude(0.050)
        .cut(cq.Workplane("XY").box(0.040, 0.0048, 0.022).translate((0, 0, 0.0275)))
        .cut(cq.Workplane("XY").box(0.0048, 0.040, 0.022).translate((0, 0, 0.0275)))
        .cut(cq.Workplane("XY").circle(0.0045).extrude(0.030).translate((0, 0, 0.025)))
        .rotate((0, 0, 0), (0, 1, 0), 90)
        .translate((0.400, 0.0, BORE_Z))
    )
    barrel.visual(
        mesh_from_cadquery(hider, "flash_hider"),
        material=barrel_steel,
        name="flash_hider",
    )
    model.articulation(
        "receiver_to_barrel",
        ArticulationType.FIXED,
        parent=receiver,
        child=barrel,
    )

    # ============================================================ quad-rail handguard
    handguard = model.part("handguard")
    hg_tube = (
        cq.Workplane("YZ")
        .rect(0.054, 0.054)
        .circle(0.018)
        .extrude(0.192)
        .translate((0.1055, 0.0, BORE_Z))
    )
    handguard.visual(
        mesh_from_cadquery(hg_tube, "handguard_tube"),
        material=polymer_black,
        name="handguard_tube",
    )
    hg_xs = [0.112 + i * 0.0178 for i in range(11)]
    handguard.visual(
        mesh_from_cadquery(
            _box_compound([((0.0075, 0.034, 0.0075), (x, 0.0, BORE_Z + 0.027)) for x in hg_xs]),
            "hg_rail_top",
        ),
        material=rail_black,
        name="hg_rail_top",
    )
    # bottom rail skips two slots where the vertical foregrip clamps on
    handguard.visual(
        mesh_from_cadquery(
            _box_compound(
                [
                    ((0.0075, 0.034, 0.0075), (x, 0.0, BORE_Z - 0.027))
                    for x in hg_xs
                    if not (0.178 < x < 0.222)
                ]
            ),
            "hg_rail_bottom",
        ),
        material=rail_black,
        name="hg_rail_bottom",
    )
    handguard.visual(
        mesh_from_cadquery(
            _box_compound([((0.0075, 0.0075, 0.034), (x, 0.027, BORE_Z)) for x in hg_xs]),
            "hg_rail_left",
        ),
        material=rail_black,
        name="hg_rail_left",
    )
    handguard.visual(
        mesh_from_cadquery(
            _box_compound([((0.0075, 0.0075, 0.034), (x, -0.027, BORE_Z)) for x in hg_xs]),
            "hg_rail_right",
        ),
        material=rail_black,
        name="hg_rail_right",
    )
    model.articulation(
        "receiver_to_handguard",
        ArticulationType.FIXED,
        parent=receiver,
        child=handguard,
    )

    # ============================================================ front sight tower
    tower = model.part("front_sight_tower")
    tower_shape = (
        cq.Workplane("XZ")
        .polyline([(0.297, 0.158), (0.342, 0.158), (0.3265, 0.230), (0.3125, 0.230)])
        .close()
        .extrude(0.007, both=True)
        # barrel-clamp boss bridging the A-frame legs above/below the bore cut
        .union(
            cq.Workplane("YZ")
            .circle(0.0145)
            .extrude(0.030)
            .translate((0.3045, 0.0, BORE_Z))
        )
        .union(cq.Workplane("XY").box(0.0045, 0.012, 0.024).translate((0.3195, 0.0, 0.240)))
        .cut(
            cq.Workplane("YZ").circle(0.0113).extrude(0.10).translate((0.290, 0.0, BORE_Z))
        )
    )
    tower.visual(
        mesh_from_cadquery(tower_shape, "sight_tower"),
        material=receiver_black,
        name="sight_tower",
    )
    model.articulation(
        "barrel_to_front_sight_tower",
        ArticulationType.FIXED,
        parent=barrel,
        child=tower,
    )

    # ============================================================ flip-up iron sights
    # Replace the powered optic with a rear aperture BUIS on the receiver rail
    # and a front post BUIS on the handguard top rail.  Both flip up/down via
    # revolute joints about the cross-bore (Y) axis; q=0 is deployed upright,
    # positive q folds the sight forward along the rail.
    #
    # Rail rib tops:
    #   receiver rail rib top ≈ 0.20925  (rib center 0.2055 + half-height 0.00375)
    #   handguard rail rib top ≈ BORE_Z + 0.027 + 0.00375 = 0.20575

    _RECV_RIB_TOP = 0.20925
    _HG_RIB_TOP = BORE_Z + 0.027 + 0.00375  # 0.20575

    # --- shared placement constants ---
    _rear_base_h = 0.012
    _rear_base_ctr = (-0.100, 0.0, _RECV_RIB_TOP + 0.5 * _rear_base_h)
    _rear_pivot_z = _RECV_RIB_TOP + _rear_base_h  # top of base clamp

    _front_base_h = 0.010
    _front_base_ctr = (0.250, 0.0, _HG_RIB_TOP + 0.5 * _front_base_h)
    _front_pivot_z = _HG_RIB_TOP + _front_base_h

    # --- inline base clamps (non-moving, parent visuals) ---
    receiver.visual(
        mesh_from_cadquery(
            _sight_base_clamp(0.026, 0.032, _rear_base_h).translate(_rear_base_ctr),
            "rear_sight_base",
        ),
        material=rail_black,
        name="rear_sight_base",
    )
    handguard.visual(
        mesh_from_cadquery(
            _sight_base_clamp(0.022, 0.028, _front_base_h).translate(_front_base_ctr),
            "front_sight_base",
        ),
        material=rail_black,
        name="front_sight_base",
    )

    # --- rear aperture sight (flip-up child part) ---
    rear_sight = model.part("rear_sight")
    # All-box construction from one workplane for reliable boolean fusion
    _rear_frame = (
        cq.Workplane("XY")
        # left ear
        .box(0.005, 0.005, 0.030).translate((0, 0.0105, 0.015))
        # pivot crossbar connecting the ear bases (touches left ear immediately)
        .union(cq.Workplane("XY").box(0.005, 0.026, 0.008).translate((0, 0, 0.004)))
        # right ear (touches crossbar)
        .union(cq.Workplane("XY").box(0.005, 0.005, 0.030).translate((0, -0.0105, 0.015)))
        # aperture housing block between the ears (bridges both ears near the top)
        .union(cq.Workplane("XY").box(0.005, 0.022, 0.014).translate((0, 0, 0.022)))
        # peep-hole: round through-hole along Y (does NOT split the housing in Z)
        .cut(cq.Workplane("XZ").circle(0.0015).extrude(0.028).translate((0, -0.014, 0.022)))
    )
    rear_sight.visual(
        mesh_from_cadquery(_rear_frame, "rear_sight_frame"),
        material=sight_black,
        name="rear_sight_frame",
    )
    # repeated grip pads on ear outsides emitted via for-loop with name_i convention
    for i, y_off in enumerate((-0.012, 0.012)):
        rear_sight.visual(
            Box((0.006, 0.003, 0.008)),
            origin=Origin(xyz=(0.0, y_off, 0.008)),
            material=sight_black,
            name=f"rear_grip_{i}",
        )
    model.articulation(
        "rear_sight_flip",
        ArticulationType.REVOLUTE,
        parent=receiver,
        child=rear_sight,
        origin=Origin(xyz=(_rear_base_ctr[0], 0.0, _rear_pivot_z)),
        # positive q folds the upright sight forward along the bore (+X)
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=5.0, lower=0.0, upper=SIGHT_FLIP_RAD
        ),
    )

    # --- front post sight (flip-up child part) ---
    front_sight = model.part("front_sight")
    # All-box construction from one workplane for reliable boolean fusion
    _front_body = (
        cq.Workplane("XY")
        # upright arm blade
        .box(0.005, 0.008, 0.030).translate((0, 0, 0.012))
        # pivot crossbar
        .union(cq.Workplane("XY").box(0.005, 0.026, 0.006).translate((0, 0, 0.003)))
        # post tip (narrow blade on top, overlaps arm at z≈0.020)
        .union(cq.Workplane("XY").box(0.003, 0.005, 0.014).translate((0, 0, 0.026)))
    )
    front_sight.visual(
        mesh_from_cadquery(_front_body, "front_sight_body"),
        material=sight_black,
        name="front_sight_body",
    )
    # repeated windage knob pair emitted via for-loop with name_i convention
    for i, y_off in enumerate((-0.003, 0.003)):
        front_sight.visual(
            Cylinder(radius=0.002, length=0.005),
            origin=Origin(xyz=(0.0, y_off, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=sight_black,
            name=f"front_knob_{i}",
        )
    model.articulation(
        "front_sight_flip",
        ArticulationType.REVOLUTE,
        parent=handguard,
        child=front_sight,
        origin=Origin(xyz=(_front_base_ctr[0], 0.0, _front_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=5.0, lower=0.0, upper=SIGHT_FLIP_RAD
        ),
    )

    # ============================================================ vertical foregrip
    foregrip = model.part("vertical_foregrip")
    foregrip.visual(
        Box((0.034, 0.024, 0.010)),
        origin=Origin(xyz=(0.200, 0.0, 0.1435)),
        material=polymer_black,
        name="foregrip_mount",
    )
    fg = cq.Workplane("XY").circle(0.0135).extrude(0.095).translate((0, 0, 0.045))
    for ring_z in (0.058, 0.074, 0.090, 0.106, 0.122):
        fg = fg.union(
            cq.Workplane("XY").circle(0.0155).extrude(0.0065).translate((0, 0, ring_z))
        )
    fg = fg.union(cq.Solid.makeCone(0.008, 0.0135, 0.009).translate(cq.Vector(0, 0, 0.036)))
    fg = fg.translate((0.200, 0.0, 0.0))
    foregrip.visual(
        mesh_from_cadquery(fg, "foregrip_body"),
        material=polymer_black,
        name="foregrip_body",
    )
    model.articulation(
        "handguard_to_vertical_foregrip",
        ArticulationType.FIXED,
        parent=handguard,
        child=foregrip,
    )

    # ============================================================ collapsible buttstock
    # Child frame sits on the buffer-tube axis at the collar front face (world -0.24, 0, 0.170).
    buttstock = model.part("buttstock")
    stock_shape = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, 0.018),
                (-0.165, 0.018),
                (-0.165, -0.082),
                (-0.150, -0.082),
                (-0.045, -0.026),
                (0.0, -0.026),
            ]
        )
        .close()
        .extrude(0.020, both=True)
        .cut(
            cq.Workplane("XZ")
            .polyline([(-0.145, -0.070), (-0.145, -0.034), (-0.062, -0.030)])
            .close()
            .extrude(0.021, both=True)
        )
        .union(cq.Workplane("XY").box(0.012, 0.044, 0.106).translate((-0.163, 0.0, -0.032)))
        .union(cq.Workplane("XY").box(0.028, 0.013, 0.008).translate((-0.022, 0.0, -0.0285)))
        .cut(cq.Workplane("XY").box(0.009, 0.060, 0.028).translate((-0.1635, 0.0, -0.058)))
        .cut(cq.Workplane("YZ").circle(0.0145).extrude(0.180).translate((-0.175, 0.0, 0.0)))
    )
    buttstock.visual(
        mesh_from_cadquery(stock_shape, "stock_body"),
        material=polymer_black,
        name="stock_body",
    )
    model.articulation(
        "stock_slide",
        ArticulationType.PRISMATIC,
        parent=receiver,
        child=buttstock,
        origin=Origin(xyz=(-0.240, 0.0, 0.170)),
        # positive q collapses the stock forward along the buffer tube
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.4, lower=0.0, upper=STOCK_TRAVEL),
    )

    # ============================================================ charging handle
    charging = model.part("charging_handle")
    charging.visual(
        Box((0.120, 0.012, 0.007)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=control_steel,
        name="handle_shaft",
    )
    charging.visual(
        Box((0.016, 0.052, 0.0085)),
        origin=Origin(xyz=(-0.0075, 0.0, 0.0)),
        material=control_steel,
        name="handle_t_wings",
    )
    charging.visual(
        Box((0.016, 0.016, 0.0105)),
        origin=Origin(xyz=(-0.0075, 0.0, 0.001)),
        material=control_steel,
        name="handle_t_spine",
    )
    model.articulation(
        "charging_handle_slide",
        ArticulationType.PRISMATIC,
        parent=receiver,
        child=charging,
        origin=Origin(xyz=(-0.120, 0.0, 0.1935)),
        # positive q pulls the handle rearward along the bore axis
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.8, lower=0.0, upper=CHARGING_TRAVEL),
    )

    # ============================================================ trigger
    trigger = model.part("trigger")
    trigger_shape = (
        cq.Workplane("XY")
        .box(0.0075, 0.010, 0.0118)
        .translate((0.0005, 0.0, -0.0051))
        .union(
            cq.Workplane("XY")
            .box(0.007, 0.010, 0.009)
            .rotate((0, 0, 0), (0, 1, 0), 14.0)
            .translate((-0.0006, 0.0, -0.0140))
        )
    )
    trigger.visual(
        mesh_from_cadquery(trigger_shape, "trigger_blade"),
        material=control_steel,
        name="trigger_blade",
    )
    model.articulation(
        "trigger_pull",
        ArticulationType.REVOLUTE,
        parent=receiver,
        child=trigger,
        origin=Origin(xyz=(-0.032, 0.0, 0.1115)),
        # positive q swings the hanging blade rearward (right-hand rule about +Y)
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=0.0, upper=TRIGGER_PULL_RAD),
    )

    # ============================================================ magazine
    # Child frame at the magwell mouth (world 0.025, 0, 0.100); the magazine cants
    # forward by MAG_CANT_DEG so its insertion axis is down-and-slightly-forward.
    magazine = model.part("magazine")
    d10 = (math.sin(_D10), 0.0, -math.cos(_D10))
    mag_top_center = (
        0.0140 * d10[0],
        0.0,
        0.012 + 0.0140 * d10[2],
    )
    mag_top = (
        cq.Workplane("XY")
        .box(0.068, 0.024, 0.026)
        .rotate((0, 0, 0), (0, 1, 0), -MAG_CANT_DEG)
        .translate(mag_top_center)
    )
    magazine.visual(
        mesh_from_cadquery(mag_top, "mag_top"),
        material=polymer_black,
        name="mag_top",
    )
    # curved 30-round body: two canted segments + flared floorplate
    top_bottom = tuple(mag_top_center[i] + 0.013 * d10[i] for i in range(3))
    c1 = tuple(top_bottom[i] + 0.018 * d10[i] for i in range(3))
    b1 = tuple(c1[i] + 0.020 * d10[i] for i in range(3))
    d22 = (math.sin(_D22), 0.0, -math.cos(_D22))
    c2 = tuple(b1[i] + 0.016 * d22[i] for i in range(3))
    b2 = tuple(c2[i] + 0.018 * d22[i] for i in range(3))
    plate_c = tuple(b2[i] - 0.001 * d22[i] for i in range(3))
    mag_body = (
        cq.Workplane("XY")
        .box(0.068, 0.024, 0.040)
        .rotate((0, 0, 0), (0, 1, 0), -MAG_CANT_DEG)
        .translate(c1)
        .union(
            cq.Workplane("XY")
            .box(0.068, 0.024, 0.036)
            .rotate((0, 0, 0), (0, 1, 0), -22.0)
            .translate(c2)
        )
        .union(
            cq.Workplane("XY")
            .box(0.071, 0.027, 0.008)
            .rotate((0, 0, 0), (0, 1, 0), -22.0)
            .translate(plate_c)
        )
    )
    magazine.visual(
        mesh_from_cadquery(mag_body, "mag_body"),
        material=polymer_black,
        name="mag_body",
    )
    model.articulation(
        "magazine_release",
        ArticulationType.PRISMATIC,
        parent=receiver,
        child=magazine,
        origin=Origin(xyz=(0.025, 0.0, 0.100)),
        # positive q drops the magazine out along the canted magwell axis
        axis=d10,
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=MAG_TRAVEL),
    )

    # ============================================================ safety selector
    selector = model.part("safety_selector")
    selector.visual(
        Cylinder(radius=0.005, length=0.0035),
        origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_steel,
        name="selector_axle",
    )
    lever_shape = (
        cq.Workplane("XY")
        .box(0.030, 0.0045, 0.0085)
        .translate((0.011, 0.00125, 0.0))
        .union(
            cq.Workplane("XY").box(0.007, 0.0045, 0.014).translate((0.0255, 0.00125, 0.0))
        )
    )
    selector.visual(
        mesh_from_cadquery(lever_shape, "selector_lever"),
        material=control_steel,
        name="selector_lever",
    )
    model.articulation(
        "safety_selector_rotate",
        ArticulationType.REVOLUTE,
        parent=receiver,
        child=selector,
        origin=Origin(xyz=(-0.085, 0.0185, 0.140)),
        # lever points forward at SAFE; positive q sweeps the tip up 90 deg to FIRE
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=4.0, lower=0.0, upper=SELECTOR_THROW_RAD
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    receiver = object_model.get_part("receiver")
    barrel = object_model.get_part("barrel")
    handguard = object_model.get_part("handguard")
    tower = object_model.get_part("front_sight_tower")
    rear_sight = object_model.get_part("rear_sight")
    front_sight = object_model.get_part("front_sight")
    foregrip = object_model.get_part("vertical_foregrip")
    buttstock = object_model.get_part("buttstock")
    charging = object_model.get_part("charging_handle")
    trigger = object_model.get_part("trigger")
    magazine = object_model.get_part("magazine")
    selector = object_model.get_part("safety_selector")

    stock_slide = object_model.get_articulation("stock_slide")
    charging_slide = object_model.get_articulation("charging_handle_slide")
    trigger_pull = object_model.get_articulation("trigger_pull")
    mag_release = object_model.get_articulation("magazine_release")
    selector_rotate = object_model.get_articulation("safety_selector_rotate")
    rear_sight_flip = object_model.get_articulation("rear_sight_flip")
    front_sight_flip = object_model.get_articulation("front_sight_flip")

    # -------- intentional snug fits (sliding/seated nests)
    ctx.allow_overlap(
        receiver,
        buttstock,
        elem_a="buffer_tube",
        elem_b="stock_body",
        reason="six-position stock collar intentionally rides the buffer tube as a snug sliding fit",
    )
    ctx.allow_overlap(
        receiver,
        charging,
        elem_a="upper_receiver",
        elem_b="handle_shaft",
        reason="charging handle shaft is captured inside the upper receiver channel and slides rearward",
    )
    ctx.allow_overlap(
        receiver,
        magazine,
        elem_a="magwell",
        elem_b="mag_top",
        reason="magazine top is seated inside the canted magwell with a snug insertion fit",
    )
    ctx.allow_overlap(
        receiver,
        magazine,
        elem_a="lower_receiver",
        elem_b="mag_top",
        reason="seated magazine continues up into the lower receiver mag pocket with a snug fit",
    )

    # -------- joint plan: types, axes, ranges
    ctx.check(
        "trigger is revolute with ~25 deg pull",
        trigger_pull.articulation_type == ArticulationType.REVOLUTE
        and abs(trigger_pull.motion_limits.upper - TRIGGER_PULL_RAD) < 1e-6
        and trigger_pull.motion_limits.lower == 0.0,
    )
    ctx.check(
        "stock is prismatic with 0.09 m travel along bore axis",
        stock_slide.articulation_type == ArticulationType.PRISMATIC
        and abs(stock_slide.motion_limits.upper - STOCK_TRAVEL) < 1e-9
        and abs(stock_slide.axis[0]) == 1.0
        and stock_slide.axis[2] == 0.0,
    )
    ctx.check(
        "charging handle is prismatic, 0.07 m rearward along bore",
        charging_slide.articulation_type == ArticulationType.PRISMATIC
        and abs(charging_slide.motion_limits.upper - CHARGING_TRAVEL) < 1e-9
        and charging_slide.axis[0] == -1.0,
    )
    ctx.check(
        "magazine is prismatic, 0.06 m down a forward-canted axis",
        mag_release.articulation_type == ArticulationType.PRISMATIC
        and abs(mag_release.motion_limits.upper - MAG_TRAVEL) < 1e-9
        and mag_release.axis[2] < -0.9
        and mag_release.axis[0] > 0.1,
    )
    ctx.check(
        "safety selector is revolute with a 90 deg throw",
        selector_rotate.articulation_type == ArticulationType.REVOLUTE
        and abs(selector_rotate.motion_limits.upper - SELECTOR_THROW_RAD) < 1e-9,
    )
    ctx.check(
        "rear sight flip is revolute with 90 deg fold",
        rear_sight_flip.articulation_type == ArticulationType.REVOLUTE
        and abs(rear_sight_flip.motion_limits.upper - SIGHT_FLIP_RAD) < 1e-9,
    )
    ctx.check(
        "front sight flip is revolute with 90 deg fold",
        front_sight_flip.articulation_type == ArticulationType.REVOLUTE
        and abs(front_sight_flip.motion_limits.upper - SIGHT_FLIP_RAD) < 1e-9,
    )

    # -------- real-world scale at rest
    bb_barrel = ctx.part_world_aabb(barrel)
    bb_stock = ctx.part_world_aabb(buttstock)
    bb_rear = ctx.part_world_aabb(rear_sight)
    bb_mag = ctx.part_world_aabb(magazine)
    oal = bb_barrel[1][0] - bb_stock[0][0]
    ctx.check(
        "overall length ~0.85 m muzzle to butt pad",
        0.82 <= oal <= 0.89,
        details=f"oal={oal:.4f}",
    )
    ctx.check(
        "flash hider reaches the muzzle end",
        bb_barrel[1][0] > 0.44,
        details=f"barrel xmax={bb_barrel[1][0]:.4f}",
    )
    ctx.check(
        "deployed rear aperture sight gives ~0.25 m total height",
        0.235 <= bb_rear[1][2] <= 0.27,
        details=f"rear sight zmax={bb_rear[1][2]:.4f}",
    )
    ctx.check(
        "seated magazine floorplate rests near the ground plane",
        -0.004 <= bb_mag[0][2] <= 0.015,
        details=f"mag zmin={bb_mag[0][2]:.4f}",
    )
    bb_hg = ctx.part_world_aabb(handguard)
    width = bb_hg[1][1] - bb_hg[0][1]
    ctx.check(
        "quad-rail width ~0.06 m",
        0.05 <= width <= 0.07,
        details=f"handguard width={width:.4f}",
    )

    # -------- mounts are seated, not floating
    ctx.expect_contact(barrel, receiver, name="barrel root seats in the upper receiver")
    ctx.expect_contact(handguard, receiver, name="handguard seats against the delta ring")
    ctx.expect_contact(tower, handguard, name="front sight tower abuts the handguard")
    ctx.expect_contact(
        rear_sight,
        receiver,
        elem_a="rear_sight_frame",
        elem_b="rear_sight_base",
        name="rear sight pivot seats on the receiver rail base clamp",
    )
    ctx.expect_contact(
        front_sight,
        handguard,
        elem_a="front_sight_body",
        elem_b="front_sight_base",
        name="front sight pivot seats on the handguard rail base clamp",
    )
    ctx.expect_contact(foregrip, handguard, name="foregrip clamps the bottom rail")
    ctx.expect_contact(trigger, receiver, name="trigger root enters the lower receiver")
    ctx.expect_contact(selector, receiver, name="selector axle enters the receiver side")

    # trigger hangs inside the guard loop
    ctx.expect_within(
        trigger,
        receiver,
        axes="x",
        outer_elem="guard_bar",
        margin=0.0,
        name="trigger blade stays inside the trigger guard span",
    )
    # magazine centered in the magwell
    ctx.expect_within(
        magazine,
        receiver,
        axes="y",
        inner_elem="mag_top",
        outer_elem="magwell",
        margin=0.0005,
        name="magazine top stays centered in the magwell",
    )
    # stock retains insertion on the buffer tube at rest
    ctx.expect_overlap(
        buttstock,
        receiver,
        axes="x",
        elem_a="stock_body",
        elem_b="buffer_tube",
        min_overlap=0.05,
        name="stock collar retains insertion on the buffer tube",
    )

    # -------- decisive articulated poses
    rest_stock = ctx.part_world_aabb(buttstock)
    with ctx.pose({stock_slide: STOCK_TRAVEL}):
        collapsed = ctx.part_world_aabb(buttstock)
        ctx.check(
            "stock collapses forward by ~0.09 m",
            collapsed[0][0] > rest_stock[0][0] + 0.07,
            details=f"rest xmin={rest_stock[0][0]:.4f}, collapsed xmin={collapsed[0][0]:.4f}",
        )
        ctx.expect_overlap(
            buttstock,
            receiver,
            axes="x",
            elem_a="stock_body",
            elem_b="buffer_tube",
            min_overlap=0.10,
            name="collapsed stock stays engaged on the buffer tube",
        )

    rest_ch = ctx.part_world_aabb(charging)
    with ctx.pose({charging_slide: CHARGING_TRAVEL}):
        pulled = ctx.part_world_aabb(charging)
        ctx.check(
            "charging handle pulls rearward by ~0.07 m at constant height",
            pulled[0][0] < rest_ch[0][0] - 0.06
            and abs(pulled[1][2] - rest_ch[1][2]) < 1e-6,
            details=f"rest xmin={rest_ch[0][0]:.4f}, pulled xmin={pulled[0][0]:.4f}",
        )

    rest_mag = ctx.part_world_aabb(magazine)
    with ctx.pose({mag_release: MAG_TRAVEL}):
        dropped = ctx.part_world_aabb(magazine)
        ctx.check(
            "released magazine drops clear of the magwell mouth",
            dropped[1][2] < 0.097 and dropped[0][2] < rest_mag[0][2] - 0.05,
            details=f"dropped zmax={dropped[1][2]:.4f}, zmin={dropped[0][2]:.4f}",
        )
        ctx.check(
            "magazine release also travels slightly forward (canted axis)",
            dropped[0][0] > rest_mag[0][0] + 0.005,
            details=f"rest xmin={rest_mag[0][0]:.4f}, dropped xmin={dropped[0][0]:.4f}",
        )

    rest_trig = ctx.part_world_aabb(trigger)
    with ctx.pose({trigger_pull: TRIGGER_PULL_RAD}):
        pulled_trig = ctx.part_world_aabb(trigger)
        ctx.check(
            "pulled trigger blade (off-axis of its pivot) swings rearward without dropping",
            pulled_trig[0][0] < rest_trig[0][0] - 0.004
            and pulled_trig[0][2] > rest_trig[0][2] - 0.0005,
            details=f"rest xmin={rest_trig[0][0]:.4f}, pulled xmin={pulled_trig[0][0]:.4f}",
        )
        ctx.expect_within(
            trigger,
            receiver,
            axes="x",
            outer_elem="guard_bar",
            margin=0.0,
            name="pulled trigger stays inside the guard loop",
        )

    rest_sel = ctx.part_world_aabb(selector)
    with ctx.pose({selector_rotate: SELECTOR_THROW_RAD}):
        fired = ctx.part_world_aabb(selector)
        ctx.check(
            "selector lever tip (off-axis) sweeps up 90 deg from SAFE to FIRE",
            fired[1][2] > rest_sel[1][2] + 0.018,
            details=f"rest zmax={rest_sel[1][2]:.4f}, fire zmax={fired[1][2]:.4f}",
        )

    # -------- flip-up iron sight poses
    rest_rear = ctx.part_world_aabb(rear_sight)
    with ctx.pose({rear_sight_flip: SIGHT_FLIP_RAD}):
        folded_rear = ctx.part_world_aabb(rear_sight)
        ctx.check(
            "rear sight folds rearward (z drops, xmin moves toward -X) when stowed",
            folded_rear[1][2] < rest_rear[1][2] - 0.015
            and folded_rear[0][0] < rest_rear[0][0] - 0.010,
            details=(
                f"rest zmax={rest_rear[1][2]:.4f}, folded zmax={folded_rear[1][2]:.4f}, "
                f"rest xmin={rest_rear[0][0]:.4f}, folded xmin={folded_rear[0][0]:.4f}"
            ),
        )

    rest_front = ctx.part_world_aabb(front_sight)
    with ctx.pose({front_sight_flip: SIGHT_FLIP_RAD}):
        folded_front = ctx.part_world_aabb(front_sight)
        ctx.check(
            "front sight folds rearward (z drops, xmin moves toward -X) when stowed",
            folded_front[1][2] < rest_front[1][2] - 0.015
            and folded_front[0][0] < rest_front[0][0] - 0.010,
            details=(
                f"rest zmax={rest_front[1][2]:.4f}, folded zmax={folded_front[1][2]:.4f}, "
                f"rest xmin={rest_front[0][0]:.4f}, folded xmin={folded_front[0][0]:.4f}"
            ),
        )

    # -------- no powered optic body remains
    _part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no reflex_sight part exists (optic removed)",
        "reflex_sight" not in _part_names,
    )

    return ctx.report()


object_model = build_object_model()
