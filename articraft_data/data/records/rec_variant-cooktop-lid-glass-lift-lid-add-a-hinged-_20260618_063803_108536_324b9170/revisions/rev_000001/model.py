from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Overall envelope (meters). Front of the cooker faces +X.
#   width  (Y): 0.50
#   depth  (X): 0.60
#   height (Z): ~0.85 including feet, burners, and pan grates
# ---------------------------------------------------------------------------
HALF_W = 0.25  # half width  (Y)
HALF_D = 0.30  # half depth  (X)

FOOT_H = 0.05
PLINTH_TOP = 0.10
DOOR_TOP = 0.62  # oven door / cavity top
COOKTOP_FLOOR = 0.785  # recessed cooktop well floor
RIM_TOP = 0.805  # raised perimeter rim around the cooktop well
GRATE_Z0 = RIM_TOP  # grate bars rest on the rim
GRATE_BAR = 0.008

DOOR_H = DOOR_TOP - PLINTH_TOP  # 0.52
DOOR_T = 0.04

KNOB_Z = 0.71
KNOB_YS = [-0.1875 + 0.075 * i for i in range(6)]

# (x, y) burner centers in the cooktop well; index order: 0=front-right view
BURNER_XY = [(0.13, 0.115), (0.13, -0.115), (-0.13, 0.115), (-0.13, -0.115)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_gas_range_cooker")

    enamel_white = model.material("enamel_white", rgba=(0.93, 0.93, 0.91, 1.0))
    knob_white = model.material("knob_white", rgba=(0.96, 0.96, 0.95, 1.0))
    iron_black = model.material("iron_black", rgba=(0.07, 0.07, 0.08, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.05, 0.06, 0.07, 0.93))
    steel_silver = model.material("steel_silver", rgba=(0.76, 0.77, 0.78, 1.0))
    cap_silver = model.material("cap_silver", rgba=(0.82, 0.83, 0.84, 1.0))
    drip_silver = model.material("drip_silver", rgba=(0.60, 0.61, 0.62, 1.0))
    cavity_grey = model.material("cavity_grey", rgba=(0.20, 0.19, 0.18, 1.0))
    rack_chrome = model.material("rack_chrome", rgba=(0.68, 0.69, 0.70, 1.0))
    foot_black = model.material("foot_black", rgba=(0.12, 0.12, 0.12, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")

    # Plinth / oven floor slab (also closes the cavity bottom).
    body.visual(
        Box((0.60, 0.50, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=enamel_white,
        name="plinth",
    )

    # Four short adjustable feet (embedded 1 mm into the plinth underside).
    for i, (fx, fy) in enumerate([(0.26, 0.21), (0.26, -0.21), (-0.26, 0.21), (-0.26, -0.21)]):
        body.visual(
            Cylinder(radius=0.022, length=0.051),
            origin=Origin(xyz=(fx, fy, 0.0255)),
            material=foot_black,
            name=f"foot_{i}",
        )

    # Oven cavity walls: hollow box open at the front (+X).
    wall_h = 0.525  # z 0.095 .. 0.62, embedded 5 mm into the plinth
    body.visual(
        Box((0.60, 0.05, wall_h)),
        origin=Origin(xyz=(0.0, -0.225, 0.3575)),
        material=enamel_white,
        name="wall_left",
    )
    body.visual(
        Box((0.60, 0.05, wall_h)),
        origin=Origin(xyz=(0.0, 0.225, 0.3575)),
        material=enamel_white,
        name="wall_right",
    )
    body.visual(
        Box((0.05, 0.40, wall_h)),
        origin=Origin(xyz=(-0.275, 0.0, 0.3575)),
        material=enamel_white,
        name="wall_rear",
    )

    # Upper housing: control fascia front band + burner box under the cooktop.
    # z 0.619 .. 0.785 (embedded 1 mm into the cavity wall tops).
    body.visual(
        Box((0.60, 0.50, 0.166)),
        origin=Origin(xyz=(0.0, 0.0, 0.702)),
        material=enamel_white,
        name="top_housing",
    )

    # Raised rim around the recessed cooktop well (z 0.784 .. 0.805).
    body.visual(
        Box((0.03, 0.50, 0.021)),
        origin=Origin(xyz=(0.285, 0.0, 0.7945)),
        material=enamel_white,
        name="rim_front",
    )
    body.visual(
        Box((0.03, 0.50, 0.021)),
        origin=Origin(xyz=(-0.285, 0.0, 0.7945)),
        material=enamel_white,
        name="rim_rear",
    )
    body.visual(
        Box((0.54, 0.03, 0.021)),
        origin=Origin(xyz=(0.0, -0.235, 0.7945)),
        material=enamel_white,
        name="rim_left",
    )
    body.visual(
        Box((0.54, 0.03, 0.021)),
        origin=Origin(xyz=(0.0, 0.235, 0.7945)),
        material=enamel_white,
        name="rim_right",
    )

    # Dark oven cavity liners (thin panels, embedded ~1 mm into their walls).
    body.visual(
        Box((0.55, 0.005, 0.52)),
        origin=Origin(xyz=(0.025, -0.1985, 0.36)),
        material=cavity_grey,
        name="liner_left",
    )
    body.visual(
        Box((0.55, 0.005, 0.52)),
        origin=Origin(xyz=(0.025, 0.1985, 0.36)),
        material=cavity_grey,
        name="liner_right",
    )
    body.visual(
        Box((0.005, 0.40, 0.52)),
        origin=Origin(xyz=(-0.2485, 0.0, 0.36)),
        material=cavity_grey,
        name="liner_rear",
    )
    body.visual(
        Box((0.55, 0.40, 0.005)),
        origin=Origin(xyz=(0.025, 0.0, 0.618)),
        material=cavity_grey,
        name="liner_top",
    )
    body.visual(
        Box((0.55, 0.40, 0.005)),
        origin=Origin(xyz=(0.025, 0.0, 0.102)),
        material=cavity_grey,
        name="liner_bottom",
    )

    # Chrome oven rack shelf spanning between the side liners.
    body.visual(
        Box((0.48, 0.398, 0.008)),
        origin=Origin(xyz=(0.02, 0.0, 0.304)),
        material=rack_chrome,
        name="oven_rack",
    )

    # Four gas burners: recessed drip cup, burner body, silver cap.
    for i, (bx, by) in enumerate(BURNER_XY):
        body.visual(
            Cylinder(radius=0.055, length=0.006),
            origin=Origin(xyz=(bx, by, 0.7875)),  # z 0.7845 .. 0.7905
            material=drip_silver,
            name=f"drip_cup_{i}",
        )
        body.visual(
            Cylinder(radius=0.030, length=0.010),
            origin=Origin(xyz=(bx, by, 0.795)),  # z 0.790 .. 0.800
            material=steel_silver,
            name=f"burner_body_{i}",
        )
        body.visual(
            Cylinder(radius=0.022, length=0.004),
            origin=Origin(xyz=(bx, by, 0.8015)),  # z 0.7995 .. 0.8035
            material=cap_silver,
            name=f"burner_cap_{i}",
        )

    # ------------------------------------------------------- cast-iron grates
    # Two black pan grates of thin square bars, one over each burner pair.
    bar_zc = GRATE_Z0 + GRATE_BAR / 2.0  # 0.809
    for gi, side in enumerate((-1.0, 1.0)):
        grate = model.part(f"grate_{gi}")
        grate.visual(
            Box((GRATE_BAR, 0.230, GRATE_BAR)),
            origin=Origin(xyz=(0.281, side * 0.125, bar_zc)),
            material=iron_black,
            name="frame_bar_front",
        )
        grate.visual(
            Box((GRATE_BAR, 0.230, GRATE_BAR)),
            origin=Origin(xyz=(-0.281, side * 0.125, bar_zc)),
            material=iron_black,
            name="frame_bar_rear",
        )
        grate.visual(
            Box((0.570, GRATE_BAR, GRATE_BAR)),
            origin=Origin(xyz=(0.0, side * 0.236, bar_zc)),
            material=iron_black,
            name="frame_bar_outer",
        )
        grate.visual(
            Box((0.570, GRATE_BAR, GRATE_BAR)),
            origin=Origin(xyz=(0.0, side * 0.014, bar_zc)),
            material=iron_black,
            name="frame_bar_inner",
        )
        for fi, fy in enumerate((0.0695, 0.125, 0.1805)):
            grate.visual(
                Box((0.570, GRATE_BAR, GRATE_BAR)),
                origin=Origin(xyz=(0.0, side * fy, bar_zc)),
                material=iron_black,
                name=f"finger_{fi}",
            )
        model.articulation(
            f"grate_{gi}_mount",
            ArticulationType.FIXED,
            parent=body,
            child=grate,
            origin=Origin(),
        )

    # -------------------------------------------------------- cooktop glass lid
    # Hinged tempered-glass lid over the burner cooktop.
    # Rear REVOLUTE hinge swings the lid up to expose the burners.
    lid = model.part("cooktop_lid")
    lid_glass_mat = model.material("lid_glass", rgba=(0.14, 0.16, 0.18, 0.80))
    lid_trim_mat = model.material("lid_trim", rgba=(0.58, 0.59, 0.60, 1.0))

    LID_DEPTH = 0.56       # X extent from hinge (covers burner area)
    LID_WIDTH = 0.44       # Y extent (fits inside side rims)
    LID_THICK = 0.004      # 4 mm tempered glass
    LID_TRIM_W = 0.006     # trim bar width
    LID_TRIM_H = 0.006     # trim bar height
    HINGE_BRACKET_H = 0.015
    HINGE_X = -0.296       # behind the grate rear bars (clear by ~3 mm)
    HINGE_Z = RIM_TOP + HINGE_BRACKET_H  # 0.820

    # Hinge bracket mounted on the rear rim (visible steel support).
    # Narrow in X to stay behind the grate frame bars.
    body.visual(
        Box((0.014, LID_WIDTH + 0.02, HINGE_BRACKET_H)),
        origin=Origin(xyz=(HINGE_X, 0.0, RIM_TOP + HINGE_BRACKET_H / 2.0)),
        material=steel_silver,
        name="lid_hinge_bracket",
    )

    # Glass panel — extends forward from the hinge along local +X.
    lid.visual(
        Box((LID_DEPTH, LID_WIDTH, LID_THICK)),
        origin=Origin(xyz=(LID_DEPTH / 2.0, 0.0, LID_THICK / 2.0)),
        material=lid_glass_mat,
        name="lid_glass",
    )

    # Metal trim frame around the glass perimeter.
    lid.visual(
        Box((LID_TRIM_W, LID_WIDTH + 2.0 * LID_TRIM_W, LID_TRIM_H)),
        origin=Origin(xyz=(LID_DEPTH + LID_TRIM_W / 2.0, 0.0, LID_TRIM_H / 2.0)),
        material=lid_trim_mat,
        name="lid_trim_front",
    )
    lid.visual(
        Box((LID_TRIM_W, LID_WIDTH + 2.0 * LID_TRIM_W, LID_TRIM_H)),
        origin=Origin(xyz=(-LID_TRIM_W / 2.0, 0.0, LID_TRIM_H / 2.0)),
        material=lid_trim_mat,
        name="lid_trim_rear",
    )
    lid.visual(
        Box((LID_DEPTH + 2.0 * LID_TRIM_W, LID_TRIM_W, LID_TRIM_H)),
        origin=Origin(xyz=(LID_DEPTH / 2.0, -(LID_WIDTH / 2.0 + LID_TRIM_W / 2.0), LID_TRIM_H / 2.0)),
        material=lid_trim_mat,
        name="lid_trim_left",
    )
    lid.visual(
        Box((LID_DEPTH + 2.0 * LID_TRIM_W, LID_TRIM_W, LID_TRIM_H)),
        origin=Origin(xyz=(LID_DEPTH / 2.0, (LID_WIDTH / 2.0 + LID_TRIM_W / 2.0), LID_TRIM_H / 2.0)),
        material=lid_trim_mat,
        name="lid_trim_right",
    )

    # Small raised grip tab at the front edge for lifting.
    lid.visual(
        Box((0.012, 0.08, 0.012)),
        origin=Origin(xyz=(LID_DEPTH + LID_TRIM_W + 0.006, 0.0, LID_THICK / 2.0 + 0.006)),
        material=lid_trim_mat,
        name="lid_handle",
    )

    # REVOLUTE hinge: axis (0, -1, 0) so positive q lifts the front edge upward.
    model.articulation(
        "cooktop_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=math.radians(85.0)
        ),
    )

    # --------------------------------------------------------------- oven door
    # Drop-down door hinged along its bottom edge at the front of the plinth.
    # Child frame: hinge line at local origin, door panel extends along +Z
    # (up, when closed) and +X (outward thickness).
    door = model.part("oven_door")
    door_frame = BezelGeometry(
        (0.36, 0.32),  # glass opening (width, height)
        (0.50, DOOR_H),  # outer door panel
        DOOR_T,
        opening_shape="rounded_rect",
        outer_shape="rect",
        opening_corner_radius=0.012,
    )
    door.visual(
        mesh_from_geometry(door_frame, "oven_door_frame"),
        # Map bezel local X->door Y (width), Y->door Z (height), Z->door X (thickness).
        origin=Origin(xyz=(DOOR_T / 2.0, 0.0, DOOR_H / 2.0), rpy=(math.pi / 2, 0.0, math.pi / 2)),
        material=enamel_white,
        name="door_frame",
    )
    # Large dark tinted glass window; edges embed into the frame walls.
    door.visual(
        Box((0.012, 0.40, 0.36)),
        origin=Origin(xyz=(0.018, 0.0, DOOR_H / 2.0)),
        material=glass_tint,
        name="window_glass",
    )
    # Slim horizontal handle bar near the door top edge on two standoffs.
    for si, hy in enumerate((-0.18, 0.18)):
        door.visual(
            Cylinder(radius=0.009, length=0.030),
            origin=Origin(xyz=(0.053, hy, 0.47), rpy=(0.0, math.pi / 2, 0.0)),
            material=steel_silver,
            name=f"handle_standoff_{si}",
        )
    door.visual(
        Cylinder(radius=0.011, length=0.44),
        origin=Origin(xyz=(0.072, 0.0, 0.47), rpy=(math.pi / 2, 0.0, 0.0)),
        material=steel_silver,
        name="handle_bar",
    )
    model.articulation(
        "oven_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HALF_D, 0.0, PLINTH_TOP)),
        # +Y axis: positive q tips the door top edge outward (+X) and down.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=0.0, upper=math.pi / 2),
    )

    # ------------------------------------------------------------- control knobs
    # Six round white knobs on the fascia, each a revolute dial about +X
    # (horizontal axis perpendicular to the front panel).
    for i, ky in enumerate(KNOB_YS):
        knob = model.part(f"knob_{i}")
        knob_geo = KnobGeometry(
            0.038,
            0.024,
            body_style="skirted",
            top_diameter=0.030,
            skirt=KnobSkirt(0.048, 0.006, flare=0.08, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            center=False,  # mounting face on z=0
        )
        knob.visual(
            mesh_from_geometry(knob_geo, f"range_knob_{i}"),
            # Rotate knob axis +Z onto +X so it points out of the fascia.
            origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
            material=knob_white,
            name="knob_cap",
        )
        model.articulation(
            f"knob_{i}_dial",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(HALF_D, ky, KNOB_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=6.0, lower=0.0, upper=math.radians(270.0)
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("oven_door")
    hinge = object_model.get_articulation("oven_door_hinge")
    knobs = [object_model.get_part(f"knob_{i}") for i in range(6)]
    grates = [object_model.get_part(f"grate_{i}") for i in range(2)]

    # --- oven door: closed pose seats flush on the body front, full width ---
    ctx.expect_gap(
        door,
        body,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0005,
        name="closed door seats against body front",
    )
    ctx.expect_within(door, body, axes="y", margin=0.002, name="door spans within body width")

    # Dark tinted glass window is large and centered in the door.
    glass_aabb = ctx.part_element_world_aabb(door, elem="window_glass")
    ok_glass = glass_aabb is not None
    if glass_aabb is not None:
        gmin, gmax = glass_aabb
        ok_glass = (
            (gmax[1] - gmin[1]) >= 0.38
            and (gmax[2] - gmin[2]) >= 0.34
            and 0.30 <= 0.5 * (gmin[2] + gmax[2]) <= 0.42
        )
    ctx.check("door has large tinted glass window", ok_glass, f"glass_aabb={glass_aabb!r}")

    # Slim handle bar sits near the door top edge, proud of the door face.
    handle_aabb = ctx.part_element_world_aabb(door, elem="handle_bar")
    ok_handle = handle_aabb is not None
    if handle_aabb is not None:
        hmin, hmax = handle_aabb
        ok_handle = hmin[2] > 0.50 and hmax[0] > 0.35 and (hmax[1] - hmin[1]) > 0.40
    ctx.check("handle bar near door top edge", ok_handle, f"handle_aabb={handle_aabb!r}")

    # Drop-down motion: at 90 deg the door lies horizontal in front, low down.
    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: math.pi / 2}):
        open_aabb = ctx.part_world_aabb(door)
    ok_open = closed_aabb is not None and open_aabb is not None
    if closed_aabb is not None and open_aabb is not None:
        ok_open = (
            open_aabb[1][0] > 0.75  # extends far forward
            and open_aabb[1][2] < 0.16  # dropped flat near plinth height
            and open_aabb[0][2] > 0.0  # stays above the floor
            and open_aabb[1][2] < closed_aabb[1][2] - 0.40  # top edge swung down
        )
    ctx.check(
        "door drops down 90 deg in front of the oven",
        ok_open,
        f"closed={closed_aabb!r} open={open_aabb!r}",
    )
    lim = hinge.motion_limits
    ctx.check(
        "door hinge limits 0..~90deg",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower) < 1e-6
        and abs(lim.upper - math.pi / 2) < 0.1,
        f"limits={lim!r}",
    )

    # --- control knobs: six dials seated on the fascia above the door -------
    knob_ys = []
    for i, knob in enumerate(knobs):
        ctx.expect_contact(knob, body, contact_tol=5e-4, name=f"knob_{i} seated on fascia")
        pos = ctx.part_world_position(knob)
        ok_pos = pos is not None and pos[2] > DOOR_TOP + 0.04 and pos[0] > 0.29
        ctx.check(f"knob_{i} on fascia above oven door", ok_pos, f"pos={pos!r}")
        if pos is not None:
            knob_ys.append(pos[1])
        klim = object_model.get_articulation(f"knob_{i}_dial").motion_limits
        ctx.check(
            f"knob_{i} dial range ~270deg about front axis",
            klim is not None
            and klim.lower is not None
            and klim.upper is not None
            and abs(klim.lower) < 1e-6
            and abs(klim.upper - math.radians(270.0)) < 0.05,
            f"limits={klim!r}",
        )
    ctx.check(
        "six knobs form a horizontal row",
        len(knob_ys) == 6
        and all(knob_ys[i] < knob_ys[i + 1] - 0.05 for i in range(5))
        and (knob_ys[-1] - knob_ys[0]) > 0.30,
        f"knob_ys={knob_ys!r}",
    )

    # --- cooktop: four silver burner caps in a 2x2 layout --------------------
    quadrants = set()
    caps_ok = True
    for i in range(4):
        cap_aabb = ctx.part_element_world_aabb(body, elem=f"burner_cap_{i}")
        if cap_aabb is None:
            caps_ok = False
            continue
        cmin, cmax = cap_aabb
        cx = 0.5 * (cmin[0] + cmax[0])
        cy = 0.5 * (cmin[1] + cmax[1])
        if not (0.79 <= cmax[2] <= 0.81 and abs(cx) > 0.05 and abs(cy) > 0.05):
            caps_ok = False
        quadrants.add((cx > 0.0, cy > 0.0))
    ctx.check(
        "four burner caps in a 2x2 layout on the recessed cooktop",
        caps_ok and len(quadrants) == 4,
        f"quadrants={quadrants!r}",
    )

    # --- grates: rest on the cooktop rim and clear the burner caps ----------
    for gi, grate in enumerate(grates):
        ctx.expect_gap(
            grate,
            body,
            axis="z",
            negative_elem="rim_rear",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"grate_{gi} rests on cooktop rim",
        )
        ctx.expect_overlap(
            grate,
            body,
            axes="xy",
            min_overlap=0.02,
            name=f"grate_{gi} bars seated over the cooktop",
        )
    ctx.expect_gap(
        grates[0],
        body,
        axis="z",
        negative_elem="burner_cap_1",
        min_gap=0.0005,
        max_gap=0.01,
        name="grate bars pass just above the burner caps (left)",
    )
    ctx.expect_gap(
        grates[1],
        body,
        axis="z",
        negative_elem="burner_cap_0",
        min_gap=0.0005,
        max_gap=0.01,
        name="grate bars pass just above the burner caps (right)",
    )

    # --- oven cavity stays hollow behind the door ----------------------------
    probe = (0.05, 0.0, 0.45)
    blockers = []
    for vis in body.visuals:
        aabb = ctx.part_element_world_aabb(body, elem=vis.name)
        if aabb is None:
            continue
        amin, amax = aabb
        if all(amin[k] <= probe[k] <= amax[k] for k in range(3)):
            blockers.append(vis.name)
    ctx.check(
        "oven cavity hollow behind the door",
        not blockers,
        f"solid geometry at cavity probe point: {blockers!r}",
    )

    # Oven rack shelf sits inside the cavity.
    rack_aabb = ctx.part_element_world_aabb(body, elem="oven_rack")
    ok_rack = rack_aabb is not None
    if rack_aabb is not None:
        rmin, rmax = rack_aabb
        rz = 0.5 * (rmin[2] + rmax[2])
        ok_rack = PLINTH_TOP < rz < DOOR_TOP and rmin[1] > -0.21 and rmax[1] < 0.21
    ctx.check("oven rack shelf inside the cavity", ok_rack, f"rack_aabb={rack_aabb!r}")

    # --- cooktop glass lid: hinged at rear, lifts upward ------------------
    lid = object_model.get_part("cooktop_lid")
    lid_hinge = object_model.get_articulation("cooktop_lid_hinge")

    # Lid hinge origin is at the rear of the cooktop near the rim.
    hinge_origin = lid_hinge.origin
    ok_hinge_origin = hinge_origin is not None
    if ok_hinge_origin:
        ok_hinge_origin = (
            hinge_origin.xyz[0] < -0.20  # rear
            and hinge_origin.xyz[2] > RIM_TOP - 0.01  # at/above rim
        )
    ctx.check(
        "cooktop lid hinge at rear of cooktop",
        ok_hinge_origin,
        f"origin={hinge_origin!r}",
    )

    # Lid hinge axis is parallel to Y (left-right), not the oven door axis.
    ok_axis = lid_hinge.axis is not None and abs(lid_hinge.axis[1]) > 0.99
    ctx.check("cooktop lid hinge axis is horizontal (Y)", ok_axis, f"axis={lid_hinge.axis!r}")

    # Lid hinge limits: 0 closed to ~85 deg open.
    llim = lid_hinge.motion_limits
    ctx.check(
        "cooktop lid hinge limits 0..~85deg",
        llim is not None
        and llim.lower is not None
        and llim.upper is not None
        and abs(llim.lower) < 1e-6
        and abs(llim.upper - math.radians(85.0)) < 0.1,
        f"limits={llim!r}",
    )

    # Closed lid sits over the cooktop area, above the burners.
    closed_lid = ctx.part_world_aabb(lid)
    ok_closed = closed_lid is not None
    if closed_lid is not None:
        cmin, cmax = closed_lid
        ok_closed = (
            cmin[2] > 0.80   # above the rim
            and cmax[2] < 0.84  # thin
            and cmax[0] > 0.20  # extends forward
            and cmin[0] < -0.20  # extends back
        )
    ctx.check(
        "closed cooktop lid sits flat over the burners",
        ok_closed,
        f"lid_aabb={closed_lid!r}",
    )

    # Lid glass element exists and is semi-transparent.
    glass_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_glass")
    ctx.check(
        "cooktop lid has glass panel",
        glass_lid_aabb is not None,
        f"lid_glass_aabb={glass_lid_aabb!r}",
    )

    # Open lid swings upward to expose the burners underneath.
    with ctx.pose({lid_hinge: math.radians(85.0)}):
        open_lid = ctx.part_world_aabb(lid)
    ok_lift = closed_lid is not None and open_lid is not None
    if ok_lift:
        cmin_c, cmax_c = closed_lid
        omin, omax = open_lid
        ok_lift = (
            omax[2] > cmax_c[2] + 0.35  # swings high
            and omin[0] < cmin_c[0] + 0.10  # rear stays near hinge
            and omax[2] > 1.1  # well above cooktop
        )
    ctx.check(
        "cooktop lid lifts up to expose burners",
        ok_lift,
        f"closed={closed_lid!r} open={open_lid!r}",
    )

    # Burners remain visible underneath the lid (still on the body).
    burner_caps = [ctx.part_element_world_aabb(body, elem=f"burner_cap_{i}") for i in range(4)]
    ctx.check(
        "four burners present under the cooktop lid",
        all(b is not None for b in burner_caps),
        f"caps={[b for b in burner_caps if b is None]!r}",
    )

    return ctx.report()


object_model = build_object_model()
