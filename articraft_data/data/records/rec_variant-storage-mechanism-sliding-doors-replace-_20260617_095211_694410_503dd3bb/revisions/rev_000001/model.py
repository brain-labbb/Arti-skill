from __future__ import annotations

"""Vintage industrial steel locker cabinet with two bypass sliding doors.

Reference image: picture/Other/Cabinet/001.png

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel. A hollow thin-wall (~0.02 m) carcass sits on four short splayed legs and
carries a thin riveted top cap strip. The front opening is covered by two
overlapping flat steel door panels that slide horizontally on top and bottom
track rails (bypass configuration). The front door rides on the front track,
the rear door on the rear track, so the panels pass each other at slightly
different depths. Each door carries a dark recessed ventilation slot with
rounded ends near the bottom, stamped vent lines near the top, and a small
recessed finger-pull grip near the grab edge.

Sliding-door variant of the four-hinge locker: carcass shell, legs, riveted
top cap, and interior shelf are identical to the parent. The front frame
loses its intermediate stiles (single wide opening) and gains top/bottom
track rails. The four hinged doors and their latch knobs are replaced by
two prismatic sliding doors.
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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Global dimensions (meters). Cabinet is centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60  # overall carcass width  (X)
CAB_D = 0.50  # overall carcass depth  (Y)
CAB_TOP = 1.80  # carcass top height   (Z)
LEG_H = 0.15  # splayed leg height; carcass starts here
WALL_T = 0.02  # thin steel wall thickness

FRONT_Y = CAB_D / 2.0  # +0.25, front face plane
BACK_Y = -CAB_D / 2.0

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.74

# --- Sliding-door specific dimensions ---
TRACK_H = 0.015  # track rail height
OPENING_INNER_X = CAB_W / 2.0 - WALL_T  # 0.78, inner side-wall face

SLIDE_DOOR_W = 0.80  # each door panel width
DOOR_T = WALL_T  # 0.02, same thickness as walls

# Door vertical extent: edges engage the track channels by ~1 mm so the
# doors read as riding in the tracks (small intentional Z overlap).
DOOR_Z0 = BOTTOM_RAIL_TOP + TRACK_H - 0.001  # 0.224
DOOR_Z1 = TOP_RAIL_BOT - TRACK_H + 0.001  # 1.726
DOOR_H = DOOR_Z1 - DOOR_Z0  # 1.502
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # 0.975

# Two bypass doors at different Y depths. Front door closer to viewer.
# 10 mm Y gap between the back of the front door and the front of the rear
# door, enough clearance for the door-face features (vent backing, grip bar).
DOOR0_Y = FRONT_Y - WALL_T - DOOR_T / 2.0 - 0.002  # 0.218
DOOR1_Y = DOOR0_Y - DOOR_T - 0.010  # 0.188
TRACK_Y = 0.5 * (DOOR0_Y + DOOR1_Y)  # 0.203
TRACK_DEPTH = 2.0 * DOOR_T + 0.012  # 0.052, spans both door depths + margin

# Rest positions: each door covers its half of the opening with centre overlap.
SLIDE_DOOR0_X = OPENING_INNER_X - SLIDE_DOOR_W / 2.0 - 0.001  # +0.379
SLIDE_DOOR1_X = -(OPENING_INNER_X - SLIDE_DOOR_W / 2.0 - 0.001)  # -0.379

# Travel: door can slide one full door-width to stack behind the other side.
SLIDE_TRAVEL = 0.75

SLOT_LEN = 0.36  # dark rounded-end vent slot near the bottom
SLOT_W = 0.030
SLOT_ZC = -0.40  # in door-local z (door centre = 0)

CAP_T = 0.022  # riveted top cap strip
CAP_OVERHANG = 0.02


def _slide_door_solid(mesh_name: str):
    """Sliding door panel: flat panel centred at origin with a rounded-end
    through slot near the bottom."""
    panel = cq.Workplane("XY").box(SLIDE_DOOR_W, DOOR_T, DOOR_H)
    # Vertical slot, rounded ends, cut through the leaf thickness.
    cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((0.0, 0.0, SLOT_ZC))
    )
    leaf = panel.cut(cutter)
    return mesh_from_cadquery(leaf, mesh_name)


def _leg_solid(mesh_name: str):
    """Splayed tapered leg: small foot on the floor, offset outward toward
    local (+x, +y); wide top section embedded into the carcass bottom."""
    leg = (
        cq.Workplane("XY")
        .center(0.03, 0.03)
        .rect(0.035, 0.035)
        .workplane(offset=LEG_H + 0.01)
        .center(-0.03, -0.03)
        .rect(0.06, 0.06)
        .loft()
    )
    return mesh_from_cadquery(leg, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_locker_cabinet_sliding")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door_a = model.material("steel_door_a", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_door_b = model.material("steel_door_b", rgba=(0.50, 0.51, 0.54, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_track = model.material("steel_track", rgba=(0.42, 0.43, 0.45, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_grip = model.material("steel_grip", rgba=(0.22, 0.22, 0.24, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + legs + front frame + tracks + cap
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - LEG_H  # 1.65
    carcass_zc = LEG_H + carcass_h / 2.0

    # Side walls (full depth, full carcass height).
    for sx, vname in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, carcass_h)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
            material=steel_body,
            name=vname,
        )
    # Back wall.
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, carcass_h - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, carcass_zc)),
        material=steel_body,
        name="back_wall",
    )
    # Bottom and top panels.
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )
    # Interior shelf (thin, embedded into side and back walls). Pulled back
    # from the door opening so it clears the rear sliding door panel.
    body.visual(
        Box((CAB_W - 2.0 * WALL_T + 0.01, 0.38, 0.015)),
        origin=Origin(xyz=(0.0, -0.04, 0.95)),
        material=steel_body,
        name="interior_shelf",
    )

    # Front frame: bottom rail and top rail only (no intermediate stiles for
    # the sliding-door variant — the front is one wide opening).
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LEG_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, CAB_TOP - TOP_RAIL_BOT + 0.01)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0)),
        material=steel_body,
        name="front_top_rail",
    )

    # Sliding-door track rails: thin channel rails at top and bottom of the
    # door opening, spanning the full opening width.
    track_w = CAB_W - 2.0 * WALL_T + 0.01  # 1.57
    for zc, vname in (
        (BOTTOM_RAIL_TOP + TRACK_H / 2.0, "bottom_track"),
        (TOP_RAIL_BOT - TRACK_H / 2.0, "top_track"),
    ):
        body.visual(
            Box((track_w, TRACK_DEPTH, TRACK_H)),
            origin=Origin(xyz=(0.0, TRACK_Y, zc)),
            material=steel_track,
            name=vname,
        )
        # Track front lip (thin raised strip on the visible front face of
        # each track, gives the channel a visible edge).
        body.visual(
            Box((track_w, 0.003, TRACK_H + 0.004)),
            origin=Origin(
                xyz=(0.0, TRACK_Y + TRACK_DEPTH / 2.0 + 0.001, zc)
            ),
            material=steel_trim,
            name=f"{vname}_lip",
        )

    # Thin riveted top cap strip with a slight overhang.
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    # Raised rivet dots along the top rail.
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, 1.77)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Splayed legs: one lofted solid reused at the four corners.
    leg_mesh = _leg_solid("splayed_leg")
    leg_corners = [
        (0.72, 0.19, 0.0),
        (-0.72, 0.19, math.pi / 2.0),
        (-0.72, -0.19, math.pi),
        (0.72, -0.19, 3.0 * math.pi / 2.0),
    ]
    for i, (lx, ly, yaw) in enumerate(leg_corners):
        body.visual(
            leg_mesh,
            origin=Origin(xyz=(lx, ly, 0.0), rpy=(0.0, 0.0, yaw)),
            material=steel_leg,
            name=f"leg_{i}",
        )

    # ------------------------------------------------------------------
    # Two bypass sliding doors.
    # door_0: front track (closer to viewer), slides LEFT to open.
    # door_1: rear track, slides RIGHT to open.
    # ------------------------------------------------------------------
    door_mesh = _slide_door_solid("slide_door_panel")
    door_specs = [
        # (rest_x, rest_y, slide_axis_x, leaf_material, grab_sign)
        (SLIDE_DOOR0_X, DOOR0_Y, -1.0, steel_door_a, +1.0),  # front, slides left
        (SLIDE_DOOR1_X, DOOR1_Y, +1.0, steel_door_b, -1.0),  # rear, slides right
    ]
    for i, (rest_x, rest_y, axis_x, leaf_mat, grab_sign) in enumerate(door_specs):
        door = model.part(f"door_{i}")

        # Door panel (shared mesh, centred at part origin).
        door.visual(door_mesh, material=leaf_mat, name="leaf")

        # Dark backing plate behind the through slot -> recessed dark slot.
        # Flush with the interior face of the door to avoid protruding into
        # the bypass gap between the two sliding panels.
        door.visual(
            Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
            origin=Origin(xyz=(0.0, -DOOR_T / 2.0 + 0.001, SLOT_ZC)),
            material=steel_dark,
            name="vent_backing",
        )

        # Stamped vent lines near the top (slightly proud thin dark strips).
        for j, dz in enumerate((0.58, 0.60, 0.62)):
            door.visual(
                Box((0.16, 0.004, 0.006)),
                origin=Origin(xyz=(0.0, DOOR_T / 2.0 + 0.001, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )

        # Recessed finger-pull grip near the grab edge.
        grip_x = grab_sign * (SLIDE_DOOR_W / 2.0 - 0.06)
        # Dark recessed cup (set slightly into the door face).
        door.visual(
            Box((0.07, 0.008, 0.035)),
            origin=Origin(xyz=(grip_x, DOOR_T / 2.0 - 0.002, 0.0)),
            material=steel_dark,
            name="grip_recess",
        )
        # Raised grip bar inside the recess.
        door.visual(
            Box((0.055, 0.005, 0.018)),
            origin=Origin(xyz=(grip_x, DOOR_T / 2.0 + 0.001, 0.0)),
            material=steel_grip,
            name="grip_bar",
        )

        # Prismatic articulation: joint origin at the door rest position,
        # axis along the cabinet width (X).
        model.articulation(
            f"door_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=door,
            origin=Origin(xyz=(rest_x, rest_y, DOOR_ZC)),
            axis=(axis_x, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=60.0, velocity=0.8, lower=0.0, upper=SLIDE_TRAVEL
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    doors = [object_model.get_part(f"door_{i}") for i in range(2)]
    slides = [object_model.get_articulation(f"door_{i}_slide") for i in range(2)]

    # --- Intentional local overlaps: door edges engage the track channels ----
    for i, door in enumerate(doors):
        for track_elem in ("bottom_track", "top_track"):
            ctx.allow_overlap(
                door,
                body,
                elem_a="leaf",
                elem_b=track_elem,
                reason=(
                    f"door_{i} panel edge engages the {track_elem} channel, "
                    "representing the sliding door riding in the track."
                ),
            )

    # --- Overall envelope, true scale, grounded on the floor ----------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.6 m (cap overhang included)",
            1.58 <= (x1 - x0) <= 1.70,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "overall depth ~0.5 m",
            0.48 <= (y1 - y0) <= 0.58,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "overall height ~1.8 m",
            1.78 <= z1 <= 1.86,
            details=f"top={z1:.3f}",
        )
        ctx.check("legs rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # --- Track rails present on the body -----------------------------------
    for track_name in ("bottom_track", "top_track"):
        taabb = ctx.part_element_world_aabb(body, elem=track_name)
        ctx.check(
            f"{track_name} spans the opening width",
            taabb is not None and (taabb[1][0] - taabb[0][0]) > 1.50,
            details=str(taabb),
        )

    # --- Sliding doors: joint type, axis, range, closed seating ------------
    for i, (door, slide) in enumerate(zip(doors, slides)):
        ctx.check(
            f"door_{i} slide is prismatic",
            slide.articulation_type == ArticulationType.PRISMATIC,
        )
        ax = slide.axis
        ctx.check(
            f"door_{i} slide axis is horizontal along X",
            abs(abs(ax[0]) - 1.0) < 1e-9
            and abs(ax[1]) < 1e-9
            and abs(ax[2]) < 1e-9,
            details=str(ax),
        )
        lim = slide.motion_limits
        ctx.check(
            f"door_{i} slide range 0..~0.75 m",
            lim is not None
            and lim.lower == 0.0
            and 0.70 <= lim.upper <= 0.80,
            details=f"lower={lim.lower}, upper={lim.upper}",
        )

        # Closed door sits within the opening width, flush with the front frame.
        ctx.expect_within(
            door,
            body,
            axes="x",
            margin=0.012,
            name=f"door_{i} stays inside the cabinet width when closed",
        )

        # Recessed dark vent slot near the bottom of the leaf.
        vb = ctx.part_element_world_aabb(door, elem="vent_backing")
        ctx.check(
            f"door_{i} vent slot sits in the lower half of the leaf",
            vb is not None and vb[1][2] < DOOR_ZC and vb[0][2] > DOOR_Z0,
            details=str(vb),
        )

        # Grip recess present near the grab edge.
        grip = ctx.part_element_world_aabb(door, elem="grip_recess")
        ctx.check(
            f"door_{i} has a grip recess at mid-height",
            grip is not None and abs(0.5 * (grip[0][2] + grip[1][2]) - DOOR_ZC) < 0.05,
            details=str(grip),
        )

    # Door 0 on the front track (higher Y) than door 1 on the rear track.
    ctx.check(
        "door_0 (front track) is closer to the viewer than door_1 (rear track)",
        slides[0].origin.xyz[1] > slides[1].origin.xyz[1],
        details=f"door0_y={slides[0].origin.xyz[1]:.3f}, door1_y={slides[1].origin.xyz[1]:.3f}",
    )

    # Door 0 covers the right half, door 1 covers the left half at rest.
    ctx.check(
        "door_0 covers the right half, door_1 the left half at rest",
        slides[0].origin.xyz[0] > 0.30 and slides[1].origin.xyz[0] < -0.30,
        details=f"x0={slides[0].origin.xyz[0]:.3f}, x1={slides[1].origin.xyz[0]:.3f}",
    )

    # Closed doors overlap in X (bypass configuration) but are separated in Y.
    ctx.expect_overlap(
        doors[0],
        doors[1],
        axes="x",
        min_overlap=0.02,
        name="closed doors overlap in X (bypass configuration)",
    )
    ctx.expect_gap(
        doors[0],
        doors[1],
        axis="y",
        min_gap=0.002,
        name="closed doors have a Y gap (no 3D interpenetration)",
    )

    # Proof checks for the track-engagement overlap allowances: each door
    # leaf stays within the track footprint on XY and has small Z overlap.
    for i, door in enumerate(doors):
        ctx.expect_within(
            door,
            body,
            axes="x",
            margin=0.02,
            inner_elem="leaf",
            outer_elem="bottom_track",
            name=f"door_{i} leaf stays within the bottom track width",
        )
        ctx.expect_gap(
            door,
            body,
            axis="z",
            max_penetration=0.002,
            positive_elem="leaf",
            negative_elem="bottom_track",
            name=f"door_{i} leaf engages the bottom track by ≤ 2 mm",
        )

    # --- Opening pose: door_0 slides left, revealing the right half ---------
    closed0 = ctx.part_world_aabb(doors[0])
    with ctx.pose({slides[0]: SLIDE_TRAVEL}):
        open0 = ctx.part_world_aabb(doors[0])
    ctx.check(
        "door_0 slides left when opened",
        closed0 is not None
        and open0 is not None
        and open0[1][0] < closed0[1][0] - 0.50,
        details=f"closed_right={closed0[1][0]:.3f}, open_right={open0[1][0]:.3f}",
    )

    # --- Opening pose: door_1 slides right, revealing the left half ---------
    closed1 = ctx.part_world_aabb(doors[1])
    with ctx.pose({slides[1]: SLIDE_TRAVEL}):
        open1 = ctx.part_world_aabb(doors[1])
    ctx.check(
        "door_1 slides right when opened",
        closed1 is not None
        and open1 is not None
        and open1[0][0] > closed1[0][0] + 0.50,
        details=f"closed_left={closed1[0][0]:.3f}, open_left={open1[0][0]:.3f}",
    )

    # Riveted top cap detail present along the top rail.
    rivet_aabb = ctx.part_element_world_aabb(body, elem="rivet_0")
    ctx.check(
        "rivet dots stand proud of the top rail face",
        rivet_aabb is not None
        and rivet_aabb[1][1] > FRONT_Y + 0.003
        and rivet_aabb[0][2] > TOP_RAIL_BOT,
        details=str(rivet_aabb),
    )

    return ctx.report()


object_model = build_object_model()
