from __future__ import annotations

"""Corner cabinet variant: angled front doors, hinged top lid, interior shelves,
and recessed panel borders on each door face.

Overall envelope ~1.60 m wide x ~0.55 m deep (at the centre ridge) x 1.80 m tall,
brushed/tarnished raw steel.  A hollow thin-wall (~0.02 m) carcass sits on four
short splayed legs.  The front face is not flat: two doors angle inward from the
outer side-wall corners to meet at a forward centre ridge, giving the cabinet
its recognisable corner-cabinet silhouette.  Each door is an independent
revolute joint about a vertical axis at its outer hinge edge, 0..~110 deg
outward.  A top lid hinges upward on a rear revolute joint.  Two interior shelf
boards are visible through the door opening.  Each door carries a recessed
panel-border frame (four raised strips forming a rectangular surround on the
outer face), a dark rounded-end ventilation slot near the bottom, and stamped
vent lines near the top.
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
# Global dimensions (meters).  Cabinet centred on X, back at −Y, front at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60          # overall carcass width
CAB_D = 0.40          # carcass depth (back wall to hinge-line plane)
BACK_Y = -CAB_D / 2   # −0.20
HINGE_Y = CAB_D / 2   # +0.20
RIDGE_FWD = 0.15      # centre-ridge protrusion forward from the hinge line
RIDGE_Y = HINGE_Y + RIDGE_FWD  # +0.35
CAB_TOP = 1.80
LEG_H = 0.15
WALL_T = 0.02

HINGE_X = CAB_W / 2 - WALL_T   # 0.78  inner face of side wall

# Door geometry (computed from the angled plan)
DOOR_DX = HINGE_X      # 0.78
DOOR_DY = RIDGE_FWD    # 0.15
DOOR_LEN = math.hypot(DOOR_DX, DOOR_DY)       # ~0.794
DOOR_ANGLE = math.atan2(DOOR_DY, DOOR_DX)      # ~0.190 rad  (~10.9°)
DOOR_ANGLE_DEG = math.degrees(DOOR_ANGLE)

BOT_RAIL_TOP = LEG_H + 0.06       # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06     # 1.74
DOOR_Z0 = BOT_RAIL_TOP + 0.002    # 0.212
DOOR_Z1 = TOP_RAIL_BOT - 0.002    # 1.738
DOOR_H = DOOR_Z1 - DOOR_Z0        # 1.526
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)

DOOR_T = WALL_T
LID_T = 0.022
DOOR_OPEN = math.radians(110.0)
LID_OPEN = math.radians(100.0)

CARCASS_H = CAB_TOP - LEG_H
CARCASS_ZC = LEG_H + CARCASS_H / 2

CAP_T = 0.022
CAP_OVERHANG = 0.02

BARREL_R = 0.0075
KNUCKLE_R = 0.0095
BARREL_LEN = DOOR_H - 0.04

# ---------------------------------------------------------------------------
# CadQuery mesh builders
# ---------------------------------------------------------------------------

def _corner_door_panel(sign: float, mesh_name: str):
    """Angled door leaf: flat panel with a rounded-end vent slot near the
    bottom.  ``sign``=+1 → left door (extends +X, rotates CCW to angle);
    ``sign``=−1 → right door (extends −X, rotates CW)."""
    panel = (
        cq.Workplane("XY")
        .box(DOOR_LEN, DOOR_T, DOOR_H)
        .translate((sign * DOOR_LEN / 2, -DOOR_T / 2, 0))
    )
    # Rounded-end vent slot near bottom
    slot_len = DOOR_H * 0.18
    slot_w = 0.025
    slot_zc = -DOOR_H * 0.34
    slot_cutter = (
        cq.Workplane("XZ")
        .slot2D(slot_len, slot_w, 90)
        .extrude(DOOR_T + 0.02, both=True)
        .translate((sign * DOOR_LEN / 2, 0, slot_zc))
    )
    panel = panel.cut(slot_cutter)
    # Rotate whole panel to the closed-door angle
    panel = panel.rotate((0, 0, 0), (0, 0, 1), sign * DOOR_ANGLE_DEG)
    return mesh_from_cadquery(panel, mesh_name)


def _border_frame(sign: float, mesh_name: str):
    """Four raised strips forming a rectangular surround on the door outer
    face.  Built in the same pre-rotation frame as the leaf, then rotated."""
    inset = 0.065
    bw = 0.025   # strip width
    bp = 0.005   # proudness above the face

    def _strip(sx, sy, sz, tx, ty, tz):
        return cq.Workplane("XY").box(sx, sy, sz).translate((tx, ty, tz))

    inner_h = DOOR_H - 2 * inset - 2 * bw
    inner_w = DOOR_LEN - 2 * inset

    top = _strip(inner_w, bp, bw,
                 sign * DOOR_LEN / 2, bp / 2, DOOR_H / 2 - inset - bw / 2)
    bot = _strip(inner_w, bp, bw,
                 sign * DOOR_LEN / 2, bp / 2, -DOOR_H / 2 + inset + bw / 2)
    hinge_side = _strip(bw, bp, inner_h,
                        sign * (inset + bw / 2), bp / 2, 0)
    free_side = _strip(bw, bp, inner_h,
                       sign * (DOOR_LEN - inset - bw / 2), bp / 2, 0)
    frame = top.union(bot).union(hinge_side).union(free_side)
    frame = frame.rotate((0, 0, 0), (0, 0, 1), sign * DOOR_ANGLE_DEG)
    return mesh_from_cadquery(frame, mesh_name)


def _hinge_barrel(mesh_name: str):
    """Piano-hinge knuckle column along the door hinge edge (local Z)."""
    barrel = cq.Workplane("XY").circle(BARREL_R).extrude(BARREL_LEN / 2, both=True)
    ring_h = 0.055
    for zc in (-0.55, -0.28, 0.0, 0.28, 0.55):
        ring = (
            cq.Workplane("XY")
            .circle(KNUCKLE_R)
            .extrude(ring_h / 2, both=True)
            .translate((0, 0, zc))
        )
        barrel = barrel.union(ring)
    return mesh_from_cadquery(barrel, mesh_name)


def _leg_solid(mesh_name: str):
    """Splayed tapered leg."""
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


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_cabinet")

    # Materials
    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_shelf = model.material("steel_shelf", rgba=(0.50, 0.51, 0.53, 1.0))
    steel_lid = model.material("steel_lid", rgba=(0.56, 0.57, 0.59, 1.0))

    # ==================================================================
    # Cabinet body
    # ==================================================================
    body = model.part("cabinet_body")

    # --- Side walls (from back to hinge-line plane) --------------------
    for sx, vn in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, CARCASS_H)),
            origin=Origin(xyz=(sx * (CAB_W / 2 - WALL_T / 2), 0.0, CARCASS_ZC)),
            material=steel_body,
            name=vn,
        )
    # --- Back wall -----------------------------------------------------
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, CARCASS_H - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2, CARCASS_ZC)),
        material=steel_body,
        name="back_wall",
    )
    # --- Bottom panel --------------------------------------------------
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + WALL_T / 2)),
        material=steel_body,
        name="bottom_panel",
    )

    # --- Front bottom rail (flat, at hinge-line plane) -----------------
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, BOT_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(xyz=(0.0, HINGE_Y - WALL_T / 2, (LEG_H + BOT_RAIL_TOP) / 2)),
        material=steel_body,
        name="front_bottom_rail",
    )
    # --- Front top rail (flat, at hinge-line plane) --------------------
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, CAB_TOP - TOP_RAIL_BOT + 0.01)),
        origin=Origin(xyz=(0.0, HINGE_Y - WALL_T / 2, (TOP_RAIL_BOT + CAB_TOP) / 2)),
        material=steel_body,
        name="front_top_rail",
    )

    # --- Angled front frame rails (connecting side-wall front edges to
    #     the centre ridge, at the door-opening top and bottom) ----------
    rail_h = 0.035
    for side, prefix in ((+1.0, "left"), (-1.0, "right")):
        # Bottom angled rail
        body.visual(
            Box((DOOR_LEN + 0.01, WALL_T, rail_h)),
            origin=Origin(
                xyz=(
                    side * (-HINGE_X / 2),   # midpoint X between hinge and centre
                    (HINGE_Y + RIDGE_Y) / 2,
                    DOOR_Z0 - rail_h / 2,
                ),
                rpy=(0, 0, side * DOOR_ANGLE),
            ),
            material=steel_trim,
            name=f"angled_rail_bottom_{prefix}",
        )
        # Top angled rail
        body.visual(
            Box((DOOR_LEN + 0.01, WALL_T, rail_h)),
            origin=Origin(
                xyz=(
                    side * (-HINGE_X / 2),
                    (HINGE_Y + RIDGE_Y) / 2,
                    DOOR_Z1 + rail_h / 2,
                ),
                rpy=(0, 0, side * DOOR_ANGLE),
            ),
            material=steel_trim,
            name=f"angled_rail_top_{prefix}",
        )

    # --- Centre post (vertical, where the two doors meet) --------------
    post_w = 0.030
    post_h = DOOR_Z1 - DOOR_Z0 + 0.04
    body.visual(
        Box((post_w, WALL_T, post_h)),
        origin=Origin(xyz=(0.0, RIDGE_Y - WALL_T / 2, DOOR_ZC)),
        material=steel_trim,
        name="center_post",
    )

    # --- Interior shelves with support brackets -------------------------
    shelf_w = CAB_W - 2 * WALL_T - 0.01
    shelf_d = CAB_D - 2 * WALL_T - 0.01
    shelf_t = 0.015
    bracket_w = 0.012
    bracket_h = 0.04
    for i, sz in enumerate((0.70, 1.20)):
        body.visual(
            Box((shelf_w, shelf_d, shelf_t)),
            origin=Origin(xyz=(0.0, 0.0, sz)),
            material=steel_shelf,
            name=f"shelf_{i}",
        )
        # Four small L-bracket supports connecting the shelf to the side walls
        for sx in (-1.0, 1.0):
            bx = sx * (shelf_w / 2 + bracket_w / 2)
            body.visual(
                Box((bracket_w, 0.04, bracket_h)),
                origin=Origin(xyz=(bx, 0.0, sz - bracket_h / 2 - shelf_t / 2)),
                material=steel_trim,
                name=f"shelf_{i}_bracket_{'l' if sx < 0 else 'r'}",
            )

    # --- Top frame lips (narrow horizontal strips around the top opening
    #     for the lid to rest on) ---------------------------------------
    lip_w = 0.030
    # Front lip
    body.visual(
        Box((CAB_W - 2 * WALL_T, lip_w, WALL_T)),
        origin=Origin(xyz=(0.0, HINGE_Y - lip_w / 2, CAB_TOP - WALL_T / 2)),
        material=steel_body,
        name="top_lip_front",
    )
    # Back lip
    body.visual(
        Box((CAB_W - 2 * WALL_T, lip_w, WALL_T)),
        origin=Origin(xyz=(0.0, BACK_Y + lip_w / 2, CAB_TOP - WALL_T / 2)),
        material=steel_body,
        name="top_lip_back",
    )
    # Side lips
    for sx, sn in ((-1.0, "top_lip_left"), (1.0, "top_lip_right")):
        body.visual(
            Box((lip_w, CAB_D - 2 * lip_w, WALL_T)),
            origin=Origin(xyz=(sx * (CAB_W / 2 - WALL_T - lip_w / 2), 0.0, CAB_TOP - WALL_T / 2)),
            material=steel_body,
            name=sn,
        )

    # --- Thin riveted top cap strip (sits just below the lid) ----------
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T * 0.5)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - CAP_T * 0.25)),
        material=steel_trim,
        name="top_cap",
    )
    # Raised rivet dots along the front top rail
    n_riv = 11
    for i in range(n_riv):
        rx = -0.68 + i * (1.36 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.004),
            origin=Origin(xyz=(rx, HINGE_Y + 0.002, 1.77)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # --- Splayed legs --------------------------------------------------
    leg_mesh = _leg_solid("splayed_leg")
    leg_corners = [
        (0.72, 0.14, 0.0),
        (-0.72, 0.14, math.pi / 2),
        (-0.72, -0.14, math.pi),
        (0.72, -0.14, 3 * math.pi / 2),
    ]
    for i, (lx, ly, yaw) in enumerate(leg_corners):
        body.visual(
            leg_mesh,
            origin=Origin(xyz=(lx, ly, 0.0), rpy=(0.0, 0.0, yaw)),
            material=steel_leg,
            name=f"leg_{i}",
        )

    # ==================================================================
    # Doors (two angled doors meeting at the centre ridge)
    # ==================================================================
    door_specs = [
        # (sign, hinge_x, axis_z)
        (+1.0, -HINGE_X, +1.0),   # left door: hinge on left, opens CCW
        (-1.0, +HINGE_X, -1.0),   # right door: hinge on right, opens CW
    ]
    doors = []
    hinges = []

    hinge_barrel_mesh = _hinge_barrel("hinge_barrel")

    for i, (sign, hinge_x, axis_z) in enumerate(door_specs):
        door = model.part(f"door_{i}")

        # Door leaf (panel with vent slot)
        door.visual(
            _corner_door_panel(sign, f"door_leaf_{i}"),
            material=steel_door,
            name="leaf",
        )

        # Recessed panel border frame (raised strips on outer face)
        door.visual(
            _border_frame(sign, f"border_{i}"),
            material=steel_trim,
            name="border_frame",
        )

        # Dark vent backing behind the slot
        slot_zc = -DOOR_H * 0.34
        slot_len = DOOR_H * 0.18
        # Position in the door's rotated frame
        mid_along = DOOR_LEN / 2
        vx = sign * mid_along * math.cos(DOOR_ANGLE)
        vy = mid_along * math.sin(DOOR_ANGLE) - DOOR_T - 0.002
        door.visual(
            Box((0.035, 0.005, slot_len + 0.04)),
            origin=Origin(xyz=(vx, vy, slot_zc)),
            material=steel_dark,
            name="vent_backing",
        )

        # Stamped vent lines near the top (dark strips on outer face)
        for j, dz in enumerate((0.58, 0.60, 0.62)):
            vent_x = sign * mid_along * math.cos(DOOR_ANGLE)
            vent_y = mid_along * math.sin(DOOR_ANGLE) + 0.001
            door.visual(
                Box((0.14, 0.004, 0.006)),
                origin=Origin(xyz=(vent_x, vent_y, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )

        # Piano-hinge knuckle column on the hinge edge
        door.visual(
            hinge_barrel_mesh,
            origin=Origin(xyz=(0.0, 0.004, 0.0)),
            material=steel_trim,
            name="hinge_barrel",
        )

        # Door articulation
        hinge = model.articulation(
            f"door_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, HINGE_Y, DOOR_ZC)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )
        doors.append(door)
        hinges.append(hinge)

    # ==================================================================
    # Top lid (hinges upward on a rear revolute joint)
    # ==================================================================
    lid = model.part("top_lid")
    lid_w = CAB_W + 2 * CAP_OVERHANG - 0.01
    lid_d = CAB_D + 2 * CAP_OVERHANG - 0.01

    # Lid panel: extends from hinge (local origin at rear top edge) along +Y
    lid.visual(
        Box((lid_w, lid_d, LID_T)),
        origin=Origin(xyz=(0.0, lid_d / 2, LID_T / 2)),
        material=steel_lid,
        name="lid_panel",
    )
    # Small handle tab on the front edge of the lid
    lid.visual(
        Box((0.08, 0.015, 0.012)),
        origin=Origin(xyz=(0.0, lid_d - 0.005, LID_T + 0.006)),
        material=steel_trim,
        name="lid_handle",
    )
    # Hinge barrel along the rear edge
    lid.visual(
        Cylinder(radius=0.008, length=lid_w * 0.6),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -math.pi / 2, 0.0)),
        material=steel_dark,
        name="lid_hinge_barrel",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, BACK_Y - CAP_OVERHANG, CAB_TOP)),
        # +X axis: right-hand rule rotates +Y toward +Z → front edge lifts up
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=2.0, lower=0.0, upper=LID_OPEN
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door0 = object_model.get_part("door_0")
    door1 = object_model.get_part("door_1")
    hinge0 = object_model.get_articulation("door_0_hinge")
    hinge1 = object_model.get_articulation("door_1_hinge")
    lid = object_model.get_part("top_lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    # ---- Intentional overlaps ----------------------------------------
    # Hinge barrels lap the side-wall front edges
    ctx.allow_overlap(
        door0, body,
        elem_a="hinge_barrel", elem_b="side_wall_0",
        reason="Piano-hinge knuckle column laps the side wall edge it pivots on.",
    )
    ctx.allow_overlap(
        door1, body,
        elem_a="hinge_barrel", elem_b="side_wall_1",
        reason="Piano-hinge knuckle column laps the side wall edge it pivots on.",
    )
    # Doors close against the centre post
    ctx.allow_overlap(
        door0, body,
        elem_a="leaf", elem_b="center_post",
        reason="Angled door leaf closes against the centre post at the ridge.",
    )
    ctx.allow_overlap(
        door1, body,
        elem_a="leaf", elem_b="center_post",
        reason="Angled door leaf closes against the centre post at the ridge.",
    )
    # Border frame sits proud on the leaf face
    ctx.allow_overlap(
        door0, door0,
        elem_a="border_frame", elem_b="leaf",
        reason="Border frame strips sit proud on the door leaf outer face.",
    )
    ctx.allow_overlap(
        door1, door1,
        elem_a="border_frame", elem_b="leaf",
        reason="Border frame strips sit proud on the door leaf outer face.",
    )
    # Angled doors meet at the centre ridge; panel thickness causes small
    # overlap where the two leaves converge on the centre post.
    ctx.allow_overlap(
        door0, door1,
        elem_a="leaf", elem_b="leaf",
        reason="Angled door leaves converge on the centre ridge; small panel-thickness overlap at the meeting line is separated by the centre post.",
    )
    # Lid hinge barrel sits just behind the top cap strip.
    ctx.allow_overlap(
        body, lid,
        elem_a="top_cap", elem_b="lid_hinge_barrel",
        reason="Lid hinge barrel is embedded behind the top cap strip at the rear hinge line.",
    )

    # ---- Overall envelope --------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.6 m",
            1.55 <= (x1 - x0) <= 1.72,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "overall height ~1.8 m",
            1.78 <= z1 <= 1.88,
            details=f"top={z1:.3f}",
        )
        ctx.check("legs rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # ---- Door articulations ------------------------------------------
    for i, hinge in enumerate((hinge0, hinge1)):
        ctx.check(
            f"door_{i} hinge is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
        )
        ax = hinge.axis
        ctx.check(
            f"door_{i} hinge axis is vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
            details=str(ax),
        )
        lim = hinge.motion_limits
        ctx.check(
            f"door_{i} opens 0..~110 deg",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - DOOR_OPEN) < 1e-6,
        )

    # Hinge positions at outer edges
    ctx.check(
        "left door hinges on left edge",
        hinge0.origin.xyz[0] < -0.5,
        details=str(hinge0.origin.xyz[0]),
    )
    ctx.check(
        "right door hinges on right edge",
        hinge1.origin.xyz[0] > 0.5,
        details=str(hinge1.origin.xyz[0]),
    )

    # ---- Angled front: doors meet at centre ridge --------------------
    leaf0 = ctx.part_element_world_aabb(door0, elem="leaf")
    leaf1 = ctx.part_element_world_aabb(door1, elem="leaf")
    ctx.check(
        "left door free edge is forward of hinge line (angled front)",
        leaf0 is not None and leaf0[1][1] > HINGE_Y + 0.05,
        details=str(leaf0),
    )
    ctx.check(
        "right door free edge is forward of hinge line (angled front)",
        leaf1 is not None and leaf1[1][1] > HINGE_Y + 0.05,
        details=str(leaf1),
    )
    # Both doors reach near the centre (x ≈ 0)
    ctx.check(
        "left door reaches near the centre",
        leaf0 is not None and leaf0[1][0] > -0.05,
        details=str(leaf0),
    )
    ctx.check(
        "right door reaches near the centre",
        leaf1 is not None and leaf1[0][0] < 0.05,
        details=str(leaf1),
    )

    # ---- Door opening swings outward ---------------------------------
    closed0 = ctx.part_world_aabb(door0)
    closed1 = ctx.part_world_aabb(door1)
    with ctx.pose({hinge0: DOOR_OPEN, hinge1: DOOR_OPEN}):
        open0 = ctx.part_world_aabb(door0)
        open1 = ctx.part_world_aabb(door1)
    ctx.check(
        "both doors swing outward past the front face when opened",
        open0 is not None
        and open1 is not None
        and open0[1][1] > HINGE_Y + 0.25
        and open1[1][1] > HINGE_Y + 0.25,
        details=f"open0={open0}, open1={open1}",
    )
    # Left door's free edge moves leftward, right door's moves rightward
    ctx.check(
        "left door opens to the left",
        closed0 is not None and open0 is not None
        and open0[0][0] < closed0[0][0] - 0.10,
    )
    ctx.check(
        "right door opens to the right",
        closed1 is not None and open1 is not None
        and open1[1][0] > closed1[1][0] + 0.10,
    )

    # ---- Lid articulation --------------------------------------------
    ctx.check(
        "lid hinge is revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
    )
    lax = lid_hinge.axis
    ctx.check(
        "lid hinge axis is horizontal along X",
        abs(lax[0]) > 0.99 and abs(lax[1]) < 0.01 and abs(lax[2]) < 0.01,
        details=str(lax),
    )
    lim_lid = lid_hinge.motion_limits
    ctx.check(
        "lid hinge has positive range",
        lim_lid is not None and lim_lid.lower == 0.0 and lim_lid.upper > 0.5,
    )

    # Lid hinge is at the rear of the cabinet
    ctx.check(
        "lid hinge is at the rear",
        lid_hinge.origin.xyz[1] < -0.1,
        details=str(lid_hinge.origin.xyz[1]),
    )

    # Lid opens upward
    lid_rest = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: LID_OPEN}):
        lid_opened = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward (max Z increases)",
        lid_rest is not None
        and lid_opened is not None
        and lid_opened[1][2] > lid_rest[1][2] + 0.10,
        details=f"rest_top={lid_rest[1][2]:.3f} opened_top={lid_opened[1][2]:.3f}",
    )

    # ---- Interior shelves --------------------------------------------
    shelf0 = ctx.part_element_world_aabb(body, elem="shelf_0")
    shelf1 = ctx.part_element_world_aabb(body, elem="shelf_1")
    ctx.check("lower shelf exists inside the carcass", shelf0 is not None)
    ctx.check("upper shelf exists inside the carcass", shelf1 is not None)
    if shelf0 is not None:
        ctx.check(
            "lower shelf is between legs and top",
            shelf0[0][2] > LEG_H and shelf0[1][2] < CAB_TOP,
            details=str(shelf0),
        )
    if shelf1 is not None:
        ctx.check(
            "upper shelf is above the lower shelf",
            shelf0 is not None and shelf1[0][2] > shelf0[1][2],
        )

    # ---- Recessed panel borders on doors -----------------------------
    border0 = ctx.part_element_world_aabb(door0, elem="border_frame")
    border1 = ctx.part_element_world_aabb(door1, elem="border_frame")
    ctx.check(
        "door_0 has a recessed panel border frame",
        border0 is not None,
        details=str(border0),
    )
    ctx.check(
        "door_1 has a recessed panel border frame",
        border1 is not None,
        details=str(border1),
    )
    # Border frames should overlap with their respective door leaves in XY
    if border0 is not None and leaf0 is not None:
        ctx.expect_overlap(
            door0, door0,
            axes="xy",
            elem_a="border_frame", elem_b="leaf",
            min_overlap=0.10,
            name="door_0 border overlaps leaf footprint",
        )
    if border1 is not None and leaf1 is not None:
        ctx.expect_overlap(
            door1, door1,
            axes="xy",
            elem_a="border_frame", elem_b="leaf",
            min_overlap=0.10,
            name="door_1 border overlaps leaf footprint",
        )

    # ---- Centre post supports the angled front -----------------------
    cp = ctx.part_element_world_aabb(body, elem="center_post")
    ctx.check(
        "centre post is at the front ridge",
        cp is not None and cp[1][1] > HINGE_Y + 0.05,
        details=str(cp),
    )

    return ctx.report()


object_model = build_object_model()
