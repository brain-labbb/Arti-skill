from __future__ import annotations

# Clear soap-dispenser bottle with a disc-top push-down cap closure.
# Forked from the press-pump parent: same bottle body, shoulder, neck, and
# threaded collar, but the pump head, swivel carrier, stem, spout, and dip
# tube are removed. In their place, a low flat disc-top cap sits on the
# collar: an annular base with a deep bore and a dispensing slot on one
# side (+X), plus a central round disc tile that depresses on a PRISMATIC
# push-down (along +Z, springing back) to open the slot.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    mesh_from_cadquery,
)

# ---- key heights along +Z (unchanged from parent) ----
BOTTLE_BOTTOM = 0.0
BOTTLE_BODY_TOP = 0.130
SHOULDER_TOP = 0.150
NECK_TOP = 0.168
BODY_R = 0.030
NECK_R = 0.0150

COLLAR_R = 0.0185
COLLAR_BOTTOM = 0.150
COLLAR_TOP = 0.176

# ---- disc-top cap dimensions ----
CAP_H = 0.012  # cap base total height
CAP_BASE_BOTTOM = COLLAR_TOP  # 0.176
CAP_BASE_TOP = COLLAR_TOP + CAP_H  # 0.188
CAP_BASE_R = COLLAR_R  # 0.0185
CAP_BORE_R = 0.013  # bore inner radius
CAP_BORE_DEPTH = CAP_H - 0.002  # 0.010, leaving 2mm floor

DISC_R = CAP_BORE_R + 0.0003  # 0.0133, slight press-fit overlap with bore wall
DISC_H = 0.004  # disc tile thickness
DISC_REST_BOTTOM = CAP_BASE_TOP - DISC_H  # 0.184

DISC_PRESS_TRAVEL = 0.004  # disc pushes down 4mm

SLOT_W = 0.010  # dispensing slot width (Y direction)

# ---- shared geometry helper: grip nub ----
_NUB_R = 0.0015
_NUB_H = 0.001
_NUB_ORBIT_R = DISC_R * 0.6
_NUB_COUNT = 4


def _grip_nub_mesh(index: int):
    """Single grip nub on the disc top face, placed by angular index."""
    ang = 2.0 * math.pi * index / _NUB_COUNT
    geo = CylinderGeometry(_NUB_R, _NUB_H, radial_segments=8)
    # Position relative to disc part frame (origin at bore floor / contact surface)
    nx = _NUB_ORBIT_R * math.cos(ang)
    ny = _NUB_ORBIT_R * math.sin(ang)
    nz = DISC_H + _NUB_H / 2.0  # on top of the disc tile
    geo.translate(nx, ny, nz)
    return mesh_from_geometry(geo, f"grip_nub_{index}")


# ---- bottle / collar geometry (unchanged from parent) ----

def _bottle_mesh():
    wall = 0.0022
    outer = [
        (0.0, BOTTLE_BOTTOM),
        (BODY_R, BOTTLE_BOTTOM + 0.004),
        (BODY_R, BOTTLE_BODY_TOP),
        (NECK_R + 0.004, SHOULDER_TOP),
        (NECK_R, SHOULDER_TOP + 0.004),
        (NECK_R, NECK_TOP),
    ]
    inner = [
        (0.0, BOTTLE_BOTTOM + 0.006),
        (BODY_R - wall, BOTTLE_BOTTOM + 0.010),
        (BODY_R - wall, BOTTLE_BODY_TOP),
        (NECK_R + 0.004 - wall, SHOULDER_TOP),
        (NECK_R - wall, SHOULDER_TOP + 0.004),
        (NECK_R - wall, NECK_TOP + 0.002),
    ]
    geo = LatheGeometry.from_shell_profiles(outer, inner, segments=48)
    return mesh_from_geometry(geo, "bottle_shell")


def _neck_threads_mesh():
    geo = None
    for zz in (0.154, 0.160, 0.166):
        ring = TorusGeometry(NECK_R + 0.0005, 0.0012, radial_segments=8, tubular_segments=36)
        ring.translate(0.0, 0.0, zz)
        geo = ring if geo is None else geo.merge(ring)
    return mesh_from_geometry(geo, "neck_threads")


def _collar_mesh():
    outer = [
        (0.0, COLLAR_BOTTOM),
        (COLLAR_R, COLLAR_BOTTOM),
        (COLLAR_R, COLLAR_TOP),
        (NECK_R + 0.0015, COLLAR_TOP),
    ]
    inner = [
        (0.0, COLLAR_BOTTOM + 0.0025),
        (NECK_R + 0.0010, COLLAR_BOTTOM + 0.0025),
        (NECK_R + 0.0010, COLLAR_TOP + 0.002),
    ]
    geo = LatheGeometry.from_shell_profiles(outer, inner, segments=48)
    return mesh_from_geometry(geo, "collar_shell")


def _collar_knurl_mesh():
    geo = None
    h = COLLAR_TOP - COLLAR_BOTTOM
    for i in range(22):
        ang = 2 * math.pi * i / 22
        rib = CylinderGeometry(0.0012, h, radial_segments=6)
        rib.translate(COLLAR_R - 0.0002, 0.0, COLLAR_BOTTOM + h / 2.0)
        rib.rotate_z(ang)
        geo = rib if geo is None else geo.merge(rib)
    return mesh_from_geometry(geo, "collar_knurl")


# ---- disc-top cap geometry ----

def _cap_base_cadquery():
    """Annular cap base with deep bore and dispensing slot on +X side.

    Built centered at origin then translated to world position.
    The bore goes most of the way through the cap (leaving a thin floor),
    and the dispensing slot is a rectangular cutout through the cap wall
    at the top of the bore (same Z range as the disc at rest).
    """
    # Solid cylinder centered at origin (z = -CAP_H/2 .. +CAP_H/2)
    cap = cq.Workplane("XY").cylinder(CAP_H, CAP_BASE_R)

    # Bore from top face downward
    cap = (
        cap.faces(">Z").workplane()
        .circle(CAP_BORE_R)
        .cutBlind(-CAP_BORE_DEPTH)
    )

    # Dispensing slot on +X side: rectangular cutout through the wall
    # Slot spans the top DISC_H of the bore (same Z as the disc at rest)
    # Local coords: slot center z = CAP_H/2 - DISC_H/2
    slot_z = CAP_H / 2.0 - DISC_H / 2.0
    slot_x = (CAP_BORE_R + CAP_BASE_R) / 2.0
    slot_radial_w = (CAP_BASE_R - CAP_BORE_R) + 0.004  # through wall + extra

    slot_cutter = (
        cq.Workplane("XY")
        .box(slot_radial_w, SLOT_W, DISC_H)
        .translate((slot_x, 0.0, slot_z))
    )
    cap = cap.cut(slot_cutter)

    # Translate to final world position
    cap = cap.translate((0.0, 0.0, CAP_BASE_BOTTOM + CAP_H / 2.0))
    return cap


def _slot_marker_mesh():
    """Small raised pad on cap exterior at the dispensing slot position.

    Makes the dispensing point clearly visible on the +X side of the cap.
    """
    pad_w = SLOT_W + 0.004
    pad_h = DISC_H + 0.004
    pad_d = 0.0015
    geo = BoxGeometry((pad_d, pad_w, pad_h))
    # Center on the slot: X = cap outer surface + half pad depth
    # Z = centered on the slot (= disc rest center Z)
    geo.translate(
        CAP_BASE_R + pad_d / 2.0,
        0.0,
        DISC_REST_BOTTOM + DISC_H / 2.0,
    )
    return mesh_from_geometry(geo, "slot_marker")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soap_dispenser_disc_cap")

    clear = model.material("clear_plastic", rgba=(0.74, 0.80, 0.82, 0.25))
    label_mat = model.material("label", rgba=(0.96, 0.96, 0.94, 1.0))
    white = model.material("cap_white", rgba=(0.93, 0.93, 0.94, 1.0))
    disc_mat = model.material("disc_tile", rgba=(0.88, 0.89, 0.91, 1.0))
    marker_mat = model.material("slot_marker_gray", rgba=(0.72, 0.73, 0.75, 1.0))

    # ---- bottle (root): clear shell + label band + neck threads ----
    bottle = model.part("bottle")
    bottle.visual(_bottle_mesh(), material=clear, name="bottle_shell")

    label = LatheGeometry.from_shell_profiles(
        [
            (BODY_R + 0.0008, 0.040),
            (BODY_R + 0.0008, 0.110),
        ],
        [
            (BODY_R - 0.0002, 0.040),
            (BODY_R - 0.0002, 0.110),
        ],
        segments=48,
    )
    bottle.visual(
        mesh_from_geometry(label, "label_band"), material=label_mat, name="label_band"
    )
    bottle.visual(_neck_threads_mesh(), material=clear, name="neck_threads")
    bottle.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BOTTLE_BODY_TOP),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, BOTTLE_BODY_TOP / 2.0)),
    )

    # ---- collar: white threaded cap, FIXED onto the bottle neck ----
    collar = model.part("collar")
    collar.visual(_collar_mesh(), material=white, name="collar_shell")
    collar.visual(_collar_knurl_mesh(), material=white, name="collar_knurl")
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_TOP - COLLAR_BOTTOM),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, (COLLAR_BOTTOM + COLLAR_TOP) / 2.0)),
    )
    model.articulation(
        "bottle_to_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- cap_base: annular housing with bore + dispensing slot, FIXED on collar ----
    cap_base = model.part("cap_base")
    cap_base.visual(
        mesh_from_cadquery(_cap_base_cadquery(), "cap_base_shell"),
        material=white,
        name="cap_base_shell",
    )
    cap_base.visual(_slot_marker_mesh(), material=marker_mat, name="slot_marker")
    cap_base.inertial = Inertial.from_geometry(
        Cylinder(CAP_BASE_R, CAP_H),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, CAP_BASE_BOTTOM + CAP_H / 2.0)),
    )
    model.articulation(
        "collar_to_cap_base",
        ArticulationType.FIXED,
        parent=collar,
        child=cap_base,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- disc: flat push-down tile with grip nubs ----
    # The disc part frame is placed at the bore floor / contact surface
    # (z = DISC_REST_BOTTOM = 0.184). Visuals are relative to this frame.
    disc = model.part("disc")
    disc_tile = CylinderGeometry(DISC_R, DISC_H, radial_segments=36)
    disc_tile.translate(0.0, 0.0, DISC_H / 2.0)  # bottom at part-frame origin
    disc.visual(
        mesh_from_geometry(disc_tile, "disc_tile"), material=disc_mat, name="disc_tile"
    )
    # Grip nubs via for-i-in-range loop with name_i naming
    for i in range(_NUB_COUNT):
        disc.visual(
            _grip_nub_mesh(i), material=disc_mat, name=f"grip_nub_{i}"
        )
    disc.inertial = Inertial.from_geometry(
        Cylinder(DISC_R, DISC_H),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, DISC_REST_BOTTOM + DISC_H / 2.0)),
    )
    # PRISMATIC push-down: joint origin at the bore floor (actual contact surface)
    model.articulation(
        "cap_to_disc",
        ArticulationType.PRISMATIC,
        parent=cap_base,
        child=disc,
        origin=Origin(xyz=(0.0, 0.0, DISC_REST_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=0.05, lower=-DISC_PRESS_TRAVEL, upper=0.0
        ),
    )

    return model


def run_tests() -> TestReport:
    from sdk import TestContext

    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    cap_base = object_model.get_part("cap_base")
    disc = object_model.get_part("disc")
    push = object_model.get_articulation("cap_to_disc")

    # --- bottle is clear (transparent shell) ---
    shell_vis = bottle.get_visual("bottle_shell")
    rgba = shell_vis.material.rgba
    ctx.check(
        "bottle is transparent",
        rgba is not None and rgba[3] < 1.0,
        details=f"bottle_shell rgba={rgba}",
    )

    # --- collar is seated on the bottle neck ---
    ctx.allow_overlap(
        collar, bottle,
        elem_a="collar_shell", elem_b="neck_threads",
        reason="The collar threads engage inside the collar shell over the neck threads.",
    )
    ctx.allow_overlap(
        collar, bottle,
        elem_a="collar_shell", elem_b="bottle_shell",
        reason="The collar skirt wraps over the bottle neck wall.",
    )
    ctx.expect_overlap(
        collar, bottle, axes="z", min_overlap=0.005,
        name="collar seated over the neck",
    )

    # --- cap_base is seated on the collar top ---
    ctx.allow_overlap(
        cap_base, collar,
        elem_a="cap_base_shell", elem_b="collar_shell",
        reason="The cap base sits on and interpenetrates the collar top rim seating surface.",
    )
    ctx.expect_gap(
        cap_base, collar, axis="z",
        min_gap=-0.003, max_gap=0.002,
        name="cap_base bottom seats on collar top",
    )

    # --- disc sits inside cap_base bore (intentional nested fit) ---
    ctx.allow_overlap(
        disc, cap_base,
        elem_a="disc_tile", elem_b="cap_base_shell",
        reason="The disc tile is a press-fit inside the cap base bore; the tile outer surface slightly overlaps the bore wall footprint.",
    )
    # Grip nubs may overlap the bore wall footprint near the edge
    for i in range(_NUB_COUNT):
        ctx.allow_overlap(
            disc, cap_base,
            elem_a=f"grip_nub_{i}", elem_b="cap_base_shell",
            reason=f"Grip nub {i} sits near the bore edge and may overlap the bore wall footprint.",
        )
    ctx.expect_within(
        disc, cap_base,
        axes="xy", margin=0.002,
        inner_elem="disc_tile", outer_elem="cap_base_shell",
        name="disc centered in cap_base bore",
    )

    # --- dispensing slot marker extends off-axis on +X ---
    marker_aabb = ctx.part_element_world_aabb(cap_base, elem="slot_marker")
    ctx.check(
        "dispensing slot marker extends off-axis on +X side",
        marker_aabb is not None and marker_aabb[1][0] > CAP_BASE_R,
        details=f"slot_marker max_x={marker_aabb[1][0] if marker_aabb else None}",
    )

    # --- disc tile has grip nubs on top ---
    for i in range(_NUB_COUNT):
        nub_aabb = ctx.part_element_world_aabb(disc, elem=f"grip_nub_{i}")
        disc_aabb = ctx.part_element_world_aabb(disc, elem="disc_tile")
        ctx.check(
            f"grip_nub_{i} sits above disc tile",
            nub_aabb is not None and disc_aabb is not None
            and nub_aabb[1][2] > disc_aabb[1][2] - 0.0001,
            details=f"nub top z={nub_aabb[1][2] if nub_aabb else None}, "
                    f"disc top z={disc_aabb[1][2] if disc_aabb else None}",
        )

    # --- PRISMATIC push-down: disc moves straight down when pressed ---
    rest_pos = ctx.part_world_position(disc)
    with ctx.pose({push: -DISC_PRESS_TRAVEL}):
        pressed_pos = ctx.part_world_position(disc)
    ctx.check(
        "disc depresses downward when pushed",
        rest_pos is not None and pressed_pos is not None
        and pressed_pos[2] < rest_pos[2] - 0.002,
        details=f"rest_z={rest_pos[2] if rest_pos else None}, "
                f"pressed_z={pressed_pos[2] if pressed_pos else None}",
    )

    # --- at rest, disc top is flush with cap_base top ---
    disc_tile_aabb = ctx.part_element_world_aabb(disc, elem="disc_tile")
    cap_shell_aabb = ctx.part_element_world_aabb(cap_base, elem="cap_base_shell")
    ctx.check(
        "disc top flush with cap_base top at rest",
        disc_tile_aabb is not None and cap_shell_aabb is not None
        and abs(disc_tile_aabb[1][2] - cap_shell_aabb[1][2]) < 0.001,
        details=f"disc_tile top z={disc_tile_aabb[1][2] if disc_tile_aabb else None}, "
                f"cap_base_shell top z={cap_shell_aabb[1][2] if cap_shell_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
