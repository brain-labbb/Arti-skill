from __future__ import annotations

"""Cylindrical knitting needle tube case.

A rigid cylindrical tube that holds a bundle of six double-pointed knitting
needles upright, with a removable pull-off cap at the mouth. The tube body
replaces the planar leather panel of the bifold case variant with a volumetric
cylindrical envelope. Brass rings at the mouth and base provide decorative
detail, and a brass pull stud sits on the cap top.

Layout (rest pose, world frame): tube stands upright with z up. The tube body
is the root part; the cap sits on the tube mouth at z = TUBE_H. The six
double-pointed needles stand upright inside the bore, arranged in a hexagonal
ring bundle.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    ConeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- tube body
TUBE_OR = 0.0175   # outer radius  (35 mm OD)
TUBE_IR = 0.0145   # inner radius  (29 mm bore, 3 mm wall)
TUBE_H = 0.160     # overall height
BOTTOM_T = 0.004   # bottom wall thickness

# ---------------------------------------------------------------- cap
CAP_T = 0.006      # cap disk thickness
PLUG_R = TUBE_IR - 0.001   # plug radius (1 mm radial clearance in bore)
PLUG_H = 0.008     # plug depth into mouth

# ---------------------------------------------------------------- needles
NEEDLE_R = 0.0025          # needle shaft radius
NEEDLE_SHAFT_L = 0.115     # shaft length
NEEDLE_TIP_L = 0.018       # each tip length
NEEDLE_COUNT = 6
NEEDLE_RING_R = 0.006      # ring radius for bundle inside bore

# Needle color palette (kept from parent)
NEEDLE_RGBAS = (
    (0.16, 0.55, 0.55, 1.0),   # teal
    (0.48, 0.30, 0.62, 1.0),   # purple
    (0.75, 0.22, 0.24, 1.0),   # red
    (0.23, 0.38, 0.72, 1.0),   # blue
    (0.33, 0.58, 0.30, 1.0),   # green
    (0.85, 0.52, 0.18, 1.0),   # orange
)

# Derived: total needle extent along Z (bottom tip apex to top tip apex)
_NEEDLE_TOTAL = NEEDLE_SHAFT_L + 2 * (NEEDLE_TIP_L - 0.002)


# ================================================================ geometry


def _tube_geometry():
    """Hollow cylindrical tube with closed bottom and open top mouth.

    Built as a lathe revolve of the tube cross-section in the r-z half-plane.
    """
    profile = [
        (0.0, 0.0),               # bottom center (on axis)
        (TUBE_OR, 0.0),           # bottom outer edge
        (TUBE_OR, TUBE_H),        # top outer edge
        (TUBE_IR, TUBE_H),        # top inner edge (mouth rim)
        (TUBE_IR, BOTTOM_T),      # inner cavity floor edge
        (0.0, BOTTOM_T),          # inner cavity floor center (on axis)
    ]
    return LatheGeometry(profile, segments=32, closed=True)


def _dpn_geometry():
    """Double-pointed needle along +Z, bottom tip apex at z = 0.

    Shaft with a cone tip on each end; 2 mm overlap at each joint so the
    merged mesh is one connected solid.
    """
    # Bottom tip: apex at z = 0, base at z = NEEDLE_TIP_L
    bot_tip = ConeGeometry(NEEDLE_R, NEEDLE_TIP_L, radial_segments=16)
    bot_tip.rotate_x(math.pi)           # apex points down
    bot_tip.translate(0.0, 0.0, NEEDLE_TIP_L / 2)

    # Shaft: overlaps 2 mm into each tip
    shaft_z0 = NEEDLE_TIP_L - 0.002
    shaft = CylinderGeometry(NEEDLE_R, NEEDLE_SHAFT_L, radial_segments=16)
    shaft.translate(0.0, 0.0, shaft_z0 + NEEDLE_SHAFT_L / 2)

    # Top tip: base at shaft end, apex up
    top_z0 = shaft_z0 + NEEDLE_SHAFT_L
    top_tip = ConeGeometry(NEEDLE_R, NEEDLE_TIP_L, radial_segments=16)
    top_tip.translate(0.0, 0.0, top_z0 + NEEDLE_TIP_L / 2 - 0.002)

    return bot_tip.merge(shaft).merge(top_tip)


def _cap_geometry():
    """Cap disk with inner plug that seats into the tube mouth bore.

    Disk extends z in [0, CAP_T] (above mouth plane); plug extends z in
    [-PLUG_H, 0] (below mouth plane, into bore).
    """
    disk = CylinderGeometry(TUBE_OR + 0.001, CAP_T, radial_segments=32)
    disk.translate(0.0, 0.0, CAP_T / 2)

    plug = CylinderGeometry(PLUG_R, PLUG_H, radial_segments=24)
    plug.translate(0.0, 0.0, -PLUG_H / 2)

    return disk.merge(plug)


# ================================================================ model


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knitting_needle_tube_case")

    # ---- materials ----
    model.material("tube_brown", rgba=(0.42, 0.24, 0.12, 1.0))
    model.material("cap_brown", rgba=(0.45, 0.26, 0.14, 1.0))
    model.material("brass", rgba=(0.76, 0.62, 0.30, 1.0))
    for i, rgba in enumerate(NEEDLE_RGBAS):
        model.material(f"needle_{i}", rgba=rgba)

    # ---- meshes ----
    tube_mesh = mesh_from_geometry(_tube_geometry(), "tube_shell")
    needle_mesh = mesh_from_geometry(_dpn_geometry(), "dpn_needle")
    cap_mesh = mesh_from_geometry(_cap_geometry(), "cap_body")

    # ============================================================ tube_body
    tube = model.part("tube_body")

    # Main hollow tube shell
    tube.visual(
        tube_mesh,
        origin=Origin(),
        material="tube_brown",
        name="tube_shell",
    )

    # Brass mouth ring (thin band wrapping just above tube outer wall)
    tube.visual(
        Cylinder(radius=TUBE_OR + 0.0004, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, TUBE_H - 0.002)),
        material="brass",
        name="mouth_ring",
    )

    # Brass base ring (thin band wrapping the lower tube wall)
    tube.visual(
        Cylinder(radius=TUBE_OR + 0.0004, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material="brass",
        name="base_ring",
    )

    # ---- needle bundle inside the bore ----
    for i in range(NEEDLE_COUNT):
        angle = i * math.pi * 2.0 / NEEDLE_COUNT
        nx = NEEDLE_RING_R * math.cos(angle)
        ny = NEEDLE_RING_R * math.sin(angle)
        tube.visual(
            needle_mesh,
            origin=Origin(xyz=(nx, ny, BOTTOM_T)),
            material=f"needle_{i}",
            name=f"needle_{i}",
        )

    # ============================================================ cap
    cap = model.part("cap")

    cap.visual(
        cap_mesh,
        origin=Origin(),
        material="cap_brown",
        name="cap_disk",
    )

    # Brass pull stud on cap top
    cap.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, CAP_T + 0.0015)),
        material="brass",
        name="cap_stud",
    )

    # ============================================================ joint
    model.articulation(
        "tube_to_cap",
        ArticulationType.PRISMATIC,
        parent=tube,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, TUBE_H)),
        axis=(0.0, 0.0, 1.0),   # positive q pulls cap upward off mouth
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.5,
            lower=0.0,
            upper=0.08,
        ),
    )

    return model


# ================================================================ tests


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tube = object_model.get_part("tube_body")
    cap = object_model.get_part("cap")
    cap_joint = object_model.get_articulation("tube_to_cap")

    # ---- structural: tube_body carries a cylindrical tube_shell -----------
    tube_shell = tube.get_visual("tube_shell")
    ctx.check(
        "tube_body carries a tube_shell visual (cylindrical envelope, not planar panel)",
        tube_shell is not None,
        details="tube_shell visual not found on tube_body",
    )

    # ---- tube_to_cap joint is prismatic along +Z --------------------------
    ctx.check(
        "tube_to_cap is a prismatic joint for pull-off cap motion",
        cap_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type = {cap_joint.articulation_type}",
    )

    # ---- cap seated on tube mouth at rest ----------------------------------
    ctx.expect_contact(
        tube, cap,
        elem_a="mouth_ring",
        elem_b="cap_disk",
        name="cap disk seats against the tube mouth ring at rest",
    )

    # ---- cap plug inside tube bore (intentional seated insertion) ----------
    ctx.allow_overlap(
        tube, cap,
        elem_a="tube_shell",
        elem_b="cap_disk",
        reason="The cap plug is intentionally seated inside the tube mouth bore for retention; the plug cylinder fits within the hollow cavity with 1 mm radial clearance.",
    )
    ctx.expect_within(
        cap, tube,
        axes="xy",
        inner_elem="cap_disk",
        outer_elem="tube_shell",
        margin=0.003,
        name="cap plug stays centered within the tube bore on XY",
    )

    # ---- needle bundle inside tube bore ------------------------------------
    for i in range(NEEDLE_COUNT):
        ctx.expect_within(
            tube, tube,
            axes="xy",
            inner_elem=f"needle_{i}",
            outer_elem="tube_shell",
            margin=0.002,
            name=f"needle_{i} stays within the tube bore on XY",
        )

    # ---- needles below tube mouth ------------------------------------------
    needle_top_z = BOTTOM_T + _NEEDLE_TOTAL
    mouth_z = TUBE_H
    ctx.check(
        "needle tips remain below the tube mouth",
        needle_top_z < mouth_z - 0.002,
        details=f"needle_top_z={needle_top_z:.4f}, mouth_z={mouth_z:.4f}",
    )

    # ---- decisive pose: cap pulls upward off the tube mouth ----------------
    rest_cap_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_joint: 0.06}):
        pulled_pos = ctx.part_world_position(cap)
        ctx.check(
            "cap pulls upward off the tube mouth when joint is extended",
            rest_cap_pos is not None
            and pulled_pos is not None
            and pulled_pos[2] > rest_cap_pos[2] + 0.04,
            details=f"rest={rest_cap_pos}, pulled={pulled_pos}",
        )

    # ---- cap clear of tube at extended pose --------------------------------
    with ctx.pose({cap_joint: 0.06}):
        ctx.expect_gap(
            cap, tube,
            axis="z",
            positive_elem="cap_disk",
            negative_elem="tube_shell",
            min_gap=0.02,
            name="cap is fully clear of the tube mouth at extended pose",
        )

    return ctx.report()


object_model = build_object_model()
