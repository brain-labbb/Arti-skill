"""Top-loading hard plastic pet travel kennel.

Tan upper shell lid with rows of oval vent holes and molded carry handle,
black lower tub, rear-hinged lift-up lid, and side/rear latch clips joining
the two shells along the rim flange.

Layout: +X = front, +Y = left, +Z = up. Ground at z = 0.
"""

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
RIM_L = 0.62  # carrier length at the rim parting line (X)
RIM_W = 0.42  # carrier width at the rim parting line (Y)
RIM_Z = 0.20  # height of the tub / parting line
TOP_L = 0.50  # roof footprint (upper shell tapers inward)
TOP_W = 0.30
SHELL_H = 0.24  # upper shell height (rim -> roof)
TUB_BOT_L = 0.56  # tub bottom footprint (slight outward taper going up)
TUB_BOT_W = 0.36
WALL = 0.006  # molded plastic wall thickness

# rim flange (both shells flare into a bolt flange at the parting line)
FLANGE_L = 0.66
FLANGE_W = 0.46
FLANGE_T = 0.023  # each flange band thickness (they overlap 2 mm at the rim)

# hinge at the rear rim edge (rear = -X)
HINGE_X = -RIM_L / 2.0  # rear rim parting line
HINGE_Y = 0.0
HINGE_Z = RIM_Z

# lid-local offsets: all visuals are expressed relative to the hinge origin.
# At q=0, the lid part frame coincides with the articulation frame at the rear
# rim, and the shell extends forward along +X.
LID_DX = RIM_L / 2.0  # shell center is RIM_L/2 ahead of the hinge

# ---------------------------------------------------------------- materials
TAN = Material(name="tan_plastic", rgba=(0.80, 0.76, 0.69, 1.0))
BLACK_PLASTIC = Material(name="black_plastic", rgba=(0.13, 0.13, 0.14, 1.0))
DARK_GREY = Material(name="dark_grey_plastic", rgba=(0.20, 0.20, 0.21, 1.0))


# ---------------------------------------------------------------- cq helpers
def _top_shell_solid() -> cq.Workplane:
    """Tapered hollow tan shell with oval vent holes (no front door opening).

    Built in shell-local coords: z=0 at the rim, z=SHELL_H at the roof.
    Centered at x=0, y=0.
    """
    shell = (
        cq.Workplane("XY")
        .rect(RIM_L, RIM_W)
        .workplane(offset=SHELL_H)
        .rect(TOP_L, TOP_W)
        .loft()
        .faces("<Z")
        .shell(-WALL)
    )

    # oval side vents: vertical stadium slots cut straight through both side
    # walls (three staggered rows, upper rows shorter as the shell narrows)
    rows = (
        (0.048, 7, 0.0),  # (row height above rim, slot count, x stagger)
        (0.106, 6, 0.029),
        (0.164, 5, 0.0),
    )
    pitch = 0.058
    pts = []
    for z_row, count, stagger in rows:
        x0 = -(count - 1) * pitch / 2.0 + stagger
        for i in range(count):
            pts.append((x0 + i * pitch, z_row))
    side_cutters = (
        cq.Workplane("XZ")
        .pushPoints(pts)
        .slot2D(0.046, 0.023, 90)
        .extrude(0.6, both=True)
    )
    shell = shell.cut(side_cutters)

    # rear vents: two rows of four, cut rearward only
    rear_pts = []
    for z_row in (0.060, 0.120):
        for i in range(4):
            rear_pts.append((-0.0825 + i * 0.055, z_row))
    rear_cutters = (
        cq.Workplane("YZ")
        .pushPoints(rear_pts)
        .slot2D(0.046, 0.023, 90)
        .extrude(-0.6)
    )
    return shell.cut(rear_cutters)


def _tub_solid() -> cq.Workplane:
    """Black lower tub: outward-tapered open-top hollow tub, floor kept."""
    return (
        cq.Workplane("XY")
        .rect(TUB_BOT_L, TUB_BOT_W)
        .workplane(offset=RIM_Z)
        .rect(RIM_L, RIM_W)
        .loft()
        .faces(">Z")
        .shell(-WALL)
    )


def _flange_ring() -> cq.Workplane:
    """Rim flange ring (rect frame), full perimeter — no door cut needed."""
    return (
        cq.Workplane("XY")
        .rect(FLANGE_L, FLANGE_W)
        .rect(RIM_L - 0.02, RIM_W - 0.02)
        .extrude(FLANGE_T)
    )


# ---------------------------------------------------------------- the model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plastic_pet_travel_kennel")

    # ============================================================= body (root)
    body = model.part("kennel_body")

    # black lower tub
    body.visual(
        mesh_from_cadquery(_tub_solid(), "lower_tub"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=BLACK_PLASTIC,
        name="lower_tub",
    )

    # black rim flange (tub lip, sits below the parting line)
    flange = _flange_ring()
    body.visual(
        mesh_from_cadquery(flange, "flange_black"),
        origin=Origin(xyz=(0.0, 0.0, RIM_Z - FLANGE_T + 0.001)),
        material=BLACK_PLASTIC,
        name="tub_rim_flange",
    )

    # dark latch clips on the sides clamping the two flange bands together
    for i, cx in enumerate((-0.15, 0.01, 0.17)):  # three per side
        for j, sy in enumerate((-1.0, 1.0)):
            body.visual(
                Box((0.036, 0.024, 0.065)),
                origin=Origin(xyz=(cx, sy * (FLANGE_W / 2.0 - 0.005), RIM_Z)),
                material=DARK_GREY,
                name=f"side_latch_clip_{i}_{j}",
            )
    body.visual(  # one clip centered on the rear flange
        Box((0.024, 0.036, 0.065)),
        origin=Origin(xyz=(-(FLANGE_L / 2.0 - 0.005), 0.0, RIM_Z)),
        material=DARK_GREY,
        name="rear_latch_clip",
    )

    # =========================================================== top shell lid
    # Part frame at the rear hinge line; at q=0 this coincides with the
    # articulation frame. All visuals are offset forward by LID_DX so the
    # shell rear edge meets the hinge.
    lid = model.part("top_shell_lid")

    # tan tapered upper shell with vents
    lid.visual(
        mesh_from_cadquery(_top_shell_solid(), "top_shell"),
        origin=Origin(xyz=(LID_DX, 0.0, 0.0)),
        material=TAN,
        name="kennel_upper_shell",
    )

    # tan rim flange (shell lip, sits above the parting line)
    lid.visual(
        mesh_from_cadquery(flange, "flange_tan"),
        origin=Origin(xyz=(LID_DX, 0.0, -0.001)),
        material=TAN,
        name="shell_rim_flange",
    )

    # front latch clips (lid catch — these snap onto the tub front flange)
    for j, sy in enumerate((-1.0, 1.0)):
        lid.visual(
            Box((0.024, 0.036, 0.065)),
            origin=Origin(
                xyz=(FLANGE_L / 2.0 - 0.005 + LID_DX, sy * 0.17, 0.0)
            ),
            material=DARK_GREY,
            name=f"front_latch_clip_{j}",
        )

    # molded black carry handle on the roof (base plate, two risers, grip bar)
    roof_z_local = SHELL_H  # roof in lid-local z (rim=0)
    lid.visual(
        Box((0.16, 0.08, 0.012)),
        origin=Origin(xyz=(LID_DX, 0.0, roof_z_local + 0.0045)),
        material=BLACK_PLASTIC,
        name="handle_base_plate",
    )
    for i, sx in enumerate((-1.0, 1.0)):
        lid.visual(
            Box((0.016, 0.05, 0.075)),
            origin=Origin(xyz=(sx * 0.062 + LID_DX, 0.0, roof_z_local + 0.047)),
            material=BLACK_PLASTIC,
            name=f"handle_riser_{i}",
        )
    lid.visual(
        Box((0.14, 0.048, 0.024)),
        origin=Origin(xyz=(LID_DX, 0.0, roof_z_local + 0.0745)),
        material=BLACK_PLASTIC,
        name="handle_grip_bar",
    )

    # ========================================================= rear lid hinge
    # Axis = (0, -1, 0): right-hand rule about -Y lifts the +X end (front of
    # the lid) upward toward +Z. Positive q opens the lid.
    model.articulation(
        "body_to_top_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=2.0, lower=0.0, upper=1.9
        ),
    )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("kennel_body")
    lid = object_model.get_part("top_shell_lid")
    hinge = object_model.get_articulation("body_to_top_lid")

    # --- intentional overlaps ---

    # The tan shell flange seats onto the black tub flange at the rim parting
    # line when the lid is closed.
    ctx.allow_overlap(
        body,
        lid,
        elem_a="tub_rim_flange",
        elem_b="shell_rim_flange",
        reason="mating rim flanges overlap ~2 mm at the parting line when closed",
    )

    # Side latch clips on the body bridge both flanges.
    for i in range(3):
        for j in range(2):
            ctx.allow_overlap(
                body,
                lid,
                elem_a=f"side_latch_clip_{i}_{j}",
                elem_b="shell_rim_flange",
                reason="latch clips clamp across the flange parting line",
            )
            ctx.allow_overlap(
                body,
                lid,
                elem_a=f"side_latch_clip_{i}_{j}",
                elem_b="kennel_upper_shell",
                reason="latch clips grip the shell rim edge",
            )

    # Rear latch clip bridges the flanges at the hinge side.
    ctx.allow_overlap(
        body,
        lid,
        elem_a="rear_latch_clip",
        elem_b="shell_rim_flange",
        reason="rear latch clamps across the flange parting line near the hinge",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="rear_latch_clip",
        elem_b="kennel_upper_shell",
        reason="rear latch grips the shell rim edge near the hinge",
    )

    # Front latch clips (on the lid) clamp down onto the tub flange.
    for j in range(2):
        ctx.allow_overlap(
            body,
            lid,
            elem_a="tub_rim_flange",
            elem_b=f"front_latch_clip_{j}",
            reason="front lid catch clips clamp onto the tub rim flange",
        )

    # --- closed pose (q=0): lid seats on the tub ---
    with ctx.pose({hinge: 0.0}):
        # lid shell overlaps body footprint in XY
        ctx.expect_overlap(
            lid, body, axes="xy", min_overlap=0.20,
            elem_a="kennel_upper_shell", elem_b="lower_tub",
            name="closed_lid_overlaps_tub_in_xy",
        )
        # lid flange contacts tub flange in Z (seated)
        ctx.expect_gap(
            lid, body, axis="z",
            max_penetration=0.005,
            max_gap=0.005,
            positive_elem="shell_rim_flange",
            negative_elem="tub_rim_flange",
            name="closed_lid_flange_seated_on_tub_flange",
        )
        # handle is above the rim
        handle_aabb = ctx.part_element_world_aabb(lid, elem="handle_grip_bar")
        ctx.check(
            "handle_above_rim",
            handle_aabb is not None and handle_aabb[0][2] > RIM_Z,
            f"handle_grip_bar aabb={handle_aabb}",
        )

    # --- open pose (q=1.5): lid lifts upward, front edge rises ---
    with ctx.pose({hinge: 1.5}):
        lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "open_lid_front_edge_rises",
            lid_aabb is not None and lid_aabb[1][2] > RIM_Z + SHELL_H + 0.10,
            f"lid aabb max_z={None if lid_aabb is None else lid_aabb[1][2]}",
        )
        # The lid's minimum z should be at the hinge (rear edge stays put),
        # but the handle (roof element) must clear the tub rim vertically.
        handle_open_aabb = ctx.part_element_world_aabb(lid, elem="handle_grip_bar")
        ctx.check(
            "open_lid_handle_clears_tub_rim",
            handle_open_aabb is not None and handle_open_aabb[0][2] > RIM_Z + 0.15,
            f"handle_grip_bar aabb={handle_open_aabb}",
        )

    # --- prompt-specific: top_shell_lid exists with vent holes and handle ---
    shell_vis = lid.get_visual("kennel_upper_shell")
    ctx.check(
        "top_shell_lid_has_upper_shell_visual",
        shell_vis is not None,
        "top_shell_lid must carry the kennel_upper_shell visual",
    )
    grip_vis = lid.get_visual("handle_grip_bar")
    ctx.check(
        "handle_mounted_on_lid",
        grip_vis is not None,
        "carry handle grip bar must be on the top_shell_lid",
    )

    return ctx.report()


object_model = build_object_model()
