from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.46          # width (X)
D = 0.30          # depth (Y)
HB = 0.18         # body height (Z)
T = 0.018         # plank wall thickness
T_LID = 0.022     # lid plank thickness

# Lid plank layout (boards running front-to-back)
N_PLANKS = 4
PLANK_GAP = 0.003
PLANK_W = (W - (N_PLANKS - 1) * PLANK_GAP) / N_PLANKS

# Iron strap layout on lid (flat bands across width)
N_STRAPS = 3
STRAP_W = 0.028
STRAP_T = 0.005
STRAP_Y_FRACS = (0.13, 0.50, 0.87)  # fraction of depth from rear edge


def _plank_x(i: int) -> float:
    """Centre-X of the i-th lid plank in the lid-local frame."""
    return -W / 2.0 + PLANK_W / 2.0 + i * (PLANK_W + PLANK_GAP)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medieval_treasure_chest")

    wood = model.material("chest_wood", rgba=(0.42, 0.30, 0.20, 1.0))
    wood_light = model.material("chest_wood_light", rgba=(0.48, 0.35, 0.23, 1.0))
    wood_dark = model.material("chest_wood_dark", rgba=(0.30, 0.21, 0.14, 1.0))
    iron = model.material("iron_band", rgba=(0.20, 0.21, 0.24, 1.0))

    # ── Body (root) ─────────────────────────────────────────────────────────
    body = model.part("chest_body")

    # Floor
    body.visual(Box((W, D, T)),
                origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
                material=wood_dark, name="floor_panel")

    # Four walls
    cz = HB / 2.0
    body.visual(Box((W, T, HB)),
                origin=Origin(xyz=(0.0, -(D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_front")
    body.visual(Box((W, T, HB)),
                origin=Origin(xyz=(0.0, (D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_back")
    body.visual(Box((T, D, HB)),
                origin=Origin(xyz=(-(W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_left")
    body.visual(Box((T, D, HB)),
                origin=Origin(xyz=((W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_right")

    # Iron corner posts on the four vertical edges
    bk = 0.034
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        body.visual(
            Box((bk, bk, HB)),
            origin=Origin(xyz=(sx * (W / 2.0 - bk / 2.0 + 0.003),
                               sy * (D / 2.0 - bk / 2.0 + 0.003),
                               HB / 2.0)),
            material=iron, name=f"corner_post_{i}")

    # Horizontal iron bands on body sides
    for i, sx in enumerate((-0.12, 0.12)):
        body.visual(Box((0.03, D + 0.004, 0.006)),
                    origin=Origin(xyz=(sx, 0.0, HB * 0.5)),
                    material=iron, name=f"body_band_{i}")

    # Front lock keeper (protruding staple that receives the hasp eye)
    body.visual(Box((0.05, 0.012, 0.035)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.006, HB * 0.55)),
                material=iron, name="lock_keeper")

    # ── Flat plank lid (hinged at rear top edge of body) ────────────────────
    # Lid-local frame: origin at hinge (rear-bottom corner of lid).
    #   +Y is toward the rear (hinge side), -Y is toward the front.
    #   +Z is upward from the plank surface.
    lid = model.part("chest_lid")

    # Individual wooden planks (boards running front-to-back)
    for i in range(N_PLANKS):
        mat = wood if i % 2 == 0 else wood_light
        lid.visual(
            Box((PLANK_W, D, T_LID)),
            origin=Origin(xyz=(_plank_x(i), -D / 2.0, T_LID / 2.0)),
            material=mat, name=f"lid_plank_{i}")

    # Flat iron straps running across the width, sitting on top of planks
    for i in range(N_STRAPS):
        y_pos = -(STRAP_Y_FRACS[i] * D)
        lid.visual(
            Box((W + 0.004, STRAP_W, STRAP_T)),
            origin=Origin(xyz=(0.0, y_pos, T_LID + STRAP_T / 2.0)),
            material=iron, name=f"lid_strap_{i}")

    # Rivet caps at every plank-strap intersection
    rivet_idx = 0
    for si in range(N_STRAPS):
        y_pos = -(STRAP_Y_FRACS[si] * D)
        for pi in range(N_PLANKS):
            lid.visual(
                Box((0.010, 0.010, 0.004)),
                origin=Origin(xyz=(_plank_x(pi), y_pos,
                                   T_LID + STRAP_T + 0.002)),
                material=iron, name=f"rivet_{rivet_idx}")
            rivet_idx += 1

    # Hasp mount plate on the front face of the lid
    lid.visual(Box((0.05, 0.012, 0.04)),
               origin=Origin(xyz=(0.0, -D - 0.006, T_LID * 0.4)),
               material=iron, name="hasp_mount")

    # Lid hinge at the rear top edge of the body
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, D / 2.0, HB)),
        # Lid extends along local -Y; axis (-1,0,0) lifts the front edge up.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0,
                                   lower=0.0, upper=1.9),
    )

    # ── Front lock hasp (hinged on the lid front, swings outward to release) ─
    hasp = model.part("lock_hasp")
    hasp.visual(Box((0.05, 0.010, 0.09)),
                origin=Origin(xyz=(0.0, 0.0, -0.045)),
                material=iron, name="hasp_arm")
    hasp.visual(Box((0.026, 0.016, 0.02)),
                origin=Origin(xyz=(0.0, -0.005, -0.08)),
                material=iron, name="hasp_eye")
    model.articulation(
        "hasp_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=hasp,
        origin=Origin(xyz=(0.0, -D - 0.005, T_LID * 0.2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0,
                                   lower=0.0, upper=1.4),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("chest_body")
    lid = object_model.get_part("chest_lid")
    hasp = object_model.get_part("lock_hasp")
    lid_hinge = object_model.get_articulation("lid_hinge")
    hasp_hinge = object_model.get_articulation("hasp_hinge")

    # ── Intentional overlaps ────────────────────────────────────────────────
    # The hasp arm top embeds slightly into the hasp mount plate at the hinge
    # point — this is the real mounting interface where the hasp hangs from
    # the plate.
    ctx.allow_overlap(
        lid, hasp,
        elem_a="hasp_mount", elem_b="hasp_arm",
        reason="Hasp arm top embeds into the hasp mount plate at the hinge point, "
               "representing the real plate-mount interface.",
    )

    # The hasp arm and eye partially embed into the lock keeper when the hasp
    # is in its closed (hanging-down) position.  This represents the real hasp
    # engagement where the arm passes over / wraps around the keeper staple.
    ctx.allow_overlap(
        hasp, body,
        elem_a="hasp_arm", elem_b="lock_keeper",
        reason="Hasp arm hangs in front of the keeper as part of the hasp-lock engagement.",
    )
    ctx.allow_overlap(
        hasp, body,
        elem_a="hasp_eye", elem_b="lock_keeper",
        reason="Hasp eye wraps around the keeper staple when the hasp is closed.",
    )

    # Proof: hasp arm top contacts the hasp mount plate (mounting interface).
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        ctx.expect_contact(
            hasp, lid,
            elem_a="hasp_arm", elem_b="hasp_mount",
            name="hasp arm contacts the mount plate at the hinge",
        )

    # ── Closed-pose checks ──────────────────────────────────────────────────
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        # Flat lid seats on the body rim (plank bottom ≈ wall top)
        ctx.expect_gap(lid, body, axis="z",
                       min_gap=-0.001, max_gap=0.006,
                       positive_elem="lid_plank_0",
                       negative_elem="wall_front",
                       name="flat lid plank seats on body rim")

        # Closed lid covers the body opening
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.20,
                           name="closed lid covers the body opening")

        closed_lid = ctx.part_world_aabb(lid)

    # ── Flat-lid profile verification ───────────────────────────────────────
    # The lid top must be close to HB + T_LID (+ straps/rivets), NOT a tall
    # dome.  A domed lid would peak at roughly HB + R ≈ 0.33 m.
    max_flat_z = HB + T_LID + STRAP_T + 0.010  # planks + strap + rivet + slack
    ctx.check(
        "lid is a flat plank profile (not domed)",
        closed_lid is not None and closed_lid[1][2] < max_flat_z,
        details=(f"lid_top_z={closed_lid[1][2] if closed_lid else None}, "
                 f"expected < {max_flat_z:.3f}"),
    )

    # ── Lid opens upward ────────────────────────────────────────────────────
    with ctx.pose({lid_hinge: 1.7}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on rear hinge",
        closed_lid is not None and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.08,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    # ── Front lock hasp lifts to release ────────────────────────────────────
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        hasp_closed = ctx.part_world_aabb(hasp)
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 1.3}):
        hasp_open = ctx.part_world_aabb(hasp)
    ctx.check(
        "front lock hasp lifts to release",
        hasp_closed is not None and hasp_open is not None
        and hasp_open[0][2] > hasp_closed[0][2] + 0.02,
        details=f"closed={hasp_closed}, open={hasp_open}",
    )

    return ctx.report()
