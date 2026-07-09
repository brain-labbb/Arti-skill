from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.50          # width (X)
D = 0.34          # depth (Y)
HB = 0.24         # body height (Z)
T = 0.018         # plank wall thickness
LID_T = 0.040     # flat lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rustic_wooden_box")

    wood = model.material("wood_plank", rgba=(0.62, 0.45, 0.26, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.45, 0.32, 0.18, 1.0))
    iron = model.material("iron_fitting", rgba=(0.22, 0.22, 0.24, 1.0))

    # --- Body (root) ---------------------------------------------------------
    body = model.part("box_body")
    body.visual(Box((W, D, T)), origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
                material=wood_dark, name="floor_panel")
    cz = HB / 2.0
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, -(D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_front")
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, (D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_back")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=(-(W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_left")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=((W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_right")

    # iron corner brackets at the four vertical edges
    bk = 0.030
    idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Box((bk, bk, HB - 0.02)),
                origin=Origin(xyz=(sx * (W / 2.0 - bk / 2.0 + 0.003),
                                   sy * (D / 2.0 - bk / 2.0 + 0.003), HB / 2.0)),
                material=iron, name=f"corner_bracket_{idx}",
            )
            idx += 1

    # vertical iron battens on the long faces
    for sx in (-0.13, 0.13):
        body.visual(Box((0.028, D + 0.004, 0.008)),
                    origin=Origin(xyz=(sx, 0.0, HB * 0.5)),
                    material=iron, name=f"batten_{'l' if sx < 0 else 'r'}")

    # Structural side brackets for a single swing bail handle.  The plates are
    # bolted through the side walls and carry visible round pivot bosses.
    pivot_z = HB + LID_T + 0.028
    lug_z = HB - 0.020
    for sx, tag in ((-1.0, "left"), (1.0, "right")):
        xface = sx * (W / 2.0)
        body.visual(Box((0.014, 0.095, 2.0 * (pivot_z - lug_z))),
                    origin=Origin(xyz=(xface + sx * 0.022, 0.0, (pivot_z + lug_z) / 2.0)),
                    material=iron, name=f"side_bracket_{tag}")
        for yoff, lug in ((-0.032, "front"), (0.032, "rear")):
            body.visual(Box((0.032, 0.018, 0.018)),
                        origin=Origin(xyz=(xface + sx * 0.015, yoff, lug_z)),
                        material=iron, name=f"bracket_lug_{tag}_{lug}")
        body.visual(
            Cylinder(radius=0.018, length=0.030),
            origin=Origin(xyz=(xface + sx * 0.028, 0.0, pivot_z),
                          rpy=(0.0, 1.5707963, 0.0)),
            material=iron, name=f"pivot_boss_{tag}",
        )

    # --- Lid (flat, hinged at the back top edge) -----------------------------
    lid = model.part("box_lid")
    lid.visual(Box((W + 0.02, D + 0.02, LID_T)),
               origin=Origin(xyz=(0.0, -D / 2.0, SEAM_GAP + LID_T / 2.0)),
               material=wood, name="lid_panel")
    # iron straps across the lid
    for sx in (-0.13, 0.13):
        lid.visual(Box((0.028, D + 0.02, 0.006)),
                   origin=Origin(xyz=(sx, -D / 2.0, SEAM_GAP + LID_T + 0.003)),
                   material=iron, name=f"lid_strap_{'l' if sx < 0 else 'r'}")
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, D / 2.0, HB)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=2.1),
    )

    # --- Swing bail handle ---------------------------------------------------
    # The child frame sits on the handle pivot axis.  At q=0 the bail stands
    # upright over the lid; positive rotation swings it down toward the front.
    bail = model.part("bail_handle")
    bail.visual(
        Cylinder(radius=0.010, length=W + 0.106),
        origin=Origin(xyz=(0.0, 0.0, 0.135), rpy=(0.0, 1.5707963, 0.0)),
        material=iron, name="top_grip",
    )
    for sx, tag in ((-1.0, "left"), (1.0, "right")):
        bail.visual(
            Cylinder(radius=0.007, length=0.135),
            origin=Origin(xyz=(sx * (W / 2.0 + 0.053), 0.0, 0.0675)),
            material=iron, name=f"side_arm_{tag}",
        )
        bail.visual(
            Cylinder(radius=0.008, length=0.032),
            origin=Origin(xyz=(sx * (W / 2.0 + 0.036), 0.0, 0.0),
                          rpy=(0.0, 1.5707963, 0.0)),
            material=iron, name=f"pivot_pin_{tag}",
        )

    model.articulation(
        "bail_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.4),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("box_body")
    lid = object_model.get_part("box_lid")
    bail = object_model.get_part("bail_handle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    bail_pivot = object_model.get_articulation("bail_pivot")

    # The bail's small pivot pins are intentionally captured inside the round
    # bracket bosses, which creates a local hidden overlap at the hinge axis.
    for tag in ("left", "right"):
        ctx.allow_overlap(
            body,
            bail,
            elem_a=f"pivot_boss_{tag}",
            elem_b=f"pivot_pin_{tag}",
            reason="The bail handle pivot pin is intentionally seated through the side bracket boss.",
        )
        ctx.allow_overlap(
            body,
            bail,
            elem_a=f"side_bracket_{tag}",
            elem_b=f"pivot_pin_{tag}",
            reason="The pivot pin passes through the side bracket plate at the handle hinge.",
        )
        ctx.expect_within(
            bail,
            body,
            axes="yz",
            inner_elem=f"pivot_pin_{tag}",
            outer_elem=f"pivot_boss_{tag}",
            margin=0.002,
            name=f"{tag} bail pin is centered in its side boss",
        )
        ctx.expect_overlap(
            bail,
            body,
            axes="x",
            elem_a=f"pivot_pin_{tag}",
            elem_b=f"pivot_boss_{tag}",
            min_overlap=0.020,
            name=f"{tag} bail pin remains inserted through the boss",
        )
        ctx.expect_overlap(
            bail,
            body,
            axes="x",
            elem_a=f"pivot_pin_{tag}",
            elem_b=f"side_bracket_{tag}",
            min_overlap=0.006,
            name=f"{tag} bail pin passes through bracket plate",
        )

    # Closed lid seats on the body rim without penetrating it.
    with ctx.pose({lid_hinge: 0.0, bail_pivot: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.006,
                       positive_elem="lid_panel", negative_elem="wall_front",
                       name="lid seats on the body rim when closed")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.25,
                           name="closed lid covers the body opening")
        ctx.expect_overlap(bail, lid, axes="x", min_overlap=0.45,
                           elem_a="top_grip", elem_b="lid_panel",
                           name="single bail handle spans across the lid")
        closed_lid = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: 1.9}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        closed_lid is not None and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.10,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    # The bail handle swings about the side bracket axis.
    with ctx.pose({bail_pivot: 0.0}):
        bail_upright = ctx.part_element_world_aabb(bail, elem="top_grip")
    with ctx.pose({bail_pivot: 1.2}):
        bail_swung = ctx.part_element_world_aabb(bail, elem="top_grip")
    ctx.check(
        "bail handle swings forward",
        bail_upright is not None and bail_swung is not None
        and bail_swung[0][1] < bail_upright[0][1] - 0.07,
        details=f"upright={bail_upright}, swung={bail_swung}",
    )

    # Side brackets replace the old rope handles and make the box a simple
    # hinged storage chest with one rotating bail.
    ctx.check(
        "side bracket bail hardware is present",
        body.get_visual("side_bracket_left") is not None
        and body.get_visual("side_bracket_right") is not None
        and bail.get_visual("top_grip") is not None,
        details="expected side brackets and a single top_grip bail handle",
    )

    return ctx.report()
