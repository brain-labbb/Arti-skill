from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

SILVER_RGBA = (0.80, 0.81, 0.79, 1.0)
HARDWARE_RGBA = (0.32, 0.33, 0.34, 1.0)
RUBBER_RGBA = (0.03, 0.03, 0.03, 1.0)

# ── Wedge geometry ────────────────────────────────────────────────────
WEDGE_WIDTH = 0.240       # Y extent (left-right)
BASE_FRONT_X = -0.100     # front edge of base
BASE_REAR_X = 0.100       # rear edge of base
WEDGE_FRONT_X = -0.060    # front edge of inclined top panel
WEDGE_REAR_X = 0.085      # rear edge of top panel
WEDGE_FRONT_H = 0.058     # height at front of top panel
WEDGE_REAR_H = 0.148      # height at rear of top panel
WALL_THK = 0.004          # shell wall thickness
BASE_THK = 0.005          # base plate thickness

INCLINE_ANGLE = math.atan2(
    WEDGE_REAR_H - WEDGE_FRONT_H, WEDGE_REAR_X - WEDGE_FRONT_X
)

HOOK_Y = 0.060            # lateral offset of each hook from center


def _lightening_slot_bar(
    *,
    length: float,
    width: float,
    thickness: float,
    cuts: tuple[tuple[float, float, float, bool], ...],
    name: str,
) -> object:
    """Flat rounded-end bar stock with a row of through cuts.

    `cuts` entries are `(center_x, cut_len, cut_wid, rounded)`.
    """
    body = cq.Workplane("XY").slot2D(length - width, width).extrude(thickness)
    body = body.translate((0.0, 0.0, -thickness / 2.0))
    for center_x, cut_len, cut_wid, rounded in cuts:
        wp = body.faces(">Z").workplane(centerOption="CenterOfMass").center(center_x, 0.0)
        if rounded:
            body = wp.slot2D(max(cut_len - cut_wid, 0.001), cut_wid).cutThruAll()
        else:
            body = wp.rect(cut_len, cut_wid).cutThruAll()
    try:
        body = body.edges("|Z").chamfer(0.0010).edges(">Z").chamfer(0.0006)
    except Exception:
        pass
    return mesh_from_cadquery(body, name, tolerance=0.0006, angular_tolerance=0.08)


def _rubber_pad_mesh(length: float, width: float, thickness: float, name: str) -> object:
    pad = cq.Workplane("XY").slot2D(max(length - width, 0.001), width).extrude(thickness)
    pad = pad.translate((0.0, 0.0, -thickness / 2.0))
    return mesh_from_cadquery(pad, name, tolerance=0.0005, angular_tolerance=0.08)


def _hook_bracket_mesh(width: float, name: str) -> object:
    """Upturned front hook: a clean L-shaped lip at the rail's front tip.

    Side profile in the local XZ plane. Local +X runs back over the panel body
    (so the floor is carried by the panel), local -X cantilevers past the front
    tip where the upturned lip rises. The laptop's front edge rests in the
    inner corner of the L.
    """
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.030, 0.0)
        .lineTo(-0.036, 0.0)      # floor underside, panel side -> front tip
        .lineTo(-0.036, 0.030)    # front lip outer face rises
        .lineTo(-0.030, 0.030)    # lip top
        .lineTo(-0.030, 0.006)    # lip inner face back down to floor top
        .lineTo(0.030, 0.006)     # floor top, front -> panel side
        .close()
        .extrude(width)
    )
    profile = profile.translate((0.0, width / 2.0, 0.0))
    try:
        profile = profile.edges("|Y").fillet(0.0022)
    except Exception:
        pass
    return mesh_from_cadquery(profile, name, tolerance=0.0005, angular_tolerance=0.08)


def _wedge_shell_mesh() -> object:
    """Hollow wedge shell: inclined top panel, front/rear walls, flat base, two side walls.

    Built as a single extruded XZ-profile (along Y), then hollowed with a
    boolean cavity cut, then ventilated with through-slots in the top panel.
    """
    # Outer profile in XZ plane
    outer = (
        cq.Workplane("XZ")
        .moveTo(BASE_FRONT_X, 0.0)
        .lineTo(BASE_REAR_X, 0.0)
        .lineTo(WEDGE_REAR_X, WEDGE_REAR_H)
        .lineTo(WEDGE_FRONT_X, WEDGE_FRONT_H)
        .close()
    )
    # CadQuery "XZ" workplane normal is -Y, so extrude goes in -Y.
    # Translate +W/2 to center the result at Y=0.
    outer_solid = outer.extrude(WEDGE_WIDTH).translate((0.0, WEDGE_WIDTH / 2.0, 0.0))

    # Inner cavity (approximate inward offset)
    inner = (
        cq.Workplane("XZ")
        .moveTo(BASE_FRONT_X + 0.008, BASE_THK)
        .lineTo(BASE_REAR_X - 0.008, BASE_THK)
        .lineTo(WEDGE_REAR_X - 0.007, WEDGE_REAR_H - 0.006)
        .lineTo(WEDGE_FRONT_X + 0.007, WEDGE_FRONT_H - 0.003)
        .close()
    )
    inner_w = WEDGE_WIDTH - 2.0 * WALL_THK
    cavity = inner.extrude(inner_w).translate((0.0, inner_w / 2.0, 0.0))

    shell = outer_solid.cut(cavity)

    # Ventilation slots through the top panel: thin boxes spanning across
    # the width, positioned along the incline at regular intervals.
    slot_x_positions = (-0.028, 0.010, 0.048)
    slot_span_y = WEDGE_WIDTH * 0.80  # across most of the panel width
    for x_c in slot_x_positions:
        # Height on the outer incline surface at this x
        t = (x_c - WEDGE_FRONT_X) / (WEDGE_REAR_X - WEDGE_FRONT_X)
        z_c = WEDGE_FRONT_H + t * (WEDGE_REAR_H - WEDGE_FRONT_H)
        # Thin box: narrow in X (0.010), wide in Y (slot_span_y), tall enough
        # in Z (0.024) to cut through the panel thickness.
        # Rotate first (around origin), then translate to incline position.
        cutter = cq.Workplane("XY").box(0.010, slot_span_y, 0.024)
        cutter = cutter.rotate((0, 0, 0), (0, 1, 0), -math.degrees(INCLINE_ANGLE))
        cutter = cutter.translate((x_c, 0.0, z_c))
        shell = shell.cut(cutter)

    # Round exterior vertical edges
    try:
        shell = shell.edges("|Y").fillet(0.002)
    except Exception:
        pass
    # Round top/bottom horizontal edges
    try:
        shell = shell.edges(">Z or <Z").fillet(0.001)
    except Exception:
        pass

    return mesh_from_cadquery(shell, "wedge_shell", tolerance=0.0006, angular_tolerance=0.08)


def _add_front_hook(part, silver, rubber) -> None:
    # Raise bracket 0.001m above the pivot plane so the floor sits just proud
    # of the panel surface (avoids mesh penetration at the seated contact).
    part.visual(
        _hook_bracket_mesh(0.030, "hook_bracket"),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=silver,
        name="hook_bracket",
    )
    # Rubber saddle seated in the L-corner: its base is embedded into the floor
    # top (z=0.006) so it reads as one bonded pad, and it nestles against the
    # upturned lip where the laptop edge rests.
    part.visual(
        Box((0.040, 0.024, 0.008)),
        origin=Origin(xyz=(-0.012, 0.0, 0.008)),
        material=rubber,
        name="rubber_saddle",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.066, 0.030, 0.030)),
        mass=0.02,
        origin=Origin(xyz=(-0.006, 0.0, 0.012)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixed_wedge_laptop_riser")

    silver = model.material("brushed_silver_aluminum", rgba=SILVER_RGBA)
    hardware = model.material("satin_pivot_hardware", rgba=HARDWARE_RGBA)
    rubber = model.material("matte_black_rubber", rgba=RUBBER_RGBA)

    # ── WEDGE BODY (root) ─────────────────────────────────────────────
    wedge = model.part("wedge_body")

    # Main hollow shell with ventilation slots
    wedge.visual(
        _wedge_shell_mesh(),
        origin=Origin(),
        material=silver,
        name="wedge_shell",
    )

    # Side-wall ventilation inserts using _lightening_slot_bar
    for side in range(2):
        y_sign = -1.0 if side == 0 else 1.0
        vent_mesh = _lightening_slot_bar(
            length=0.080,
            width=0.016,
            thickness=0.002,
            cuts=(
                (-0.015, 0.028, 0.007, True),
                (0.020, 0.022, 0.006, True),
            ),
            name=f"side_vent_{side}",
        )
        # Place embedded into the outer face of the side wall so the vent
        # mesh shares geometry with the shell wall (connectivity requirement).
        mid_x = (WEDGE_FRONT_X + WEDGE_REAR_X) / 2.0
        mid_z = (WEDGE_FRONT_H + WEDGE_REAR_H) / 2.0
        wedge.visual(
            vent_mesh,
            origin=Origin(
                xyz=(mid_x, y_sign * (WEDGE_WIDTH / 2.0 - 0.001), mid_z - 0.010),
                rpy=(0.0, -INCLINE_ANGLE, 0.0),
            ),
            material=silver,
            name=f"side_vent_{side}",
        )

    # Rubber feet under the base plate (four corners).
    # Embedded 0.001m into the base so they read as bonded pads, not floating.
    foot_positions = (
        (-0.080, -0.088),
        (-0.080, 0.088),
        (0.080, -0.088),
        (0.080, 0.088),
    )
    for i, (fx, fy) in enumerate(foot_positions):
        wedge.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(fx, fy, -0.001)),
            material=rubber,
            name=f"rail_foot_{i}",
        )

    # Hook pivot bosses (small bearing cylinders at the mount points)
    for side in range(2):
        y = -HOOK_Y if side == 0 else HOOK_Y
        wedge.visual(
            Cylinder(radius=0.005, length=0.016),
            origin=Origin(
                xyz=(WEDGE_FRONT_X, y, WEDGE_FRONT_H),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware,
            name=f"hook_pivot_boss_{side}",
        )

    wedge.inertial = Inertial.from_geometry(
        Box((0.200, 0.240, 0.150)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    # ── FRONT HOOKS (revolute retaining lips) ─────────────────────────
    for side in range(2):
        y = -HOOK_Y if side == 0 else HOOK_Y
        hook = model.part(f"front_hook_{side}")
        _add_front_hook(hook, silver, rubber)

        model.articulation(
            f"wedge_to_hook_{side}",
            ArticulationType.REVOLUTE,
            parent=wedge,
            child=hook,
            # Origin at the front edge of the incline; tilted to match panel
            origin=Origin(
                xyz=(WEDGE_FRONT_X, y, WEDGE_FRONT_H),
                rpy=(0.0, -INCLINE_ANGLE, 0.0),
            ),
            # axis=(0,-1,0): positive q folds the lip down for storage
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=1.0, lower=0.0, upper=1.4,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wedge = object_model.get_part("wedge_body")
    hook_0 = object_model.get_part("front_hook_0")
    hook_1 = object_model.get_part("front_hook_1")
    hook_joint_0 = object_model.get_articulation("wedge_to_hook_0")
    hook_joint_1 = object_model.get_articulation("wedge_to_hook_1")

    def visual_names(p) -> set[str]:
        return {v.name for v in p.visuals}

    # ── Wedge body structure ──────────────────────────────────────────
    ctx.check(
        "wedge body has continuous shell mesh",
        "wedge_shell" in visual_names(wedge),
        details=f"{visual_names(wedge)=}",
    )
    ctx.check(
        "wedge shell is a single ventilated CadQuery mesh",
        "wedge_shell" in visual_names(wedge)
        and "wedge" in wedge.get_visual("wedge_shell").geometry.filename,
        details="ventilation slots are cut through the inclined top panel",
    )
    ctx.check(
        "side-wall ventilation inserts present",
        "side_vent_0" in visual_names(wedge) and "side_vent_1" in visual_names(wedge),
        details=f"{visual_names(wedge)=}",
    )
    ctx.check(
        "rubber feet under the base",
        all(f"rail_foot_{i}" in visual_names(wedge) for i in range(4)),
        details=f"{visual_names(wedge)=}",
    )

    # ── Front hooks preserved ─────────────────────────────────────────
    ctx.check(
        "front retaining hooks with rubber saddles exist",
        "hook_bracket" in visual_names(hook_0)
        and "rubber_saddle" in visual_names(hook_0)
        and "hook_bracket" in visual_names(hook_1)
        and "rubber_saddle" in visual_names(hook_1),
        details=f"{visual_names(hook_0)=}, {visual_names(hook_1)=}",
    )

    # ── Non-fixed articulation: hook pivots ───────────────────────────
    non_fixed = [
        j for j in object_model.articulations
        if j.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "hook pivots are the only non-fixed joints (fold linkage removed)",
        len(non_fixed) == 2
        and all(j.name.startswith("wedge_to_hook") for j in non_fixed),
        details=f"non_fixed={[j.name for j in non_fixed]}",
    )

    # Hook pivot motion: at q=0 the lip is upright (retaining);
    # at positive q the lip folds downward for storage.
    rest_aabb = ctx.part_world_aabb(hook_0)
    with ctx.pose({hook_joint_0: 1.2}):
        folded_aabb = ctx.part_world_aabb(hook_0)
    ctx.check(
        "hook lip folds downward when pivoted",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] < rest_aabb[1][2] - 0.008,
        details=f"rest={rest_aabb}, folded={folded_aabb}",
    )

    # ── Overlap allowances ────────────────────────────────────────────
    for side in range(2):
        hook = hook_0 if side == 0 else hook_1
        ctx.allow_overlap(
            wedge, hook,
            elem_a=f"hook_pivot_boss_{side}",
            elem_b="hook_bracket",
            reason="Hook pivot boss is a bearing seated inside the hook bracket pivot zone.",
        )
        ctx.allow_overlap(
            wedge, hook,
            elem_a=f"hook_pivot_boss_{side}",
            elem_b="rubber_saddle",
            reason="Hook pivot boss protrudes into the rubber saddle seat zone at the pivot.",
        )
        ctx.expect_contact(
            wedge, hook,
            elem_a=f"hook_pivot_boss_{side}",
            elem_b="hook_bracket",
            contact_tol=0.003,
            name=f"hook {side} bracket engages its pivot boss",
        )

    return ctx.report()


object_model = build_object_model()
