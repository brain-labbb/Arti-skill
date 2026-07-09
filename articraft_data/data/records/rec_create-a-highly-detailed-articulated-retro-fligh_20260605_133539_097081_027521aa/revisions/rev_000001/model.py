from __future__ import annotations

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


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> str:
    model.material(name, rgba=rgba)
    return name


def _hollow_shell() -> cq.Workplane:
    """Hollow helmet shell: outer sphere minus inner sphere, open at the bottom
    for the head and cut open at the front for the face."""
    center = (0.0, 0.012, 0.100)
    r_out = 0.112
    r_in = 0.102  # ~10 mm wall

    outer = cq.Workplane("XY").sphere(r_out)
    inner = cq.Workplane("XY").sphere(r_in)
    shell = outer.cut(inner).translate(center)

    # Open the bottom: keep only the part above the ear/neck line.
    keep_top = cq.Workplane("XY").box(0.6, 0.6, 0.6).translate((0.0, 0.012, 0.300 + 0.034))
    shell = shell.intersect(keep_top)

    # Cut the front face opening (front is -Y).
    face_window = cq.Workplane("XY").box(0.120, 0.120, 0.078).translate((0.0, -0.060, 0.074))
    shell = shell.cut(face_window)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_flight_pilot_helmet")

    shell_mat = _mat(model, "aged_ivory_shell", (0.87, 0.85, 0.77, 1.0))
    liner = _mat(model, "tan_padded_liner", (0.62, 0.52, 0.40, 1.0))
    metal = _mat(model, "brushed_aluminum", (0.60, 0.61, 0.58, 1.0))
    dark = _mat(model, "smoked_visor_glass", (0.03, 0.032, 0.035, 0.74))
    gasket = _mat(model, "black_rubber", (0.02, 0.02, 0.02, 1.0))
    strap = _mat(model, "ivory_webbing", (0.80, 0.76, 0.64, 1.0))

    # ----- Hollow helmet shell (root) -----
    helmet = model.part("helmet_shell")
    helmet.visual(mesh_from_cadquery(_hollow_shell(), "helmet_shell"), material=shell_mat, name="hollow_shell")

    # Interior padded liner ring just inside the bottom opening.
    helmet.visual(
        Cylinder(radius=0.094, length=0.030),
        origin=Origin(xyz=(0.0, 0.012, 0.052)),
        material=liner,
        name="interior_liner_band",
    )
    # Padded liner visible through the front face window.
    helmet.visual(
        Box((0.104, 0.016, 0.052)),
        origin=Origin(xyz=(0.0, -0.082, 0.066)),
        material=liner,
        name="front_face_pad",
    )

    # Ear cups bulging from both sides, embedded into the shell.
    for side, x in enumerate((-0.100, 0.100)):
        helmet.visual(
            Cylinder(radius=0.034, length=0.022),
            origin=Origin(xyz=(x, 0.006, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=shell_mat,
            name=f"ear_cup_{side}",
        )
        helmet.visual(
            Cylinder(radius=0.019, length=0.008),
            origin=Origin(xyz=(x + (0.013 if x > 0 else -0.013), 0.006, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=liner,
            name=f"ear_pad_{side}",
        )

    # Round top access plate with a small latch.
    helmet.visual(
        Cylinder(radius=0.036, length=0.008),
        origin=Origin(xyz=(0.0, 0.028, 0.200)),
        material=shell_mat,
        name="top_plate",
    )
    helmet.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, 0.028, 0.206)),
        material=gasket,
        name="top_plate_recess",
    )
    helmet.visual(
        Box((0.030, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, -0.004, 0.205)),
        material=metal,
        name="top_latch",
    )

    # Chin straps anchored to the lower sides of the shell.
    for side, x in enumerate((-0.066, 0.066)):
        helmet.visual(
            Box((0.014, 0.010, 0.085)),
            origin=Origin(xyz=(x, -0.050, 0.012), rpy=(0.42, 0.0, 0.0)),
            material=strap,
            name=f"chin_strap_{side}",
        )

    # ----- Visor assembly (child) authored in the brow-pivot frame -----
    visor = model.part("visor_assembly")
    visor.visual(
        Box((0.150, 0.012, 0.066)),
        origin=Origin(xyz=(0.0, -0.028, -0.045)),
        material=dark,
        name="dark_visor_lens",
    )
    visor.visual(
        Box((0.162, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, -0.028, -0.008)),
        material=metal,
        name="ribbed_top_band",
    )
    for i, x in enumerate((-0.060, -0.040, -0.020, 0.0, 0.020, 0.040, 0.060)):
        visor.visual(
            Box((0.005, 0.006, 0.013)),
            origin=Origin(xyz=(x, -0.037, -0.008)),
            material=gasket,
            name=f"visor_rib_{i}",
        )
    visor.visual(
        Box((0.150, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.028, -0.080)),
        material=metal,
        name="lower_visor_trim",
    )
    # Side metal hinge plates that reach back and clamp onto the shell sides.
    for side, x in enumerate((-0.082, 0.082)):
        visor.visual(
            Box((0.026, 0.056, 0.062)),
            origin=Origin(xyz=(x, 0.006, -0.020)),
            material=metal,
            name=f"side_hinge_plate_{side}",
        )
        visor.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(xyz=(x, -0.024, -0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gasket,
            name=f"plate_rivet_{side}_0",
        )
        visor.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(xyz=(x, -0.024, -0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gasket,
            name=f"plate_rivet_{side}_1",
        )
    # Short hinge pins at the pivot, seated inside the side plates.
    for side, x in enumerate((-0.090, 0.090)):
        visor.visual(
            Cylinder(radius=0.006, length=0.026),
            origin=Origin(xyz=(x, -0.008, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"hinge_pin_{side}",
        )

    model.articulation(
        "helmet_to_visor",
        ArticulationType.REVOLUTE,
        parent=helmet,
        child=visor,
        origin=Origin(xyz=(0.0, -0.082, 0.132)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.2, lower=0.0, upper=1.10),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    helmet = object_model.get_part("helmet_shell")
    visor = object_model.get_part("visor_assembly")
    hinge = object_model.get_articulation("helmet_to_visor")

    # The metal hinge plates and pins are intentionally seated onto the shell.
    ctx.allow_overlap(helmet, visor, elem_a="hollow_shell", elem_b="hinge_pin_0", reason="The visor hinge pin is seated into the shell pivot boss.")
    ctx.allow_overlap(helmet, visor, elem_a="hollow_shell", elem_b="hinge_pin_1", reason="The visor hinge pin is seated into the shell pivot boss.")
    ctx.allow_overlap(helmet, visor, elem_a="hollow_shell", elem_b="side_hinge_plate_0", reason="The side hinge plate clamps onto the shell side.")
    ctx.allow_overlap(helmet, visor, elem_a="hollow_shell", elem_b="side_hinge_plate_1", reason="The side hinge plate clamps onto the shell side.")

    # The closed visor should cover the face opening across its width.
    ctx.expect_overlap(visor, helmet, axes="x", elem_a="dark_visor_lens", elem_b="front_face_pad", min_overlap=0.090, name="closed visor spans the face opening width")
    # The metal brow band reads as part of one visor assembly.
    ctx.expect_overlap(visor, visor, axes="x", elem_a="ribbed_top_band", elem_b="dark_visor_lens", min_overlap=0.140, name="metal brow band covers the visor lens")

    closed = ctx.part_element_world_aabb(visor, elem="dark_visor_lens")
    with ctx.pose({hinge: 0.9}):
        opened = ctx.part_element_world_aabb(visor, elem="dark_visor_lens")
    closed_bottom = closed[0][2] if closed else None
    opened_bottom = opened[0][2] if opened else None
    ctx.check(
        "visor lifts up when opened",
        closed_bottom is not None and opened_bottom is not None and opened_bottom > closed_bottom + 0.02,
        details=f"closed_bottom={closed_bottom}, opened_bottom={opened_bottom}",
    )
    ctx.check(
        "helmet has hollow shell plus hardware and visor detail",
        len(helmet.visuals) >= 11 and len(visor.visuals) >= 16,
        details="helmet should include hollow shell, liner, ear cups, top plate, straps; visor should include lens, ribbed band, plates, rivets, pins",
    )
    return ctx.report()


object_model = build_object_model()
