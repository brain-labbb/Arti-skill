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


# ---------------------------------------------------------------------------
# Ellipse parameters (oval footprint, longer along X)
# ---------------------------------------------------------------------------
RX_OUTER = 0.080  # outer bowl half-width (X)
RY_OUTER = 0.052  # outer bowl half-depth (Y)
RX_INNER = 0.068  # inner cavity half-width
RY_INNER = 0.042  # inner cavity half-depth
BOWL_H = 0.022    # bowl total height
WALL_T = 0.006    # wall thickness below cavity
CAV_H = 0.016     # cavity depth

LID_RX = 0.079
LID_RY = 0.051
LID_H = 0.012
LID_FILLET = 0.006

HINGE_Y = RY_OUTER - 0.002  # rear hinge line Y
LATCH_Y = -(RY_OUTER - 0.002)  # front latch Y


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _oval_extrude(rx: float, ry: float, height: float, z0: float = 0.0) -> cq.Workplane:
    """Extrude an ellipse from z0 to z0+height."""
    return (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .ellipse(rx, ry)
        .extrude(height)
    )


def _oval_ring(rx_out: float, ry_out: float, rx_in: float, ry_in: float, height: float, z0: float = 0.0) -> cq.Workplane:
    """Elliptical ring (annular disk): outer ellipse minus inner ellipse."""
    outer = _oval_extrude(rx_out, ry_out, height, z0)
    inner = _oval_extrude(rx_in, ry_in, height, z0)
    return outer.cut(inner)


def _hollow_oval_bowl() -> cq.Workplane:
    """Thick translucent smoked-acrylic oval bowl with inner recess."""
    outer = _oval_extrude(RX_OUTER, RY_OUTER, BOWL_H)
    cavity = _oval_extrude(RX_INNER, RY_INNER, CAV_H, z0=WALL_T)
    return outer.cut(cavity)


def _oval_domed_lid() -> cq.Workplane:
    """White domed oval lid with filleted top edge."""
    body = _oval_extrude(LID_RX, LID_RY, LID_H)
    return body.edges(">Z").fillet(LID_FILLET)


def _build_oval_feature(rx: float, ry: float, height: float) -> cq.Workplane:
    """Shared helper for small oval feature disks (gasket, pan, mirror, label)."""
    return _oval_extrude(rx, ry, height)


# ---------------------------------------------------------------------------
# Material helper
# ---------------------------------------------------------------------------

def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> str:
    model.material(name, rgba=rgba)
    return name


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="oval_cushion_compact")

    clear = _mat(model, "smoked_translucent_acrylic", (0.46, 0.42, 0.37, 0.38))
    white = _mat(model, "smooth_white", (0.93, 0.92, 0.88, 1.0))
    black = _mat(model, "black_gasket", (0.02, 0.02, 0.02, 1.0))
    mirror = _mat(model, "oval_mirror", (0.72, 0.78, 0.80, 0.55))
    powder = _mat(model, "beige_powder", (0.86, 0.69, 0.54, 1.0))
    ink = _mat(model, "label_ink", (0.18, 0.20, 0.22, 1.0))
    pink = _mat(model, "pink_emblem", (0.72, 0.20, 0.38, 1.0))
    metal = _mat(model, "hinge_metal", (0.56, 0.55, 0.52, 1.0))

    # ----- Translucent oval bowl base (root) -----
    base = model.part("base")

    # Main hollow oval bowl
    base.visual(
        mesh_from_cadquery(_hollow_oval_bowl(), "translucent_oval_bowl"),
        material=clear,
        name="clear_bowl",
    )

    # Rim gasket – thin elliptical ring at top of bowl
    base.visual(
        mesh_from_cadquery(
            _oval_ring(RX_OUTER - 0.001, RY_OUTER - 0.001,
                       RX_INNER + 0.002, RY_INNER + 0.002, 0.003,
                       z0=BOWL_H - 0.003),
            "rim_gasket_oval",
        ),
        material=black,
        name="rim_gasket",
    )

    # Powder pan – oval dish seated on cavity floor
    base.visual(
        mesh_from_cadquery(
            _build_oval_feature(RX_INNER - 0.002, RY_INNER - 0.002, 0.012),
            "powder_pan_oval",
        ),
        origin=Origin(xyz=(0.0, 0.0, WALL_T)),
        material=powder,
        name="powder_pan",
    )

    # Powder surface dome – slightly raised oval on top of powder
    base.visual(
        mesh_from_cadquery(
            _build_oval_feature(RX_INNER - 0.012, RY_INNER - 0.012, 0.003),
            "powder_dome_oval",
        ),
        origin=Origin(xyz=(0.0, 0.0, WALL_T + 0.012)),
        material=powder,
        name="powder_dome",
    )

    # Front latch socket
    base.visual(
        Box((0.032, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, LATCH_Y, 0.016)),
        material=black,
        name="front_latch_socket",
    )

    # Rear hinge leaf
    base.visual(
        Box((0.072, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, HINGE_Y, 0.024)),
        material=metal,
        name="rear_hinge_leaf",
    )

    # Hinge saddles and barrels (repeated via loop)
    for side, x in enumerate((-0.026, 0.026)):
        base.visual(
            Box((0.022, 0.014, 0.016)),
            origin=Origin(xyz=(x, HINGE_Y, 0.024)),
            material=metal,
            name=f"hinge_saddle_{side}",
        )
        base.visual(
            Cylinder(radius=0.005, length=0.026),
            origin=Origin(xyz=(x, HINGE_Y, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"hinge_barrel_{side}",
        )

    # ----- White domed oval lid (child) in rear-hinge pivot frame -----
    lid = model.part("lid")

    # Main domed oval lid panel – offset from hinge to cover the bowl
    lid.visual(
        mesh_from_cadquery(_oval_domed_lid(), "white_oval_domed_lid"),
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.006)),
        material=white,
        name="white_domed_lid",
    )

    # Label disc – oval on top of lid
    lid.visual(
        mesh_from_cadquery(
            _build_oval_feature(0.050, 0.034, 0.003),
            "label_disc_oval",
        ),
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.013)),
        material=white,
        name="label_disc",
    )

    # Printed label ring – oval ink outline
    lid.visual(
        mesh_from_cadquery(
            _oval_ring(0.046, 0.031, 0.038, 0.025, 0.0015),
            "printed_label_ring_oval",
        ),
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.0145)),
        material=ink,
        name="printed_label_ring",
    )

    # Flower emblem center (small circular features – hardware detail)
    lid.visual(
        Cylinder(radius=0.006, length=0.002),
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.015)),
        material=pink,
        name="flower_core",
    )
    for i, a in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        lid.visual(
            Cylinder(radius=0.004, length=0.002),
            origin=Origin(
                xyz=(0.013 * math.cos(a), -HINGE_Y + 0.013 * math.sin(a), 0.015)
            ),
            material=pink,
            name=f"flower_petal_{i}",
        )

    # Large oval mirror mounted on lid underside
    lid.visual(
        mesh_from_cadquery(
            _build_oval_feature(LID_RX - 0.006, LID_RY - 0.006, 0.003),
            "inner_oval_mirror",
        ),
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.003)),
        material=mirror,
        name="inner_mirror",
    )

    # Front latch tab
    lid.visual(
        Box((0.030, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, LATCH_Y - HINGE_Y, 0.002)),
        material=black,
        name="front_latch_tab",
    )

    # Lid hinge pins and knuckle arms (circular pins + box arms bridging to lid panel)
    for side, x in enumerate((-0.026, 0.026)):
        lid.visual(
            Cylinder(radius=0.0035, length=0.020),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"lid_hinge_pin_{side}",
        )
        # Hinge knuckle arm bridges from pin pivot to lid rear edge
        lid.visual(
            Box((0.014, 0.008, 0.012)),
            origin=Origin(xyz=(x, 0.003, 0.004)),
            material=metal,
            name=f"lid_hinge_knuckle_{side}",
        )

    # ----- Articulation: rear single hinge -----
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, 0.030)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.3, lower=0.0, upper=1.70),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("base_to_lid")

    # Hinge hardware intentional embeddings
    for side in (0, 1):
        ctx.allow_overlap(
            lid, base,
            elem_a=f"lid_hinge_pin_{side}",
            elem_b=f"hinge_barrel_{side}",
            reason="The lid hinge pin is captured by the rear hinge barrel.",
        )
        ctx.allow_overlap(
            base, lid,
            elem_a=f"hinge_barrel_{side}",
            elem_b="white_domed_lid",
            reason="The rear hinge barrel is locally nested under the rear lid edge.",
        )
        ctx.allow_overlap(
            base, lid,
            elem_a=f"hinge_saddle_{side}",
            elem_b=f"lid_hinge_pin_{side}",
            reason="The hinge pin passes through the saddle mount area at the pivot.",
        )
        # Knuckle arm is seated into the lid rear edge and wraps around the pin
        ctx.allow_overlap(
            lid, lid,
            elem_a=f"lid_hinge_knuckle_{side}",
            elem_b="white_domed_lid",
            reason="The hinge knuckle arm is embedded into the lid rear edge to bridge pin to panel.",
        )
        ctx.allow_overlap(
            base, lid,
            elem_a=f"hinge_saddle_{side}",
            elem_b=f"lid_hinge_knuckle_{side}",
            reason="The hinge knuckle passes through the saddle area at the pivot.",
        )
        ctx.allow_overlap(
            base, lid,
            elem_a=f"hinge_barrel_{side}",
            elem_b=f"lid_hinge_knuckle_{side}",
            reason="The hinge knuckle wraps around the barrel at the pivot.",
        )

    # --- Oval footprint checks ---

    # Closed lid covers the oval bowl footprint
    ctx.expect_overlap(
        lid, base, axes="xy", min_overlap=0.080,
        name="closed oval lid covers the bowl footprint",
    )

    # The bowl is wider in X than in Y (proves oval, not round)
    bowl_aabb = ctx.part_world_aabb(base)
    if bowl_aabb is not None:
        mn, mx = bowl_aabb
        dx = mx[0] - mn[0]
        dy = mx[1] - mn[1]
        ctx.check(
            "oval footprint is wider in X than Y",
            dx > dy + 0.020,
            details=f"dx={dx:.4f}, dy={dy:.4f}",
        )

    # Powder pan seated inside the translucent oval bowl
    ctx.expect_overlap(
        base, base, axes="xy",
        elem_a="powder_pan", elem_b="clear_bowl",
        min_overlap=0.075,
        name="powder pan seated inside the oval bowl",
    )

    # Oval mirror fills the inside of the lid
    ctx.expect_overlap(
        lid, lid, axes="xy",
        elem_a="inner_mirror", elem_b="white_domed_lid",
        min_overlap=0.085,
        name="oval mirror fills the inside of the lid",
    )

    # Front latch lines up with the socket
    ctx.expect_overlap(
        lid, base, axes="x",
        elem_a="front_latch_tab", elem_b="front_latch_socket",
        min_overlap=0.020,
        name="front latch lines up with the socket",
    )

    # Hinge knuckle arms bridge pin to lid panel (proof for allow_overlap)
    for side in (0, 1):
        ctx.expect_overlap(
            lid, lid, axes="xy",
            elem_a=f"lid_hinge_knuckle_{side}", elem_b="white_domed_lid",
            min_overlap=0.001,
            name=f"hinge knuckle {side} contacts lid panel",
        )

    # --- Lid articulation checks ---
    closed = ctx.part_element_world_aabb(lid, elem="white_domed_lid")
    with ctx.pose({hinge: 1.20}):
        opened = ctx.part_element_world_aabb(lid, elem="white_domed_lid")
    ctx.check(
        "lid opens upward",
        closed is not None and opened is not None and opened[1][2] > closed[1][2] + 0.040,
        details=f"closed={closed}, opened={opened}",
    )

    # Visual richness: bowl has hollow shell + gasket + pan + hinge,
    # lid has label + flower + mirror + latch
    ctx.check(
        "oval compact has hollow bowl, label, mirror and hinge knuckles",
        len(base.visuals) >= 6 and len(lid.visuals) >= 11,
        details="base should be a hollow translucent oval bowl with powder pan and hinge; "
                "lid should include label, flower, mirror, latch and hinge knuckles",
    )

    return ctx.report()


object_model = build_object_model()
