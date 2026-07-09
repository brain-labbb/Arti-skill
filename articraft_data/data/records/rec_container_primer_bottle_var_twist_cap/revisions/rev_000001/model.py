from __future__ import annotations

# Black airless cosmetic PRIMER bottle with a THREADED SCREW CAP.
# Fork of the cushion-bodied primer bottle: same body, gold band, and label,
# but the press-down pump actuator is replaced by a cylindrical threaded screw
# cap that seats over the neck collar and unscrews via rotation about the
# vertical axis (REVOLUTE joint).
# Frame: bottle stands upright along +Z. Base at z=0.
# Articulation:
#   - body_to_cap: REVOLUTE about Z, positive q unscrews the cap.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
BODY_W = 0.034  # across X (slim)
BODY_D = 0.024  # front-to-back along Y
BODY_H = 0.086  # body height
BODY_FILLET = 0.008  # rounded vertical edges -> cushion cross-section

SHOULDER_H = 0.006  # tapered shoulder above the main body
SHOULDER_TOP_W = 0.020
SHOULDER_TOP_D = 0.016

NECK_W = 0.018  # short collar the cap seats over
NECK_D = 0.014
NECK_H = 0.006
NECK_TOP_Z = BODY_H + SHOULDER_H + NECK_H  # 0.098

GOLD_BAND_Z = 0.050  # gold accent band center height on the body
GOLD_BAND_H = 0.005

# Screw cap dimensions
CAP_OD = 0.024  # outer diameter of the cylindrical cap
CAP_H = 0.015  # total cap height
CAP_SEAT_OVERLAP = 0.005  # how far the skirt slips down over the neck
CAP_WALL = 0.002  # wall thickness above the bore

N_GRIP_RIBS = 16  # vertical grip ribs around the circumference
GRIP_RIB_DEPTH = 0.0008  # radial protrusion from outer wall
GRIP_RIB_WIDTH = 0.0015  # tangential width of each rib


# ---- shared geometry helpers ----


def _rounded_prism(w: float, d: float, h: float, fillet: float, z0: float = 0.0) -> cq.Workplane:
    """Upright rounded-rectangle prism: footprint w(X) × d(Y), height h, base at z0."""
    f = min(fillet, 0.49 * min(w, d))
    wp = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .rect(w, d)
        .extrude(h)
    )
    if f > 1e-5:
        wp = wp.edges("|Z").fillet(f)
    return wp


def _body_solid() -> cq.Workplane:
    """Main cushion body + tapered shoulder + short neck collar (identical to parent)."""
    body = _rounded_prism(BODY_W, BODY_D, BODY_H, BODY_FILLET, z0=0.0)

    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .rect(BODY_W - 2 * BODY_FILLET + 0.004, BODY_D - 2 * BODY_FILLET + 0.004)
        .workplane(offset=SHOULDER_H)
        .rect(SHOULDER_TOP_W, SHOULDER_TOP_D)
        .loft(ruled=True)
    )
    body = body.union(shoulder)

    neck = _rounded_prism(NECK_W, NECK_D, NECK_H, 0.003, z0=BODY_H + SHOULDER_H)
    body = body.union(neck)
    return body


def _gold_band_solid() -> cq.Workplane:
    """Thin band wrapping the body at GOLD_BAND_Z; slightly proud of the surface."""
    return _rounded_prism(
        BODY_W + 0.0008,
        BODY_D + 0.0008,
        GOLD_BAND_H,
        BODY_FILLET,
        z0=GOLD_BAND_Z - GOLD_BAND_H / 2.0,
    )


def _label_plate_solid() -> cq.Workplane:
    """Slightly raised label area on the front (+Y) face, below the gold band."""
    return (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .center(0.0, BODY_D / 2.0 - 0.0005)
        .rect(BODY_W - 0.010, 0.0018)
        .extrude(GOLD_BAND_Z - GOLD_BAND_H / 2.0 - 0.012 - 0.002)
    )


# ---- screw cap geometry (neck-top local frame: z=0 is neck top surface) ----


def _cap_shell_solid() -> cq.Workplane:
    """Cylindrical screw cap shell in neck-top frame.

    Outer cylindrical wall, rectangular bore matching the neck collar with
    clearance, and a flat top with a small center boss (common cosmetic cap
    detail). Thread ridges inside the bore suggest internal threading.
    """
    z0 = -CAP_SEAT_OVERLAP  # skirt bottom is below neck top
    R = CAP_OD / 2.0

    # Outer cylinder
    cap = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .circle(R)
        .extrude(CAP_H)
    )

    # Rectangular bore matching neck shape with clearance for seating
    bore_w = NECK_W - 0.0008
    bore_d = NECK_D - 0.0008
    bore_depth = CAP_SEAT_OVERLAP + 0.003
    bore = (
        cq.Workplane("XY")
        .workplane(offset=z0 - 0.001)
        .rect(bore_w, bore_d)
        .extrude(bore_depth)
    )
    cap = cap.cut(bore)

    # Internal thread ridges: thin rectangular rings protruding inward from
    # the bore walls to suggest threading engagement with the neck.
    thread_depth = 0.0006
    thread_pitch = 0.002
    for i in range(2):
        tz = z0 + 0.001 + i * thread_pitch
        outer_ring = (
            cq.Workplane("XY")
            .workplane(offset=tz)
            .rect(bore_w + 0.0002, bore_d + 0.0002)
            .extrude(0.0008)
        )
        inner_cut = (
            cq.Workplane("XY")
            .workplane(offset=tz - 0.0001)
            .rect(bore_w - 2 * thread_depth, bore_d - 2 * thread_depth)
            .extrude(0.001)
        )
        thread_ring = outer_ring.cut(inner_cut)
        cap = cap.union(thread_ring)

    # Small center boss on top face (common on cosmetic screw caps)
    boss = (
        cq.Workplane("XY")
        .workplane(offset=z0 + CAP_H - 0.0005)
        .circle(0.004)
        .extrude(0.001)
    )
    cap = cap.union(boss)

    return cap


def _grip_rib_solid(index: int) -> cq.Workplane:
    """Single vertical grip rib at given index, in neck-top local frame.

    A thin radial protrusion on the cap exterior for twisting grip.
    The rib overlaps with the cap shell wall for structural connectivity.
    """
    angle_deg = index * (360.0 / N_GRIP_RIBS)
    R = CAP_OD / 2.0
    z0 = -CAP_SEAT_OVERLAP

    # Rib centered on the cylinder surface: extends GRIP_RIB_DEPTH inward
    # (overlapping the shell wall) and GRIP_RIB_DEPTH outward (visible rib).
    rib = (
        cq.Workplane("XY")
        .workplane(offset=z0 + 0.002)
        .transformed(rotate=(0, 0, angle_deg))
        .center(R, 0)
        .rect(GRIP_RIB_DEPTH * 2, GRIP_RIB_WIDTH)
        .extrude(CAP_H - 0.004)
    )
    return rib


# ---- model assembly ----


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="primer_screw_cap_bottle")

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    gold = model.material("gold_accent", rgba=(0.80, 0.62, 0.22, 1.0))
    cap_charcoal = model.material("cap_charcoal", rgba=(0.10, 0.10, 0.12, 1.0))

    # ---- bottle body (root, identical to parent) ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell"),
        material=matte_black,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_gold_band_solid(), "gold_band"),
        material=gold,
        name="gold_band",
    )
    body.visual(
        mesh_from_cadquery(_label_plate_solid(), "label_plate"),
        material=gold,
        name="label_plate",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=0.090,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- screw cap: rotates about Z to unscrew ----
    cap = model.part("screw_cap")
    cap.visual(
        mesh_from_cadquery(_cap_shell_solid(), "cap_shell"),
        material=cap_charcoal,
        name="cap_shell",
    )

    # Grip ribs: repeated sub-parts emitted via for-i-in-range with name_i
    # naming, shared geometry helper (_grip_rib_solid), regular angular
    # placement, and uniform joint policy (all on the same cap part).
    for i in range(N_GRIP_RIBS):
        cap.visual(
            mesh_from_cadquery(_grip_rib_solid(i), f"grip_rib_{i}"),
            material=cap_charcoal,
            name=f"grip_rib_{i}",
        )

    cap.inertial = Inertial.from_geometry(
        Box((CAP_OD, CAP_OD, CAP_H)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, CAP_H / 2.0 - CAP_SEAT_OVERLAP)),
    )

    # Revolute joint at the neck top contact surface: positive q unscrews.
    model.articulation(
        "body_to_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=math.pi * 2.0
        ),
    )

    return model


# ---- tests ----


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cap = object_model.get_part("screw_cap")
    twist = object_model.get_articulation("body_to_cap")

    # Cap skirt intentionally seats over the neck collar for threaded engagement.
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell",
        elem_b="body_shell",
        reason="The screw cap skirt intentionally seats down over the neck collar for threaded engagement.",
    )

    # ---- body unchanged: tall and slim ----
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is tall (height dominates footprint)",
        body_ext[2] > 0.070 and body_ext[2] > body_ext[0] + 0.030 and body_ext[2] > body_ext[1] + 0.030,
        details=f"body extents={body_ext}",
    )
    ctx.check(
        "body is slim",
        body_ext[0] < 0.045 and body_ext[1] < 0.045,
        details=f"body extents={body_ext}",
    )

    # ---- gold accent band present on the body, partway up ----
    band_aabb = ctx.part_element_world_aabb(body, elem="gold_band")
    band_z = (band_aabb[0][2] + band_aabb[1][2]) / 2.0
    ctx.check(
        "gold accent band sits partway up the body",
        0.030 < band_z < 0.070,
        details=f"gold band center z={band_z}",
    )

    # ---- label plate present on the front face ----
    label_aabb = ctx.part_element_world_aabb(body, elem="label_plate")
    ctx.check(
        "label plate present on front face",
        label_aabb is not None and label_aabb[1][1] > 0.008,
        details=f"label aabb={label_aabb}",
    )

    # ---- cap is on TOP and seated over the neck ----
    cap_aabb = ctx.part_world_aabb(cap)
    cap_center_z = (cap_aabb[0][2] + cap_aabb[1][2]) / 2.0
    ctx.check(
        "cap mounted at the top of the bottle",
        cap_aabb[0][2] > BODY_H - 0.010 and cap_center_z > BODY_H,
        details=f"cap aabb z=[{cap_aabb[0][2]:.4f}, {cap_aabb[1][2]:.4f}]",
    )
    ctx.expect_overlap(
        cap, body, axes="xy", min_overlap=0.010,
        name="cap seated over neck (XY footprint overlap)",
    )

    # ---- joint is REVOLUTE about Z (not prismatic like the parent pump) ----
    ctx.check(
        "cap joint is revolute (twist, not press)",
        twist.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={twist.articulation_type}",
    )

    # ---- cap rotates about Z: track grip_rib_0 center before and after ----
    rib0_rest = ctx.part_element_world_aabb(cap, elem="grip_rib_0")
    rib0_cx_rest = (rib0_rest[0][0] + rib0_rest[1][0]) / 2.0
    rib0_cy_rest = (rib0_rest[0][1] + rib0_rest[1][1]) / 2.0
    rib0_cz_rest = (rib0_rest[0][2] + rib0_rest[1][2]) / 2.0

    with ctx.pose({twist: math.pi / 2.0}):
        rib0_rot = ctx.part_element_world_aabb(cap, elem="grip_rib_0")
        rib0_cx_rot = (rib0_rot[0][0] + rib0_rot[1][0]) / 2.0
        rib0_cy_rot = (rib0_rot[0][1] + rib0_rot[1][1]) / 2.0
        rib0_cz_rot = (rib0_rot[0][2] + rib0_rot[1][2]) / 2.0

        # Cap stays seated during rotation (no vertical lift-off).
        ctx.expect_overlap(
            cap, body, axes="xy", min_overlap=0.010,
            name="cap stays seated when unscrewed 90°",
        )

    dx = rib0_cx_rot - rib0_cx_rest
    dy = rib0_cy_rot - rib0_cy_rest
    dz = rib0_cz_rot - rib0_cz_rest

    ctx.check(
        "cap rotates about Z (grip rib moves in XY plane)",
        abs(dx) > 0.002 or abs(dy) > 0.002,
        details=f"rib0 delta: dx={dx:.4f}, dy={dy:.4f}",
    )
    ctx.check(
        "cap rotation is pure twist (no Z displacement)",
        abs(dz) < 0.001,
        details=f"rib0 dz={dz:.6f}",
    )

    # ---- grip ribs present (all 16) ----
    for i in range(N_GRIP_RIBS):
        rib_name = f"grip_rib_{i}"
        rib_aabb = ctx.part_element_world_aabb(cap, elem=rib_name)
        ctx.check(
            f"grip rib {rib_name} exists and is above the body",
            rib_aabb is not None and rib_aabb[0][2] > BODY_H,
            details=f"aabb={rib_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
