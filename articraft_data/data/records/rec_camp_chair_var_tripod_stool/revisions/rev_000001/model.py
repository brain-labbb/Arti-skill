from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


# ---------------------------------------------------------------------------
# Geometry helpers (KEEP: _add_tube, _add_ball, _add_box)
# ---------------------------------------------------------------------------

def _tube_pose(p0: tuple[float, float, float], p1: tuple[float, float, float]):
    """Return an Origin that aligns a local +Z cylinder from p0 to p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    center = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _add_tube(part, name: str, p0, p1, radius: float, material: Material) -> None:
    origin, length = _tube_pose(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_ball(part, name: str, xyz, radius: float, material: Material) -> None:
    part.visual(Sphere(radius=radius), origin=Origin(xyz=xyz), material=material, name=name)


def _add_box(part, name: str, size, xyz, material: Material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


# ---------------------------------------------------------------------------
# Stool constants
# ---------------------------------------------------------------------------
N_LEGS = 3
APEX_Z = 0.38          # seat-ring height (m)
RING_R = 0.13          # seat-ring centreline radius
RING_TUBE = 0.012      # ring cross-section radius
FOOT_R = 0.30          # foot spread radius
FOOT_Z = 0.015         # foot-centre height
TUBE_R = 0.012         # leg tube radius
SEAT_R = 0.11          # seat-triangle vertex radius
SEAT_Z = 0.348         # seat fabric height (below ring for clearance)
SEAT_THICK = 0.008     # seat extrusion thickness
COLLAR_R = 0.016       # pivot collar radius (wraps ring tube)
COLLAR_HALF = 0.024    # half-length of pivot collar along ring tangent


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="folding_tripod_camp_stool",
        meta={
            "source_image": "picture/Camping_Outdoor Gear/Camp chair/001.png",
            "asset_category": "Camping_Outdoor Gear",
            "asset_subcategory": "Camp chair",
            "description": "Compact three-legged folding tripod camp stool for backpacking and fishing.",
        },
    )

    # -- materials ----------------------------------------------------------
    black_fabric = model.material("black_oxford_fabric", rgba=(0.02, 0.02, 0.025, 1.0))
    gray_fabric = model.material("charcoal_gray_fabric", rgba=(0.27, 0.29, 0.27, 1.0))
    orange_trim = model.material("orange_piping", rgba=(1.0, 0.32, 0.04, 1.0))
    metal = model.material("dark_tubular_steel", rgba=(0.43, 0.43, 0.39, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.015, 1.0))
    silver = model.material("silver_rivet", rgba=(0.86, 0.88, 0.86, 1.0))

    # ======================================================================
    # ROOT: chair_frame  (apex ring + triangular sling seat)
    # ======================================================================
    frame = model.part("chair_frame")

    # Seat ring – torus at apex where legs pivot
    ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=RING_R, tube=RING_TUBE, radial_segments=16, tubular_segments=36),
        "apex_ring",
    )
    frame.visual(ring_mesh, origin=Origin(xyz=(0.0, 0.0, APEX_Z)), material=metal, name="apex_ring")

    # Central hub cap above the ring
    _add_ball(frame, "apex_hub", (0.0, 0.0, APEX_Z + 0.018), 0.014, black_plastic)

    # Hub post connecting seat centre to hub cap (structural connectivity)
    _add_tube(frame, "hub_post",
              (0.0, 0.0, SEAT_Z + SEAT_THICK * 0.5),
              (0.0, 0.0, APEX_Z + 0.014),
              0.005, black_plastic)

    # Triangular sling seat – gray fabric centre (KEEP: seat_gray_center)
    seat_profile = [
        (SEAT_R * math.cos(i * 2.0 * math.pi / N_LEGS),
         SEAT_R * math.sin(i * 2.0 * math.pi / N_LEGS))
        for i in range(N_LEGS)
    ]
    seat_mesh = mesh_from_geometry(
        ExtrudeGeometry(seat_profile, SEAT_THICK, cap=True, center=True),
        "seat_sling",
    )
    frame.visual(seat_mesh, origin=Origin(xyz=(0.0, 0.0, SEAT_Z)), material=gray_fabric, name="seat_gray_center")

    # Black fabric border underneath (slightly larger triangle)
    border_r = SEAT_R + 0.015
    border_profile = [
        (border_r * math.cos(i * 2.0 * math.pi / N_LEGS),
         border_r * math.sin(i * 2.0 * math.pi / N_LEGS))
        for i in range(N_LEGS)
    ]
    border_mesh = mesh_from_geometry(
        ExtrudeGeometry(border_profile, SEAT_THICK * 0.7, cap=True, center=True),
        "seat_border",
    )
    frame.visual(border_mesh, origin=Origin(xyz=(0.0, 0.0, SEAT_Z - 0.004)), material=black_fabric, name="seat_border")

    # Orange piping along each seat edge
    for i in range(N_LEGS):
        theta_a = i * 2.0 * math.pi / N_LEGS
        theta_b = ((i + 1) % N_LEGS) * 2.0 * math.pi / N_LEGS
        _add_tube(
            frame, f"seat_piping_{i}",
            (SEAT_R * math.cos(theta_a), SEAT_R * math.sin(theta_a), SEAT_Z + 0.005),
            (SEAT_R * math.cos(theta_b), SEAT_R * math.sin(theta_b), SEAT_Z + 0.005),
            0.003, orange_trim,
        )

    # Corner reinforcement dots
    for i in range(N_LEGS):
        theta = i * 2.0 * math.pi / N_LEGS
        _add_ball(
            frame, f"seat_corner_{i}",
            (SEAT_R * math.cos(theta), SEAT_R * math.sin(theta), SEAT_Z + 0.005),
            0.005, silver,
        )

    # Seat-to-ring attachment loops (fabric webbing connecting seat to ring)
    for i in range(N_LEGS):
        theta = i * 2.0 * math.pi / N_LEGS
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        _add_tube(
            frame, f"seat_loop_{i}",
            (SEAT_R * cos_t, SEAT_R * sin_t, SEAT_Z + SEAT_THICK * 0.5),
            ((RING_R - RING_TUBE * 0.5) * cos_t, (RING_R - RING_TUBE * 0.5) * sin_t, APEX_Z - RING_TUBE * 0.5),
            0.004, black_fabric,
        )

    # ======================================================================
    # LEG PARTS  (loop over N_LEGS, radial 120° layout)
    # ======================================================================
    splay = FOOT_R - RING_R   # horizontal distance ring→foot
    drop = APEX_Z - FOOT_Z    # vertical distance ring→foot

    for i in range(N_LEGS):
        theta = i * 2.0 * math.pi / N_LEGS
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        # World-space pivot on the ring
        pivot_xyz = (RING_R * cos_t, RING_R * sin_t, APEX_Z)

        # Child part
        leg = model.part(f"leg_{i}")

        # Leg tube from pivot (local origin) to foot (local coords)
        foot_local = (splay * cos_t, splay * sin_t, -drop)
        _add_tube(leg, f"leg_tube_{i}", (0.0, 0.0, 0.0), foot_local, TUBE_R, metal)

        # Pivot collar – short tube along ring tangent wrapping the ring
        # Tangent at θ = (-sin θ, cos θ, 0)
        c0 = (COLLAR_HALF * sin_t, -COLLAR_HALF * cos_t, 0.0)
        c1 = (-COLLAR_HALF * sin_t, COLLAR_HALF * cos_t, 0.0)
        _add_tube(leg, f"pivot_collar_{i}", c0, c1, COLLAR_R, black_plastic)

        # Rubber foot cap
        _add_box(leg, f"leg_foot_{i}", (0.040, 0.040, 0.022), foot_local, black_plastic)

        # Foot rivet on top of the cap
        _add_ball(
            leg, f"foot_rivet_{i}",
            (foot_local[0], foot_local[1], foot_local[2] + 0.016),
            0.005, silver,
        )

        # Revolute joint – positive q folds leg inward toward centre axis
        axis = (-sin_t, cos_t, 0.0)
        model.articulation(
            f"frame_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=leg,
            origin=Origin(xyz=pivot_xyz),
            axis=axis,
            motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=0.80),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("chair_frame")

    # -- metadata -----------------------------------------------------------
    ctx.check(
        "provenance metadata preserved",
        object_model.meta.get("source_image") == "picture/Camping_Outdoor Gear/Camp chair/001.png"
        and object_model.meta.get("asset_category") == "Camping_Outdoor Gear"
        and object_model.meta.get("asset_subcategory") == "Camp chair",
        details=f"metadata={object_model.meta}",
    )

    # -- seat ---------------------------------------------------------------
    seat_vis = frame.get_visual("seat_gray_center")
    ctx.check(
        "triangular sling seat seat_gray_center exists on chair_frame",
        seat_vis is not None,
        details="seat_gray_center visual required on chair_frame",
    )

    # -- legs and joints ----------------------------------------------------
    legs = []
    joints = []
    for i in range(N_LEGS):
        leg = object_model.get_part(f"leg_{i}")
        joint = object_model.get_articulation(f"frame_to_leg_{i}")
        legs.append(leg)
        joints.append(joint)

        # Collar captures ring – intentional local overlap
        ctx.allow_overlap(
            frame, leg,
            elem_a="apex_ring",
            elem_b=f"pivot_collar_{i}",
            reason=f"leg_{i} pivot collar wraps around the apex ring for the fold pivot bearing.",
        )
        # Leg tube passes through ring volume at the pivot (shaft through bearing)
        ctx.allow_overlap(
            frame, leg,
            elem_a="apex_ring",
            elem_b=f"leg_tube_{i}",
            reason=f"leg_{i} tube emerges from the apex ring at the pivot point, like a shaft through a bearing.",
        )
        # Seat loop wraps around leg tube near the ring (fabric webbing attachment)
        ctx.allow_overlap(
            frame, leg,
            elem_a=f"seat_loop_{i}",
            elem_b=f"leg_tube_{i}",
            reason=f"leg_{i} seat loop is fabric webbing sewn around the leg tube near the apex ring attachment.",
        )
        # Seat loop overlaps pivot collar at the ring attachment point
        ctx.allow_overlap(
            frame, leg,
            elem_a=f"seat_loop_{i}",
            elem_b=f"pivot_collar_{i}",
            reason=f"leg_{i} seat loop fabric webbing passes through the pivot collar area at the ring attachment.",
        )
        ctx.expect_overlap(
            frame, leg,
            axes="xy", min_overlap=0.004,
            elem_a="apex_ring", elem_b=f"pivot_collar_{i}",
            name=f"leg_{i} pivot collar retained on apex ring",
        )

    ctx.check(
        "three leg parts exist",
        len(legs) == N_LEGS and all(l is not None for l in legs),
        details=f"leg count={len(legs)}",
    )

    # Verify fold axes are tangent to ring at each leg
    axes_ok = True
    for i, jnt in enumerate(joints):
        theta = i * 2.0 * math.pi / N_LEGS
        expected = (-math.sin(theta), math.cos(theta), 0.0)
        actual = tuple(jnt.axis)
        if any(abs(a - e) > 0.01 for a, e in zip(actual, expected)):
            axes_ok = False
    ctx.check(
        "leg fold axes tangent to apex ring",
        axes_ok,
        details=f"axes={[tuple(j.axis) for j in joints]}",
    )

    # Verify all joints are revolute with positive range
    for i, jnt in enumerate(joints):
        lim = jnt.motion_limits
        ctx.check(
            f"leg_{i} joint is revolute with positive fold range",
            jnt.articulation_type == ArticulationType.REVOLUTE
            and lim is not None
            and lim.lower is not None and lim.upper is not None
            and lim.lower == 0.0 and lim.upper > 0.5,
            details=f"type={jnt.articulation_type}, limits={lim}",
        )

    # -- fold behaviour -----------------------------------------------------
    rest_aabbs = [ctx.part_world_aabb(leg) for leg in legs]

    fold_pose = {jnt: 0.45 for jnt in joints}
    with ctx.pose(fold_pose):
        fold_aabbs = [ctx.part_world_aabb(leg) for leg in legs]
        for i, leg in enumerate(legs):
            ctx.expect_overlap(
                frame, leg,
                axes="xy", min_overlap=0.003,
                elem_a="apex_ring", elem_b=f"pivot_collar_{i}",
                name=f"leg_{i} pivot retained during fold",
            )

    # Legs move inward (max XY radius from origin decreases during fold)
    def _max_xy_radius(aabb):
        if aabb is None:
            return None
        corners = [
            (aabb[0][0], aabb[0][1]), (aabb[0][0], aabb[1][1]),
            (aabb[1][0], aabb[0][1]), (aabb[1][0], aabb[1][1]),
        ]
        return max(math.sqrt(x * x + y * y) for x, y in corners)

    inward_ok = True
    details_parts = []
    for i, (ra, fa) in enumerate(zip(rest_aabbs, fold_aabbs)):
        rr = _max_xy_radius(ra)
        fr = _max_xy_radius(fa)
        details_parts.append(f"leg_{i}: rest_r={rr:.4f} fold_r={fr:.4f}")
        if rr is None or fr is None or fr >= rr - 0.01:
            inward_ok = False
    ctx.check(
        "positive fold q moves legs inward toward centre axis",
        inward_ok,
        details="; ".join(details_parts),
    )

    return ctx.report()


object_model = build_object_model()
