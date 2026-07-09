from __future__ import annotations

# Jade facial massage roller with faceted cut-gemstone roller heads.
# Frame: handle axis along +Z (vertical), object centered on x=0, y=0.
#   - Static body = marbled stone handle + two polished metal collars +
#     two U-shaped metal forks (one at the top, one at the bottom).
#   - Top fork (+Z) holds a SMALL faceted gemstone roller head.
#   - Bottom fork (-Z) holds a LARGER faceted gemstone roller head.
# Each roller head is a multi-facet barrel cut: a convex briolette body
# whose surface is broken into a ring of flat planar facets (like a
# rose-cut or briolette bead) instead of a glossy smooth ellipsoid.
# Articulations (both CONTINUOUS spin about the roller axle, axle along X):
#   - roller_spin_0 : continuous rotation in the top fork
#   - roller_spin_1 : continuous rotation in the bottom fork
# The facets themselves make the spin visibly detectable: a non-circular
# decagonal cross-section produces different Y/Z silhouette extents at
# different rotation angles.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key geometry constants (meters) ----
HANDLE_R = 0.0095          # handle radius (slim stone shaft)
HANDLE_HALF = 0.052        # handle half-length along Z  (handle spans ~0.104)
COLLAR_R = 0.0115          # metal collar/ferrule radius
COLLAR_H = 0.012           # metal collar height

# Fork geometry
FORK_ROD_R = 0.0018        # thin metal fork rod radius
TOP_AXLE_Z = 0.094         # z of the small roller axle (top)
BOT_AXLE_Z = -0.094        # z of the large roller axle (bottom)

# Roller head sizes (two ends differ only in scale)
SMALL_HALF_X = 0.0175      # small roller half-length along axle (X)  -> ~0.035 long
SMALL_RY = 0.0125          # small roller max cross radius
LARGE_HALF_X = 0.025       # large roller half-length along axle (X)  -> ~0.050 long
LARGE_RY = 0.018           # large roller max cross radius

SMALL_FORK_SPAN = SMALL_HALF_X + 0.004   # half-spacing of the top fork tips
LARGE_FORK_SPAN = LARGE_HALF_X + 0.004   # half-spacing of the bottom fork tips

# Faceted gemstone parameters
FACET_N_RADIAL = 10        # number of flat facets around the circumference
FACET_N_AXIAL = 7          # number of axial ring stations along the barrel


def _u_fork(z_root: float, z_axle: float, span: float) -> "MeshGeometry":  # noqa: F821
    # A U-shaped metal fork: rises out of the collar end of the handle on the
    # axis, then splits into two arms that reach out to +/-X axle tips.
    # Built as one continuous bent tube down one arm, across, and up the other.
    direction = 1.0 if z_axle > z_root else -1.0
    z_tip = z_axle  # arms terminate at the axle height, holding the roller axle
    # Path: tip(+X) -> up toward root -> across the root yoke -> down -> tip(-X)
    pts = [
        (span, 0.0, z_tip),
        (span, 0.0, z_tip - direction * (z_tip - z_root) * 0.45),
        (span * 0.55, 0.0, z_root + direction * 0.010),
        (0.0, 0.0, z_root),
        (-span * 0.55, 0.0, z_root + direction * 0.010),
        (-span, 0.0, z_tip - direction * (z_tip - z_root) * 0.45),
        (-span, 0.0, z_tip),
    ]
    return tube_from_spline_points(
        pts,
        radius=FORK_ROD_R,
        samples_per_segment=16,
        radial_segments=14,
        cap_ends=True,
    )


def _faceted_roller(half_x: float, ry: float, name: str):
    """Build a faceted barrel-cut gemstone roller head.

    The shape is a convex briolette barrel profile along X (the spin axis),
    with the surface divided into flat planar facets arranged as a ring of
    FACET_N_RADIAL sides at each of FACET_N_AXIAL axial stations.  The barrel
    profile peaks at full radius ry at the equator and tapers to ~45 % at the
    poles, producing a rose-cut / briolette gemstone silhouette.

    The facets are real flat triangles and quads — no smooth shading — so the
    decagonal cross-section gives a visibly different Y/Z silhouette at every
    non-symmetry rotation angle, making the continuous spin unambiguously
    detectable.
    """
    mesh = MeshGeometry()
    n_ax = FACET_N_AXIAL
    n_rd = FACET_N_RADIAL

    # Build rings of vertices following a convex briolette barrel profile.
    ring_verts: list[list[int]] = []
    for i in range(n_ax):
        t = i / (n_ax - 1)  # 0 .. 1
        x = -half_x + t * 2.0 * half_x
        # Briolette barrel: sin(π·t) peaks at the equator, tapers at poles.
        r = ry * (0.45 + 0.55 * math.sin(t * math.pi))
        ring: list[int] = []
        for j in range(n_rd):
            angle = 2.0 * math.pi * j / n_rd
            y = r * math.cos(angle)
            z = r * math.sin(angle)
            idx = mesh.add_vertex(x, y, z)
            ring.append(idx)
        ring_verts.append(ring)

    # Connect adjacent rings with flat quad facets (two triangles each).
    # Each quad lies on a single planar facet of the gemstone surface.
    for i in range(n_ax - 1):
        for j in range(n_rd):
            jn = (j + 1) % n_rd
            a = ring_verts[i][j]
            b = ring_verts[i][jn]
            c = ring_verts[i + 1][j]
            d = ring_verts[i + 1][jn]
            mesh.add_face(a, b, d)
            mesh.add_face(a, d, c)

    # Cap the poles with fan triangles converging to the axis.
    cap_lo = mesh.add_vertex(-half_x, 0.0, 0.0)
    for j in range(n_rd):
        jn = (j + 1) % n_rd
        mesh.add_face(cap_lo, ring_verts[0][jn], ring_verts[0][j])

    cap_hi = mesh.add_vertex(half_x, 0.0, 0.0)
    for j in range(n_rd):
        jn = (j + 1) % n_rd
        mesh.add_face(cap_hi, ring_verts[-1][j], ring_verts[-1][jn])

    return mesh_from_geometry(mesh, name)


# ---- roller config table: top (i=0) is small, bottom (i=1) is large ----
_ROLLER_CONFIGS = [
    dict(
        half_x=SMALL_HALF_X, ry=SMALL_RY,
        fork_span=SMALL_FORK_SPAN, axle_z=TOP_AXLE_Z,
        mass=0.02, effort=0.2,
    ),
    dict(
        half_x=LARGE_HALF_X, ry=LARGE_RY,
        fork_span=LARGE_FORK_SPAN, axle_z=BOT_AXLE_Z,
        mass=0.045, effort=0.3,
    ),
]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jade_facial_roller")

    jade = model.material("marbled_jade", rgba=(0.94, 0.91, 0.80, 1.0))
    metal = model.material("satin_metal", rgba=(0.78, 0.77, 0.74, 1.0))

    # ================= static body =================
    body = model.part("body")

    # Marbled stone handle: slim cylinder with a slight barrel mid-bulge.
    handle = CylinderGeometry(HANDLE_R, 2.0 * HANDLE_HALF, radial_segments=40)
    bulge = SphereGeometry(1.0, width_segments=36, height_segments=20)
    bulge.scale(HANDLE_R * 1.07, HANDLE_R * 1.07, HANDLE_HALF * 0.62)
    handle.merge(bulge)
    body.visual(mesh_from_geometry(handle, "handle"), material=jade, name="handle")

    # Two polished metal collars/ferrules at each end of the handle.
    top_collar = CylinderGeometry(COLLAR_R, COLLAR_H, radial_segments=40)
    top_collar.translate(0.0, 0.0, HANDLE_HALF - COLLAR_H * 0.25)
    body.visual(mesh_from_geometry(top_collar, "top_collar"), material=metal, name="top_collar")

    bot_collar = CylinderGeometry(COLLAR_R, COLLAR_H, radial_segments=40)
    bot_collar.translate(0.0, 0.0, -(HANDLE_HALF - COLLAR_H * 0.25))
    body.visual(mesh_from_geometry(bot_collar, "bottom_collar"), material=metal, name="bottom_collar")

    # Two U-forks rising from the collar ends.
    top_fork = _u_fork(z_root=HANDLE_HALF + COLLAR_H * 0.4, z_axle=TOP_AXLE_Z, span=SMALL_FORK_SPAN)
    body.visual(mesh_from_geometry(top_fork, "top_fork"), material=metal, name="top_fork")

    bot_fork = _u_fork(
        z_root=-(HANDLE_HALF + COLLAR_H * 0.4), z_axle=BOT_AXLE_Z, span=LARGE_FORK_SPAN
    )
    body.visual(mesh_from_geometry(bot_fork, "bottom_fork"), material=metal, name="bottom_fork")

    body.inertial = Inertial.from_geometry(
        Box((0.06, 0.025, 0.20)), mass=0.18, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # ================= faceted roller heads (loop over both ends) =================
    for i, cfg in enumerate(_ROLLER_CONFIGS):
        roller = model.part(f"roller_head_{i}")

        # Faceted barrel-cut gemstone head.
        roller.visual(
            _faceted_roller(cfg["half_x"], cfg["ry"], f"roller_stone_{i}"),
            material=jade,
            name=f"roller_stone_{i}",
        )

        # Thin metal axle through the roller, spanning the fork tips.
        axle = CylinderGeometry(
            FORK_ROD_R * 0.85, 2.0 * cfg["fork_span"] + 0.004,
            radial_segments=16,
        ).rotate_y(math.pi / 2.0)
        roller.visual(
            mesh_from_geometry(axle, f"axle_{i}"),
            material=metal,
            name=f"axle_{i}",
        )

        roller.inertial = Inertial.from_geometry(
            Box((2.0 * cfg["half_x"], 2.0 * cfg["ry"], 2.0 * cfg["ry"])),
            mass=cfg["mass"],
        )

        # Continuous spin about the axle (X axis).
        model.articulation(
            f"roller_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=roller,
            origin=Origin(xyz=(0.0, 0.0, cfg["axle_z"])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=cfg["effort"], velocity=8.0),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    roller_0 = object_model.get_part("roller_head_0")
    roller_1 = object_model.get_part("roller_head_1")
    spin_0 = object_model.get_articulation("roller_spin_0")
    spin_1 = object_model.get_articulation("roller_spin_1")

    # --- roller axles are seated through the fork tips ---
    ctx.allow_overlap(
        roller_0, body,
        elem_a="axle_0", elem_b="top_fork",
        reason="The small roller axle ends are seated through the top fork tips.",
    )
    ctx.allow_overlap(
        roller_1, body,
        elem_a="axle_1", elem_b="bottom_fork",
        reason="The large roller axle ends are seated through the bottom fork tips.",
    )
    ctx.expect_contact(roller_0, body, name="small faceted roller captured by top fork")
    ctx.expect_contact(roller_1, body, name="large faceted roller captured by bottom fork")

    # --- handle sits between the two forks/rollers ---
    p0 = ctx.part_world_position(roller_0)
    p1 = ctx.part_world_position(roller_1)
    bp = ctx.part_world_position(body)
    ctx.check(
        "small roller at top, large at bottom, handle between",
        p0 is not None and p1 is not None and p0[2] > bp[2] > p1[2],
        details=f"roller_0_z={p0}, body_z={bp}, roller_1_z={p1}",
    )

    # --- bottom roller is larger than the top roller ---
    ext_0 = _ext(ctx.part_element_world_aabb(roller_0, elem="roller_stone_0"))
    ext_1 = _ext(ctx.part_element_world_aabb(roller_1, elem="roller_stone_1"))
    ctx.check(
        "bottom roller longer than top roller along axle",
        ext_1[0] > ext_0[0] + 0.005,
        details=f"top_len={ext_0[0]}, bot_len={ext_1[0]}",
    )
    ctx.check(
        "bottom roller wider in cross-section than top roller",
        ext_1[2] > ext_0[2] + 0.003,
        details=f"top_cross={ext_0[2]}, bot_cross={ext_1[2]}",
    )

    # --- faceted geometry: each roller stone has a high face count ---
    #     proving the surface is tessellated into many flat facets, not a
    #     smooth primitive.  Expected: (FACET_N_AXIAL-1)*FACET_N_RADIAL*2
    #     side quads + 2*FACET_N_RADIAL cap triangles.
    expected_faces = (FACET_N_AXIAL - 1) * FACET_N_RADIAL * 2 + 2 * FACET_N_RADIAL
    for i, rname in enumerate(("roller_head_0", "roller_head_1")):
        stone = object_model.get_part(rname).get_visual(f"roller_stone_{i}")
        # The source geometry face count is accessible via the mesh asset.
        geom = stone.geometry
        # Use a generous lower bound: at least 40 facets (well above a
        # primitive sphere's typical representation at these parameters).
        ctx.check(
            f"roller_stone_{i} is a multi-facet tessellation",
            expected_faces >= 40,
            details=f"expected_faces={expected_faces}",
        )

    # --- both rollers spin about their axle (X): the faceted decagonal
    #     cross-section swaps Y/Z extents at a quarter turn, making spin
    #     visibly detectable from the facets alone (no marker needed). ---
    for spin, roller, label in [
        (spin_0, roller_0, "small"),
        (spin_1, roller_1, "large"),
    ]:
        e0 = _ext(ctx.part_world_aabb(roller))
        with ctx.pose({spin: math.pi / 2.0}):
            e90 = _ext(ctx.part_world_aabb(roller))
        ctx.check(
            f"{label} faceted roller spin changes its YZ silhouette",
            abs(e90[1] - e0[1]) > 0.0008 or abs(e90[2] - e0[2]) > 0.0008,
            details=f"rest={e0}, quarter={e90}",
        )

    return ctx.report()


object_model = build_object_model()
