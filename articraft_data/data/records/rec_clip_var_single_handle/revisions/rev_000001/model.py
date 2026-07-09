from __future__ import annotations

# Articraft model: a binder clip (foldback clip) — single-handle variant.
#
# Real object (from picture/Stationary/Clip/001.png):
#   - A single piece of folded spring steel forms a roughly triangular-prism
#     body, painted glossy orange with a speckled finish. The wide flat bottom
#     is the clamping mouth; the two sloped faces meet at a folded ridge on top.
#     The two bottom front/back edges are rolled into small lips (barrels) that
#     run across the full width of the clip.
#   - ONE stiff steel wire handle (lever loop) is threaded through the front
#     rolled lip. It pivots about that lip axis: you squeeze the handle upward
#     to spring the mouth open, and it folds back down flat against the body at
#     rest. The rear lip is bare — no handle.
#
# Articulation:
#   - Root: clip_body (the folded triangular spring-steel body).
#   - handle_0: a REVOLUTE lever pivoting about the front rolled lip axis (the
#     width / Y axis of the clip). Positive q lifts the free end up and away
#     from the body (the squeeze/open gesture).
#
# Frame convention:
#   - +Y is the clip width and the lip/pivot axis.
#   - +X is depth (front-to-back); the front lip is at -X, the rear lip at +X.
#   - +Z is up; the clamping mouth (bottom of the triangle) sits on z = 0 and
#     the folded apex is the highest point.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Dimensions (meters) -- a standard ~32 mm "medium" binder clip.
# ----------------------------------------------------------------------------
WIDTH = 0.032          # clip width along Y (the lip / pivot axis span)
HALF_W = WIDTH / 2.0

DEPTH = 0.026          # front-to-back span of the clamping mouth (X)
HALF_D = DEPTH / 2.0
APEX_Z = 0.018         # height of the folded ridge above the mouth (Z)
APEX_X = 0.0           # ridge sits centered front-to-back

SHEET_T = 0.0010       # spring-steel sheet thickness
LIP_R = 0.0018         # outer radius of each rolled lip (barrel)
LIP_INNER_R = 0.0011   # hollow inner radius of each rolled lip

WIRE_R = 0.00085       # handle wire radius
HANDLE_LEN = 0.030     # how far a handle reaches from its lip when laid flat
HANDLE_HALF_W = 0.0085  # half the span between a handle's two legs

# Lip centers: the two bottom corners of the triangle.
FRONT_LIP = (-HALF_D, 0.0)   # (x, z)
REAR_LIP = (HALF_D, 0.0)

# --- Handle multiplicity ----------------------------------------------------
# This variant threads a single wire lever handle through the front lip only.
# The rear lip barrel remains intact but bare.
NUM_HANDLES = 1

# Handle configuration table: each entry defines the lip position, reach
# direction, and joint axis sign for one handle. Index i -> handle_{i}.
HANDLE_CONFIGS: list[dict] = [
    {
        "lip_x": FRONT_LIP[0],
        "lip_z": FRONT_LIP[1],
        "reach_dir": -1.0,   # front handle reaches toward -X
        "axis_y": 1.0,       # +Y axis: positive rotation lifts -X toward +Z
    },
    # (A second config for a rear handle would go here; this variant has one.)
]


def _band_profile() -> list[tuple[float, float]]:
    """2D (x, z) center-line of the folded triangular spring-steel band.

    Traced front-bottom -> up the front face -> over the apex -> down the rear
    face -> rear-bottom. The clamping mouth is the open bottom between the two
    lips; we model the band as the visible folded sheet, so the bottom stays
    open (hollow mouth).
    """
    return [
        (FRONT_LIP[0], LIP_R),        # just above the front lip
        (-HALF_D * 0.55, APEX_Z * 0.62),
        (APEX_X, APEX_Z),             # folded ridge / apex
        (HALF_D * 0.55, APEX_Z * 0.62),
        (REAR_LIP[0], LIP_R),         # just above the rear lip
    ]


def _build_body_mesh():
    """Folded sheet-steel triangular body with two rolled lips, in CadQuery."""
    centerline = _band_profile()

    pts = [(x, z) for (x, z) in centerline]

    # Outward normals (in XZ) at each vertex to give the band its thickness.
    n = len(pts)
    outer: list[tuple[float, float]] = []
    inner: list[tuple[float, float]] = []
    for i in range(n):
        px, pz = pts[i]
        # tangent from neighbours
        x0, z0 = pts[max(i - 1, 0)]
        x1, z1 = pts[min(i + 1, n - 1)]
        tx, tz = (x1 - x0), (z1 - z0)
        tl = math.hypot(tx, tz) or 1.0
        tx, tz = tx / tl, tz / tl
        # normal (rotate tangent +90deg): points generally "outward/up"
        nx, nz = -tz, tx
        # ensure the normal points away from interior (upward-ish)
        if nz < 0:
            nx, nz = -nx, -nz
        h = SHEET_T / 2.0
        outer.append((px + nx * h, pz + nz * h))
        inner.append((px - nx * h, pz - nz * h))

    loop = outer + list(reversed(inner))
    band = (
        cq.Workplane("XZ")
        .polyline([(x, z) for (x, z) in loop])
        .close()
        .extrude(WIDTH)
    )
    # Extrude along XZ workplane pushes into -Y by default; recenter on Y=0.
    band = band.translate((0, HALF_W, 0))

    # Rolled lips: hollow tubes (barrels) running along Y at the two bottom
    # corners. Both remain intact even though only one carries a handle.
    def lip(cx: float, cz: float):
        outer_tube = (
            cq.Workplane("XY")
            .workplane(offset=0)
            .center(cx, 0)
            .circle(LIP_R)
            .extrude(WIDTH)
        )
        # The XY circle extrudes along +Z; rotate so the barrel runs along Y.
        outer_tube = outer_tube.rotate((0, 0, 0), (1, 0, 0), -90)
        inner_tube = (
            cq.Workplane("XY")
            .center(cx, 0)
            .circle(LIP_INNER_R)
            .extrude(WIDTH)
        )
        inner_tube = inner_tube.rotate((0, 0, 0), (1, 0, 0), -90)
        barrel = outer_tube.cut(inner_tube)
        barrel = barrel.translate((0, -HALF_W, cz))
        return barrel

    body = band.union(lip(*FRONT_LIP)).union(lip(*REAR_LIP))
    return mesh_from_cadquery(body, "clip_body", tolerance=0.0004, angular_tolerance=0.2)


def _handle_points(lip_x: float, lip_z: float, reach_dir: float) -> list[tuple[float, float, float]]:
    """Center-line points for one U-shaped wire handle laid flat against body.

    The handle is authored in the joint frame whose origin is at the lip center,
    so points are relative to the lip. ``reach_dir`` is +1 (reaches toward -X /
    outward-front) or -1 depending on which lip. We author the *closed* (folded
    flat) pose: the loop lies low, hugging the body, with its free end out near
    the mouth plane.
    """
    ex = reach_dir * HANDLE_LEN          # free-end x offset from lip
    yw = HANDLE_HALF_W
    z_flat = LIP_R + WIRE_R + 0.0003     # leg height when laid against the body

    # Wrap radius: the bent wire centerline rides just on the barrel surface,
    # with a hair of nest so it reads as captured (allowed via allow_overlap).
    wrap = LIP_R + WIRE_R * 0.2          # centerline distance from lip center
    # Hook point on the far (inboard) side of the barrel, slightly under it,
    # so the wire clearly encircles the lip.
    hook_x = -reach_dir * wrap * 0.85
    hook_z = -wrap * 0.45

    # Trace one continuous loop:
    #   +y leg: hook around barrel -> out to tip -> across to -y -> back -> hook.
    return [
        (hook_x, +yw, hook_z),               # +y hook under/behind the barrel
        (reach_dir * wrap, +yw, z_flat),     # +y leg rises onto the body
        (ex * 0.55, +yw, z_flat * 0.9),
        (ex, +yw * 0.7, z_flat * 0.6),
        (ex * 1.02, 0.0, z_flat * 0.55),     # rounded free tip
        (ex, -yw * 0.7, z_flat * 0.6),
        (ex * 0.55, -yw, z_flat * 0.9),
        (reach_dir * wrap, -yw, z_flat),     # -y leg
        (hook_x, -yw, hook_z),               # -y hook under/behind the barrel
    ]


def _build_handle_wire(cfg: dict) -> object:
    """Shared geometry helper: build a wire handle mesh from a config entry."""
    pts = _handle_points(cfg["lip_x"], cfg["lip_z"], cfg["reach_dir"])
    wire = tube_from_spline_points(
        pts,
        radius=WIRE_R,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    return wire


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="binder_clip")

    steel_orange = Material(name="clip_orange", rgba=(0.86, 0.34, 0.13, 1.0))
    wire_steel = Material(name="wire_steel", rgba=(0.30, 0.30, 0.33, 1.0))

    # --- Root: folded triangular spring-steel body --------------------------
    body = model.part("clip_body")
    body.visual(_build_body_mesh(), material=steel_orange, name="body_shell")

    # --- Wire lever handle(s) -----------------------------------------------
    # Emit handles via a for-i-in-range(n) loop. NUM_HANDLES=1 for this
    # single-handle variant; the rear lip barrel remains bare.
    handle_parts = []
    handle_joints = []
    for i in range(NUM_HANDLES):
        cfg = HANDLE_CONFIGS[i]
        part_name = f"handle_{i}"
        visual_name = f"handle_loop_{i}"
        joint_name = f"handle_pivot_{i}"

        handle_part = model.part(part_name)
        wire = _build_handle_wire(cfg)
        handle_part.visual(
            mesh_from_geometry(wire, visual_name),
            material=wire_steel,
            name=visual_name,
        )
        handle_parts.append(handle_part)

        # Articulation: the handle pivots about its rolled-lip (Y) axis.
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=handle_part,
            origin=Origin(xyz=(cfg["lip_x"], 0.0, cfg["lip_z"])),
            axis=(0.0, cfg["axis_y"], 0.0),
            motion_limits=MotionLimits(
                effort=0.4, velocity=4.0, lower=0.0, upper=2.0,
            ),
        )
        handle_joints.append(joint_name)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("clip_body")

    # Collect handle parts and joints by name pattern.
    handle_parts = []
    handle_joints = []
    for i in range(NUM_HANDLES):
        handle_parts.append(object_model.get_part(f"handle_{i}"))
        handle_joints.append(object_model.get_articulation(f"handle_pivot_{i}"))

    # --- Single-handle variant: exactly one handle exists ---------------------
    all_part_names = [p.name for p in object_model.parts]
    handle_names = [n for n in all_part_names if n.startswith("handle_")]
    ctx.check(
        "exactly one handle part exists",
        len(handle_names) == 1,
        details=f"handle parts: {handle_names}",
    )
    ctx.check(
        "handle_0 is the single handle",
        "handle_0" in handle_names,
        details=f"handle parts: {handle_names}",
    )
    # The rear lip must be bare — no rear_handle part.
    ctx.check(
        "rear lip is bare (no rear handle)",
        "rear_handle" not in all_part_names and "handle_1" not in all_part_names,
        details=f"parts: {all_part_names}",
    )

    # --- Mechanism type and axis claims ------------------------------------
    for i in range(NUM_HANDLES):
        joint = handle_joints[i]
        cfg = HANDLE_CONFIGS[i]
        ctx.check(
            f"handle_{i} is a revolute lever",
            joint.joint_type == "revolute",
            details=f"got {joint.joint_type}",
        )
        ctx.check(
            f"handle_{i} pivot axis is the lip (Y) axis",
            abs(joint.axis[1]) > 0.99 and abs(joint.axis[0]) < 1e-6,
            details=f"axis={joint.axis}",
        )

    # Body is the single root; handle hangs off it.
    roots = [p.name for p in object_model.root_parts()]
    ctx.check(
        "clip body is the sole root",
        roots == ["clip_body"],
        details=f"roots={roots}",
    )

    # --- Geometry / proportion claims --------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    assert body_aabb is not None
    (bx0, by0, bz0), (bx1, by1, bz1) = body_aabb
    # Triangular body: rises clearly in +Z (folded apex) and spans the width Y.
    ctx.check(
        "body has the folded apex height",
        (bz1 - bz0) > 0.012,
        details=f"height={bz1 - bz0:.4f}",
    )
    ctx.check(
        "body spans the full clip width",
        (by1 - by0) > 0.028,
        details=f"width={by1 - by0:.4f}",
    )
    ctx.check(
        "body mouth sits near z=0",
        abs(bz0) < 0.002,
        details=f"z_min={bz0:.4f}",
    )

    # --- Both lips present on the body (front and rear barrels) -------------
    # The body mesh spans from front lip to rear lip in X.
    ctx.check(
        "body spans both lips front-to-rear",
        bx0 < FRONT_LIP[0] + 0.002 and bx1 > REAR_LIP[0] - 0.002,
        details=f"body x=[{bx0:.4f}, {bx1:.4f}], lips at {FRONT_LIP[0]:.4f} and {REAR_LIP[0]:.4f}",
    )

    # --- Handle geometry and placement checks -------------------------------
    for i in range(NUM_HANDLES):
        cfg = HANDLE_CONFIGS[i]
        hp = handle_parts[i]
        hp_aabb = ctx.part_world_aabb(hp)
        assert hp_aabb is not None
        visual_name = f"handle_loop_{i}"

        # Handle reaches outward past its lip.
        if cfg["reach_dir"] < 0:
            ctx.check(
                f"handle_{i} reaches outward past its lip",
                hp_aabb[0][0] < cfg["lip_x"] - 0.005,
                details=f"handle x_min={hp_aabb[0][0]:.4f}, lip_x={cfg['lip_x']:.4f}",
            )
        else:
            ctx.check(
                f"handle_{i} reaches outward past its lip",
                hp_aabb[1][0] > cfg["lip_x"] + 0.005,
                details=f"handle x_max={hp_aabb[1][0]:.4f}, lip_x={cfg['lip_x']:.4f}",
            )

        # Handle is captured at its lip: the wire loop contacts the body barrel.
        ctx.expect_contact(
            hp, body,
            elem_a=visual_name, elem_b="body_shell",
            contact_tol=0.0015,
            name=f"handle_{i} threaded through its lip",
        )

        # The wire threading through the rolled lip is an intentional capture nest.
        ctx.allow_overlap(
            hp, body,
            elem_a=visual_name, elem_b="body_shell",
            reason=f"The wire handle_{i} is threaded through the rolled lip barrel; a small wire/lip nest is the real capture fit.",
        )

    # --- Decisive motion check: squeezing the handle lifts its free end -----
    for i in range(NUM_HANDLES):
        hp = handle_parts[i]
        joint = handle_joints[i]
        rest_aabb = ctx.part_world_aabb(hp)
        assert rest_aabb is not None
        rest_top = rest_aabb[1][2]
        with ctx.pose({joint: 1.6}):
            open_aabb = ctx.part_world_aabb(hp)
            assert open_aabb is not None
            open_top = open_aabb[1][2]
        ctx.check(
            f"raising handle_{i} lifts its free end upward",
            open_top > rest_top + 0.008,
            details=f"rest_top={rest_top:.4f}, open_top={open_top:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
