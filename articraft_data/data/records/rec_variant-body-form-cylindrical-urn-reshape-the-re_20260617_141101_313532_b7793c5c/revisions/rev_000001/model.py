from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Global layout (world frame, meters)
# ---------------------------------------------------------------------------
# Round base platform (replaces the parent's rectangular drip tray).
BASE_RADIUS = 0.150
BASE_HEIGHT = 0.020
BASE_RIM_STEP = 0.006
BASE_RIM_LIP = 0.004

# Urn body profile (radius, z) pairs for LatheGeometry.
URN_HEIGHT = 0.300
URN_R_BELLY = 0.105
URN_R_BOTTOM = 0.088
URN_R_TOP = 0.082

# Lid, band, and knob (inline visuals on the urn part).
LID_R = URN_R_TOP - 0.002
LID_T = 0.006
BAND_R = URN_R_TOP + 0.005
BAND_T = 0.010
BAND_Z = URN_HEIGHT - 0.028
KNOB_BASE_Z = URN_HEIGHT + LID_T

# Faucet geometry (in faucet local frame; origin on the urn outer wall).
FAUCET_SHANK_X = -0.015
FAUCET_BODY_END_X = 0.060
FAUCET_BODY_R = 0.013
FAUCET_FLANGE_R = 0.017
FAUCET_BONNET_X = 0.032
FAUCET_PIVOT_Z = 0.028

# Spout (in faucet local frame).
SPOUT_START_X = 0.057
SPOUT_BEND_R = 0.016

# Handle
HANDLE_RAKE = -0.09  # slight backward rake (rad)
LEVER_TRAVEL = math.radians(35.0)

# Faucet placement on urn: one tap at the lower front.
FAUCET_COUNT = 1
FAUCET_HEIGHT_ABOVE_URN = 0.060
URN_R_AT_FAUCET = URN_R_BELLY  # belly radius at faucet height

FAUCET_SPECS = [
    {"angle_rad": 0.0, "height": FAUCET_HEIGHT_ABOVE_URN},
]


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------
def _base_geometry() -> LatheGeometry:
    """Round platform with a shallow raised rim (single lathed solid)."""
    return LatheGeometry(
        [
            (BASE_RADIUS, 0.0),
            (BASE_RADIUS, BASE_HEIGHT + BASE_RIM_LIP),
            (BASE_RADIUS - BASE_RIM_STEP, BASE_HEIGHT + BASE_RIM_LIP),
            (BASE_RADIUS - BASE_RIM_STEP, BASE_HEIGHT),
            (0.001, BASE_HEIGHT),
        ],
        segments=48,
    )


def _urn_shell_geometry() -> LatheGeometry:
    """Cylindrical urn body with gentle belly, tapered neck, and flat caps."""
    return LatheGeometry(
        [
            (0.001, 0.0),
            (0.038, 0.0),
            (URN_R_BOTTOM, 0.005),
            (0.098, 0.018),
            (URN_R_BELLY, 0.055),
            (URN_R_BELLY, 0.220),
            (0.100, 0.250),
            (URN_R_TOP, 0.280),
            (URN_R_TOP + 0.003, URN_HEIGHT),
            (0.001, URN_HEIGHT),
        ],
        segments=48,
    )


def _spout_geometry() -> MeshGeometry:
    """Downward-curving tapered spout (faucet local frame)."""
    n_pts = 24
    n_bend = 10
    cx, cz = SPOUT_START_X, -SPOUT_BEND_R
    specs: list[tuple[float, float, float, float]] = []
    for i in range(n_bend + 1):
        t = (math.pi / 2.0) * i / n_bend
        r = 0.009 + (0.006 - 0.009) * (i / n_bend)
        specs.append(
            (t, cx + SPOUT_BEND_R * math.sin(t), cz + SPOUT_BEND_R * math.cos(t), r)
        )
    specs.append((math.pi / 2.0, cx + SPOUT_BEND_R, cz - 0.008, 0.0055))
    specs.append((math.pi / 2.0, cx + SPOUT_BEND_R, cz - 0.013, 0.005))

    geom = MeshGeometry()
    ring_ids: list[list[int]] = []
    for t, rcx, rcz, r in specs:
        wx, wz = math.sin(t), math.cos(t)
        ids: list[int] = []
        for j in range(n_pts):
            a = 2.0 * math.pi * j / n_pts
            px = rcx + r * math.sin(a) * wx
            py = r * math.cos(a)
            pz = rcz + r * math.sin(a) * wz
            ids.append(geom.add_vertex(px, py, pz))
        ring_ids.append(ids)

    for i in range(len(ring_ids) - 1):
        ra, rb = ring_ids[i], ring_ids[i + 1]
        for j in range(n_pts):
            j2 = (j + 1) % n_pts
            geom.add_face(ra[j], ra[j2], rb[j2])
            geom.add_face(ra[j], rb[j2], rb[j])

    # End caps (outward winding).
    c0x, c0z = specs[0][1], specs[0][2]
    start_center = geom.add_vertex(c0x, 0.0, c0z)
    for j in range(n_pts):
        j2 = (j + 1) % n_pts
        geom.add_face(start_center, ring_ids[0][j2], ring_ids[0][j])
    cnx, cnz = specs[-1][1], specs[-1][2]
    end_center = geom.add_vertex(cnx, 0.0, cnz)
    last = ring_ids[-1]
    for j in range(n_pts):
        j2 = (j + 1) % n_pts
        geom.add_face(end_center, last[j], last[j2])

    return geom


def _faucet_body_geometry() -> MeshGeometry:
    """Chrome faucet body with shank and wall flange (faucet local frame)."""
    body_len = FAUCET_BODY_END_X - FAUCET_SHANK_X
    body = (
        CylinderGeometry(FAUCET_BODY_R, body_len, radial_segments=32)
        .rotate_y(math.pi / 2.0)
        .translate((FAUCET_SHANK_X + FAUCET_BODY_END_X) / 2.0, 0.0, 0.0)
    )
    flange = (
        CylinderGeometry(FAUCET_FLANGE_R, 0.012, radial_segments=32)
        .rotate_y(math.pi / 2.0)
        .translate(FAUCET_SHANK_X + 0.006, 0.0, 0.0)
    )
    body.merge(flange)
    return body


def _faucet_bonnet_geometry() -> MeshGeometry:
    """Chrome bonnet stack above the faucet body."""
    bonnet = CylinderGeometry(0.011, 0.017, radial_segments=32).translate(
        FAUCET_BONNET_X, 0.0, 0.0155
    )
    bonnet.merge(
        CylinderGeometry(0.013, 0.005, radial_segments=32).translate(
            FAUCET_BONNET_X, 0.0, 0.0095
        )
    )
    return bonnet


def _handle_grip_geometry() -> LatheGeometry:
    """Glossy black tapered tap handle grip."""
    return (
        LatheGeometry(
            [
                (0.0078, 0.000),
                (0.0068, 0.018),
                (0.0072, 0.045),
                (0.0092, 0.075),
                (0.0113, 0.098),
                (0.0122, 0.110),
                (0.0110, 0.1165),
                (0.0072, 0.1192),
                (0.0030, 0.1200),
            ],
            segments=40,
        )
        .translate(0.0, 0.0, 0.008)
        .rotate_y(HANDLE_RAKE)
    )


def _handle_ferrule_geometry() -> LatheGeometry:
    """Chrome ferrule sleeve at the grip base."""
    return (
        LatheGeometry(
            [(0.0095, 0.0), (0.0085, 0.012), (0.0080, 0.020)], segments=32
        )
        .translate(0.0, 0.0, 0.0085)
        .rotate_y(HANDLE_RAKE)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylindrical_urn_beverage_dispenser")

    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.67, 0.69, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.80, 0.81, 0.84, 1.0))
    chrome = model.material("chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark_matte = model.material("dark_matte", rgba=(0.16, 0.16, 0.17, 1.0))
    gloss_black = model.material("gloss_black", rgba=(0.05, 0.05, 0.06, 1.0))

    # ------------------------------------------------------------------ base
    base = model.part("base")
    base.visual(
        mesh_from_geometry(_base_geometry(), "base_platform"),
        material=brushed_steel,
        name="base_platform",
    )

    # ------------------------------------------------------------------- urn
    urn = model.part("urn")
    urn.visual(
        mesh_from_geometry(_urn_shell_geometry(), "urn_shell"),
        material=dark_matte,
        name="urn_shell",
    )
    # Chrome decorative band near the top (inline visual, no separate part).
    urn.visual(
        mesh_from_geometry(
            CylinderGeometry(BAND_R, BAND_T, radial_segments=48).translate(
                0.0, 0.0, BAND_Z + BAND_T / 2.0
            ),
            "urn_band",
        ),
        material=chrome,
        name="urn_band",
    )
    # Flat polished steel lid (sits on the urn top cap).
    LID_CENTER_Z = URN_HEIGHT + LID_T / 2.0
    LID_TOP_Z = URN_HEIGHT + LID_T
    urn.visual(
        mesh_from_geometry(
            CylinderGeometry(LID_R, LID_T, radial_segments=48).translate(
                0.0, 0.0, LID_CENTER_Z
            ),
            "urn_lid",
        ),
        material=polished_steel,
        name="urn_lid",
    )
    # Small chrome knob on the lid.
    urn.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.010, LID_TOP_Z),
                    (0.012, LID_TOP_Z + 0.004),
                    (0.010, LID_TOP_Z + 0.014),
                    (0.005, LID_TOP_Z + 0.020),
                    (0.001, LID_TOP_Z + 0.020),
                ],
                segments=24,
            ),
            "urn_knob",
        ),
        material=chrome,
        name="urn_knob",
    )

    model.articulation(
        "base_to_urn",
        ArticulationType.FIXED,
        parent=base,
        child=urn,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
    )

    # -------------------------------------------------- faucets and handles
    for i in range(FAUCET_COUNT):
        spec = FAUCET_SPECS[i]
        angle = spec["angle_rad"]
        height = spec["height"]

        # Mount point on the urn outer wall surface.
        mount_r = URN_R_AT_FAUCET
        mount_x = mount_r * math.cos(angle)
        mount_y = mount_r * math.sin(angle)
        mount_z = height  # in urn local frame

        # ---- faucet (fixed to urn wall) ----
        faucet = model.part(f"faucet_{i}")
        faucet.visual(
            mesh_from_geometry(_faucet_body_geometry(), f"faucet_body_{i}"),
            material=chrome,
            name=f"faucet_body_{i}",
        )
        faucet.visual(
            mesh_from_geometry(_faucet_bonnet_geometry(), f"faucet_bonnet_{i}"),
            material=chrome,
            name=f"faucet_bonnet_{i}",
        )
        faucet.visual(
            mesh_from_geometry(_spout_geometry(), f"faucet_spout_{i}"),
            material=chrome,
            name=f"faucet_spout_{i}",
        )

        model.articulation(
            f"urn_to_faucet_{i}",
            ArticulationType.FIXED,
            parent=urn,
            child=faucet,
            origin=Origin(
                xyz=(mount_x, mount_y, mount_z),
                rpy=(0.0, 0.0, angle),
            ),
        )

        # ---- tap handle (revolute on faucet bonnet) ----
        handle = model.part(f"tap_handle_{i}")
        handle.visual(
            mesh_from_geometry(
                CylinderGeometry(0.0095, 0.026, radial_segments=32).rotate_x(
                    math.pi / 2.0
                ),
                f"lever_collar_{i}",
            ),
            material=chrome,
            name=f"lever_collar_{i}",
        )
        handle.visual(
            mesh_from_geometry(
                _handle_ferrule_geometry(), f"handle_ferrule_{i}"
            ),
            material=chrome,
            name=f"handle_ferrule_{i}",
        )
        handle.visual(
            mesh_from_geometry(_handle_grip_geometry(), f"handle_grip_{i}"),
            material=gloss_black,
            name=f"handle_grip_{i}",
        )

        model.articulation(
            f"faucet_lever_{i}",
            ArticulationType.REVOLUTE,
            parent=faucet,
            child=handle,
            origin=Origin(xyz=(FAUCET_BONNET_X, 0.0, FAUCET_PIVOT_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=4.0, lower=0.0, upper=LEVER_TRAVEL
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    urn = object_model.get_part("urn")
    faucet = object_model.get_part("faucet_0")
    handle = object_model.get_part("tap_handle_0")
    lever = object_model.get_articulation("faucet_lever_0")

    # Overlap allowances: faucet shank through urn wall, collar on bonnet.
    ctx.allow_overlap(
        faucet,
        urn,
        elem_a="faucet_body_0",
        elem_b="urn_shell",
        reason="The faucet shank passes through the urn wall like a real "
        "spigot threaded into the vessel.",
    )
    ctx.allow_overlap(
        handle,
        faucet,
        elem_a="lever_collar_0",
        elem_b="faucet_bonnet_0",
        reason="The lever pivot collar is captured on the faucet bonnet boss "
        "(seated pivot insertion).",
    )

    # --- Cylindrical urn: XY footprint is roughly circular ---
    urn_bb = ctx.part_world_aabb(urn)
    if urn_bb is not None:
        dx = urn_bb[1][0] - urn_bb[0][0]
        dy = urn_bb[1][1] - urn_bb[0][1]
        ctx.check(
            "urn XY extents are roughly equal (circular cross-section)",
            abs(dx - dy) < 0.020,
            details=f"dx={dx:.4f}, dy={dy:.4f}",
        )
        ctx.check(
            "urn body is about 0.30 m tall",
            0.28 <= (urn_bb[1][2] - urn_bb[0][2]) <= 0.36,
            details=f"height={urn_bb[1][2] - urn_bb[0][2]:.4f}",
        )

    # --- Round base ---
    base_bb = ctx.part_world_aabb(base)
    if base_bb is not None:
        bx = base_bb[1][0] - base_bb[0][0]
        by = base_bb[1][1] - base_bb[0][1]
        ctx.check(
            "base has a round (circular) footprint",
            abs(bx - by) < 0.012,
            details=f"bx={bx:.4f}, by={by:.4f}",
        )

    # --- Urn sits on the base ---
    ctx.expect_contact(
        urn,
        base,
        elem_a="urn_shell",
        elem_b="base_platform",
        contact_tol=5e-4,
        name="urn seats on the base platform",
    )
    ctx.expect_within(
        urn,
        base,
        axes="xy",
        inner_elem="urn_shell",
        outer_elem="base_platform",
        name="urn stands within the base footprint",
    )

    # --- Faucet at lower front of urn ---
    body_bb = ctx.part_element_world_aabb(faucet, elem="faucet_body_0")
    spout_bb = ctx.part_element_world_aabb(faucet, elem="faucet_spout_0")
    urn_mid_z = (urn_bb[0][2] + urn_bb[1][2]) / 2.0 if urn_bb else 0.2

    ctx.check(
        "faucet is mounted at the lower half of the urn",
        body_bb is not None and body_bb[0][2] < urn_mid_z,
        details=f"body_min_z={body_bb[0][2]:.4f}, urn_mid_z={urn_mid_z:.4f}"
        if body_bb
        else "no bb",
    )
    ctx.check(
        "faucet body projects forward of the urn",
        body_bb is not None
        and urn_bb is not None
        and body_bb[1][0] > urn_bb[1][0] + 0.02,
        details=f"body={body_bb}, urn={urn_bb}",
    )
    ctx.check(
        "spout curves downward below the faucet body",
        spout_bb is not None
        and body_bb is not None
        and spout_bb[0][2] < body_bb[0][2] - 0.003,
        details=f"spout={spout_bb}, body={body_bb}",
    )

    # --- Handle assembly ---
    ctx.expect_contact(
        handle,
        faucet,
        elem_a="lever_collar_0",
        elem_b="faucet_bonnet_0",
        name="lever collar mounts on the faucet bonnet",
    )
    grip_bb = ctx.part_element_world_aabb(handle, elem="handle_grip_0")
    ctx.check(
        "tap handle is about 0.12 m long",
        grip_bb is not None and 0.100 <= (grip_bb[1][2] - grip_bb[0][2]) <= 0.145,
        details=f"grip={grip_bb}",
    )
    ctx.check(
        "handle rises above the faucet body",
        grip_bb is not None
        and body_bb is not None
        and grip_bb[0][2] > body_bb[1][2],
        details=f"grip={grip_bb}, body={body_bb}",
    )

    # --- Lever axis and limits ---
    axis_ok = (
        abs(lever.axis[0]) < 1e-6
        and abs(abs(lever.axis[1]) - 1.0) < 1e-6
        and abs(lever.axis[2]) < 1e-6
    )
    ctx.check(
        "lever axis is horizontal left-right", axis_ok, details=f"axis={lever.axis}"
    )
    limits = lever.motion_limits
    ctx.check(
        "lever travels from upright to ~35 deg forward",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-6
        and abs(limits.upper - math.radians(35.0)) < 0.05,
        details=f"limits={limits}",
    )

    # --- Pose: handle tilts forward when pulled ---
    rest_bb = ctx.part_world_aabb(handle)
    upper = (
        limits.upper
        if limits is not None and limits.upper is not None
        else LEVER_TRAVEL
    )
    with ctx.pose({lever: upper}):
        open_bb = ctx.part_world_aabb(handle)
        ctx.check(
            "pulled handle tilts forward",
            rest_bb is not None
            and open_bb is not None
            and open_bb[1][0] > rest_bb[1][0] + 0.015,
            details=f"rest={rest_bb}, open={open_bb}",
        )
        ctx.check(
            "pulled handle tips down from vertical",
            rest_bb is not None
            and open_bb is not None
            and open_bb[1][2] < rest_bb[1][2] - 0.004,
            details=f"rest={rest_bb}, open={open_bb}",
        )
        ctx.expect_gap(
            handle,
            faucet,
            axis="z",
            positive_elem="handle_grip_0",
            negative_elem="faucet_spout_0",
            min_gap=0.01,
            name="pulled handle clears the spout",
        )

    return ctx.report()


object_model = build_object_model()
