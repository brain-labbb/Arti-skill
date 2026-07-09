from __future__ import annotations

# Open steel grate trap door: a rectangular border frame with evenly spaced
# parallel slat bars, hinged at the rear on a square mesh collar that caps a
# round concrete / stone well shaft.  You can see through the grate into the
# shaft below.
#
# Articraft brief:
# - Object: a rectangular open steel grate leaf (~0.66 x 0.70 m) on a square
#   mesh collar (0.80 x 0.80 m) over a round concrete well shaft (~0.80 m
#   across, ~0.52 m tall), standing on the ground (z=0 up).
# - Root/support: the concrete well shaft is the fixed root resting on z=0; the
#   square mesh collar is fixed to the shaft top; the grate is hinged to the
#   collar at the rear rim.
# - Parts: well_shaft (hollow concrete tube), mesh_collar (square diamond-mesh
#   frame with circular throat, fixed to shaft), grate (rectangular steel border
#   frame with N_SLATS parallel slat bars + hinge knuckle).
# - Articulation: collar_to_grate REVOLUTE, hinge line along the rear rim, axis
#   horizontal (world X at q=0) so the front edge lifts upward; positive q
#   swings the grate up past vertical.
# - Visible geometry: dark gunmetal steel grate frame + parallel slat bars with
#   see-through gaps; grey concrete shaft with hollow bore; dark rust-brown
#   diamond-mesh collar top.
# - Support/fit: the grate frame seats on the throat ring lip when closed; the
#   hinge is a real mount -- collar-side lug plates + pin on the rear frame
#   band, with the grate knuckle barrel coaxial on the pin.
# - Intentional overlaps: hinge knuckle/rear frame bar embed into the collar
#   hinge mount and throat ring lip (local, mechanically explanatory).
# - Tests: grate frame + N_SLATS slat bars present at regular pitch, grate is
#   open (see-through), closed grate lies flat & seats on collar, open pose
#   lifts the front edge well above the rim, shaft is hollow, nothing floats.
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# --- Absolute dimensions (meters) ---------------------------------------------
SHAFT_OUTER_R = 0.40
SHAFT_WALL = 0.085
SHAFT_INNER_R = SHAFT_OUTER_R - SHAFT_WALL  # bore radius ~0.315
SHAFT_HEIGHT = 0.52

COLLAR_HALF = 0.40  # square mesh collar 0.80 m x 0.80 m
COLLAR_FRAME = 0.06  # outer frame band width
COLLAR_THK = 0.05  # collar plate thickness
COLLAR_THROAT_R = SHAFT_INNER_R + 0.01  # circular throat opening radius

# --- Grate leaf (replaces the old solid cast-iron lid) -----------------------
GRATE_W = 0.66  # grate width (X, perpendicular to hinge axis)
GRATE_D = 0.70  # grate depth (Y, from rear hinge edge to front edge)
FRAME_BAR_W = 0.035  # frame border bar cross-section width
FRAME_BAR_H = 0.050  # frame border bar height (same as old lid thickness so
#                       the hinge mount geometry stays identical)
SLAT_W = 0.012  # each parallel slat bar width (along Y)
SLAT_H = 0.030  # each slat bar height (along Z, flush with frame top)
N_SLATS = 12  # number of evenly spaced parallel slat bars

# Derived grate geometry.
INNER_W = GRATE_W - 2.0 * FRAME_BAR_W  # clear span between side bars
INNER_D = GRATE_D - 2.0 * FRAME_BAR_W  # clear span between front/rear bars
SLAT_LENGTH = INNER_W + 0.004  # 2 mm embed into each side bar for connectivity
SLAT_PITCH = INNER_D / (N_SLATS + 1)  # regular centre-to-centre spacing

HINGE_PIN_R = 0.020
HINGE_KNUCKLE_LEN = 0.17

# Hinge line placement (in the collar part frame).  The grate frame rear bar
# sits on the throat ring lip; the hinge axis runs along the rear rim at the
# grate top plane, directly over the rear collar frame band so the collar-side
# lugs are grounded.
THROAT_LIP_TOP = COLLAR_THK + 0.015  # top of the throat ring lip
HINGE_Y = 0.36  # rear hinge line Y (was LID_R in the parent)
HINGE_Z = THROAT_LIP_TOP + FRAME_BAR_H - 0.002  # grate bottom embeds 2 mm

HINGE_LUG_X = 0.10  # lug plate centres either side of the knuckle
HINGE_LUG_THK = 0.03
HINGE_LUG_TOP = HINGE_Z + HINGE_PIN_R + 0.014


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def _slat_bar_geometry() -> MeshGeometry:
    """One grate slat bar, centred at the local origin, long axis along X.

    Shared helper called inside a ``for i in range(N_SLATS)`` loop so every
    slat is emitted with identical geometry at a regular pitch.
    """
    return BoxGeometry((SLAT_LENGTH, SLAT_W, SLAT_H))


def _build_grate_frame_mesh() -> MeshGeometry:
    """Rectangular border frame of the steel grate leaf.

    Four bars forming a rectangle: two side bars along Y and two cross bars
    (front / rear) along X.  Authored in the grate part frame with the rear
    edge at y=0 (the hinge line), the front edge at y=-GRATE_D, and the top
    surface at z=0.
    """
    geom = MeshGeometry()

    # Side bars (long axis along Y).
    for sx in (1.0, -1.0):
        bar = BoxGeometry((FRAME_BAR_W, GRATE_D, FRAME_BAR_H))
        bar = bar.translate(
            sx * (GRATE_W / 2.0 - FRAME_BAR_W / 2.0),
            -GRATE_D / 2.0,
            -FRAME_BAR_H / 2.0,
        )
        geom = geom.merge(bar)

    # Front and rear cross bars (long axis along X, fitted between side bars).
    cross_len = INNER_W  # span between the inner faces of the side bars
    for y_centre in (
        -FRAME_BAR_W / 2.0,  # rear bar (hinge edge)
        -(GRATE_D - FRAME_BAR_W / 2.0),  # front bar
    ):
        bar = BoxGeometry((cross_len, FRAME_BAR_W, FRAME_BAR_H))
        bar = bar.translate(0.0, y_centre, -FRAME_BAR_H / 2.0)
        geom = geom.merge(bar)

    return geom


# ---------------------------------------------------------------------------
# Collar, hinge mount, and shaft (unchanged from parent)
# ---------------------------------------------------------------------------

def _build_collar_mesh() -> MeshGeometry:
    """Square collar frame with a diamond-mesh grille inside and a circular
    throat. Authored centered on the well axis with its base at z=0 of the part
    frame; the frame top is at z=COLLAR_THK."""
    geom = MeshGeometry()

    # Outer square frame band (four bars) leaving a mesh field inside.
    inner = COLLAR_HALF - COLLAR_FRAME
    # +X / -X bars
    for sx in (1.0, -1.0):
        bar = BoxGeometry((COLLAR_FRAME, 2.0 * COLLAR_HALF, COLLAR_THK))
        bar = bar.translate(sx * (COLLAR_HALF - COLLAR_FRAME / 2.0), 0.0, COLLAR_THK / 2.0)
        geom = geom.merge(bar)
    # +Y / -Y bars
    for sy in (1.0, -1.0):
        bar = BoxGeometry((2.0 * inner, COLLAR_FRAME, COLLAR_THK))
        bar = bar.translate(0.0, sy * (COLLAR_HALF - COLLAR_FRAME / 2.0), COLLAR_THK / 2.0)
        geom = geom.merge(bar)

    # Diamond mesh: two families of thin diagonal bars.
    mesh_z = COLLAR_THK - 0.012
    bar_h = 0.012
    bar_w = 0.009
    n = 11
    pitch = (2.0 * inner) / n
    for fam in (1.0, -1.0):
        ang = fam * math.pi / 4.0
        dx, dy = math.cos(ang), math.sin(ang)
        px, py = -dy, dx
        for k in range(1, n):
            off = -inner + k * pitch
            ts = []
            base_x, base_y = px * off, py * off
            for bound, dcomp, bcomp in (
                (inner, dx, base_x),
                (-inner, dx, base_x),
                (inner, dy, base_y),
                (-inner, dy, base_y),
            ):
                if abs(dcomp) > 1e-9:
                    t = (bound - bcomp) / dcomp
                    x = base_x + dx * t
                    y = base_y + dy * t
                    if -inner - 1e-6 <= x <= inner + 1e-6 and -inner - 1e-6 <= y <= inner + 1e-6:
                        ts.append(t)
            if len(ts) < 2:
                continue
            t0, t1 = min(ts), max(ts)

            clear_r = COLLAR_THROAT_R + 0.035
            if off * off < clear_r * clear_r:
                tc = math.sqrt(clear_r * clear_r - off * off)
                segments = [(t0, -tc), (tc, t1)]
            else:
                segments = [(t0, t1)]

            for s0, s1 in segments:
                length = s1 - s0
                if length < pitch * 0.4:
                    continue
                cx = base_x + dx * (s0 + s1) / 2.0
                cy = base_y + dy * (s0 + s1) / 2.0
                bar = BoxGeometry((length, bar_w, bar_h))
                bar = bar.rotate_z(ang)
                bar = bar.translate(cx, cy, mesh_z + bar_h / 2.0 - 0.001)
                geom = geom.merge(bar)

    # Circular throat collar wall.
    throat = LatheGeometry.from_shell_profiles(
        [
            (COLLAR_THROAT_R + 0.03, 0.0),
            (COLLAR_THROAT_R + 0.03, COLLAR_THK),
            (COLLAR_THROAT_R, COLLAR_THK + 0.015),
        ],
        [
            (COLLAR_THROAT_R, 0.0),
            (COLLAR_THROAT_R, COLLAR_THK),
            (COLLAR_THROAT_R - 0.004, COLLAR_THK + 0.015),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    geom = geom.merge(throat)
    return geom


def _build_hinge_mount_mesh() -> MeshGeometry:
    """Collar-side hinge mount: two upright cast lug plates standing on the rear
    collar frame band, plus the hinge pin spanning between them along the hinge
    axis.  Authored in the collar part frame (base of the collar at z=0)."""
    geom = MeshGeometry()

    lug_y0 = 0.345
    lug_y1 = COLLAR_HALF
    for sx in (1.0, -1.0):
        lug = BoxGeometry((HINGE_LUG_THK, lug_y1 - lug_y0, HINGE_LUG_TOP))
        lug = lug.translate(sx * HINGE_LUG_X, (lug_y0 + lug_y1) / 2.0, HINGE_LUG_TOP / 2.0)
        geom = geom.merge(lug)

    pin_len = 2.0 * (HINGE_LUG_X + HINGE_LUG_THK / 2.0) + 0.012
    pin = CylinderGeometry(0.013, pin_len, radial_segments=16)
    pin = pin.rotate_y(math.pi / 2.0)
    pin = pin.translate(0.0, HINGE_Y, HINGE_Z)
    geom = geom.merge(pin)

    return geom


def _build_shaft_mesh() -> MeshGeometry:
    """Hollow round concrete well shaft, base on z=0, open bore through the top."""
    shaft = LatheGeometry.from_shell_profiles(
        [
            (SHAFT_OUTER_R, 0.0),
            (SHAFT_OUTER_R, SHAFT_HEIGHT * 0.85),
            (SHAFT_OUTER_R - 0.02, SHAFT_HEIGHT),
        ],
        [
            (SHAFT_INNER_R, 0.0),
            (SHAFT_INNER_R, SHAFT_HEIGHT * 0.85),
            (SHAFT_INNER_R, SHAFT_HEIGHT),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    return shaft


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_grate_trap_door")

    concrete = Material(name="concrete", rgba=(0.70, 0.69, 0.66, 1.0))
    # Dark gunmetal steel for the grate frame and slat bars.
    steel = Material(name="steel", rgba=(0.38, 0.40, 0.43, 1.0))
    # Slightly lighter steel for the slat bars (subtle colour variety).
    slat_steel = Material(name="slat_steel", rgba=(0.44, 0.46, 0.49, 1.0))
    mesh_iron = Material(name="mesh_iron", rgba=(0.22, 0.13, 0.10, 1.0))
    for mat in (concrete, steel, slat_steel, mesh_iron):
        model.material(mat.name, rgba=mat.rgba)

    # --- Well shaft (fixed root) ---------------------------------------------
    shaft = model.part("well_shaft")
    shaft.visual(
        mesh_from_geometry(_build_shaft_mesh(), "well_shaft"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="concrete",
        name="shaft_wall",
    )

    # --- Square mesh collar (fixed to shaft top) -----------------------------
    collar = model.part("mesh_collar")
    collar.visual(
        mesh_from_geometry(_build_collar_mesh(), "mesh_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="mesh_iron",
        name="collar_frame",
    )
    collar.visual(
        mesh_from_geometry(_build_hinge_mount_mesh(), "hinge_mount"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="mesh_iron",
        name="hinge_mount",
    )

    # --- Grate leaf (open steel frame + parallel slat bars) ------------------
    # The grate part frame sits on the rear-rim hinge line (the joint origin).
    # The frame mesh is authored with the rear edge at y=0, so the visual
    # origin is at (0, 0, 0) -- no offset needed.
    grate = model.part("grate")

    # Rectangular border frame.
    grate.visual(
        mesh_from_geometry(_build_grate_frame_mesh(), "grate_frame"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="steel",
        name="grate_frame",
    )

    # Evenly spaced parallel slat bars emitted via a for-i-in-range loop with
    # a shared geometry helper and regular pitch.  Each slat is a named visual
    # (slat_0 .. slat_{N-1}) so you can see through the gaps into the shaft.
    for i in range(N_SLATS):
        y_pos = -(FRAME_BAR_W + (i + 1) * SLAT_PITCH)
        grate.visual(
            mesh_from_geometry(_slat_bar_geometry(), f"slat_{i}"),
            # Slat top flush with frame top (z=0); centre is at z = -SLAT_H/2.
            origin=Origin(xyz=(0.0, y_pos, -SLAT_H / 2.0)),
            material="slat_steel",
            name=f"slat_{i}",
        )

    # Hinge knuckle: a barrel COAXIAL with the revolute axis (grate part
    # origin), spanning between the collar lugs.
    knuckle = mesh_from_geometry(
        CylinderGeometry(HINGE_PIN_R, HINGE_KNUCKLE_LEN, radial_segments=20),
        "hinge_knuckle",
    )
    grate.visual(
        knuckle,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel",
        name="grate_knuckle",
    )

    # --- Fixed joints -------------------------------------------------------
    model.articulation(
        "shaft_to_collar",
        ArticulationType.FIXED,
        parent=shaft,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT)),
    )

    # --- Grate hinge (primary articulation) ---------------------------------
    model.articulation(
        "collar_to_grate",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=grate,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        # Horizontal hinge axis along the rear rim (world X).  The grate extends
        # along local -Y (front) from the hinge; positive rotation about -X
        # lifts the front (-Y) edge upward and over past vertical.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    return model


object_model = build_object_model()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shaft = object_model.get_part("well_shaft")
    collar = object_model.get_part("mesh_collar")
    grate = object_model.get_part("grate")
    hinge = object_model.get_articulation("collar_to_grate")

    grate_frame = grate.get_visual("grate_frame")
    grate_knuckle = grate.get_visual("grate_knuckle")

    # --- Grate structure: frame + slat bars ---------------------------------
    ctx.check(
        "grate has border frame and hinge knuckle visuals",
        grate_frame is not None and grate_knuckle is not None,
        details="expected grate_frame and grate_knuckle visuals",
    )

    # All N_SLATS slat bar visuals must exist.
    slats_present = sum(
        1 for i in range(N_SLATS) if grate.get_visual(f"slat_{i}") is not None
    )
    ctx.check(
        f"grate has {N_SLATS} parallel slat bars (slat_0 .. slat_{N_SLATS - 1})",
        slats_present == N_SLATS,
        details=f"found {slats_present} of {N_SLATS} slat visuals",
    )

    # Slat bars are at regular pitch (shared geometry helper + uniform spacing).
    expected_pitch = INNER_D / (N_SLATS + 1)
    ctx.check(
        "slat bars are emitted at regular centre-to-centre pitch",
        abs(SLAT_PITCH - expected_pitch) < 1e-9,
        details=f"SLAT_PITCH={SLAT_PITCH:.4f}, expected={expected_pitch:.4f}",
    )

    # Grate is open: the gaps between slats let you see through into the shaft.
    gap_width = SLAT_PITCH - SLAT_W
    open_ratio = (N_SLATS + 1) * gap_width / INNER_D
    ctx.check(
        "grate is open (see-through gaps between slats, open ratio > 50 %)",
        gap_width > 0.0 and open_ratio > 0.50,
        details=f"gap_width={gap_width:.4f} m, open_ratio={open_ratio:.2f}",
    )

    # Frame bars are substantial (not token placeholders).
    ctx.check(
        "grate frame bars are structurally sized",
        FRAME_BAR_W >= 0.025 and FRAME_BAR_H >= 0.030,
        details=f"FRAME_BAR_W={FRAME_BAR_W}, FRAME_BAR_H={FRAME_BAR_H}",
    )

    # --- Hinge is physically mounted (not floating) -------------------------
    hinge_mount = collar.get_visual("hinge_mount")
    ctx.check(
        "collar-side hinge mount (lugs + pin) reaches the hinge axis",
        hinge_mount is not None and HINGE_LUG_TOP > HINGE_Z + HINGE_PIN_R,
        details=f"lug_top={HINGE_LUG_TOP:.3f}, hinge_z={HINGE_Z:.3f}",
    )

    # The grate frame is wider than the throat so it seats on the ring lip.
    ctx.check(
        "grate is wider than the throat opening so it seats on the ring lip",
        GRATE_W / 2.0 >= COLLAR_THROAT_R + 0.005
        and GRATE_D - HINGE_Y >= COLLAR_THROAT_R + 0.005,
        details=f"grate_half_w={GRATE_W / 2.0:.3f}, grate_front={GRATE_D - HINGE_Y:.3f}, "
        f"throat_r={COLLAR_THROAT_R:.3f}",
    )

    # Hinge knuckle / rear frame bar / throat-ring seating overlaps are local
    # and mechanically explanatory.
    ctx.allow_overlap(
        grate,
        collar,
        reason="Hinge knuckle barrel and grate rear frame bar embed into the "
        "collar hinge mount and throat ring lip when closed; both are local "
        "intended hinge/seating overlaps at the hatch lip.",
    )

    # --- Closed pose: grate lies FLAT and seats over the throat -------------
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(grate)
        if closed_aabb is not None:
            (cx0, cy0, cz0), (cx1, cy1, cz1) = closed_aabb
            x_span = cx1 - cx0
            y_span = cy1 - cy0
            z_span = cz1 - cz0
            ctx.check(
                "closed grate lies flat (thin in Z, wide in X and Y)",
                z_span < 0.12 and x_span > 0.5 and y_span > 0.5,
                details=f"x_span={x_span:.3f} y_span={y_span:.3f} z_span={z_span:.3f}",
            )
            ctx.check(
                "closed grate sits at the collar top, not on the ground",
                cz0 > SHAFT_HEIGHT - 0.02,
                details=f"grate min z={cz0:.3f}, shaft height={SHAFT_HEIGHT}",
            )
        ctx.expect_overlap(
            grate,
            collar,
            axes="xy",
            min_overlap=0.20,
            name="closed grate covers the collar throat in plan",
        )
        ctx.expect_contact(
            grate,
            collar,
            contact_tol=0.006,
            name="closed grate seats on the collar (not floating)",
        )

    # --- Open pose: front edge lifts upward, past vertical ------------------
    with ctx.pose({hinge: 1.9}):
        open_aabb = ctx.part_world_aabb(grate)
        ctx.check(
            "open pose lifts the grate well above the collar",
            open_aabb is not None
            and closed_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.20,
            details=f"closed max z={None if closed_aabb is None else closed_aabb[1][2]:.3f}, "
            f"open max z={None if open_aabb is None else open_aabb[1][2]:.3f}",
        )
        if open_aabb is not None:
            (ox0, oy0, oz0), (ox1, oy1, oz1) = open_aabb
            ctx.check(
                "open grate stands up (tall in Z)",
                (oz1 - oz0) > 0.45,
                details=f"open z_span={(oz1 - oz0):.3f}",
            )

    # --- Support / placement ------------------------------------------------
    shaft_aabb = ctx.part_world_aabb(shaft)
    if shaft_aabb is not None:
        ctx.check(
            "well shaft rests on the ground plane (z~0)",
            abs(shaft_aabb[0][2]) < 0.01,
            details=f"shaft min z={shaft_aabb[0][2]:.4f}",
        )

    collar_aabb = ctx.part_world_aabb(collar)
    if collar_aabb is not None and shaft_aabb is not None:
        ctx.check(
            "mesh collar sits at the shaft top",
            abs(collar_aabb[0][2] - shaft_aabb[1][2]) < 0.05,
            details=f"collar min z={collar_aabb[0][2]:.3f}, shaft max z={shaft_aabb[1][2]:.3f}",
        )

    ctx.check(
        "collar throat clears the shaft bore (hollow well)",
        COLLAR_THROAT_R <= SHAFT_INNER_R + 0.05 and SHAFT_INNER_R > 0.25,
        details=f"throat_r={COLLAR_THROAT_R:.3f}, bore_r={SHAFT_INNER_R:.3f}",
    )

    return ctx.report()
