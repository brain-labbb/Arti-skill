from __future__ import annotations

# Square plank-built timber trap door leaf hinged at the rear, sitting on a
# square metal-mesh collar that caps a round concrete / stone well shaft.
#
# Fork of the round cast-iron hatch: the disc is replaced by a square timber
# leaf built from 9 narrow wooden planks laid edge to edge, banded by two
# cross battens on top. The hinge, collar, and shaft remain unchanged.
#
# Articraft brief:
# - Object: a square timber access-hatch leaf ~0.72 m per side, on a square
#   mesh collar (0.80 m x 0.80 m) over a round concrete well shaft ~0.80 m
#   across and ~0.52 m tall, standing on the ground (z=0 up).
# - Root/support: the concrete well shaft is the fixed root resting on z=0;
#   the square mesh collar is fixed to the shaft top; the leaf is hinged to
#   the collar at the rear rim.
# - Parts: well_shaft (hollow concrete tube), mesh_collar (square diamond-mesh
#   frame with circular throat, fixed to shaft), lid (square timber leaf: 9
#   planks + 2 battens + hinge knuckle).
# - Articulation: collar_to_lid REVOLUTE, hinge line along the rear rim, axis
#   horizontal (world X at q=0) so the front edge lifts upward; positive q
#   swings the leaf up past vertical.
# - Visible geometry: warm-brown timber planks with slight color variety,
#   darker cross battens on top, cast-iron hinge knuckle at the rear; grey
#   concrete shaft; dark rust-brown diamond-mesh collar.
# - Support/fit: the leaf bottom seats on the throat ring lip when closed; the
#   hinge is a real mount -- collar-side lug plates + pin, with the lid
#   knuckle barrel coaxial on the pin.
# - Intentional overlaps: hinge knuckle/pin/lugs at the rear edge embed into
#   the collar (local, mechanically explanatory); the closed leaf bottom
#   embeds ~2mm into the ring lip seat.
# - Tests: 9 planks + 2 battens present, leaf lies flat & seats on the collar
#   when closed, open pose lifts the front edge well above the rim, shaft is
#   hollow, nothing floats.
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

# --- Timber leaf dimensions ---------------------------------------------------
LEAF_SIDE = 0.72  # square leaf side length
N_PLANKS = 9  # exactly 9 narrow planks
PLANK_GAP = 0.002  # small gap between planks for visual realism
PLANK_W = (LEAF_SIDE - (N_PLANKS - 1) * PLANK_GAP) / N_PLANKS  # ~0.0782
PLANK_PITCH = PLANK_W + PLANK_GAP  # regular center-to-center pitch
PLANK_LEN = LEAF_SIDE  # planks span the full leaf depth
PLANK_THK = 0.040  # plank board thickness

N_BATTENS = 2
BATTEN_LEN = LEAF_SIDE  # battens span the full leaf width
BATTEN_W = 0.060  # batten width (in the plank-length direction)
BATTEN_THK = 0.025  # batten thickness (stands proud on top of planks)
# Batten Y centers in the lid frame: at 25% and 75% of the leaf depth from the
# hinge (rear edge at y=0, front edge at y=-LEAF_SIDE).
BATTEN_Y_FRAC = (0.25, 0.75)

HINGE_PIN_R = 0.020
HINGE_KNUCKLE_LEN = 0.17

# Hinge line placement (in the collar part frame). The leaf bottom rests on the
# throat ring lip; the hinge axis runs along the rear edge at the plank top
# plane, directly over the rear collar frame band so the collar-side lugs are
# grounded.
THROAT_LIP_TOP = COLLAR_THK + 0.015  # top of the throat ring lip
HINGE_Y = LEAF_SIDE / 2.0  # rear edge of the square leaf
HINGE_Z = THROAT_LIP_TOP + PLANK_THK - 0.002  # plank bottom embeds 2mm into lip

HINGE_LUG_X = 0.10  # lug plate centers either side of the knuckle
HINGE_LUG_THK = 0.03
HINGE_LUG_TOP = HINGE_Z + HINGE_PIN_R + 0.014


def _board_geometry(width: float, length: float, thickness: float) -> MeshGeometry:
    """Shared board geometry helper: a rectangular plank / batten mesh."""
    return BoxGeometry((width, length, thickness))


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

    # Diamond mesh: two families of thin diagonal bars (the expanded-metal /
    # diamond grille look). Each bar is clipped to the chord it spans inside the
    # inner square so no bar pokes past the frame.
    mesh_z = COLLAR_THK - 0.012
    bar_h = 0.012
    bar_w = 0.009
    n = 11
    pitch = (2.0 * inner) / n
    for fam in (1.0, -1.0):  # +45 and -45 families => diamond pattern
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

            # Carve the circular throat clear so the hatch opening is a real hole.
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

    # Circular throat collar wall: a short ring around the opening tying the
    # mesh field down to the shaft bore and giving the leaf a seat.
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
    axis. Authored in the collar part frame (base of the collar at z=0)."""
    geom = MeshGeometry()

    # Lug plates sit fully on the rear frame band (y in [0.345, COLLAR_HALF]),
    # behind the leaf rim so the closed timber only kisses their front faces.
    lug_y0 = 0.345
    lug_y1 = COLLAR_HALF
    for sx in (1.0, -1.0):
        lug = BoxGeometry((HINGE_LUG_THK, lug_y1 - lug_y0, HINGE_LUG_TOP))
        lug = lug.translate(sx * HINGE_LUG_X, (lug_y0 + lug_y1) / 2.0, HINGE_LUG_TOP / 2.0)
        geom = geom.merge(lug)

    # Hinge pin along the axis (world X), passing through both lugs and the
    # leaf knuckle; the ends stand slightly proud as pin heads.
    pin_len = 2.0 * (HINGE_LUG_X + HINGE_LUG_THK / 2.0) + 0.012
    pin = CylinderGeometry(0.013, pin_len, radial_segments=16)
    pin = pin.rotate_y(math.pi / 2.0)  # cylinder long axis Z -> X (hinge axis)
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_trap_door")

    concrete = Material(name="concrete", rgba=(0.70, 0.69, 0.66, 1.0))
    cast_iron = Material(name="cast_iron", rgba=(0.38, 0.34, 0.30, 1.0))
    mesh_iron = Material(name="mesh_iron", rgba=(0.22, 0.13, 0.10, 1.0))
    # Timber materials for the plank leaf with slight variety.
    timber_light = Material(name="timber_light", rgba=(0.62, 0.44, 0.25, 1.0))
    timber_mid = Material(name="timber_mid", rgba=(0.56, 0.39, 0.22, 1.0))
    timber_dark = Material(name="timber_dark", rgba=(0.50, 0.34, 0.19, 1.0))
    batten_wood = Material(name="batten_wood", rgba=(0.40, 0.27, 0.15, 1.0))
    for mat in (concrete, cast_iron, mesh_iron, timber_light, timber_mid, timber_dark, batten_wood):
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

    # --- Timber leaf (9 planks + 2 battens + knuckle) -----------------------
    # The lid part frame sits on the rear-edge hinge line. Planks extend forward
    # (-Y) from the hinge, centered in X. Plank tops are at z=0 (hinge axis);
    # battens sit on top (z > 0).
    lid = model.part("lid")

    # Cycle through timber shades for visual variety across planks.
    timber_shades = ("timber_light", "timber_mid", "timber_dark")

    for i in range(N_PLANKS):
        plank_x = -LEAF_SIDE / 2.0 + PLANK_W / 2.0 + i * PLANK_PITCH
        lid.visual(
            mesh_from_geometry(
                _board_geometry(PLANK_W, PLANK_LEN, PLANK_THK),
                f"plank_{i}",
            ),
            origin=Origin(xyz=(plank_x, -LEAF_SIDE / 2.0, -PLANK_THK / 2.0)),
            material=timber_shades[i % len(timber_shades)],
            name=f"plank_{i}",
        )

    # Two cross battens on top of the planks, running in X (perpendicular to
    # the plank grain).
    for i in range(N_BATTENS):
        batten_y = -LEAF_SIDE * BATTEN_Y_FRAC[i]
        lid.visual(
            mesh_from_geometry(
                _board_geometry(BATTEN_LEN, BATTEN_W, BATTEN_THK),
                f"batten_{i}",
            ),
            origin=Origin(xyz=(0.0, batten_y, BATTEN_THK / 2.0)),
            material="batten_wood",
            name=f"batten_{i}",
        )

    # Hinge knuckle: a barrel COAXIAL with the revolute axis (lid part origin),
    # spanning between the collar lugs. Represents the iron hinge barrel that
    # wraps around the collar-side pin.
    knuckle = mesh_from_geometry(
        CylinderGeometry(HINGE_PIN_R, HINGE_KNUCKLE_LEN, radial_segments=20),
        "hinge_knuckle",
    )
    lid.visual(
        knuckle,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="cast_iron",
        name="lid_knuckle",
    )

    # --- Fixed joints -------------------------------------------------------
    model.articulation(
        "shaft_to_collar",
        ArticulationType.FIXED,
        parent=shaft,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT)),
    )

    # --- Leaf hinge (primary articulation) -----------------------------------
    # Hinge line along the rear edge of the leaf, at the collar top plane.
    # The lid part frame is placed at the hinge; planks extend forward (-Y).
    # axis=(-1, 0, 0): positive rotation lifts the front (-Y) edge upward.
    model.articulation(
        "collar_to_lid",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shaft = object_model.get_part("well_shaft")
    collar = object_model.get_part("mesh_collar")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("collar_to_lid")

    # --- Hero geometry: 9 planks + 2 battens + knuckle ----------------------
    planks = [lid.get_visual(f"plank_{i}") for i in range(N_PLANKS)]
    ctx.check(
        "leaf has exactly 9 timber planks built by loop",
        all(p is not None for p in planks),
        details=f"found {sum(1 for p in planks if p is not None)} of {N_PLANKS} planks",
    )

    battens = [lid.get_visual(f"batten_{i}") for i in range(N_BATTENS)]
    ctx.check(
        "leaf has exactly 2 cross battens",
        all(b is not None for b in battens),
        details=f"found {sum(1 for b in battens if b is not None)} of {N_BATTENS} battens",
    )

    lid_knuckle = lid.get_visual("lid_knuckle")
    hinge_mount = collar.get_visual("hinge_mount")
    ctx.check(
        "hinge knuckle present on the timber leaf",
        lid_knuckle is not None,
        details="expected lid_knuckle visual",
    )

    # Planks are narrow boards laid at regular pitch.
    ctx.check(
        "planks are narrow boards (width < 0.10 m, > 0.04 m)",
        0.04 < PLANK_W < 0.10,
        details=f"plank_w={PLANK_W:.4f}",
    )
    ctx.check(
        "planks laid at regular pitch with small gaps",
        N_PLANKS == 9
        and abs(N_PLANKS * PLANK_W + (N_PLANKS - 1) * PLANK_GAP - LEAF_SIDE) < 1e-6,
        details=f"9*{PLANK_W:.4f} + 8*{PLANK_GAP} = {N_PLANKS * PLANK_W + (N_PLANKS - 1) * PLANK_GAP:.4f} "
        f"vs leaf_side={LEAF_SIDE}",
    )

    # Battens span the leaf width and sit on top.
    ctx.check(
        "battens span the full leaf width and stand proud on top",
        BATTEN_LEN >= LEAF_SIDE - 0.01 and BATTEN_THK > 0.01,
        details=f"batten_len={BATTEN_LEN:.3f}, batten_thk={BATTEN_THK:.3f}",
    )

    # --- Hinge is physically mounted (not floating) ---------------------------
    ctx.check(
        "collar-side hinge mount (lugs + pin) reaches the hinge axis",
        hinge_mount is not None and HINGE_LUG_TOP > HINGE_Z + HINGE_PIN_R,
        details=f"lug_top={HINGE_LUG_TOP:.3f}, hinge_z={HINGE_Z:.3f}",
    )
    ctx.check(
        "square leaf is wider than the throat opening so it seats on the ring lip",
        LEAF_SIDE / 2.0 >= COLLAR_THROAT_R + 0.01,
        details=f"leaf_half={LEAF_SIDE / 2.0:.3f}, throat_r={COLLAR_THROAT_R:.3f}",
    )

    # Seating and hinge overlaps: local, intended.
    ctx.allow_overlap(
        lid,
        collar,
        reason="Timber leaf bottom embeds ~2mm into the throat ring lip when "
        "closed (seated fit), and the hinge knuckle wraps around the collar-side "
        "pin at the rear edge; both are local intended seated/hinge overlaps.",
    )

    # --- Closed pose: leaf lies FLAT and seats over the throat ----------------
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(lid)
        if closed_aabb is not None:
            (cx0, cy0, cz0), (cx1, cy1, cz1) = closed_aabb
            x_span = cx1 - cx0
            y_span = cy1 - cy0
            z_span = cz1 - cz0
            ctx.check(
                "closed leaf lies flat (thin in Z, wide in X and Y)",
                z_span < 0.12 and x_span > 0.5 and y_span > 0.5,
                details=f"x_span={x_span:.3f} y_span={y_span:.3f} z_span={z_span:.3f}",
            )
            ctx.check(
                "closed leaf sits at the collar top, not on the ground",
                cz0 > SHAFT_HEIGHT - 0.02,
                details=f"lid min z={cz0:.3f}, shaft height={SHAFT_HEIGHT}",
            )
        ctx.expect_overlap(
            lid,
            collar,
            axes="xy",
            min_overlap=0.20,
            name="closed leaf covers the collar throat in plan",
        )
        ctx.expect_contact(
            lid,
            collar,
            contact_tol=0.006,
            name="closed leaf seats on the collar (not floating)",
        )

    closed_front = ctx.part_world_aabb(lid)

    # --- Open pose: front edge lifts upward, past vertical -------------------
    with ctx.pose({hinge: 1.9}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "open pose lifts the leaf well above the collar",
            open_aabb is not None
            and closed_front is not None
            and open_aabb[1][2] > closed_front[1][2] + 0.20,
            details=f"closed max z={None if closed_front is None else closed_front[1][2]:.3f}, "
            f"open max z={None if open_aabb is None else open_aabb[1][2]:.3f}",
        )
        if open_aabb is not None:
            (ox0, oy0, oz0), (ox1, oy1, oz1) = open_aabb
            ctx.check(
                "open leaf stands up (tall in Z)",
                (oz1 - oz0) > 0.45,
                details=f"open z_span={(oz1 - oz0):.3f}",
            )

    # --- Support / placement -------------------------------------------------
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
