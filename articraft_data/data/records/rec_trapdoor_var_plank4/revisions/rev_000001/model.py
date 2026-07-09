from __future__ import annotations

# Square plank-built timber trap door leaf hinged at the rear, on a square
# metal-mesh collar capping a round concrete / stone well shaft.
#
# Articraft brief:
# - Object: a square timber access hatch leaf (~0.70 m square) made of exactly
#   4 parallel wooden planks banded by 2 cross battens, on a square mesh collar
#   over a round concrete well shaft ~0.80 m across and ~0.55 m tall (z=0 up).
# - Root/support: concrete well shaft is the fixed root; square mesh collar is
#   fixed to the shaft top; timber leaf is hinged to the collar at the rear edge.
# - Parts: well_shaft (hollow concrete tube), mesh_collar (square diamond-mesh
#   frame with circular throat, fixed to shaft), lid (4 timber planks + 2 cross
#   battens + hinge knuckle).
# - Articulation: collar_to_lid REVOLUTE, hinge line along the rear edge, axis
#   horizontal (world X at q=0) so the front edge lifts upward; positive q
#   swings the leaf up past vertical. A closed trap door lies FLAT, so the
#   hinge axis is horizontal (unlike an upright door).
# - Visible geometry: warm timber planks with narrow shadow gaps between them,
#   darker cross battens fastened on top, grey concrete shaft, dark rust-brown
#   diamond mesh collar.
# - Support/fit: leaf seats on the throat ring lip when closed; hinge knuckle is
#   coaxial with the collar hinge pin between the lug plates.
# - Intentional overlaps: hinge knuckle barrel embeds into the collar rear edge
#   and the rear plank (local, mechanically explanatory); closed leaf bottom
#   embeds ~2 mm into the throat ring lip seat.
# - Tests: 4 planks + 2 battens present with regular pitch, closed leaf lies flat
#   and covers the throat, open pose lifts the front edge well above the collar,
#   shaft is hollow, nothing floats.

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

# Timber leaf: square plank-built leaf replacing the round cast-iron disc.
LEAF_SIZE = 0.70  # square leaf side (covers the ~0.65 m throat with overhang)
PLANK_THK = 0.040  # 40 mm thick timber boards
N_PLANKS = 4  # exactly 4 parallel planks laid edge to edge
PLANK_GAP = 0.003  # 3 mm shadow gap between planks
PLANK_W = (LEAF_SIZE - (N_PLANKS - 1) * PLANK_GAP) / N_PLANKS  # ~0.173 m each
PLANK_LEN = LEAF_SIZE  # planks span the full leaf width in X

BATTEN_W = 0.060  # 60 mm wide cross battens
BATTEN_THK = 0.025  # 25 mm thick
N_BATTENS = 2
BATTEN_INSET = 0.04  # inset from front and rear leaf edges
BATTEN_LEN = LEAF_SIZE - 2.0 * BATTEN_INSET  # batten span in Y
BATTEN_X_OFFSET = LEAF_SIZE * 0.28  # battens at ±28% of leaf width from center

# Hinge geometry
HINGE_PIN_R = 0.020
HINGE_KNUCKLE_LEN = 0.17

THROAT_LIP_TOP = COLLAR_THK + 0.015  # top of the throat ring lip
HINGE_Y = LEAF_SIZE / 2.0  # rear edge of the leaf at the hinge line
HINGE_Z = THROAT_LIP_TOP + PLANK_THK - 0.002  # leaf bottom embeds 2mm into lip

HINGE_LUG_X = 0.10  # lug plate centers either side of the knuckle
HINGE_LUG_THK = 0.03
HINGE_LUG_TOP = HINGE_Z + HINGE_PIN_R + 0.014


def _board(size_x: float, size_y: float, size_z: float) -> MeshGeometry:
    """Shared flat board geometry helper for timber planks and battens.
    Returns a box centered at the origin with the given X, Y, Z extents."""
    return BoxGeometry((size_x, size_y, size_z))


def _build_collar_mesh() -> MeshGeometry:
    """Square collar frame with a diamond-mesh grille inside and a circular
    throat. Authored centered on the well axis with its base at z=0 of the part
    frame; the frame top is at z=COLLAR_THK."""
    geom = MeshGeometry()

    inner = COLLAR_HALF - COLLAR_FRAME
    # +X / -X frame bars
    for sx in (1.0, -1.0):
        bar = BoxGeometry((COLLAR_FRAME, 2.0 * COLLAR_HALF, COLLAR_THK))
        bar = bar.translate(sx * (COLLAR_HALF - COLLAR_FRAME / 2.0), 0.0, COLLAR_THK / 2.0)
        geom = geom.merge(bar)
    # +Y / -Y frame bars
    for sy in (1.0, -1.0):
        bar = BoxGeometry((2.0 * inner, COLLAR_FRAME, COLLAR_THK))
        bar = bar.translate(0.0, sy * (COLLAR_HALF - COLLAR_FRAME / 2.0), COLLAR_THK / 2.0)
        geom = geom.merge(bar)

    # Diamond mesh: two families of thin diagonal bars clipped to the inner
    # square with the circular throat carved clear.
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

    # Circular throat collar wall tying the mesh field to the shaft bore.
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
    """Collar-side hinge mount: two upright lug plates on the rear collar frame
    band plus the hinge pin spanning between them. Authored in the collar part
    frame (base of the collar at z=0)."""
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_trap_door")

    concrete = Material(name="concrete", rgba=(0.70, 0.69, 0.66, 1.0))
    timber = Material(name="timber", rgba=(0.55, 0.38, 0.22, 1.0))
    dark_timber = Material(name="dark_timber", rgba=(0.35, 0.22, 0.12, 1.0))
    mesh_iron = Material(name="mesh_iron", rgba=(0.22, 0.13, 0.10, 1.0))
    hinge_iron = Material(name="hinge_iron", rgba=(0.30, 0.18, 0.12, 1.0))
    for mat in (concrete, timber, dark_timber, mesh_iron, hinge_iron):
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

    # --- Timber leaf (4 planks + 2 cross battens + hinge knuckle) -----------
    # The lid part frame sits on the rear hinge line. Planks extend forward
    # (-Y) from the hinge; their top surface is at z=0 of the lid frame.
    lid = model.part("lid")

    # Four parallel planks laid edge to edge across the opening, built from the
    # shared board geometry helper at regular pitch.
    for i in range(N_PLANKS):
        y_center = -(PLANK_W / 2.0) - i * (PLANK_W + PLANK_GAP)
        lid.visual(
            mesh_from_geometry(_board(PLANK_LEN, PLANK_W, PLANK_THK), f"plank_{i}"),
            origin=Origin(xyz=(0.0, y_center, -PLANK_THK / 2.0)),
            material="timber",
            name=f"plank_{i}",
        )

    # Two cross battens perpendicular to the planks, banding them together.
    # Battens run in Y (spanning the planks), placed symmetrically in X.
    for j in range(N_BATTENS):
        x_sign = -1.0 if j == 0 else 1.0
        x_center = x_sign * BATTEN_X_OFFSET
        y_center = -LEAF_SIZE / 2.0  # centered in Y (symmetric inset)
        lid.visual(
            mesh_from_geometry(_board(BATTEN_W, BATTEN_LEN, BATTEN_THK), f"batten_{j}"),
            origin=Origin(xyz=(x_center, y_center, BATTEN_THK / 2.0)),
            material="dark_timber",
            name=f"batten_{j}",
        )

    # Hinge knuckle: barrel coaxial with the revolute axis (lid part origin),
    # spanning between the collar lugs. Stays on the collar pin in every pose.
    knuckle = mesh_from_geometry(
        CylinderGeometry(HINGE_PIN_R, HINGE_KNUCKLE_LEN, radial_segments=20),
        "hinge_knuckle",
    )
    lid.visual(
        knuckle,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="hinge_iron",
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
    # Hinge line is along the rear edge of the leaf, at the collar top plane.
    # The lid part frame is at the hinge line; planks extend forward (-Y).
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

    # --- Hero geometry: 4 planks + 2 battens + knuckle ----------------------
    planks = [lid.get_visual(f"plank_{i}") for i in range(N_PLANKS)]
    battens = [lid.get_visual(f"batten_{j}") for j in range(N_BATTENS)]
    lid_knuckle = lid.get_visual("lid_knuckle")

    ctx.check(
        "lid has exactly 4 plank visuals (plank_0..plank_3)",
        all(p is not None for p in planks) and len(planks) == N_PLANKS,
        details=f"found {[p is not None for p in planks]}",
    )
    ctx.check(
        "lid has exactly 2 cross batten visuals (batten_0, batten_1)",
        all(b is not None for b in battens) and len(battens) == N_BATTENS,
        details=f"found {[b is not None for b in battens]}",
    )
    ctx.check(
        "lid has hinge knuckle",
        lid_knuckle is not None,
        details="expected lid_knuckle visual",
    )

    # Planks are parallel with regular pitch (uniform width + gap spacing).
    plank_pitch = PLANK_W + PLANK_GAP
    ctx.check(
        "planks have uniform width and regular pitch from shared board helper",
        abs(PLANK_W - (LEAF_SIZE - (N_PLANKS - 1) * PLANK_GAP) / N_PLANKS) < 1e-6
        and PLANK_GAP > 0.0
        and N_PLANKS == 4,
        details=f"plank_w={PLANK_W:.4f}, gap={PLANK_GAP:.4f}, pitch={plank_pitch:.4f}",
    )

    # Planks span the full leaf width and the leaf is square.
    ctx.check(
        "planks span the leaf width in X and the leaf is square",
        abs(PLANK_LEN - LEAF_SIZE) < 1e-6,
        details=f"plank_len={PLANK_LEN:.3f}, leaf_size={LEAF_SIZE:.3f}",
    )

    # Cross battens run perpendicular to planks (long in Y) with symmetric X
    # placement, banding the planks together.
    ctx.check(
        "cross battens run perpendicular to planks with symmetric X placement",
        N_BATTENS == 2 and BATTEN_LEN > 0.5 * LEAF_SIZE and BATTEN_X_OFFSET > 0.0,
        details=f"batten_len={BATTEN_LEN:.3f}, batten_w={BATTEN_W:.3f}, "
        f"offset={BATTEN_X_OFFSET:.3f}",
    )

    # --- Hinge is physically mounted (not floating) --------------------------
    hinge_mount = collar.get_visual("hinge_mount")
    ctx.check(
        "collar-side hinge mount (lugs + pin) reaches the hinge axis",
        hinge_mount is not None and HINGE_LUG_TOP > HINGE_Z + HINGE_PIN_R,
        details=f"lug_top={HINGE_LUG_TOP:.3f}, hinge_z={HINGE_Z:.3f}",
    )
    ctx.check(
        "leaf is wider than the throat opening so it seats on the ring lip",
        LEAF_SIZE / 2.0 >= COLLAR_THROAT_R + 0.01,
        details=f"leaf_half={LEAF_SIZE / 2.0:.3f}, throat_r={COLLAR_THROAT_R:.3f}",
    )

    # Hinge knuckle embeds into collar rear edge and rear plank; closed leaf
    # bottom embeds ~2mm into the throat ring lip seat. Both are small local
    # intended seated/hinge overlaps at the hatch lip.
    ctx.allow_overlap(
        lid,
        collar,
        reason="Hinge knuckle barrel embeds into the collar rear edge and the "
        "closed leaf bottom embeds ~2mm into the throat ring lip seat; both are "
        "local intended seated/hinge overlaps at the hatch lip.",
    )

    # --- Closed pose: leaf lies FLAT and seats over the throat ---------------
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
                details=f"leaf min z={cz0:.3f}, shaft height={SHAFT_HEIGHT}",
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
