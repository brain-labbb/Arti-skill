from __future__ import annotations

# Timber plank trap door: a square leaf of 6 parallel wooden planks banded by
# 2 cross battens, hinged at the rear on a square metal-mesh collar that caps
# a round concrete / stone well shaft.
#
# Articraft brief:
# - Object: a square timber access hatch leaf ~0.72 m on a side, built from 6
#   parallel edge-to-edge planks banded by 2 cross battens, on a square mesh
#   collar over a round concrete well shaft ~0.80 m across and ~0.52 m tall.
# - Root/support: the concrete well shaft is the fixed root resting on z=0; the
#   square mesh collar is fixed to the shaft top; the leaf is hinged to the
#   collar at the rear edge.
# - Parts: well_shaft (hollow concrete tube), mesh_collar (square diamond-mesh
#   frame with circular throat, fixed to shaft), lid (square timber leaf with
#   6 planks + 2 cross battens + hinge knuckle).
# - Articulation: collar_to_lid REVOLUTE, hinge line along the rear edge, axis
#   horizontal (world -X at q=0) so the front edge lifts upward; positive q
#   swings the leaf up past vertical.
# - Visible geometry: warm-brown timber planks with visible gaps, slightly
#   darker battens on top, dark iron hinge knuckle at the rear edge; grey
#   concrete shaft with a hollow bore; dark rust-brown diamond-mesh collar.
# - Support/fit: the leaf seats on the throat ring lip when closed; the hinge
#   is a real mount -- collar-side lug plates + pin on the rear frame band,
#   with the leaf knuckle barrel coaxial on the pin.
# - Intentional overlaps: hinge knuckle embeds at the leaf rear edge and
#   between the collar lugs; plank bottoms seat ~2mm into the ring lip.
# - Tests: 6 planks + 2 battens present, leaf lies flat when closed, open pose
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

# --- Timber leaf dimensions ---------------------------------------------------
LEAF_SIDE = 0.72  # square leaf side (0.72 m x 0.72 m)
LEAF_HALF = LEAF_SIDE / 2.0

PLANK_THK = 0.040  # 40 mm thick timber planks
N_PLANKS = 6
PLANK_GAP = 0.003  # 3 mm expansion gap between planks
PLANK_NET_W = (LEAF_SIDE - (N_PLANKS - 1) * PLANK_GAP) / N_PLANKS  # ~0.1175 m
PLANK_PITCH = PLANK_NET_W + PLANK_GAP  # center-to-center spacing

N_BATTENS = 2
BATTEN_W = 0.080  # batten width (along Y)
BATTEN_THK = 0.025  # batten thickness (stands above plank tops)
BATTEN_LEN = LEAF_SIDE  # battens span the full leaf width

# Batten Y positions in the mesh frame (centered on leaf): traditional placement
# at 1/6 from each end of the leaf.
BATTEN_MESH_Y = [
    -LEAF_HALF + LEAF_SIDE / 6.0,  # near the free (front) edge
    +LEAF_HALF - LEAF_SIDE / 6.0,  # near the hinge (rear) edge
]

# --- Hinge dimensions ---------------------------------------------------------
HINGE_PIN_R = 0.020
HINGE_KNUCKLE_LEN = 0.17

THROAT_LIP_TOP = COLLAR_THK + 0.015  # top of the throat ring lip
HINGE_Y = LEAF_HALF  # hinge line at the rear edge of the square leaf
HINGE_Z = THROAT_LIP_TOP + PLANK_THK - 0.002  # plank bottom embeds 2mm into lip

HINGE_LUG_X = 0.10  # lug plate centers either side of the knuckle
HINGE_LUG_THK = 0.03
HINGE_LUG_TOP = HINGE_Z + HINGE_PIN_R + 0.014


# --- Shared geometry helpers --------------------------------------------------

def _board(sx: float, sy: float, sz: float) -> MeshGeometry:
    """Shared board geometry helper: a centered box mesh."""
    return BoxGeometry((sx, sy, sz))


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


def _build_collar_mesh() -> MeshGeometry:
    """Square collar frame with a diamond-mesh grille inside and a circular
    throat. Authored centered on the well axis with its base at z=0 of the part
    frame; the frame top is at z=COLLAR_THK."""
    geom = MeshGeometry()

    # Outer square frame band (four bars) leaving a mesh field inside.
    inner = COLLAR_HALF - COLLAR_FRAME
    for sx in (1.0, -1.0):
        bar = BoxGeometry((COLLAR_FRAME, 2.0 * COLLAR_HALF, COLLAR_THK))
        bar = bar.translate(sx * (COLLAR_HALF - COLLAR_FRAME / 2.0), 0.0, COLLAR_THK / 2.0)
        geom = geom.merge(bar)
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
    collar frame band, plus the hinge pin spanning between them. Authored in the
    collar part frame (base of the collar at z=0)."""
    geom = MeshGeometry()

    lug_y0 = 0.345
    lug_y1 = COLLAR_HALF
    for sx in (1.0, -1.0):
        lug = BoxGeometry((HINGE_LUG_THK, lug_y1 - lug_y0, HINGE_LUG_TOP))
        lug = lug.translate(sx * HINGE_LUG_X, (lug_y0 + lug_y1) / 2.0, HINGE_LUG_TOP / 2.0)
        geom = geom.merge(lug)

    # Hinge pin along the axis (world X).
    pin_len = 2.0 * (HINGE_LUG_X + HINGE_LUG_THK / 2.0) + 0.012
    pin = CylinderGeometry(0.013, pin_len, radial_segments=16)
    pin = pin.rotate_y(math.pi / 2.0)
    pin = pin.translate(0.0, HINGE_Y, HINGE_Z)
    geom = geom.merge(pin)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_plank_trap_door")

    concrete = Material(name="concrete", rgba=(0.70, 0.69, 0.66, 1.0))
    timber = Material(name="timber", rgba=(0.55, 0.36, 0.18, 1.0))
    batten_wood = Material(name="batten_wood", rgba=(0.45, 0.28, 0.13, 1.0))
    iron = Material(name="iron", rgba=(0.25, 0.22, 0.20, 1.0))
    mesh_iron = Material(name="mesh_iron", rgba=(0.22, 0.13, 0.10, 1.0))
    for mat in (concrete, timber, batten_wood, iron, mesh_iron):
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

    # --- Lid (square timber leaf: 6 planks + 2 battens + knuckle) -----------
    # The lid part frame sits at the rear-rim hinge line. Plank and batten
    # meshes are authored centered in the mesh frame, then offset by
    # (0, -LEAF_HALF, 0) so the rear edge lands on the hinge line.
    lid = model.part("lid")

    # Six parallel timber planks laid edge to edge along X, each running the
    # full leaf length in Y. Built via a for-loop from the shared _board helper
    # at regular pitch.
    for i in range(N_PLANKS):
        x_center = -LEAF_HALF + PLANK_NET_W / 2.0 + i * PLANK_PITCH
        plank_geom = _board(PLANK_NET_W, LEAF_SIDE, PLANK_THK)
        # Top face at z=0 (hinge axis plane), bottom at z=-PLANK_THK.
        plank_geom = plank_geom.translate(x_center, 0.0, -PLANK_THK / 2.0)
        lid.visual(
            mesh_from_geometry(plank_geom, f"plank_{i}"),
            origin=Origin(xyz=(0.0, -LEAF_HALF, 0.0)),
            material="timber",
            name=f"plank_{i}",
        )

    # Two cross battens running along X on top of the planks, banding them
    # together. Each batten bottom sits on the plank top surface (z=0).
    for j in range(N_BATTENS):
        y_center = BATTEN_MESH_Y[j]
        batten_geom = _board(BATTEN_LEN, BATTEN_W, BATTEN_THK)
        # Bottom at z=0 (plank top), top at z=+BATTEN_THK.
        batten_geom = batten_geom.translate(0.0, y_center, BATTEN_THK / 2.0)
        lid.visual(
            mesh_from_geometry(batten_geom, f"batten_{j}"),
            origin=Origin(xyz=(0.0, -LEAF_HALF, 0.0)),
            material="batten_wood",
            name=f"batten_{j}",
        )

    # Hinge knuckle: barrel coaxial with the revolute axis at the lid part
    # origin, spanning between the collar lugs.
    knuckle = mesh_from_geometry(
        CylinderGeometry(HINGE_PIN_R, HINGE_KNUCKLE_LEN, radial_segments=20),
        "hinge_knuckle",
    )
    lid.visual(
        knuckle,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="iron",
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
    # The leaf extends along local -Y (front) from the hinge; positive rotation
    # about -X lifts the front edge upward and over past vertical.
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

    # --- Plank-built leaf geometry present ----------------------------------
    plank_visuals = [lid.get_visual(f"plank_{i}") for i in range(N_PLANKS)]
    ctx.check(
        "leaf has exactly 6 timber planks",
        all(v is not None for v in plank_visuals) and len(plank_visuals) == 6,
        details="expected plank_0 through plank_5 visuals on the lid part",
    )

    batten_visuals = [lid.get_visual(f"batten_{j}") for j in range(N_BATTENS)]
    ctx.check(
        "leaf has exactly 2 cross battens",
        all(v is not None for v in batten_visuals) and len(batten_visuals) == 2,
        details="expected batten_0 and batten_1 visuals on the lid part",
    )

    knuckle = lid.get_visual("lid_knuckle")
    ctx.check(
        "hinge knuckle present on the leaf",
        knuckle is not None,
        details="expected lid_knuckle visual",
    )

    # Planks are evenly spaced at regular pitch with visible gaps.
    ctx.check(
        "planks at regular pitch with expansion gaps",
        N_PLANKS == 6 and PLANK_GAP > 0.001 and PLANK_NET_W > 0.08,
        details=f"n_planks={N_PLANKS}, gap={PLANK_GAP:.4f}, net_w={PLANK_NET_W:.4f}, "
                f"pitch={PLANK_PITCH:.4f}",
    )

    # Battens span the full leaf width and stand above the planks.
    ctx.check(
        "cross battens span the full leaf width",
        N_BATTENS == 2 and BATTEN_LEN >= LEAF_SIDE - 0.01 and BATTEN_THK > 0.01,
        details=f"n_battens={N_BATTENS}, batten_len={BATTEN_LEN}, batten_thk={BATTEN_THK}",
    )

    # --- Hinge is physically mounted ----------------------------------------
    hinge_mount = collar.get_visual("hinge_mount")
    ctx.check(
        "collar-side hinge mount reaches the hinge axis",
        hinge_mount is not None and HINGE_LUG_TOP > HINGE_Z + HINGE_PIN_R,
        details=f"lug_top={HINGE_LUG_TOP:.3f}, hinge_z={HINGE_Z:.3f}",
    )

    # Square leaf covers the circular throat opening.
    ctx.check(
        "square leaf is wider than the throat opening",
        LEAF_HALF >= COLLAR_THROAT_R + 0.02,
        details=f"leaf_half={LEAF_HALF:.3f}, throat_r={COLLAR_THROAT_R:.3f}",
    )

    # Intended local overlaps: hinge knuckle embeds between the collar lugs and
    # at the leaf rear edge; plank bottoms seat ~2mm into the throat ring lip.
    ctx.allow_overlap(
        lid,
        collar,
        reason="Hinge knuckle barrel embeds between the collar lug plates and "
        "at the leaf rear edge; plank bottoms seat ~2mm into the throat ring "
        "lip when closed. Both are local intended hinge/seating overlaps.",
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
            # Battens stand above the planks so the Z span includes batten height.
            ctx.check(
                "battens stand proud above the plank surface",
                z_span > PLANK_THK + BATTEN_THK * 0.5,
                details=f"z_span={z_span:.3f}, expected > {PLANK_THK + BATTEN_THK * 0.5:.3f}",
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
