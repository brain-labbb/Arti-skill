from __future__ import annotations

# Round cast-iron access hatch / trap door lid hinged at the rear, seated on a
# raised rectangular curb coaming frame that stands up above a round concrete
# well shaft.  The curb is a tall upstanding kerb wall ringing the opening; the
# lid drops down onto the curb top.
#
# Articraft brief:
# - Object: circular cast-iron access hatch, lid ~0.72 m dia, on a raised
#   square curb coaming (0.84 m x 0.84 m, walls 0.14 m tall) over a round
#   concrete shaft ~0.80 m across and ~0.52 m tall (z=0 up).
# - Root/support: concrete shaft is the fixed root; curb frame is fixed to the
#   shaft top; lid hinges at the rear curb wall.
# - Parts: well_shaft (hollow concrete tube), curb_frame (four upstanding
#   rectangular walls + base plate + circular throat ring, fixed to shaft top),
#   lid (round cast-iron disc with recessed spoked relief and rim bolts).
# - Articulation: curb_to_lid REVOLUTE, hinge line at the rear inner wall face,
#   axis horizontal (world X at q=0) so positive q lifts the front edge upward
#   past vertical.  Closed hatch lies FLAT on the curb top.
# - Visible geometry: reddish-brown cast-iron lid with dark cross-wheel relief
#   and 12-bolt rim; dark industrial steel curb with tall walls and visible
#   throat ring; grey concrete shaft with hollow bore.
# - Support/fit: lid rim seats on the curb wall tops when closed; hinge lugs
#   and pin on the rear curb wall carry the lid knuckle.
# - Intentional overlaps: hinge knuckle / pin / lugs embed at the lid-curb
#   interface; lid centering step nests inside the throat ring; lid bottom
#   embeds ~2 mm into the curb top seating surface.
# - Tests: curb walls present and tall, lid seats flat on curb when closed,
#   open pose lifts front edge well above curb, shaft is hollow, nothing floats.
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
    TorusGeometry,
    mesh_from_geometry,
)

# --- Absolute dimensions (meters) ---------------------------------------------
SHAFT_OUTER_R = 0.40
SHAFT_WALL = 0.085
SHAFT_INNER_R = SHAFT_OUTER_R - SHAFT_WALL  # bore radius ~0.315
SHAFT_HEIGHT = 0.52

# Raised curb coaming frame: a square kerb wall standing up above the shaft top.
CURB_HALF = 0.42        # outer square half-side (0.84 m x 0.84 m)
CURB_WALL = 0.06        # wall thickness (substantial steel plate)
CURB_INNER = CURB_HALF - CURB_WALL  # inner clear half-side = 0.36 m (= LID_R)
CURB_WALL_H = 0.14      # wall height above base plate (tall upstanding kerb)
CURB_BASE_THK = 0.025   # thin base plate connecting curb to shaft top
CURB_TOP_Z = CURB_BASE_THK + CURB_WALL_H  # top of walls = 0.165 m
CURB_THROAT_R = SHAFT_INNER_R + 0.01  # circular throat ~0.325 m
THROAT_WALL = 0.015     # throat ring wall thickness
N_CURB_WALLS = 4        # four walls forming the rectangular kerb

LID_R = 0.36            # lid radius (0.72 m dia) -- matches curb inner so it seats
LID_THK = 0.05          # cast-iron lid thickness
LID_RIM_SEAT = 0.015    # shallow centering step nesting inside the throat ring

RELIEF_RING_R = 0.245   # recessed relief ring outer radius on lid top
HUB_R = 0.070           # central hub radius
SPOKE_HALF_W = 0.050    # half-width of each radial spoke
N_SPOKES = 4

BOLT_R = 0.014
N_BOLTS = 12
BOLT_RING_R = 0.320     # bolt circle radius on the lid rim

HINGE_PIN_R = 0.020
HINGE_KNUCKLE_LEN = 0.17

# Hinge line placement (in the curb part frame).  The lid rim seats on the curb
# wall tops; the hinge axis runs along the rear inner wall face at the lid top
# plane, directly above the rear curb wall so the lugs are grounded on the wall.
HINGE_Y = CURB_INNER    # 0.36 -- rear inner wall face = lid rear rim
HINGE_Z = CURB_TOP_Z + LID_THK - 0.002  # pin near lid top, 2 mm seating embed

HINGE_LUG_X = 0.10      # lug plate centres either side of the knuckle
HINGE_LUG_THK = 0.03
HINGE_LUG_TOP = HINGE_Z + HINGE_PIN_R + 0.020  # lugs extend above the pin

RECESS_OUTER_R = 0.290  # outer radius of the sunken top panel
RECESS_DEPTH = 0.026    # how deep the top panel is recessed below the rim
RELIEF_TOP_Z = 0.018    # top of the raised hub / spokes above the rim


def _disc(radius: float, height: float, segments: int = 64) -> MeshGeometry:
    return CylinderGeometry(radius, height, radial_segments=segments)


def _build_lid_body_mesh() -> MeshGeometry:
    """Round cast-iron lid BODY.  A solid disc whose TOP is stepped: a flat
    outer rim band (z=0) carrying a ring of bolts, then a recessed circular
    panel that the dark cross-wheel relief sits in.  Authored in the lid part
    frame, centred on the lid axis, rim top at z=0."""
    geom = MeshGeometry()

    profile = [
        (0.0, -LID_THK),
        (LID_R, -LID_THK),
        (LID_R, 0.0),
        (RECESS_OUTER_R, 0.0),
        (RECESS_OUTER_R, -RECESS_DEPTH),
        (0.0, -RECESS_DEPTH),
    ]
    body = LatheGeometry(profile, segments=72, closed=True)
    geom = geom.merge(body)

    # Shallow centreing step under the lid nesting inside the throat ring.
    seat = _disc(CURB_THROAT_R - 0.02, LID_RIM_SEAT, segments=64)
    seat = seat.translate(0.0, 0.0, -LID_THK - LID_RIM_SEAT / 2.0)
    geom = geom.merge(seat)

    # Rim bolts around the flat outer rim band.
    bolt_h = 0.012
    for i in range(N_BOLTS):
        ang = (2.0 * math.pi / N_BOLTS) * i
        bolt = CylinderGeometry(BOLT_R, bolt_h, radial_segments=12)
        bolt = bolt.translate(BOLT_RING_R, 0.0, bolt_h / 2.0)
        bolt = bolt.rotate_z(ang)
        geom = geom.merge(bolt)

    return geom


def _build_lid_relief_mesh() -> MeshGeometry:
    """Bold raised cast cross-wheel relief sitting in the lid recess."""
    geom = MeshGeometry()

    relief_h = RELIEF_TOP_Z - (-RECESS_DEPTH)
    relief_base_z = -RECESS_DEPTH

    hub = CylinderGeometry(HUB_R, relief_h, radial_segments=48)
    hub = hub.translate(0.0, 0.0, relief_base_z + relief_h / 2.0)
    geom = geom.merge(hub)

    spoke_inner = HUB_R - 0.010
    spoke_outer = RELIEF_RING_R + 0.004
    spoke_len = spoke_outer - spoke_inner
    for i in range(N_SPOKES):
        ang = (math.pi / 2.0) * i + math.pi / 4.0
        bar = BoxGeometry((spoke_len, 2.0 * SPOKE_HALF_W, relief_h))
        bar = bar.translate(spoke_inner + spoke_len / 2.0, 0.0, relief_base_z + relief_h / 2.0)
        bar = bar.rotate_z(ang)
        geom = geom.merge(bar)

    ring_tube = 0.020
    ring = TorusGeometry(RELIEF_RING_R, ring_tube, radial_segments=18, tubular_segments=72)
    ring = ring.translate(0.0, 0.0, RELIEF_TOP_Z - ring_tube + 0.006)
    geom = geom.merge(ring)

    return geom


# ---------------------------------------------------------------------------
# Curb coaming frame geometry
# ---------------------------------------------------------------------------

def _curb_wall_section(is_x_wall: bool, sign: float) -> MeshGeometry:
    """Build one wall section of the rectangular curb kerb.  Walls along Y
    (is_x_wall=False) span the full outer X width; walls along X (is_x_wall=True)
    span the inner Y clear so corners meet without overlap."""
    if not is_x_wall:
        # +Y or -Y wall: full outer width along X
        wall = BoxGeometry((2.0 * CURB_HALF, CURB_WALL, CURB_WALL_H))
        wall = wall.translate(
            0.0,
            sign * (CURB_HALF - CURB_WALL / 2.0),
            CURB_BASE_THK + CURB_WALL_H / 2.0,
        )
    else:
        # +X or -X wall: spans between the Y walls
        inner_span = 2.0 * CURB_INNER
        wall = BoxGeometry((CURB_WALL, inner_span, CURB_WALL_H))
        wall = wall.translate(
            sign * (CURB_HALF - CURB_WALL / 2.0),
            0.0,
            CURB_BASE_THK + CURB_WALL_H / 2.0,
        )
    return wall


def _build_curb_mesh() -> MeshGeometry:
    """Raised rectangular curb coaming frame.  Four tall upstanding walls
    forming a square kerb, a thin base plate, and a circular throat ring inside.
    Authored centred on the well axis with base at z=0 of the part frame."""
    geom = MeshGeometry()

    # Base plate: thin solid square tying the curb to the shaft top.
    base = BoxGeometry((2.0 * CURB_HALF, 2.0 * CURB_HALF, CURB_BASE_THK))
    base = base.translate(0.0, 0.0, CURB_BASE_THK / 2.0)
    geom = geom.merge(base)

    # Four upstanding kerb walls (repeated via for-i-in-range with name_i logic
    # inside the mesh build; walls are merged into one visual).
    wall_specs = [
        (False, +1.0),  # +Y rear wall
        (False, -1.0),  # -Y front wall
        (True,  +1.0),  # +X right wall
        (True,  -1.0),  # -X left wall
    ]
    for i in range(N_CURB_WALLS):
        is_x, sign = wall_specs[i]
        geom = geom.merge(_curb_wall_section(is_x, sign))

    # Circular throat ring: cylindrical shell from the base up to the curb top,
    # defining the round opening that the lid plugs.
    throat = LatheGeometry.from_shell_profiles(
        [
            (CURB_THROAT_R + THROAT_WALL, 0.0),
            (CURB_THROAT_R + THROAT_WALL, CURB_TOP_Z - 0.015),
            (CURB_THROAT_R + 0.003, CURB_TOP_Z),
        ],
        [
            (CURB_THROAT_R, 0.0),
            (CURB_THROAT_R, CURB_TOP_Z - 0.015),
            (CURB_THROAT_R - 0.004, CURB_TOP_Z),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    geom = geom.merge(throat)

    return geom


def _build_hinge_mount_mesh() -> MeshGeometry:
    """Curb-side hinge mount: two upright lug plates standing on the rear curb
    wall top, plus the hinge pin spanning between them.  Authored in the curb
    part frame (base of the curb at z=0)."""
    geom = MeshGeometry()

    # Lug plates sit on the rear wall top (y in [CURB_INNER, CURB_HALF]),
    # extending from the wall top up past the pin.
    lug_y0 = CURB_INNER
    lug_y1 = CURB_HALF
    lug_z0 = CURB_TOP_Z
    for i in range(2):
        sx = 1.0 if i == 0 else -1.0
        lug = BoxGeometry((HINGE_LUG_THK, lug_y1 - lug_y0, HINGE_LUG_TOP - lug_z0))
        lug = lug.translate(
            sx * HINGE_LUG_X,
            (lug_y0 + lug_y1) / 2.0,
            (lug_z0 + HINGE_LUG_TOP) / 2.0,
        )
        geom = geom.merge(lug)

    # Hinge pin along the axis (world X), through both lugs and the lid knuckle.
    pin_len = 2.0 * (HINGE_LUG_X + HINGE_LUG_THK / 2.0) + 0.012
    pin = CylinderGeometry(0.013, pin_len, radial_segments=16)
    pin = pin.rotate_y(math.pi / 2.0)
    pin = pin.translate(0.0, HINGE_Y, HINGE_Z)
    geom = geom.merge(pin)

    return geom


def _build_shaft_mesh() -> MeshGeometry:
    """Hollow round concrete well shaft, base on z=0, open bore through top."""
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
    model = ArticulatedObject(name="cast_iron_trap_door")

    concrete = Material(name="concrete", rgba=(0.70, 0.69, 0.66, 1.0))
    cast_iron = Material(name="cast_iron", rgba=(0.46, 0.21, 0.15, 1.0))
    relief_iron = Material(name="relief_iron", rgba=(0.12, 0.10, 0.09, 1.0))
    curb_iron = Material(name="curb_iron", rgba=(0.28, 0.26, 0.23, 1.0))
    for mat in (concrete, cast_iron, relief_iron, curb_iron):
        model.material(mat.name, rgba=mat.rgba)

    # --- Well shaft (fixed root) ---------------------------------------------
    shaft = model.part("well_shaft")
    shaft.visual(
        mesh_from_geometry(_build_shaft_mesh(), "well_shaft"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="concrete",
        name="shaft_wall",
    )

    # --- Raised curb coaming frame (fixed to shaft top) ----------------------
    curb = model.part("curb_frame")
    curb.visual(
        mesh_from_geometry(_build_curb_mesh(), "curb_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="curb_iron",
        name="curb_body",
    )
    curb.visual(
        mesh_from_geometry(_build_hinge_mount_mesh(), "hinge_mount"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="curb_iron",
        name="hinge_mount",
    )

    # --- Lid (cast-iron disc + spoked relief + bolts) ------------------------
    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_body_mesh(), "lid_body"),
        origin=Origin(xyz=(0.0, -LID_R, 0.0)),
        material="cast_iron",
        name="lid_disc",
    )
    lid.visual(
        mesh_from_geometry(_build_lid_relief_mesh(), "lid_relief"),
        origin=Origin(xyz=(0.0, -LID_R, 0.0)),
        material="relief_iron",
        name="lid_relief",
    )

    # Hinge knuckle: barrel coaxial with the revolute axis (lid part origin).
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

    # --- Fixed joint: curb on shaft top -------------------------------------
    model.articulation(
        "shaft_to_curb",
        ArticulationType.FIXED,
        parent=shaft,
        child=curb,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT)),
    )

    # --- Lid hinge (primary articulation) -----------------------------------
    model.articulation(
        "curb_to_lid",
        ArticulationType.REVOLUTE,
        parent=curb,
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
    curb = object_model.get_part("curb_frame")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("curb_to_lid")

    lid_disc = lid.get_visual("lid_disc")
    lid_relief = lid.get_visual("lid_relief")
    lid_knuckle = lid.get_visual("lid_knuckle")
    curb_body = curb.get_visual("curb_body")
    hinge_mount = curb.get_visual("hinge_mount")

    # --- Hero geometry present ----------------------------------------------
    ctx.check(
        "lid has disc, cross-wheel relief, and knuckle geometry",
        lid_disc is not None and lid_relief is not None and lid_knuckle is not None,
        details="expected lid_disc, lid_relief, and lid_knuckle visuals",
    )
    ctx.check(
        "curb body and hinge mount are present on the curb frame",
        curb_body is not None and hinge_mount is not None,
        details="expected curb_body and hinge_mount visuals",
    )
    ctx.check(
        "lid top is recessed and the cross-wheel relief stands proud of the rim",
        RECESS_DEPTH >= 0.015 and RELIEF_TOP_Z > 0.0,
        details=f"recess_depth={RECESS_DEPTH}, relief_top_z={RELIEF_TOP_Z}",
    )
    ctx.check(
        "spoked wheel relief and rim-bolt ring are present",
        N_SPOKES == 4 and N_BOLTS == 12 and BOLT_RING_R > RECESS_OUTER_R,
        details=f"spokes={N_SPOKES}, bolts={N_BOLTS}, bolt_ring_r={BOLT_RING_R}, "
        f"recess_outer_r={RECESS_OUTER_R}",
    )

    # --- Curb is a raised rectangular kerb (the key structural change) ------
    ctx.check(
        "curb walls are tall and clearly upstanding above the base plate",
        CURB_WALL_H >= 0.10,
        details=f"curb_wall_h={CURB_WALL_H}",
    )
    ctx.check(
        "curb is rectangular (square outer) with substantial walls",
        CURB_HALF > CURB_INNER and CURB_WALL >= 0.04
        and abs(CURB_HALF - 0.42) < 0.01,
        details=f"curb_half={CURB_HALF}, curb_inner={CURB_INNER}, curb_wall={CURB_WALL}",
    )
    ctx.check(
        "curb inner matches lid radius so the lid seats on the wall tops",
        abs(CURB_INNER - LID_R) < 0.005,
        details=f"curb_inner={CURB_INNER}, lid_r={LID_R}",
    )
    ctx.check(
        "circular throat ring is inside the square curb",
        CURB_THROAT_R < CURB_INNER and CURB_THROAT_R > SHAFT_INNER_R - 0.01,
        details=f"throat_r={CURB_THROAT_R}, curb_inner={CURB_INNER}, "
        f"shaft_inner_r={SHAFT_INNER_R}",
    )

    # --- Hinge is physically mounted on the curb ----------------------------
    ctx.check(
        "curb-side hinge mount (lugs + pin) reaches the hinge axis",
        hinge_mount is not None and HINGE_LUG_TOP > HINGE_Z + HINGE_PIN_R,
        details=f"lug_top={HINGE_LUG_TOP:.3f}, hinge_z={HINGE_Z:.3f}",
    )
    ctx.check(
        "hinge axis is at the curb top, well above the shaft",
        HINGE_Z > CURB_TOP_Z and CURB_TOP_Z > CURB_BASE_THK + 0.05,
        details=f"hinge_z={HINGE_Z:.3f}, curb_top_z={CURB_TOP_Z:.3f}",
    )

    # Intended local overlaps at the lid/curb interface.
    ctx.allow_overlap(
        lid,
        curb,
        reason="Hinge knuckle barrel embeds into the curb rear wall top and "
        "the shallow lid centreing step nests inside the throat ring when "
        "closed; both are local intended seated / hinge overlaps at the hatch lip.",
    )

    # --- Closed pose: lid lies FLAT and seats on the curb top ---------------
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(lid)
        if closed_aabb is not None:
            (cx0, cy0, cz0), (cx1, cy1, cz1) = closed_aabb
            x_span = cx1 - cx0
            y_span = cy1 - cy0
            z_span = cz1 - cz0
            ctx.check(
                "closed lid lies flat (thin in Z, wide in X and Y)",
                z_span < 0.12 and x_span > 0.5 and y_span > 0.5,
                details=f"x_span={x_span:.3f} y_span={y_span:.3f} z_span={z_span:.3f}",
            )
            ctx.check(
                "closed lid sits at the curb top, well above the ground",
                cz0 > SHAFT_HEIGHT + CURB_BASE_THK - 0.02,
                details=f"lid min z={cz0:.3f}, shaft_h+base={SHAFT_HEIGHT + CURB_BASE_THK:.3f}",
            )
        ctx.expect_overlap(
            lid,
            curb,
            axes="xy",
            min_overlap=0.20,
            name="closed lid covers the curb throat in plan",
        )
        ctx.expect_contact(
            lid,
            curb,
            contact_tol=0.006,
            name="closed lid seats on the curb top (not floating)",
        )

    closed_front = ctx.part_world_aabb(lid)

    # --- Open pose: front edge lifts upward, past vertical ------------------
    with ctx.pose({hinge: 1.9}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "open pose lifts the lid well above the curb",
            open_aabb is not None
            and closed_front is not None
            and open_aabb[1][2] > closed_front[1][2] + 0.20,
            details=f"closed max z={None if closed_front is None else closed_front[1][2]:.3f}, "
            f"open max z={None if open_aabb is None else open_aabb[1][2]:.3f}",
        )
        if open_aabb is not None:
            (ox0, oy0, oz0), (ox1, oy1, oz1) = open_aabb
            ctx.check(
                "open lid stands up (tall in Z)",
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

    curb_aabb = ctx.part_world_aabb(curb)
    if curb_aabb is not None and shaft_aabb is not None:
        ctx.check(
            "curb frame sits at the shaft top",
            abs(curb_aabb[0][2] - shaft_aabb[1][2]) < 0.05,
            details=f"curb min z={curb_aabb[0][2]:.3f}, shaft max z={shaft_aabb[1][2]:.3f}",
        )

    ctx.check(
        "curb throat clears the shaft bore (hollow well)",
        CURB_THROAT_R <= SHAFT_INNER_R + 0.05 and SHAFT_INNER_R > 0.25,
        details=f"throat_r={CURB_THROAT_R:.3f}, bore_r={SHAFT_INNER_R:.3f}",
    )

    return ctx.report()
