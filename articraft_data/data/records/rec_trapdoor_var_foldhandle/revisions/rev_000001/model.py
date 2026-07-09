from __future__ import annotations

# Round cast-iron access hatch / trap door lid hinged at the rear, sitting on a
# square metal-mesh collar that caps a round concrete / stone well shaft.
# VARIANT: the cross-wheel relief is replaced by a flush folding lift handle --
# a flat bar lying in a recessed pocket in the lid face that pivots up on its
# own small hinge at one end of the pocket.
#
# Articraft brief:
# - Object: a circular cast-iron access hatch / manhole-style trap door, lid
#   ~0.72 m diameter (wider than the throat so it seats on the ring lip), on a
#   square mesh collar over a round concrete well shaft ~0.80 m across and
#   ~0.55 m tall, standing on the ground (z=0 up).
# - Root/support: the concrete well shaft is the fixed root resting on z=0; the
#   square mesh collar is fixed to the shaft top; the lid is hinged to the
#   collar at the rear rim.
# - Parts: well_shaft (hollow concrete tube), mesh_collar (square diamond-mesh
#   frame with circular throat, fixed to shaft), lid (round cast-iron disc with
#   recessed pocket and rim bolts), lift_handle (flat bar with hinge barrel).
# - Articulations:
#   * collar_to_lid REVOLUTE, hinge line along the rear rim, axis horizontal
#     (world X at q=0) so the front edge lifts upward; positive q swings the
#     lid up past vertical.
#   * lid_to_handle REVOLUTE, small hinge at the rear end of the pocket, axis
#     horizontal (world X at q=0); positive q pivots the free end of the bar
#     upward out of the pocket.
# - Visible geometry: reddish-brown cast iron lid with a recessed pocket, rim
#   bolts, hinge lugs for the handle; dark pocket recess; steel-grey handle
#   bar; grey concrete shaft; dark rust-brown diamond-mesh collar.
# - Support/fit: the lid rim seats on the throat ring lip when closed; the main
#   hinge is collar-side lug plates + pin with lid knuckle barrel; the handle
#   hinge is two lid-side lugs + pin with handle barrel between them.
# - Intentional overlaps: main hinge knuckle/pin/lugs embed into lid and collar;
#   lid centering step nests in throat ring; handle bar sits in the lid pocket;
#   handle barrel nests between the lid-side lugs.
# - Tests: handle part + articulation present, handle lies flush in pocket when
#   closed, handle lifts up at positive q, pocket visual present, cross-wheel
#   relief removed, main lid hinge unchanged.
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

COLLAR_HALF = 0.40  # square mesh collar 0.80 m x 0.80 m
COLLAR_FRAME = 0.06  # outer frame band width
COLLAR_THK = 0.05  # collar plate thickness
COLLAR_THROAT_R = SHAFT_INNER_R + 0.01  # circular throat opening radius

LID_R = 0.36  # lid radius (0.72 m diameter) -- wider than the throat so it seats
LID_THK = 0.05  # cast-iron lid thickness
LID_RIM_SEAT = 0.015  # shallow centering step that nests into the throat ring

RECESS_OUTER_R = 0.290  # outer radius of the sunken top panel
RECESS_DEPTH = 0.026  # how deep the top panel is recessed below the rim

BOLT_R = 0.014
N_BOLTS = 12
BOLT_RING_R = 0.320  # bolt circle radius on the lid rim

HINGE_PIN_R = 0.020
HINGE_KNUCKLE_LEN = 0.17

# Hinge line placement (in the collar part frame). The lid rim rests on the
# throat ring lip; the hinge axis runs along the rear rim at the lid top plane,
# directly over the rear collar frame band so the collar-side lugs are grounded.
THROAT_LIP_TOP = COLLAR_THK + 0.015  # top of the throat ring lip
HINGE_Y = LID_R
HINGE_Z = THROAT_LIP_TOP + LID_THK - 0.002  # lid bottom embeds 2mm into the lip seat

HINGE_LUG_X = 0.10  # lug plate centers either side of the knuckle
HINGE_LUG_THK = 0.03
HINGE_LUG_TOP = HINGE_Z + HINGE_PIN_R + 0.014

# --- Handle pocket dimensions --------------------------------------------------
POCKET_LEN = 0.22  # pocket length along Y (front-to-back on lid face)
POCKET_WID = 0.065  # pocket width along X
POCKET_DEPTH = 0.018  # pocket depth below the recess floor

# --- Lift handle dimensions ----------------------------------------------------
HANDLE_LEN = 0.19  # flat bar length (fits in pocket with clearance)
HANDLE_WID = 0.040  # flat bar width
HANDLE_THK = 0.013  # flat bar thickness
HANDLE_HINGE_R = 0.007  # hinge barrel radius at the handle end
HANDLE_HINGE_LEN = 0.038  # hinge barrel length along X

# Handle hinge mount lugs (on the lid, flanking the handle barrel)
N_HANDLE_LUGS = 2
HANDLE_LUG_WID = 0.010  # lug thickness along X
HANDLE_LUG_DEPTH = 0.014  # lug extent along Y
HANDLE_LUG_H = POCKET_DEPTH + HANDLE_HINGE_R + 0.004  # lug height from pocket floor

# Handle hinge pin
HANDLE_PIN_R = 0.005
HANDLE_PIN_LEN = HANDLE_HINGE_LEN + 2.0 * HANDLE_LUG_WID + 0.008


def _disc(radius: float, height: float, segments: int = 64) -> MeshGeometry:
    return CylinderGeometry(radius, height, radial_segments=segments)


def _build_lid_body_mesh() -> MeshGeometry:
    """Round cast-iron lid BODY (reddish-brown). A solid disc whose TOP is
    stepped: a flat outer rim band (z=0) carrying a ring of bolts, then a
    recessed circular panel. The pocket for the folding handle is a separate
    visual. Authored in the lid part frame, centered on the lid axis, rim top
    at z=0."""
    geom = MeshGeometry()

    # Disc body as a solid of revolution with a stepped (recessed) top.
    # Closed cross-section outline in (radius, z), counter-clockwise:
    #   bottom face -> outer wall -> flat rim top -> step down -> recess floor.
    profile = [
        (0.0, -LID_THK),  # bottom center
        (LID_R, -LID_THK),  # bottom outer edge
        (LID_R, 0.0),  # outer wall up to rim top
        (RECESS_OUTER_R, 0.0),  # flat outer rim band
        (RECESS_OUTER_R, -RECESS_DEPTH),  # step down into recess
        (0.0, -RECESS_DEPTH),  # recessed panel floor
    ]
    body = LatheGeometry(profile, segments=72, closed=True)
    geom = geom.merge(body)

    # Shallow centering step under the lid that nests inside the throat ring so
    # the closed lid reads as seated (small, intended, mostly hidden overlap).
    seat = _disc(COLLAR_THROAT_R - 0.02, LID_RIM_SEAT, segments=64)
    seat = seat.translate(0.0, 0.0, -LID_THK - LID_RIM_SEAT / 2.0)
    geom = geom.merge(seat)

    # Rim bolts: raised cylinders around the flat outer rim band.
    bolt_h = 0.012
    for i in range(N_BOLTS):
        ang = (2.0 * math.pi / N_BOLTS) * i
        bolt = CylinderGeometry(BOLT_R, bolt_h, radial_segments=12)
        bolt = bolt.translate(BOLT_RING_R, 0.0, bolt_h / 2.0)
        bolt = bolt.rotate_z(ang)
        geom = geom.merge(bolt)

    return geom


def _build_pocket_mesh() -> MeshGeometry:
    """Dark rectangular pocket recess in the lid face. The pocket is a shallow
    rectangular cavity below the recessed panel floor where the folding handle
    bar nests when closed. Authored in the lid body frame (centered on lid axis,
    rim top at z=0). The pocket top is at the recess floor (z=-RECESS_DEPTH) and
    extends downward by POCKET_DEPTH."""
    pocket = BoxGeometry((POCKET_WID, POCKET_LEN, POCKET_DEPTH))
    # Center the pocket vertically: top at z=-RECESS_DEPTH, bottom at
    # z=-RECESS_DEPTH-POCKET_DEPTH, so center z = -RECESS_DEPTH - POCKET_DEPTH/2.
    pocket = pocket.translate(0.0, 0.0, -RECESS_DEPTH - POCKET_DEPTH / 2.0)
    return pocket


def _lug_geometry() -> MeshGeometry:
    """Shared geometry helper: a single handle hinge lug -- a small rectangular
    tab that rises from the pocket floor to support the handle hinge pin."""
    return BoxGeometry((HANDLE_LUG_WID, HANDLE_LUG_DEPTH, HANDLE_LUG_H))


def _handle_hinge_position_in_body(lug_index: int) -> tuple[float, float, float]:
    """Return the (x, y, z) center of handle lug `lug_index` (0 or 1) in the lid
    body mesh frame (centered on lid axis, rim top at z=0)."""
    sx = 1.0 if lug_index == 0 else -1.0
    lug_x = sx * (HANDLE_HINGE_LEN / 2.0 + HANDLE_LUG_WID / 2.0)
    # Lugs are at the rear end of the pocket (+Y in body frame), slightly inside.
    lug_y = POCKET_LEN / 2.0 - HANDLE_LUG_DEPTH / 2.0 - 0.003
    # Lug base at pocket floor, center at base + H/2.
    lug_z = -RECESS_DEPTH - POCKET_DEPTH + HANDLE_LUG_H / 2.0
    return (lug_x, lug_y, lug_z)


def _build_handle_mesh() -> MeshGeometry:
    """Folding lift handle: a flat bar with a hinge barrel at one end. Authored
    in the handle part frame (origin at the hinge pin axis). At q=0 the bar
    extends in -Y from the hinge with its top surface at z=0 (flush with the
    hinge axis / pocket lip). The barrel is coaxial with the hinge axis (X)."""
    geom = MeshGeometry()

    # Flat bar: extends from y=0 (hinge end) to y=-HANDLE_LEN (free end).
    # Top at z=0, bottom at z=-HANDLE_THK (bar hangs below the hinge axis).
    bar = BoxGeometry((HANDLE_WID, HANDLE_LEN, HANDLE_THK))
    bar = bar.translate(0.0, -HANDLE_LEN / 2.0, -HANDLE_THK / 2.0)
    geom = geom.merge(bar)

    # Hinge barrel at the hinge end: cylinder along X, centered at origin.
    barrel = CylinderGeometry(HANDLE_HINGE_R, HANDLE_HINGE_LEN, radial_segments=16)
    barrel = barrel.rotate_y(math.pi / 2.0)  # cylinder Z -> X
    geom = geom.merge(barrel)

    return geom


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
        # unit direction of the bar and the perpendicular (offset) direction
        dx, dy = math.cos(ang), math.sin(ang)
        px, py = -dy, dx
        for k in range(1, n):
            off = -inner + k * pitch  # perpendicular offset of this bar
            # Sample the diagonal line p_perp*off + t*dir and find the t-range
            # that stays inside the inner square [-inner, inner]^2.
            ts = []
            base_x, base_y = px * off, py * off
            # intersect with the four square edges
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

            # Carve the circular throat clear so the hatch opening is a real hole
            # (the lid plugs an actual hole and the shaft is visible when open),
            # instead of letting the mesh span across the opening.
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

    # Circular throat collar wall: a short ring around the opening tying the mesh
    # field down to the shaft bore and giving the lid a seat.
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
    axis. The lid knuckle barrel rides on this pin, so the joint is physically
    anchored to the collar instead of hanging in the air. Authored in the collar
    part frame (base of the collar at z=0)."""
    geom = MeshGeometry()

    # Lug plates sit fully on the rear frame band (y in [0.345, COLLAR_HALF]),
    # behind the lid rim so the closed disc only kisses their front faces.
    lug_y0 = 0.345
    lug_y1 = COLLAR_HALF
    for sx in (1.0, -1.0):
        lug = BoxGeometry((HINGE_LUG_THK, lug_y1 - lug_y0, HINGE_LUG_TOP))
        lug = lug.translate(sx * HINGE_LUG_X, (lug_y0 + lug_y1) / 2.0, HINGE_LUG_TOP / 2.0)
        geom = geom.merge(lug)

    # Hinge pin along the axis (world X), passing through both lugs and the lid
    # knuckle; the ends stand slightly proud as pin heads.
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


# --- Handle hinge placement in lid part frame ----------------------------------
# The handle hinge axis is at the rear end of the pocket, at the recess floor
# level. In the lid body frame (centered on lid axis), the pocket rear is at
# y = +POCKET_LEN/2. The lid body mesh is offset by (0, -LID_R, 0) in the lid
# part frame, so the hinge Y in the lid part frame is:
HANDLE_HINGE_Y_LID = -LID_R + POCKET_LEN / 2.0
HANDLE_HINGE_Z_LID = -RECESS_DEPTH  # at the recess floor level


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_trap_door")

    concrete = Material(name="concrete", rgba=(0.70, 0.69, 0.66, 1.0))
    cast_iron = Material(name="cast_iron", rgba=(0.46, 0.21, 0.15, 1.0))
    # Dark pocket recess (shadow-like, reads as a cavity in the lid face).
    pocket_iron = Material(name="pocket_iron", rgba=(0.08, 0.07, 0.06, 1.0))
    # Steel-grey handle bar contrasting with the rusty lid.
    handle_iron = Material(name="handle_iron", rgba=(0.38, 0.36, 0.34, 1.0))
    mesh_iron = Material(name="mesh_iron", rgba=(0.22, 0.13, 0.10, 1.0))
    for mat in (concrete, cast_iron, pocket_iron, handle_iron, mesh_iron):
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
    # Hinge lugs + pin on the rear frame band; the lid knuckle rides on this pin.
    collar.visual(
        mesh_from_geometry(_build_hinge_mount_mesh(), "hinge_mount"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="mesh_iron",
        name="hinge_mount",
    )

    # --- Lid (cast-iron disc + pocket + rim bolts + handle hinge mount) ------
    # The lid part frame sits on the rear-rim hinge line. The lid disc mesh is
    # centered on the lid axis, so we offset it forward (-Y) by LID_R so the
    # rear rim of the disc lands on the hinge line at the lid part origin.
    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_body_mesh(), "lid_body"),
        origin=Origin(xyz=(0.0, -LID_R, 0.0)),
        material="cast_iron",
        name="lid_disc",
    )
    # Dark rectangular pocket recess in the lid face.
    lid.visual(
        mesh_from_geometry(_build_pocket_mesh(), "lid_pocket"),
        origin=Origin(xyz=(0.0, -LID_R, 0.0)),
        material="pocket_iron",
        name="lid_pocket",
    )

    # Handle hinge mount lugs: two small tabs on the lid that support the
    # handle hinge pin. Emitted via a for loop with name_i style naming.
    for i in range(N_HANDLE_LUGS):
        lx, ly, lz = _handle_hinge_position_in_body(i)
        lug = _lug_geometry()
        lug = lug.translate(lx, ly, lz)
        lid.visual(
            mesh_from_geometry(lug, f"handle_lug_{i}"),
            origin=Origin(xyz=(0.0, -LID_R, 0.0)),
            material="cast_iron",
            name=f"handle_lug_{i}",
        )

    # Handle hinge pin: thin cylinder along X through the lugs and handle barrel.
    pin = CylinderGeometry(HANDLE_PIN_R, HANDLE_PIN_LEN, radial_segments=12)
    pin = pin.rotate_y(math.pi / 2.0)  # cylinder Z -> X
    # Pin center at the hinge axis in lid body frame:
    pin_y_body = POCKET_LEN / 2.0 - HANDLE_LUG_DEPTH / 2.0 - 0.003
    pin_z_body = -RECESS_DEPTH
    pin = pin.translate(0.0, pin_y_body, pin_z_body)
    lid.visual(
        mesh_from_geometry(pin, "handle_hinge_pin"),
        origin=Origin(xyz=(0.0, -LID_R, 0.0)),
        material="handle_iron",
        name="handle_hinge_pin",
    )

    # Hinge knuckle: a barrel COAXIAL with the revolute axis (lid part origin),
    # spanning between the collar lugs. Because it sits exactly on the axis it
    # stays on the collar pin in every pose instead of orbiting away from it.
    knuckle = mesh_from_geometry(
        CylinderGeometry(HINGE_PIN_R, HINGE_KNUCKLE_LEN, radial_segments=20),
        "hinge_knuckle",
    )
    # Knuckle barrel lies along the hinge line (world X at q=0): rotate the
    # cylinder's local +Z to +X via a pitch of pi/2 about Y.
    lid.visual(
        knuckle,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="cast_iron",
        name="lid_knuckle",
    )

    # --- Lift handle (flat bar that folds into the pocket) -------------------
    # The handle part frame sits at the hinge axis. At q=0 the bar extends in
    # -Y from the hinge (toward the front of the lid) with its top at z=0.
    lift_handle = model.part("lift_handle")
    lift_handle.visual(
        mesh_from_geometry(_build_handle_mesh(), "handle_bar"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="handle_iron",
        name="handle_bar",
    )

    # --- Fixed joints -------------------------------------------------------
    # Collar sits on top of the shaft.
    model.articulation(
        "shaft_to_collar",
        ArticulationType.FIXED,
        parent=shaft,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT)),
    )

    # --- Lid hinge (primary articulation) -----------------------------------
    # Hinge line is along the rear rim of the throat, at the collar top plane.
    # The lid part frame is placed there; the lid disc geometry is authored
    # centered on the lid axis, so we offset the lid mesh forward (-Y) via the
    # joint origin so the rear rim aligns with the hinge.
    model.articulation(
        "collar_to_lid",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        # Horizontal hinge axis along the rear rim (world X). The lid disc extends
        # along local -Y (front) from the hinge; positive rotation about -X lifts
        # the front (-Y) edge upward and over past vertical.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    # --- Handle hinge (folding lift handle articulation) ---------------------
    # The handle hinges at the rear end of the pocket. The articulation origin
    # is at the hinge pin axis on the lid face (in the lid part frame).
    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=lift_handle,
        origin=Origin(xyz=(0.0, HANDLE_HINGE_Y_LID, HANDLE_HINGE_Z_LID)),
        # Horizontal axis along X. The handle bar extends in -Y from the hinge;
        # positive rotation about -X lifts the free (-Y) end upward (+Z).
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0, lower=0.0, upper=1.75),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shaft = object_model.get_part("well_shaft")
    collar = object_model.get_part("mesh_collar")
    lid = object_model.get_part("lid")
    lift_handle = object_model.get_part("lift_handle")
    main_hinge = object_model.get_articulation("collar_to_lid")
    handle_hinge = object_model.get_articulation("lid_to_handle")

    lid_disc = lid.get_visual("lid_disc")
    lid_pocket = lid.get_visual("lid_pocket")
    lid_knuckle = lid.get_visual("lid_knuckle")
    hinge_mount = collar.get_visual("hinge_mount")
    handle_bar = lift_handle.get_visual("handle_bar")

    # --- Hero geometry present ----------------------------------------------
    ctx.check(
        "lid has disc, pocket, and knuckle geometry",
        lid_disc is not None and lid_pocket is not None and lid_knuckle is not None,
        details="expected lid_disc, lid_pocket, and lid_knuckle visuals",
    )
    ctx.check(
        "handle bar visual exists on lift_handle part",
        handle_bar is not None,
        details="expected handle_bar visual on lift_handle",
    )
    ctx.check(
        "handle hinge lugs exist on the lid",
        lid.get_visual("handle_lug_0") is not None
        and lid.get_visual("handle_lug_1") is not None,
        details="expected handle_lug_0 and handle_lug_1 visuals on lid",
    )
    ctx.check(
        "handle hinge pin exists on the lid",
        lid.get_visual("handle_hinge_pin") is not None,
        details="expected handle_hinge_pin visual on lid",
    )

    # --- Cross-wheel relief is removed (variant check) -----------------------
    relief_removed = True
    for vname in ("lid_relief",):
        try:
            if lid.get_visual(vname) is not None:
                relief_removed = False
        except Exception:
            pass
    ctx.check(
        "cross-wheel relief is removed (variant fork)",
        relief_removed,
        details="lid_relief visual should not exist in this variant",
    )

    # --- Handle hinge is REVOLUTE with correct limits -----------------------
    ctx.check(
        "handle hinge is a revolute articulation",
        handle_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {handle_hinge.articulation_type}",
    )
    handle_limits = handle_hinge.motion_limits
    ctx.check(
        "handle hinge has positive upper limit (folds up)",
        handle_limits is not None
        and handle_limits.upper is not None
        and handle_limits.upper > 0.5,
        details=f"upper={None if handle_limits is None else handle_limits.upper}",
    )

    # --- Main lid hinge unchanged -------------------------------------------
    ctx.check(
        "main lid hinge is still revolute with same axis",
        main_hinge.articulation_type == ArticulationType.REVOLUTE
        and main_hinge.axis is not None
        and abs(main_hinge.axis[0]) > 0.9,
        details=f"type={main_hinge.articulation_type}, axis={main_hinge.axis}",
    )

    # --- Hinge is physically mounted (not floating) ---------------------------
    ctx.check(
        "collar-side hinge mount (lugs + pin) reaches the hinge axis",
        hinge_mount is not None and HINGE_LUG_TOP > HINGE_Z + HINGE_PIN_R,
        details=f"lug_top={HINGE_LUG_TOP:.3f}, hinge_z={HINGE_Z:.3f}",
    )
    ctx.check(
        "lid is wider than the throat opening so it seats on the ring lip",
        LID_R >= COLLAR_THROAT_R + 0.02,
        details=f"lid_r={LID_R:.3f}, throat_r={COLLAR_THROAT_R:.3f}",
    )

    # --- Pocket geometry claims ----------------------------------------------
    ctx.check(
        "pocket is recessed below the lid recess floor",
        POCKET_DEPTH > 0.010 and RECESS_DEPTH > 0.015,
        details=f"pocket_depth={POCKET_DEPTH}, recess_depth={RECESS_DEPTH}",
    )
    ctx.check(
        "handle bar fits within the pocket",
        HANDLE_LEN < POCKET_LEN and HANDLE_WID < POCKET_WID
        and HANDLE_THK < POCKET_DEPTH + HANDLE_HINGE_R,
        details=f"handle=({HANDLE_LEN},{HANDLE_WID},{HANDLE_THK}), "
        f"pocket=({POCKET_LEN},{POCKET_WID},{POCKET_DEPTH})",
    )

    # --- Overlap allowances --------------------------------------------------
    # Main hinge knuckle/pin/lugs embed into the lid and collar; the lid
    # centering step nests inside the throat ring when closed.
    ctx.allow_overlap(
        lid,
        collar,
        reason="Hinge knuckle barrel embeds into the collar rear edge and the "
        "shallow lid centering step nests inside the throat ring when closed; "
        "both are local intended seated/hinge overlaps at the hatch lip.",
    )
    # Handle bar sits in the lid pocket; handle barrel nests between the lugs.
    ctx.allow_overlap(
        lid,
        lift_handle,
        reason="Handle bar sits in the recessed lid pocket and the handle hinge "
        "barrel nests between the lid-side mount lugs; both are intentional "
        "local nesting at the handle hinge interface.",
    )

    # --- Handle closed pose: bar lies flush in pocket -----------------------
    with ctx.pose({main_hinge: 0.0, handle_hinge: 0.0}):
        # Handle is within the lid footprint when folded flat.
        ctx.expect_within(
            lift_handle,
            lid,
            axes="xy",
            margin=0.02,
            name="folded handle stays within the lid footprint",
        )
        # Handle is in contact with (or very close to) the lid when folded flat.
        ctx.expect_contact(
            lift_handle,
            lid,
            contact_tol=0.008,
            name="folded handle seats in the lid pocket",
        )

    # --- Handle open pose: bar pivots upward --------------------------------
    with ctx.pose({main_hinge: 0.0, handle_hinge: 1.5}):
        handle_aabb = ctx.part_world_aabb(lift_handle)
        lid_aabb = ctx.part_world_aabb(lid)
        if handle_aabb is not None and lid_aabb is not None:
            # The handle top should be well above the lid top when deployed.
            ctx.check(
                "deployed handle rises above the lid face",
                handle_aabb[1][2] > lid_aabb[1][2] + 0.05,
                details=f"handle_max_z={handle_aabb[1][2]:.3f}, "
                f"lid_max_z={lid_aabb[1][2]:.3f}",
            )
            # The deployed handle has significant Z extent (stands up).
            z_span = handle_aabb[1][2] - handle_aabb[0][2]
            ctx.check(
                "deployed handle stands up (significant Z extent)",
                z_span > 0.08,
                details=f"handle z_span={z_span:.3f}",
            )

    # --- Closed lid pose: lies FLAT and seats over the throat ----------------
    with ctx.pose({main_hinge: 0.0, handle_hinge: 0.0}):
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
                "closed lid sits at the collar top, not on the ground",
                cz0 > SHAFT_HEIGHT - 0.02,
                details=f"lid min z={cz0:.3f}, shaft height={SHAFT_HEIGHT}",
            )
        # The lid footprint overlaps the throat/collar in plan when closed.
        ctx.expect_overlap(
            lid,
            collar,
            axes="xy",
            min_overlap=0.20,
            name="closed lid covers the collar throat in plan",
        )
        # The closed lid actually rests on the collar (seated, not floating above
        # the opening). Allowed local seating overlap, so require near contact.
        ctx.expect_contact(
            lid,
            collar,
            contact_tol=0.006,
            name="closed lid seats on the collar (not floating)",
        )

    closed_front = ctx.part_world_aabb(lid)

    # --- Open lid pose: front edge lifts upward, past vertical ----------------
    with ctx.pose({main_hinge: 1.9, handle_hinge: 0.0}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "open pose lifts the lid well above the collar",
            open_aabb is not None
            and closed_front is not None
            and open_aabb[1][2] > closed_front[1][2] + 0.20,
            details=f"closed max z={None if closed_front is None else closed_front[1][2]:.3f}, "
            f"open max z={None if open_aabb is None else open_aabb[1][2]:.3f}",
        )
        # When swung up, the lid becomes tall (large Z extent) and shallow in Y.
        if open_aabb is not None:
            (ox0, oy0, oz0), (ox1, oy1, oz1) = open_aabb
            ctx.check(
                "open lid stands up (tall in Z)",
                (oz1 - oz0) > 0.45,
                details=f"open z_span={(oz1 - oz0):.3f}",
            )

    # --- Support / placement -------------------------------------------------
    # Shaft is the fixed root standing on the ground.
    shaft_aabb = ctx.part_world_aabb(shaft)
    if shaft_aabb is not None:
        ctx.check(
            "well shaft rests on the ground plane (z~0)",
            abs(shaft_aabb[0][2]) < 0.01,
            details=f"shaft min z={shaft_aabb[0][2]:.4f}",
        )

    # Collar sits on top of the shaft and supports the lid (no floating).
    collar_aabb = ctx.part_world_aabb(collar)
    if collar_aabb is not None and shaft_aabb is not None:
        ctx.check(
            "mesh collar sits at the shaft top",
            abs(collar_aabb[0][2] - shaft_aabb[1][2]) < 0.05,
            details=f"collar min z={collar_aabb[0][2]:.3f}, shaft max z={shaft_aabb[1][2]:.3f}",
        )

    # Collar throat is wider than the shaft bore so the opening reads through.
    ctx.check(
        "collar throat clears the shaft bore (hollow well)",
        COLLAR_THROAT_R <= SHAFT_INNER_R + 0.05 and SHAFT_INNER_R > 0.25,
        details=f"throat_r={COLLAR_THROAT_R:.3f}, bore_r={SHAFT_INNER_R:.3f}",
    )

    return ctx.report()
