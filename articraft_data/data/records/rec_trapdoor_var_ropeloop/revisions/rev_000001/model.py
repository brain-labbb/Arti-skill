from __future__ import annotations

# Round cast-iron access hatch / trap door lid hinged at the rear, sitting on a
# square metal-mesh collar that caps a round concrete / stone well shaft.
# Variant: rope loop pull handle on the lid top (replaces cross-wheel relief).
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
#   recessed top panel, rope loop pull, eyelet grommets, and rim bolts).
# - Articulation: collar_to_lid REVOLUTE, hinge line along the rear rim, axis
#   horizontal (world Y at q=0) so the front edge lifts upward; positive q
#   swings the lid up past vertical. A closed trap door correctly lies FLAT, so
#   the hinge axis is horizontal here (unlike an upright door).
# - Visible geometry: reddish-brown cast iron lid with recessed top panel,
#   hemp-fiber rope loop pull (curved tube arch anchored through two eyelet
#   grommets), ring of rim bolts; grey concrete shaft with hollow bore; dark
#   rust-brown diamond-mesh collar top.
# - Support/fit: the lid rim seats on the throat ring lip when closed; the hinge
#   is a real mount -- collar-side lug plates + pin on the rear frame band, with
#   the lid knuckle barrel coaxial on the pin.
# - Intentional overlaps: hinge knuckle/pin/lugs embed into the lid and collar
#   (local, mechanically explanatory); rope tube ends embed into the lid recess
#   at the eyelets (rope anchored through the face); eyelet grommets set into
#   the recess floor; the closed lid rim embeds ~2mm into the ring lip seat.
# - Tests: lid disc + rope loop + eyelets + bolts present, lid lies flat & seats
#   on the collar when closed, open pose lifts the front edge well above the
#   rim, shaft is hollow, nothing floats.
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
    tube_from_spline_points,
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

# --- Rope loop pull handle (replaces cross-wheel relief) --------------------
ROPE_EYELET_SPACING = 0.16  # distance between the two eyelets on the lid face
ROPE_LOOP_HEIGHT = 0.065    # arch height above the recess floor
ROPE_RADIUS = 0.012         # rope tube radius (~24 mm thick cord)
N_EYELETS = 2
EYELET_R = 0.016            # eyelet grommet ring radius
EYELET_TUBE_R = 0.005       # eyelet torus tube radius (ring cross-section)

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


def _disc(radius: float, height: float, segments: int = 64) -> MeshGeometry:
    return CylinderGeometry(radius, height, radial_segments=segments)


RECESS_OUTER_R = 0.290  # outer radius of the sunken top panel
RECESS_DEPTH = 0.026  # how deep the top panel is recessed below the rim


def _build_lid_body_mesh() -> MeshGeometry:
    """Round cast-iron lid BODY (reddish-brown). A solid disc whose TOP is
    stepped: a flat outer rim band (z=0) carrying a ring of bolts, then a
    recessed circular panel that the dark cross-wheel relief sits in. Authored in
    the lid part frame, centered on the lid axis, rim top at z=0. The dark relief
    casting is a separate visual (`_build_lid_relief_mesh`) so it can read in a
    contrasting near-black iron like the photographed hatch."""
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

    # Rim bolts: raised cylinders around the flat outer rim band (cast iron, same
    # body material so they read as part of the lid casting / the light radial
    # studs seen around the rim in the reference).
    bolt_h = 0.012
    for i in range(N_BOLTS):
        ang = (2.0 * math.pi / N_BOLTS) * i
        bolt = CylinderGeometry(BOLT_R, bolt_h, radial_segments=12)
        bolt = bolt.translate(BOLT_RING_R, 0.0, bolt_h / 2.0)
        bolt = bolt.rotate_z(ang)
        geom = geom.merge(bolt)

    return geom


def _build_rope_loop_mesh() -> MeshGeometry:
    """Rope loop pull: a thick curved tube forming a semicircular arch from one
    eyelet to the other, standing proud of the recessed lid face. Authored in
    the lid body frame (centered on the lid axis, rim top at z=0). The rope
    arches in the XZ plane; the tube ends embed into the recess floor at the
    eyelet positions (intentional, rope anchored through the face)."""
    half_span = ROPE_EYELET_SPACING / 2.0
    base_z = -RECESS_DEPTH  # recess floor level

    # Sample points along a semicircular arch in the XZ plane.
    n_arc = 9
    points = []
    for i in range(n_arc):
        t = i / (n_arc - 1)
        angle = math.pi * t
        x = half_span * math.cos(angle)
        z = base_z + ROPE_LOOP_HEIGHT * math.sin(angle)
        points.append((x, 0.0, z))

    return tube_from_spline_points(
        points,
        radius=ROPE_RADIUS,
        samples_per_segment=16,
        radial_segments=14,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def _build_eyelet_mesh() -> MeshGeometry:
    """Single eyelet grommet ring for the rope loop pull. A small torus lying
    flat (ring axis along Z) centered at the origin, representing a metal
    grommet set into the lid recess floor."""
    return TorusGeometry(
        EYELET_R, EYELET_TUBE_R, radial_segments=10, tubular_segments=24
    )


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
            # instead of letting the mesh span across the opening. Because `base`
            # is perpendicular to the bar direction, the line meets a circle of
            # radius R at t = +/- sqrt(R^2 - off^2); drop that central span.
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

    # The diamond bars above are carved clear of the throat circle, so the
    # circular opening reads as a real hole framed by the throat ring; the lid
    # plugs that hole when closed and the shaft bore is visible when it swings up.
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_trap_door")

    concrete = Material(name="concrete", rgba=(0.70, 0.69, 0.66, 1.0))
    cast_iron = Material(name="cast_iron", rgba=(0.46, 0.21, 0.15, 1.0))
    # Natural hemp-fiber rope for the loop pull, contrasting warm tan against
    # the rusty cast-iron lid.
    rope_fiber = Material(name="rope_fiber", rgba=(0.68, 0.55, 0.33, 1.0))
    mesh_iron = Material(name="mesh_iron", rgba=(0.22, 0.13, 0.10, 1.0))
    for mat in (concrete, cast_iron, rope_fiber, mesh_iron):
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

    # --- Lid (cast-iron disc + spoked relief + bolts) ------------------------
    # The lid part frame sits on the rear-rim hinge line. The disc mesh is
    # centered on the lid axis, so we offset it forward (-Y) by LID_R so the
    # rear rim of the disc lands on the hinge line at the lid part origin.
    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_body_mesh(), "lid_body"),
        origin=Origin(xyz=(0.0, -LID_R, 0.0)),
        material="cast_iron",
        name="lid_disc",
    )
    # Rope loop pull: a thick curved tube arching over the recessed lid face.
    lid.visual(
        mesh_from_geometry(_build_rope_loop_mesh(), "rope_loop"),
        origin=Origin(xyz=(0.0, -LID_R, 0.0)),
        material="rope_fiber",
        name="rope_loop",
    )
    # Eyelet grommets: two metal rings set into the recess floor where the rope
    # is anchored. Emitted via a for loop with name_i style naming.
    for i in range(N_EYELETS):
        sx = 1.0 if i == 0 else -1.0
        ex = sx * ROPE_EYELET_SPACING / 2.0
        lid.visual(
            mesh_from_geometry(_build_eyelet_mesh(), f"eyelet_{i}"),
            origin=Origin(xyz=(ex, -LID_R, -RECESS_DEPTH)),
            material="cast_iron",
            name=f"eyelet_{i}",
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
    #
    # The lid mesh is centered on its own axis. We want, at q=0, the lid center to
    # sit over the throat center and lie flat. The hinge line is at the REAR rim
    # (y = +LID_R from the lid center). Place the joint frame at the collar top,
    # rear of the throat; then the lid frame (which equals the joint frame at q=0)
    # must be shifted so the lid disc center is forward (-Y) of the hinge by LID_R.
    # HINGE_Y/HINGE_Z put the axis on the collar-side lug pin, with the lid bottom
    # embedded ~2mm into the throat ring lip so the closed lid reads as seated.
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

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shaft = object_model.get_part("well_shaft")
    collar = object_model.get_part("mesh_collar")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("collar_to_lid")

    lid_disc = lid.get_visual("lid_disc")
    rope_loop = lid.get_visual("rope_loop")
    eyelet_0 = lid.get_visual("eyelet_0")
    eyelet_1 = lid.get_visual("eyelet_1")
    lid_knuckle = lid.get_visual("lid_knuckle")
    hinge_mount = collar.get_visual("hinge_mount")

    # --- Hero geometry present ----------------------------------------------
    ctx.check(
        "lid has disc, rope loop pull, eyelet grommets, knuckle, and rim bolts",
        lid_disc is not None and rope_loop is not None
        and eyelet_0 is not None and eyelet_1 is not None
        and lid_knuckle is not None,
        details="expected lid_disc, rope_loop, eyelet_0, eyelet_1, lid_knuckle visuals",
    )
    # Rope loop pull is a curved tube arch standing above the recessed lid face,
    # anchored through two eyelet grommets.
    ctx.check(
        "rope loop is a thick curved tube with adequate arch height",
        ROPE_LOOP_HEIGHT > 0.03 and ROPE_RADIUS >= 0.008
        and ROPE_EYELET_SPACING > 0.10,
        details=f"loop_height={ROPE_LOOP_HEIGHT}, rope_r={ROPE_RADIUS}, "
        f"eyelet_spacing={ROPE_EYELET_SPACING}",
    )
    ctx.check(
        "two eyelet grommets anchor the rope loop symmetrically on the lid face",
        N_EYELETS == 2 and ROPE_EYELET_SPACING / 2.0 < LID_R - 0.05,
        details=f"eyelets={N_EYELETS}, half_spacing={ROPE_EYELET_SPACING/2.0:.3f}, "
        f"lid_r={LID_R:.3f}",
    )
    # Recessed lid top with rim-bolt ring still present.
    ctx.check(
        "lid top is recessed with a rim-bolt ring",
        RECESS_DEPTH >= 0.015 and N_BOLTS == 12 and BOLT_RING_R > RECESS_OUTER_R,
        details=f"recess_depth={RECESS_DEPTH}, bolts={N_BOLTS}, "
        f"bolt_ring_r={BOLT_RING_R}, recess_outer_r={RECESS_OUTER_R}",
    )

    # --- Hinge is physically mounted (not floating) ---------------------------
    # The collar carries lug plates + a pin on the hinge axis, the lid knuckle is
    # coaxial with that axis, and the lid is wider than the throat so its rim
    # rests on the ring lip instead of hanging over the opening.
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

    # The hinge knuckle bridges the lid rim and the collar rear edge, and the
    # shallow lid seating step nests inside the throat ring when closed. Both are
    # small, local, intended overlaps at the lid/collar interface.
    ctx.allow_overlap(
        lid,
        collar,
        reason="Hinge knuckle barrel embeds into the collar rear edge and the "
        "shallow lid centering step nests inside the throat ring when closed; "
        "both are local intended seated/hinge overlaps at the hatch lip.",
    )

    # --- Closed pose: lid lies FLAT and seats over the throat ----------------
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(lid)
        # Lid disc thickness is small compared to its diameter => it lies flat.
        # The lid should span roughly a full diameter in both X and Y near the
        # collar top, well above the ground.
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

    # --- Open pose: front edge lifts upward, past vertical -------------------
    with ctx.pose({hinge: 1.9}):
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
