from __future__ import annotations

# Blue ribbed plastic barrel keg with a round black screw-on lid.
# Frame: barrel axis along +Z, base sits on z=0, lid on top.
#   - body (root): blue lathed barrel shell with a closed bottom, hollow
#     interior, visible open mouth, horizontal rib tori, and threaded neck.
#   - lid: black round screw cap with a knurled skirt and an off-axis marker
#     notch so rotation is detectable.
# Articulations (SCREW CAP, two INDEPENDENT decoupled joints via a massless
# carrier link, both sharing the vertical +Z cap axis):
#   - lid_rotate : CONTINUOUS, body -> lid_carrier (spin about +Z)
#   - lid_slide  : PRISMATIC,  lid_carrier -> lid    (lift along +Z)

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
BODY_BOTTOM_Z = 0.0
BARREL_TOP_Z = 0.255          # top shoulder of the blue barrel body
NECK_TOP_Z = 0.270            # top rim of the open threaded neck
RIM_TOP_Z = NECK_TOP_Z        # cap rotation joint sits at the neck top
BOTTOM_FLOOR_Z = 0.012        # inner floor height, leaving a real bottom thickness
MOUTH_RADIUS = 0.055          # visible open mouth radius
NECK_RADIUS = 0.070           # outer radius of the screw neck
BARREL_MAX_R = 0.110          # widest barrel radius (belly)
LID_HEIGHT = 0.030            # cap height
LID_REST_CLEARANCE = 0.040    # default gap so the open mouth is visible
LID_OUTER_R = 0.082           # solid cap outer radius


def _barrel_body() -> object:
    # Bulged barrel shell as a revolved material cross-section.  The profile
    # includes a center bottom underside and an inner floor, so the barrel has
    # a bottom while retaining an open, hollow interior up to the mouth.
    outer = [
        (0.000, 0.000),   # bottom underside center: closes the base
        (0.090, 0.000),
        (0.098, 0.006),
        (0.105, 0.030),
        (0.110, 0.110),   # belly
        (0.108, 0.160),
        (0.098, 0.215),
        (0.082, 0.245),   # shoulder
        (0.070, 0.255),   # top of body, meeting the neck
        (NECK_RADIUS, 0.270),
    ]
    inner = [
        (MOUTH_RADIUS, 0.270),  # open mouth: no center cap at the top
        (0.060, 0.255),
        (0.074, 0.245),
        (0.090, 0.215),
        (0.100, 0.160),
        (0.102, 0.110),
        (0.097, 0.035),
        (0.090, 0.018),
        (0.082, BOTTOM_FLOOR_Z),
        (0.000, BOTTOM_FLOOR_Z),  # interior floor center
    ]
    return LatheGeometry(outer + inner, segments=64, closed=True)


def _body_mesh():
    body = _barrel_body()
    # Horizontal rib tori wrapped around the belly at the local outer radius.
    # Each rib radius follows the barrel profile so the ribs sit proud of it.
    rib_specs = [
        (0.045, 0.106),
        (0.075, 0.109),
        (0.105, 0.110),
        (0.135, 0.109),
        (0.165, 0.107),
        (0.195, 0.099),
        (0.222, 0.086),
    ]
    for z, r in rib_specs:
        rib = TorusGeometry(r + 0.004, 0.009, radial_segments=12, tubular_segments=64)
        rib.translate(0.0, 0.0, z)
        body.merge(rib)
    # Threaded neck rings sit around the open mouth without sealing its center.
    for i, tz in enumerate((0.252, 0.262, 0.272)):
        thread = TorusGeometry(NECK_RADIUS - 0.002, 0.004, radial_segments=10, tubular_segments=48)
        thread.translate(0.0, 0.0, tz)
        body.merge(thread)
    return mesh_from_geometry(body, "barrel_shell")


def _lid_geometry():
    # Black screw cap with a closed top/center; the cap is lifted in the
    # default pose so the barrel mouth remains visible.
    cap = CylinderGeometry(LID_OUTER_R, LID_HEIGHT, radial_segments=48)
    cap.translate(0.0, 0.0, LID_HEIGHT / 2.0)
    # Knurl ridges around the skirt.
    n = 28
    for k in range(n):
        a = 2.0 * math.pi * k / n
        ridge = CylinderGeometry(0.0035, LID_HEIGHT * 0.85, radial_segments=6)
        ridge.translate(LID_OUTER_R, 0.0, LID_HEIGHT / 2.0)
        ridge.rotate_z(a)
        cap.merge(ridge)
    # Slightly domed top cap rim.
    rim = TorusGeometry(LID_OUTER_R - 0.006, 0.006, radial_segments=10, tubular_segments=48)
    rim.translate(0.0, 0.0, LID_HEIGHT)
    cap.merge(rim)
    return cap


def _lid_mesh():
    cap = _lid_geometry()
    return mesh_from_geometry(cap, "lid_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blue_barrel_keg")

    blue = model.material("barrel_blue", rgba=(0.16, 0.34, 0.78, 1.0))
    black = model.material("cap_black", rgba=(0.10, 0.10, 0.11, 1.0))

    # ---- body (root) --------------------------------------------------------
    body = model.part("body")
    body.visual(_body_mesh(), material=blue, name="barrel_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(0.11, 0.27), mass=2.2, origin=Origin(xyz=(0.0, 0.0, 0.13))
    )

    # ---- massless carrier (decouples the two cap joints) --------------------
    carrier = model.part("lid_carrier")  # NO visuals
    carrier.inertial = Inertial.from_geometry(Box((0.01, 0.01, 0.01)), mass=1e-4)

    # ---- lid (black screw cap) ----------------------------------------------
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=black, name="lid_shell")
    # Same-material molded key so rotation is visibly detectable.
    lid.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(LID_OUTER_R - 0.010, 0.0, LID_HEIGHT + 0.004)),
        material=black,
        name="lid_molded_key",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_OUTER_R, LID_HEIGHT), mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, LID_HEIGHT / 2.0)),
    )

    # ---- articulations: decoupled screw cap, both about +Z ------------------
    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_REST_CLEARANCE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-LID_REST_CLEARANCE,
            upper=LID_HEIGHT,
            effort=1.0,
            velocity=1.0,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")

    # The cap skirt intentionally slips over the threaded neck when the cap is
    # lowered from the default open display pose.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="barrel_shell",
        reason="The screw-cap skirt is intentionally seated over the barrel neck/threads when lowered.",
    )
    ctx.allow_isolated_part(
        lid,
        reason="Default display pose lifts the solid screw cap so the open barrel mouth is visible.",
    )

    # Lid is on top of the barrel.
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits on top of the barrel",
        lid_pos is not None and lid_pos[2] >= BARREL_TOP_Z - 0.02,
        details=f"lid origin={lid_pos}",
    )

    # The default pose leaves the solid cap lifted so the open mouth is visible.
    ctx.check(
        "default lid is raised clear of the open mouth",
        lid_pos is not None and lid_pos[2] >= NECK_TOP_Z + LID_REST_CLEARANCE - 0.002,
        details=f"lid origin={lid_pos}, neck_top={NECK_TOP_Z}",
    )

    # Sliding the cap down seats it over the neck; sliding up lifts it farther.
    with ctx.pose({slide: -LID_REST_CLEARANCE}):
        ctx.expect_overlap(
            lid, body, axes="z", min_overlap=0.004, name="cap seated over neck when lowered"
        )
    rest_z = ctx.part_world_position(lid)[2]
    with ctx.pose({slide: LID_HEIGHT}):
        lifted_z = ctx.part_world_position(lid)[2]
    ctx.check(
        "sliding lid_slide lifts the lid up off the neck",
        lifted_z > rest_z + 0.02,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # Rotating lid_rotate spins the lid: the off-axis molded key swings,
    # so the key's world AABB center moves.
    def _center(aabb):
        mn, mx = aabb
        return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0)

    rest_c = _center(ctx.part_element_world_aabb(lid, elem="lid_molded_key"))
    with ctx.pose({rotate: math.pi / 2.0}):
        spun_c = _center(ctx.part_element_world_aabb(lid, elem="lid_molded_key"))
    moved = abs(spun_c[0] - rest_c[0]) + abs(spun_c[1] - rest_c[1])
    ctx.check(
        "rotating lid_rotate spins the lid (off-axis molded key moves)",
        moved > 0.005,
        details=f"rest_c={rest_c}, spun_c={spun_c}, moved={moved}",
    )

    # Barrel has horizontal ribs: the belly is wider than the neck (bulged,
    # ribbed body reads as a barrel, not a plain cylinder).
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "barrel body is bulged/ribbed (wide belly)",
        body_ext[0] > 0.20 and body_ext[1] > 0.20,
        details=f"body extents={body_ext}",
    )

    # Geometry-specific checks for this requested correction: bottom present,
    # hollow interior, and visible mouth opening.
    body_geom = _barrel_body()
    radii_z = [(math.hypot(x, y), z) for x, y, z in body_geom.vertices]
    bottom_has_center = any(
        r < 0.003 and abs(z - BODY_BOTTOM_Z) < 1e-6 for r, z in radii_z
    )
    ctx.check(
        "barrel bottom has a center face",
        bottom_has_center,
        details="bottom includes vertices on the center axis at z=0",
    )
    mid_band = [r for r, z in radii_z if 0.100 <= z <= 0.120]
    ctx.check(
        "barrel interior remains hollow at the belly",
        bool(mid_band) and min(mid_band) > 0.090,
        details=f"mid-band min radius={min(mid_band) if mid_band else None}",
    )
    mouth_band = [r for r, z in radii_z if abs(z - NECK_TOP_Z) < 1e-6]
    ctx.check(
        "barrel mouth is a visible opening",
        bool(mouth_band) and min(mouth_band) >= MOUTH_RADIUS - 0.001,
        details=f"mouth radii min/max={((min(mouth_band), max(mouth_band)) if mouth_band else None)}",
    )
    lid_geom = _lid_geometry()
    lid_top_radii = [
        math.hypot(x, y) for x, y, z in lid_geom.vertices if abs(z - LID_HEIGHT) < 1e-6
    ]
    ctx.check(
        "screw cap has a closed center",
        bool(lid_top_radii) and min(lid_top_radii) <= 0.003,
        details=(
            "lid top radii min/max="
            f"{((min(lid_top_radii), max(lid_top_radii)) if lid_top_radii else None)}"
        ),
    )

    return ctx.report()
