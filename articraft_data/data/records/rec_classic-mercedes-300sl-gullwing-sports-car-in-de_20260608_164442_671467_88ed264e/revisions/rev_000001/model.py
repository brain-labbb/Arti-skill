from __future__ import annotations

# Classic Mercedes 300SL "Gullwing" sports car in deep maroon red.
# Z-up world. Long axis of the car runs along +Y (nose at +Y), width along X
# (driver/left side at +X), up along +Z. Wheels touch z = 0.
#
# Modeled in the spirit of the Diablo wedge supercar in this dataset: a finely
# lofted solid lower body with CARVED-OPEN wheel arches, axle channels bored
# clean through, a HOLLOWED cabin (open floor, seats, steering wheel) and door
# apertures cut THROUGH the flanks so an open door reveals the cabin (not a
# solid cross-section).
#
# Primary articulation: the TWO GULLWING doors. Unlike the Diablo's scissor
# doors (which hinge at the front-top SIDE edge and rise along the flank), a
# gullwing door hinges along a LONGITUDINAL (fore-aft, +/-Y) axis at the ROOF
# RIDGE, just off the centerline. Each door carries its half of the roof, the
# wraparound side window and the upper flank skin, and swings UP-AND-OUTWARD --
# away from the body, peaking above the roofline like a seagull's wing.
# Secondary: all four wheels spin (continuous, lateral X axle); the two FRONT
# wheels additionally STEER about a vertical king-pin (revolute about Z).
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TireCarcass,
    TireGeometry,
    TireSidewall,
    TorusGeometry,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    boolean_difference,
    boolean_intersection,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
)

# ----------------------------------------------------------------------------
# Global proportions (meters). Real 300SL: ~4.52 L x 1.79 W x 1.30 H,
# wheelbase ~2.40, track ~1.38, wheel radius ~0.34.
# ----------------------------------------------------------------------------
WHEEL_R = 0.335
WHEEL_W = 0.205
HALF_TRACK = 0.745
FRONT_AXLE_Y = 1.21
REAR_AXLE_Y = -1.19

# Wheel arches are carved out of the solid lower body so the wheels sit in tight
# open wells. Each cutter is a lateral (X-axis) cylinder hugging the wheel that
# opens only the OUTBOARD flank: it spans |x| from an inboard wall (just past the
# tire's inner face) out through the body side. Fronts reach a touch deeper
# inboard so the steered front tire clears.
ARCH_RADIUS = 0.36
# (x_center_sign*track, y_center, inboard_wall_abs_x, outboard_wall_abs_x)
WHEEL_ARCHES = (
    (HALF_TRACK, FRONT_AXLE_Y, 0.46, 1.02),
    (-HALF_TRACK, FRONT_AXLE_Y, 0.46, 1.02),
    (HALF_TRACK, REAR_AXLE_Y, 0.55, 1.02),
    (-HALF_TRACK, REAR_AXLE_Y, 0.55, 1.02),
)

# Straight axle rods run hub-to-hub at wheel-center height; they show in the open
# wheel wells and tuck under the centre of the car.
AXLE_BAR_RADIUS = 0.042

# The cabin is hollowed out of the solid body so the seats + steering wheel sit
# in real open space, and each door aperture is cut clean THROUGH the upper
# flank so opening a door reveals the cabin. (x is half-width; y/z are (min,max).)
CABIN_HALF_X = 0.56
CABIN_Y = (-0.82, 0.86)
# Hollow runs a little below the door-aperture floor so each door-skin flank
# chunk keeps a clean inboard wall at the cabin (no inward dip into the seats).
CABIN_Z = (0.42, 0.70)  # above the floor pan, up to the body top
# The door aperture is sized just inside the door panel (which spans y -0.42..0.34)
# so the closed door fully seats over and seals it -- no open gap in the flank.
DOOR_APERTURE_X = (0.46, 0.84)  # inboard (into cabin) .. outboard (through flank)
DOOR_APERTURE_Y = (-0.38, 0.30)
DOOR_APERTURE_Z = (0.44, 0.71)

# Gullwing hinge: a LONGITUDINAL (fore-aft) axis at the ROOF SHOULDER -- the top
# edge of the side window, where the flank meets the fixed roof. Each door
# (side window + flank skin) hangs below it; the left (+X) door pivots about -Y,
# the right (-X) door about +Y, so positive q lifts the whole panel UP-AND-
# OUTWARD, away from the body. The roof stays one complete fixed dome.
DOOR_HINGE_X = 0.52
HINGE_Z = 0.88
DOOR_AXIS_LEFT = (0.0, -1.0, 0.0)
DOOR_AXIS_RIGHT = (0.0, 1.0, 0.0)
DOOR_OPEN_MAX = 1.20

# Temporarily drop the two gullwing doors and the cant-rail "bar" they hinge
# against, to inspect the bare body / greenhouse structure. Flip back to True to
# restore the doors.
INCLUDE_DOORS = False

# Steering wheel on the driver (+X) side: the column is fixed on the body; the
# detailed wheel is a separate part that SPINS about the raked column axis.
STEER_WHEEL_X = 0.32
STEER_WHEEL_RAKE = 0.95  # column tilt about X (toward the driver)
STEER_WHEEL_HUB = (STEER_WHEEL_X, 0.40, 0.70)

# Finely sectioned lower-body side rails: (y, z_min, z_max, width). Soft rounded
# nose, swelling front fenders with the headlamps on the leading crown, a tucked
# door-sill waist, swelling rear fenders, and a short rounded tail. Authored
# nose-first (decreasing y) -- the same ordering the Diablo body uses.
LOWER_SECTIONS = [
    (2.32, 0.21, 0.40, 0.46),
    (2.24, 0.16, 0.45, 0.78),
    (2.12, 0.13, 0.51, 1.10),
    (1.97, 0.11, 0.60, 1.42),
    (1.80, 0.10, 0.70, 1.57),
    (1.62, 0.10, 0.75, 1.62),  # front fender crown (leading)
    (1.42, 0.10, 0.76, 1.63),
    (1.21, 0.10, 0.76, 1.62),  # over the front axle
    (1.00, 0.11, 0.70, 1.55),
    (0.72, 0.12, 0.65, 1.47),  # door-sill waist
    (0.30, 0.12, 0.64, 1.45),
    (-0.15, 0.12, 0.64, 1.46),
    (-0.58, 0.12, 0.67, 1.52),
    (-0.90, 0.11, 0.74, 1.61),  # rear fender crown
    (-1.19, 0.10, 0.76, 1.63),  # over the rear axle
    (-1.45, 0.11, 0.73, 1.58),
    (-1.70, 0.13, 0.67, 1.42),
    (-1.94, 0.16, 0.57, 1.10),
    (-2.16, 0.20, 0.49, 0.74),
    (-2.30, 0.26, 0.44, 0.48),
]

# Cab greenhouse, built (like the Diablo) as thin shells that share seam rails so
# the windshield / roof / rear window meet seamlessly and each reads as a thin
# skin. The ROOF is ONE complete fixed dome; the gullwing doors hinge at its
# shoulder and carry only the side glass + flank skin.
_SEAM_FRONT = (0.46, 0.88, 1.12, 1.10)  # roof leading edge
_SEAM_REAR = (-0.56, 0.88, 1.13, 1.12)  # roof trailing edge
ROOF_SECTIONS = [
    _SEAM_FRONT,
    (0.16, 0.88, 1.17, 1.20),
    (-0.24, 0.88, 1.17, 1.20),
    _SEAM_REAR,
]
GLASS_SHELL_T = 0.016


def _save(name: str, geom):
    return mesh_from_geometry(geom, name)


def _box_cutter(x0, x1, y0, y1, z0, z1):
    # Axis-aligned box spanning the given world ranges, for boolean carving.
    # BoxGeometry ships INWARD-wound (negative volume); manifold3d reads that as
    # empty space, so a raw box is a silent no-op as a subtrahend. Flip the face
    # winding so it is a real solid cutter.
    box = BoxGeometry((x1 - x0, y1 - y0, z1 - z0)).translate(
        (x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0
    )
    return MeshGeometry(
        vertices=list(box.vertices),
        faces=[(f[0], f[2], f[1]) for f in box.faces],
    )


def _solid(geom):
    # Export the WATERTIGHT native manifold triangulation. The SDK's boolean
    # output stores the real manifold on `geom._manifold_source`, but the
    # triangle mesh it writes (`geom.vertices/faces`) is a coplanar
    # re-triangulation that leaves T-junctions -> the mesh is NOT watertight, so
    # the body shows torn holes / jagged "burr" spikes and open seams. Re-reading
    # the cached manifold's own triangulation gives a fully closed (watertight)
    # body with no gaps.
    import numpy as _np

    man = getattr(geom, "_manifold_source", None)
    if man is None:
        return geom
    out = man.to_mesh()
    vp = _np.asarray(out.vert_properties)
    tv = _np.asarray(out.tri_verts)
    if vp.size == 0 or tv.size == 0:
        return geom
    vp = vp[:, :3]
    return MeshGeometry(
        vertices=[(float(v[0]), float(v[1]), float(v[2])) for v in vp],
        faces=[(int(f[0]), int(f[1]), int(f[2])) for f in tv],
    )


_BODY_PARTS_CACHE = None


def _build_body_parts():
    # Rounded solid body, then: carve a lateral wheel-arch tunnel at each wheel,
    # hollow the cabin, and cut a door aperture through each upper flank. The exact
    # flank chunk each aperture removes is captured (boolean_intersection) and
    # reused as that door's skin, so the closed door fills its hole perfectly
    # flush -- same curvature, no gap and no protrusion. Cached because the carve
    # is reused by the visual export, the door skins and the QC checks.
    global _BODY_PARTS_CACHE
    if _BODY_PARTS_CACHE is None:
        body = superellipse_side_loft(LOWER_SECTIONS, exponents=2.6, segments=72)
        for ax, ay, inboard, outboard in WHEEL_ARCHES:
            sign = 1.0 if ax > 0 else -1.0
            arch = (
                CylinderGeometry(
                    radius=ARCH_RADIUS, height=outboard - inboard, radial_segments=48
                )
                .rotate_y(pi / 2.0)  # long axis Z -> X (lateral wheel-arch tunnel)
                .translate(sign * (inboard + outboard) / 2.0, ay, WHEEL_R)
            )
            body = boolean_difference(body, arch)
        body = boolean_difference(
            body,
            _box_cutter(-CABIN_HALF_X, CABIN_HALF_X, CABIN_Y[0], CABIN_Y[1], CABIN_Z[0], CABIN_Z[1]),
        )
        skins = {}
        for side, sgn in (("left", 1.0), ("right", -1.0)):
            xa, xb = sorted((sgn * DOOR_APERTURE_X[0], sgn * DOOR_APERTURE_X[1]))
            apbox = _box_cutter(
                xa, xb, DOOR_APERTURE_Y[0], DOOR_APERTURE_Y[1], DOOR_APERTURE_Z[0], DOOR_APERTURE_Z[1]
            )
            skins[side] = _solid(boolean_intersection(body, apbox))
            body = boolean_difference(body, apbox)
        _BODY_PARTS_CACHE = {
            "body": _solid(body),
            "skin_left": skins["left"],
            "skin_right": skins["right"],
        }
    return _BODY_PARTS_CACHE


def _lower_body_mesh():
    return _build_body_parts()["body"].clone()


def _door_skin_mesh(side):
    return _build_body_parts()["skin_" + side].clone()


def _shell(sections, t=GLASS_SHELL_T):
    # Thin, uniform-thickness, open-bottom shell of a side-loft: subtract an inset
    # copy (top/sides pulled in by t, bottom dropped well below so the underside
    # opens into the cabin). Shells sharing seam sections meet seamlessly.
    outer = superellipse_side_loft(sections, exponents=2.7, segments=60)
    inner = superellipse_side_loft(
        [(y, zmin - 0.18, zmax - t, max(w - 2.0 * t, 0.04)) for (y, zmin, zmax, w) in sections],
        exponents=2.7,
        segments=60,
    )
    # superellipse_side_loft authored nose-first is wound so that, as a subtrahend
    # here, manifold3d would read it as empty -> flip the inner faces so it carves.
    inner = MeshGeometry(
        vertices=list(inner.vertices),
        faces=[(f[0], f[2], f[1]) for f in inner.faces],
    )
    return boolean_difference(outer, inner)


# Shared wheel/tire geometry: smooth chrome disc wheel with a domed hubcap (the
# 300SL does not use openwork spoke wheels), low-profile classic tire.
_TIRE_GEOM = TireGeometry(
    WHEEL_R,
    WHEEL_W,
    inner_radius=WHEEL_R * 0.64,
    carcass=TireCarcass(belt_width_ratio=0.62, sidewall_bulge=0.05),
    sidewall=TireSidewall(style="rounded", bulge=0.05),
)
_WHEEL_GEOM = WheelGeometry(
    WHEEL_R * 0.64,
    WHEEL_W * 0.58,
    rim=WheelRim(inner_radius=WHEEL_R * 0.50, flange_height=0.012, flange_thickness=0.006),
    hub=WheelHub(radius=WHEEL_R * 0.32, width=WHEEL_W * 0.5, cap_style="domed"),
    face=WheelFace(dish_depth=0.010, front_inset=0.004),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mercedes_300sl_gullwing")

    maroon = model.material("maroon", rgba=(0.43, 0.05, 0.07, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.85, 1.0))
    dark_chrome = model.material("dark_chrome", rgba=(0.27, 0.28, 0.30, 1.0))
    model.material("glass_dark", rgba=(0.07, 0.08, 0.10, 1.0))  # referenced by QC by name
    glass_tint = model.material("glass_tint", rgba=(0.10, 0.12, 0.15, 0.40))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.055, 1.0))
    lens_pale = model.material("lens_pale", rgba=(0.86, 0.88, 0.82, 1.0))
    amber = model.material("amber", rgba=(0.85, 0.52, 0.10, 1.0))
    red_tail = model.material("tail_red", rgba=(0.60, 0.06, 0.05, 1.0))
    interior_dk = model.material("interior_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.64, 0.65, 0.68, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")
    body.visual(_save("lower_body.obj", _lower_body_mesh()), material=maroon, name="lower_body")

    # Greenhouse: a complete fixed roof dome PLUS a body-color frame (A-pillars,
    # rear-quarter sails, cant rails) that fully ENCLOSES the cabin sides -- the
    # only openings are glass: the windshield, the side windows (on the doors) and
    # the rear window. Windshield + rear glass are clean flat raked panels (Box ->
    # no shell side-spikes).
    body.visual(_save("roof.obj", _solid(_shell(ROOF_SECTIONS))), material=maroon, name="roof")
    # Windshield + rear window: thin glass panels with ROUNDED corners (extruded
    # rounded rectangles) so there are no sharp poking corners, tucked just inside
    # the A/C-pillars and header.
    body.visual(
        _save(
            "windshield.obj",
            ExtrudeGeometry(rounded_rect_profile(0.92, 0.62, 0.16), 0.04)
            .rotate_x(-0.70)
            .translate(0.0, 0.735, 0.88),
        ),
        material=glass_tint,
        name="windshield",
    )
    body.visual(
        _save(
            "rear_window.obj",
            ExtrudeGeometry(rounded_rect_profile(0.88, 0.70, 0.16), 0.04)
            .rotate_x(0.44)
            .translate(0.0, -0.90, 0.94),
        ),
        material=glass_tint,
        name="rear_window",
    )
    # Clean body-color header panels capping the roof's open front and rear edges
    # (otherwise the roof shell's end-cap triangulation pokes into the cabin as
    # stray triangles).
    body.visual(
        Box((1.06, 0.10, 0.27)),
        origin=Origin(xyz=(0.0, 0.46, 1.005)),
        material=maroon,
        name="roof_header_front",
    )
    body.visual(
        Box((1.04, 0.10, 0.27)),
        origin=Origin(xyz=(0.0, -0.56, 1.01)),
        material=maroon,
        name="roof_header_rear",
    )
    for s, side in ((1.0, "left"), (-1.0, "right")):
        # A-pillar: solid body-color post framing the windshield side, cowl->roof.
        body.visual(
            Box((0.09, 0.70, 0.13)),
            origin=Origin(xyz=(s * 0.515, 0.735, 0.85), rpy=(-0.70, 0.0, 0.0)),
            material=maroon,
            name=f"a_pillar_{side}",
        )
        # Rear quarter: solid body-color sail enclosing the rear cabin side (a
        # coupe has a solid quarter here, no rear side glass).
        body.visual(
            Box((0.10, 0.74, 0.30)),
            origin=Origin(xyz=(s * 0.52, -0.82, 0.76)),
            material=maroon,
            name=f"rear_quarter_{side}",
        )
        # C-pillar: body-color post framing the rear-window side, roof->deck.
        body.visual(
            Box((0.09, 0.74, 0.13)),
            origin=Origin(xyz=(s * 0.49, -0.90, 0.90), rpy=(0.44, 0.0, 0.0)),
            material=maroon,
            name=f"c_pillar_{side}",
        )
        if INCLUDE_DOORS:
            # Cant rail: body-color rail along the roof shoulder joining the
            # pillars and capping the side window (the bar the door hinges against).
            body.visual(
                Box((0.08, 1.18, 0.09)),
                origin=Origin(xyz=(s * 0.55, -0.06, 0.85)),
                material=maroon,
                name=f"cant_rail_{side}",
            )
            # Slim bright chrome drip strip along the cant rail.
            body.visual(
                Box((0.05, 1.16, 0.022)),
                origin=Origin(xyz=(s * 0.575, -0.06, 0.895)),
                material=chrome,
                name=f"drip_trim_{side}",
            )
    # Bright chrome header across the windshield + rear window tops.
    body.visual(
        Box((1.02, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.48, 1.10)),
        material=chrome,
        name="windshield_header",
    )
    body.visual(
        Box((1.00, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, -0.56, 1.10)),
        material=chrome,
        name="rear_header",
    )

    # ---- Cabin interior (floor pan, trim panels, two seats, dashboard) --------
    body.visual(
        Box((1.10, 1.50, 0.07)),
        origin=Origin(xyz=(0.0, 0.02, 0.45)),
        material=interior_dk,
        name="cabin_floor",
    )
    # Flat interior trim panels covering the raw structural cabin walls (sides +
    # a rear bulkhead) so the cabin reads smoothly trimmed, not bumpy sheetmetal.
    for sx, side in ((0.55, "left"), (-0.55, "right")):
        body.visual(
            Box((0.05, 1.52, 0.26)),
            origin=Origin(xyz=(sx, 0.02, 0.55)),
            material=interior_dk,
            name=f"cabin_side_trim_{side}",
        )
    body.visual(
        Box((1.08, 0.05, 0.30)),
        origin=Origin(xyz=(0.0, -0.78, 0.57)),
        material=interior_dk,
        name="cabin_bulkhead",
    )
    for sx, side in ((0.27, "left"), (-0.27, "right")):
        body.visual(
            Box((0.38, 0.44, 0.10)),
            origin=Origin(xyz=(sx, -0.06, 0.50)),
            material=interior_dk,
            name=f"seat_cushion_{side}",
        )
        body.visual(
            Box((0.38, 0.14, 0.34)),
            origin=Origin(xyz=(sx, -0.34, 0.64)),
            material=interior_dk,
            name=f"seat_back_{side}",
        )
    # Dashboard across the cowl, under the windshield.
    body.visual(
        Box((1.04, 0.18, 0.16)),
        origin=Origin(xyz=(0.0, 0.66, 0.60)),
        material=interior_dk,
        name="dashboard",
    )
    # Steering column (fixed on the body): a raked shaft from the dashboard up to
    # the wheel hub, perfectly in line with the wheel's spin axis. The rotating
    # wheel itself is a separate articulated part (see make_steering_wheel).
    body.visual(
        Cylinder(radius=0.018, length=0.34),
        origin=Origin(xyz=(STEER_WHEEL_X, 0.54, 0.60), rpy=(STEER_WHEEL_RAKE, 0.0, 0.0)),
        material=interior_dk,
        name="steering_column",
    )

    # ---- Twin raised hood "power domes" (a 300SL signature) -------------------
    # Half-buried rounded capsules (no flat end caps to poke through) raked to
    # follow the hood's drop toward the nose, so they read as smooth swells.
    for sx, side in ((0.18, "left"), (-0.18, "right")):
        dome = (
            CapsuleGeometry(radius=0.065, length=0.50, radial_segments=20)
            .rotate_x(pi / 2.0 - 0.20)
            .translate(sx, 1.66, 0.705)
        )
        body.visual(_save(f"hood_dome_{side}.obj", dome), material=maroon, name=f"hood_dome_{side}")

    # ---- Front face: split chrome bumper, oval grille with the 3-pointed star --
    for sx, side in ((0.40, "left"), (-0.40, "right")):
        body.visual(
            Box((0.50, 0.07, 0.09)),
            origin=Origin(xyz=(sx, 2.16, 0.40)),
            material=chrome,
            name=f"front_bumper_{side}",
        )
        body.visual(
            Cylinder(radius=0.05, length=0.12),
            origin=Origin(xyz=(sx - (0.21 if sx > 0 else -0.21), 2.15, 0.40), rpy=(0.0, 0.0, 0.0)),
            material=chrome,
            name=f"front_bumper_guard_{side}",
        )
    body.visual(
        Box((0.76, 0.06, 0.32)),
        origin=Origin(xyz=(0.0, 2.13, 0.45)),
        material=chrome,
        name="grille_surround",
    )
    body.visual(
        Box((0.64, 0.05, 0.22)),
        origin=Origin(xyz=(0.0, 2.15, 0.45)),
        material=dark_chrome,
        name="grille_mesh",
    )
    # Big 3-pointed star in the grille: a chrome ring + three radial bars.
    body.visual(
        _save(
            "star_ring.obj",
            TorusGeometry(radius=0.085, tube=0.012, radial_segments=10, tubular_segments=36),
        ),
        origin=Origin(xyz=(0.0, 2.15, 0.47), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="grille_star_ring",
    )
    for k, ang in enumerate((0.0, 2.0944, 4.1888)):  # 0, 120, 240 deg
        body.visual(
            Box((0.012, 0.04, 0.085)),
            origin=Origin(xyz=(0.0, 2.16, 0.47), rpy=(0.0, ang, 0.0)),
            material=chrome,
            name=f"grille_star_bar_{k}",
        )

    # ---- Round chrome-ringed headlamps set into the front-fender crowns -------
    for sx, side in ((0.46, "left"), (-0.46, "right")):
        body.visual(
            Cylinder(radius=0.115, length=0.10),
            origin=Origin(xyz=(sx, 1.92, 0.54), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"headlight_ring_{side}",
        )
        body.visual(
            Sphere(radius=0.098),
            origin=Origin(xyz=(sx, 1.95, 0.54)),
            material=lens_pale,
            name=f"headlight_{side}",
        )
        # Small amber turn/parking lamp tucked below the headlamp.
        body.visual(
            Cylinder(radius=0.042, length=0.09),
            origin=Origin(xyz=(sx, 1.96, 0.38), rpy=(pi / 2.0, 0.0, 0.0)),
            material=amber,
            name=f"front_indicator_{side}",
        )

    # ---- Chrome rocker sill trim along each lower flank -----------------------
    for sx, side in ((0.71, "left"), (-0.71, "right")):
        body.visual(
            Box((0.06, 1.96, 0.07)),
            origin=Origin(xyz=(sx, -0.05, 0.24)),
            material=chrome,
            name=f"rocker_trim_{side}",
        )

    # ---- Round chrome "bullet" wing mirrors mounted beside the windshield -----
    # The chrome stalk roots into the SOLID cowl/A-pillar base just outboard of
    # the windshield (its lower end plunges down into the sheetmetal at ~z 0.50)
    # and rises into a round chrome housing whose rear face carries a dark
    # reflective glass disc -- firmly attached to the body, not floating.
    for s, side in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Cylinder(radius=0.015, length=0.18),
            origin=Origin(xyz=(s * 0.63, 0.93, 0.62), rpy=(0.0, s * 0.45, 0.0)),
            material=chrome,
            name=f"mirror_stalk_{side}",
        )
        # Round chrome housing, axis along Y so the disc faces fore/aft.
        body.visual(
            Cylinder(radius=0.052, length=0.05),
            origin=Origin(xyz=(s * 0.69, 0.92, 0.73), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"mirror_housing_{side}",
        )
        # Domed chrome back of the housing.
        body.visual(
            Sphere(radius=0.05),
            origin=Origin(xyz=(s * 0.69, 0.945, 0.73)),
            material=chrome,
            name=f"mirror_back_{side}",
        )
        # Dark reflective glass face toward the driver.
        body.visual(
            Cylinder(radius=0.044, length=0.014),
            origin=Origin(xyz=(s * 0.69, 0.895, 0.73), rpy=(pi / 2.0, 0.0, 0.0)),
            material=glass_tint,
            name=f"mirror_glass_{side}",
        )

    # ---- Front + rear axle rods (hub to hub, shown in the open wheel wells) ----
    for ay, axle_name in ((FRONT_AXLE_Y, "front_axle_bar"), (REAR_AXLE_Y, "rear_axle_bar")):
        body.visual(
            Cylinder(radius=AXLE_BAR_RADIUS, length=2.0 * HALF_TRACK),
            origin=Origin(xyz=(0.0, ay, WHEEL_R), rpy=(0.0, pi / 2.0, 0.0)),
            material=axle_steel,
            name=axle_name,
        )

    # ---- Detailed tail: split chrome bumper + overriders, framed vertical tail-
    # lights (red + amber), recessed license plate, a rear star badge, and twin
    # chrome exhaust tips tucked into the rear valance --------------------------
    for sx, side in ((0.42, "left"), (-0.42, "right")):
        body.visual(
            Box((0.46, 0.07, 0.08)),
            origin=Origin(xyz=(sx, -2.16, 0.39)),
            material=chrome,
            name=f"rear_bumper_{side}",
        )
        body.visual(
            Box((0.07, 0.11, 0.20)),
            origin=Origin(xyz=(sx + (-0.14 if sx > 0 else 0.14), -2.16, 0.41)),
            material=chrome,
            name=f"rear_overrider_{side}",
        )
        # Vertical taillight: chrome surround, red lens over an amber lower section.
        body.visual(
            Box((0.15, 0.05, 0.24)),
            origin=Origin(xyz=(sx, -2.09, 0.46)),
            material=chrome,
            name=f"taillight_surround_{side}",
        )
        body.visual(
            Box((0.11, 0.05, 0.14)),
            origin=Origin(xyz=(sx, -2.105, 0.50)),
            material=red_tail,
            name=f"taillight_{side}",
        )
        body.visual(
            Box((0.11, 0.05, 0.05)),
            origin=Origin(xyz=(sx, -2.105, 0.40)),
            material=amber,
            name=f"rear_indicator_{side}",
        )
    # Recessed license plate with a chrome surround, low on the rear panel.
    body.visual(
        Box((0.36, 0.04, 0.15)),
        origin=Origin(xyz=(0.0, -2.15, 0.30)),
        material=chrome,
        name="plate_surround",
    )
    body.visual(
        Box((0.30, 0.05, 0.11)),
        origin=Origin(xyz=(0.0, -2.165, 0.30)),
        material=lens_pale,
        name="license_plate",
    )
    # Rear three-pointed star badge seated on the rear panel above the plate.
    body.visual(
        _save("rear_star_ring.obj", TorusGeometry(radius=0.042, tube=0.008, radial_segments=8, tubular_segments=28)),
        origin=Origin(xyz=(0.0, -2.15, 0.43), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="rear_star_ring",
    )
    for k, ang in enumerate((0.0, 2.0944, 4.1888)):
        body.visual(
            Box((0.008, 0.02, 0.042)),
            origin=Origin(xyz=(0.0, -2.16, 0.43), rpy=(0.0, ang, 0.0)),
            material=chrome,
            name=f"rear_star_{k}",
        )
    # Twin chrome exhaust tips tucked up under the rear valance (rooted in the body).
    for sx, side in ((0.18, "left"), (-0.18, "right")):
        body.visual(
            Cylinder(radius=0.034, length=0.20),
            origin=Origin(xyz=(sx, -2.20, 0.28), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"exhaust_{side}",
        )

    body.inertial = Inertial.from_geometry(
        Box((1.74, 4.40, 1.24)),
        mass=1295.0,
        origin=Origin(xyz=(0.0, -0.02, 0.55)),
    )

    # ----------------------------------------------------------- gullwing doors
    def make_door(side: str):
        s = 1.0 if side == "left" else -1.0
        hx = s * DOOR_HINGE_X  # hinge x at the roof shoulder
        hz = HINGE_Z
        door = model.part(f"door_{side}")

        # Side window: a clean tinted pane that overlaps the skin top at the
        # beltline (so skin + glass read as ONE continuous door, not floating
        # pieces) and rises to the cant rail. It fills the body's side-window
        # opening between the fixed A-pillar and rear quarter.
        glass_sections = [
            (0.40, 0.60, 0.80, 0.10),
            (0.0, 0.60, 0.83, 0.11),
            (-0.45, 0.60, 0.78, 0.10),
        ]
        glass = superellipse_side_loft(glass_sections, exponents=2.4, segments=44)
        door.visual(
            _save(f"door_{side}_glass.obj", glass.translate(s * 0.555 - hx, 0.0, -hz)),
            material=glass_tint,
            name="door_glass",
        )

        # Lower door skin = the exact flank chunk the body aperture removed, moved
        # into the door's hinge frame. Closed, it refills its hole seamlessly
        # (matching the body's curvature); open, it lifts away with the door.
        door.visual(
            _save(f"door_{side}_skin.obj", _door_skin_mesh(side).translate(-hx, 0.0, -hz)),
            material=maroon,
            name="door_skin",
        )

        # Slim chrome beltline strip along the skin/glass seam + a flush handle.
        # The window's top and side frame is the fixed body greenhouse, so the
        # door itself is simply skin + glass + this trim.
        door.visual(
            Box((0.05, 0.84, 0.022)),
            origin=Origin(xyz=(s * 0.585 - hx, -0.02, 0.605 - hz)),
            material=chrome,
            name="door_belt_rail",
        )
        door.visual(
            Box((0.05, 0.13, 0.03)),
            origin=Origin(xyz=(s * 0.76 - hx, -0.28, 0.55 - hz)),
            material=chrome,
            name="door_handle",
        )

        door.inertial = Inertial.from_geometry(
            Box((0.26, 0.90, 0.50)),
            mass=28.0,
            origin=Origin(xyz=(s * 0.13, -0.05, -0.22)),
        )
        return door

    door_left = make_door("left") if INCLUDE_DOORS else None
    door_right = make_door("right") if INCLUDE_DOORS else None

    # ---------------------------------------------------------------- wheels
    def make_wheel(name: str, outboard_sign: float):
        w = model.part(name)
        face_rpy = (0.0, 0.0, 0.0) if outboard_sign > 0 else (0.0, 0.0, pi)
        w.visual(
            _save(f"{name}_tire.obj", _TIRE_GEOM.clone()),
            origin=Origin(rpy=face_rpy),
            material=rubber,
            name="tire",
        )
        w.visual(
            _save(f"{name}_disc.obj", _WHEEL_GEOM.clone()),
            origin=Origin(xyz=(outboard_sign * 0.03, 0.0, 0.0), rpy=face_rpy),
            material=chrome,
            name="rim",
        )
        # Chrome spinner cap at the hub center.
        w.visual(
            Cylinder(radius=WHEEL_R * 0.13, length=WHEEL_W * 0.7),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_chrome,
            name="hub_center",
        )
        w.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_R, length=WHEEL_W),
            mass=20.0,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        )
        return w

    make_wheel("wheel_front_left", 1.0)
    make_wheel("wheel_front_right", -1.0)
    make_wheel("wheel_rear_left", 1.0)
    make_wheel("wheel_rear_right", -1.0)

    # ----------------------------------------------- front steering knuckles
    def make_knuckle(name: str):
        k = model.part(name)
        k.inertial = Inertial.from_geometry(Box((0.10, 0.10, 0.22)), mass=6.0)
        return k

    knuckle_fl = make_knuckle("steer_knuckle_front_left")
    knuckle_fr = make_knuckle("steer_knuckle_front_right")

    # ------------------------------------------------- rotating steering wheel
    # Authored in the wheel's own (un-raked) frame: rim/spokes lie in the XY
    # plane, the hub runs along +Z. The joint origin applies the column rake and
    # the joint spins the whole wheel about its local +Z (the column axis), so the
    # 3 spokes + 3-pointed-star horn visibly turn.
    steer_wheel = model.part("steering_wheel")
    steer_wheel.visual(
        _save("sw_rim.obj", TorusGeometry(radius=0.150, tube=0.018, radial_segments=16, tubular_segments=56)),
        material=interior_dk,
        name="sw_rim",
    )
    steer_wheel.visual(
        _save("sw_rim_chrome.obj", TorusGeometry(radius=0.150, tube=0.009, radial_segments=10, tubular_segments=56)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=chrome,
        name="sw_rim_chrome",
    )
    for k, ang in enumerate((-pi / 2.0, pi / 6.0, pi - pi / 6.0)):  # 3-spoke layout
        steer_wheel.visual(
            Box((0.145, 0.022, 0.010)),
            origin=Origin(xyz=(0.072 * cos(ang), 0.072 * sin(ang), 0.0), rpy=(0.0, 0.0, ang)),
            material=chrome,
            name=f"sw_spoke_{k}",
        )
    steer_wheel.visual(Cylinder(radius=0.042, length=0.05), material=interior_dk, name="sw_hub")
    steer_wheel.visual(
        Cylinder(radius=0.034, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=chrome,
        name="sw_horn",
    )
    # 3-pointed star on the horn cap (off-axis -> proves the wheel turns).
    steer_wheel.visual(
        _save("sw_star_ring.obj", TorusGeometry(radius=0.025, tube=0.004, radial_segments=8, tubular_segments=24)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=chrome,
        name="sw_star_ring",
    )
    for k, ang in enumerate((pi / 2.0, pi / 2.0 + 2.0944, pi / 2.0 + 4.1888)):
        steer_wheel.visual(
            Box((0.006, 0.025, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.048), rpy=(0.0, 0.0, ang)),
            material=chrome,
            name=f"sw_star_{k}",
        )
    steer_wheel.inertial = Inertial.from_geometry(Cylinder(radius=0.16, length=0.06), mass=2.5)

    # ----------------------------------------------------------- articulations
    # Gullwing doors: revolute about a LONGITUDINAL (fore-aft) axis at the roof
    # shoulder; positive q swings each door UP-AND-OUTWARD.
    if INCLUDE_DOORS:
        model.articulation(
            "door_left_gullwing",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door_left,
            origin=Origin(xyz=(DOOR_HINGE_X, 0.0, HINGE_Z)),
            axis=DOOR_AXIS_LEFT,
            motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN_MAX),
        )
        model.articulation(
            "door_right_gullwing",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door_right,
            origin=Origin(xyz=(-DOOR_HINGE_X, 0.0, HINGE_Z)),
            axis=DOOR_AXIS_RIGHT,
            motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN_MAX),
        )

    # Steering wheel spins about the raked column axis (origin rpy applies the
    # rake; the joint turns the wheel about its local +Z = the column line).
    model.articulation(
        "steering_wheel_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steer_wheel,
        origin=Origin(xyz=STEER_WHEEL_HUB, rpy=(STEER_WHEEL_RAKE, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0, lower=-3.14, upper=3.14),
    )

    # Front steering: revolute about the vertical (Z) king-pin axis, pivoting the
    # knuckle (and the wheel it carries) in place.
    STEER_LOCK = 0.38
    for knuckle, sx in ((knuckle_fl, HALF_TRACK), (knuckle_fr, -HALF_TRACK)):
        model.articulation(
            f"{knuckle.name}_steer",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knuckle,
            origin=Origin(xyz=(sx, FRONT_AXLE_Y, WHEEL_R)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=120.0, velocity=4.0, lower=-STEER_LOCK, upper=STEER_LOCK
            ),
        )

    # Wheel spin: continuous about the lateral X axle. Front wheels spin off their
    # steering knuckle (origin at the knuckle = wheel center, so the spin axle
    # swings with the steer angle); rear wheels spin directly off the body.
    for name, spin_parent, origin_xyz in (
        ("wheel_front_left", knuckle_fl, (0.0, 0.0, 0.0)),
        ("wheel_front_right", knuckle_fr, (0.0, 0.0, 0.0)),
        ("wheel_rear_left", body, (HALF_TRACK, REAR_AXLE_Y, WHEEL_R)),
        ("wheel_rear_right", body, (-HALF_TRACK, REAR_AXLE_Y, WHEEL_R)),
    ):
        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=spin_parent,
            child=name,
            origin=Origin(xyz=origin_xyz),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=200.0, velocity=60.0),
        )

    return model


# >>> USER_CODE_END

object_model = build_object_model()


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door_l = object_model.get_part("door_left") if INCLUDE_DOORS else None
    door_r = object_model.get_part("door_right") if INCLUDE_DOORS else None
    wheels = {
        name: object_model.get_part(name)
        for name in (
            "wheel_front_left",
            "wheel_front_right",
            "wheel_rear_left",
            "wheel_rear_right",
        )
    }
    hinge_l = object_model.get_articulation("door_left_gullwing") if INCLUDE_DOORS else None
    hinge_r = object_model.get_articulation("door_right_gullwing") if INCLUDE_DOORS else None

    # --- Intentional overlap allowances --------------------------------------
    _sw_part = object_model.get_part("steering_wheel")
    for _selem in ("sw_hub", "sw_spoke_0", "sw_spoke_1", "sw_spoke_2"):
        ctx.allow_overlap(
            body,
            _sw_part,
            elem_a="steering_column",
            elem_b=_selem,
            reason="The fixed steering column meets the wheel hub/spokes at the center, on the spin axis.",
        )
    axle_of = {
        "wheel_front_left": "front_axle_bar",
        "wheel_front_right": "front_axle_bar",
        "wheel_rear_left": "rear_axle_bar",
        "wheel_rear_right": "rear_axle_bar",
    }
    for wname, w in wheels.items():
        side = "left" if wname.endswith("left") else "right"
        for elem in ("tire", "rim", "hub_center"):
            ctx.allow_overlap(
                body,
                w,
                elem_a="lower_body",
                elem_b=elem,
                reason="Wheel seated flush inside the carved fender wheel arch of the solid body shell.",
            )
            ctx.allow_overlap(
                body,
                w,
                elem_a=f"rocker_trim_{side}",
                elem_b=elem,
                reason="Chrome rocker sill runs past the wheel arch; minor tuck is intentional.",
            )
            ctx.allow_overlap(
                body,
                w,
                elem_a=axle_of[wname],
                elem_b=elem,
                reason="Straight axle rod runs out to the wheel hub on the spin/steer axis.",
            )
    # Each gullwing door seats flush into the body apertures (side window opening,
    # upper-flank opening, roof shoulder); thin embed with the shells/sill is intended.
    for door, side in ((door_l, "left"), (door_r, "right")) if INCLUDE_DOORS else ():
        for shell in (
            "lower_body",
            "roof",
            "windshield",
            "rear_window",
            f"a_pillar_{side}",
            f"c_pillar_{side}",
            f"rear_quarter_{side}",
            f"cant_rail_{side}",
            f"drip_trim_{side}",
            f"rocker_trim_{side}",
        ):
            for delem in (
                "door_glass",
                "door_skin",
                "door_belt_rail",
            ):
                ctx.allow_overlap(
                    body,
                    door,
                    elem_a=shell,
                    elem_b=delem,
                    reason="Gullwing door seats flush in the body apertures; thin flush embed is intentional.",
                )

    # --- Hero features present and legible -----------------------------------
    vis_names = {v.name for v in body.visuals}
    ctx.check(
        "lofted lower body + complete fixed roof dome present (not a box)",
        {"lower_body", "roof"} <= vis_names,
        details=f"body visuals={sorted(vis_names)}",
    )
    ctx.check(
        "windshield and rear window glass present",
        {"windshield", "rear_window"} <= vis_names,
    )
    ctx.check(
        "round chrome headlamps both sides",
        {"headlight_left", "headlight_right", "headlight_ring_left", "headlight_ring_right"}
        <= vis_names,
    )
    ctx.check(
        "oval grille with 3-pointed star",
        {"grille_surround", "grille_mesh", "grille_star_ring"} <= vis_names
        and sum(1 for v in body.visuals if v.name.startswith("grille_star_bar_")) == 3,
    )
    ctx.check(
        "chrome split bumpers, rocker sills, two detailed wing mirrors",
        {
            "front_bumper_left",
            "front_bumper_right",
            "rear_bumper_left",
            "rear_bumper_right",
            "rocker_trim_left",
            "rocker_trim_right",
            "mirror_housing_left",
            "mirror_housing_right",
            "mirror_glass_left",
            "mirror_glass_right",
        }
        <= vis_names,
    )
    ctx.check(
        "front and rear axle shafts present",
        {"front_axle_bar", "rear_axle_bar"} <= vis_names,
    )

    # --- Hollowed cabin: open interior with seats + steering wheel ------------
    ctx.check(
        "cabin has a floor, two seats and a dashboard",
        {
            "cabin_floor",
            "seat_cushion_left",
            "seat_cushion_right",
            "seat_back_left",
            "seat_back_right",
            "dashboard",
        }
        <= vis_names,
    )
    sw = object_model.get_part("steering_wheel")
    sw_names = {v.name for v in sw.visuals}
    ctx.check(
        "detailed steering wheel: rim, chrome ring, 3 spokes, hub, horn + star",
        {"sw_rim", "sw_rim_chrome", "sw_hub", "sw_horn", "sw_star_ring"} <= sw_names
        and sum(1 for n in sw_names if n.startswith("sw_spoke_")) == 3
        and "steering_column" in vis_names,
        details=f"sw visuals={sorted(sw_names)}",
    )
    # Steering wheel TURNS about the column axis: a spoke tip swings while the hub
    # center stays put (the column line runs through it).
    sw_turn = object_model.get_articulation("steering_wheel_turn")
    ctx.check(
        "steering wheel joint is revolute about its column axis",
        tuple(sw_turn.axis) == (0.0, 0.0, 1.0)
        and sw_turn.articulation_type == ArticulationType.REVOLUTE,
        details=f"axis={sw_turn.axis}",
    )
    hub_rest = ctx.part_world_position(sw)
    spoke_rest = ctx.part_element_world_aabb(sw, elem="sw_spoke_0")
    assert hub_rest is not None and spoke_rest is not None
    with ctx.pose({sw_turn: 1.2}):
        hub_turn = ctx.part_world_position(sw)
        spoke_turn = ctx.part_element_world_aabb(sw, elem="sw_spoke_0")
    assert hub_turn is not None and spoke_turn is not None
    ctx.check(
        "steering wheel spins in place (hub center fixed, a spoke sweeps)",
        all(abs(hub_rest[i] - hub_turn[i]) < 1e-3 for i in range(3))
        and max(abs(spoke_rest[0][i] - spoke_turn[0][i]) for i in range(3)) > 0.05,
        details=f"hub rest={hub_rest}, turned={hub_turn}",
    )
    # The cabin is genuinely hollowed: the body mesh has no solid sheetmetal in
    # the middle of the cabin volume (a vertical ray near the centerline above the
    # floor passes through open space, not buried in the loft interior).
    cabin_pts = [
        (x, y, z)
        for (x, y, z) in _lower_body_mesh().vertices
        if abs(x) < 0.30 and -0.5 < y < 0.5 and 0.50 < z < 0.62
    ]
    ctx.check(
        "cabin interior is hollow (no body sheetmetal mid-cabin above the floor)",
        len(cabin_pts) == 0,
        details=f"stray mid-cabin body verts={len(cabin_pts)}",
    )

    # --- Front fender drapes over the wheel (well capped from above) ----------
    fender_cover = max(
        (
            z
            for (x, y, z) in _lower_body_mesh().vertices
            if abs(abs(x) - HALF_TRACK) < 0.18 and abs(y - FRONT_AXLE_Y) < 0.30
        ),
        default=0.0,
    )
    ctx.check(
        "front fender drapes above the front wheel top (caps the well from above)",
        fender_cover > 2.0 * WHEEL_R + 0.02,
        details=f"front fender top z={fender_cover:.3f}, wheel top z={2.0 * WHEEL_R:.3f}",
    )

    # --- Scale sanity ---------------------------------------------------------
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    lo, hi = bb
    ctx.check("car length ~4.5 m", 4.3 <= hi[1] - lo[1] <= 4.7, details=f"L={hi[1] - lo[1]:.3f}")
    ctx.check("car width ~1.7 m", 1.55 <= hi[0] - lo[0] <= 1.95, details=f"W={hi[0] - lo[0]:.3f}")
    ctx.check("car height ~1.25 m", 1.12 <= hi[2] <= 1.40, details=f"H={hi[2]:.3f}")

    # --- Glass reads darker than the maroon paint ----------------------------
    mats = {m.name: m for m in object_model.materials}
    glass_rgb = sum(mats["glass_dark"].rgba[:3])
    body_rgb = sum(mats["maroon"].rgba[:3])
    ctx.check(
        "glass is darker than the maroon body",
        glass_rgb < body_rgb,
        details=f"glass={glass_rgb:.2f}, maroon={body_rgb:.2f}",
    )

    # --- Gullwing doors: LONGITUDINAL revolute hinges that lift up & out -------
    _door_cases = ((hinge_l, door_l, "left"), (hinge_r, door_r, "right")) if INCLUDE_DOORS else ()
    for hinge, door, side in _door_cases:
        ax = tuple(hinge.axis)
        ctx.check(
            f"door_{side} hinge axis is LONGITUDINAL (gullwing, not scissor/normal)",
            abs(ax[1]) > 0.9 and abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6,
            details=f"axis={ax}",
        )
        ml = hinge.motion_limits
        ctx.check(
            f"door_{side} hinge limits ~[0, {DOOR_OPEN_MAX}] rad",
            ml is not None and abs(ml.lower) < 1e-6 and 0.9 <= ml.upper <= 1.3,
            details=f"limits=({ml.lower}, {ml.upper})" if ml else "no limits",
        )
        # Door carries the side window + flank skin (the roof stays fixed/complete).
        dnames = {v.name for v in door.visuals}
        ctx.check(
            f"door_{side} carries side window + skin + handle",
            {"door_glass", "door_skin", "door_handle"} <= dnames,
            details=f"door_{side} visuals={sorted(dnames)}",
        )
        ctx.expect_contact(body, door, contact_tol=0.06, name=f"door_{side} seated on body")

        rest = ctx.part_world_aabb(door)
        assert rest is not None
        with ctx.pose({hinge: 1.15}):
            opened = ctx.part_world_aabb(door)
            assert opened is not None
        # Gullwing motion: the whole panel lifts UP-AND-OUT about the shoulder --
        # its LOWER edge rises clear of the sill and it swings outboard.
        ctx.check(
            f"door_{side} lifts UP when open (bottom edge clears the sill)",
            opened[0][2] > rest[0][2] + 0.25,
            details=f"rest bottom z={rest[0][2]:.3f}, open bottom z={opened[0][2]:.3f}",
        )
        # ... and swings OUTWARD, away from the body (wing kicks out, not in).
        if side == "left":
            ctx.check(
                "door_left swings outboard (+X) as it lifts",
                opened[1][0] > rest[1][0] + 0.10,
                details=f"rest max x={rest[1][0]:.3f}, open max x={opened[1][0]:.3f}",
            )
        else:
            ctx.check(
                "door_right swings outboard (-X) as it lifts",
                opened[0][0] < rest[0][0] - 0.10,
                details=f"rest min x={rest[0][0]:.3f}, open min x={opened[0][0]:.3f}",
            )

    # --- Wheels: four continuous lateral spins, grounded and mirrored --------
    ground_zs = []
    for name, w in wheels.items():
        j = object_model.get_articulation(f"{name}_spin")
        ctx.check(
            f"{name} spin axis is lateral (X) and continuous",
            tuple(j.axis) == (1.0, 0.0, 0.0) and j.articulation_type == ArticulationType.CONTINUOUS,
            details=f"axis={j.axis}, type={j.articulation_type}",
        )
        wbb = ctx.part_world_aabb(w)
        assert wbb is not None
        ground_zs.append(wbb[0][2])
        ctx.check(
            f"{name} touches the ground plane",
            abs(wbb[0][2]) <= 0.02,
            details=f"min z={wbb[0][2]:.4f}",
        )
    ctx.check(
        "all four wheels touch the ground consistently",
        max(ground_zs) - min(ground_zs) <= 0.01,
        details=f"ground zs={['%.4f' % z for z in ground_zs]}",
    )
    flp = ctx.part_world_position(wheels["wheel_front_left"])
    frp = ctx.part_world_position(wheels["wheel_front_right"])
    assert flp is not None and frp is not None
    ctx.check(
        "front wheels mirror across the centerline",
        abs(flp[0] + frp[0]) < 0.02 and abs(flp[1] - frp[1]) < 0.02,
        details=f"fl={flp}, fr={frp}",
    )
    fl_spin = object_model.get_articulation("wheel_front_left_spin")
    rest_center = ctx.part_world_position(wheels["wheel_front_left"])
    with ctx.pose({fl_spin: 0.9}):
        spun_center = ctx.part_world_position(wheels["wheel_front_left"])
    assert rest_center is not None and spun_center is not None
    ctx.check(
        "front-left wheel rolls in place (center fixed under spin)",
        all(abs(rest_center[i] - spun_center[i]) < 1e-4 for i in range(3)),
        details=f"rest={rest_center}, spun={spun_center}",
    )
    ctx.check(
        "wheel center sits one radius above the ground",
        abs(rest_center[2] - WHEEL_R) < 1e-4,
        details=f"center z={rest_center[2]:.4f}, R={WHEEL_R}",
    )

    # --- Front steering: vertical king-pin pivots, wheels steer in place ------
    for kname, wname, side in (
        ("steer_knuckle_front_left", "wheel_front_left", "left"),
        ("steer_knuckle_front_right", "wheel_front_right", "right"),
    ):
        steer = object_model.get_articulation(f"{kname}_steer")
        ax = tuple(steer.axis)
        ctx.check(
            f"front-{side} steering axis is vertical (Z) and revolute",
            abs(ax[2]) > 0.9
            and abs(ax[0]) < 1e-6
            and abs(ax[1]) < 1e-6
            and steer.articulation_type == ArticulationType.REVOLUTE,
            details=f"axis={ax}, type={steer.articulation_type}",
        )
        w = wheels[wname]
        rest_c = ctx.part_world_position(w)
        rest_bb = ctx.part_world_aabb(w)
        assert rest_c is not None and rest_bb is not None
        with ctx.pose({steer: 0.35}):
            steer_c = ctx.part_world_position(w)
            steer_bb = ctx.part_world_aabb(w)
        assert steer_c is not None and steer_bb is not None
        ctx.check(
            f"front-{side} wheel steers in place (king-pin runs through its center)",
            all(abs(rest_c[i] - steer_c[i]) < 1e-3 for i in range(3)),
            details=f"rest={rest_c}, steered={steer_c}",
        )
        rest_xw = rest_bb[1][0] - rest_bb[0][0]
        steer_xw = steer_bb[1][0] - steer_bb[0][0]
        ctx.check(
            f"front-{side} wheel actually turns about the vertical axis",
            steer_xw > rest_xw + 0.08,
            details=f"rest x-extent={rest_xw:.3f}, steered x-extent={steer_xw:.3f}",
        )

    return ctx.report()
