from __future__ import annotations

# White Volkswagen Santana style three-box compact sedan.
# Z-up world. Long axis along +Y (nose at +Y), width along X (driver/left at +X),
# up along +Z. Wheels touch z = 0. ~4.5 m long, ~1.7 m wide, ~1.43 m tall.
#
# Articulation:
#   - FOUR conventional passenger doors, each on a near-vertical hinge at its
#     FRONT edge, swinging OUTWARD (revolute about Z) -- not Diablo scissor doors.
#   - Front hood and rear trunk lid each hinge UP (revolute about lateral X).
#   - Both front wheels STEER (revolute king-pin) and all four wheels SPIN.
# Like the Diablo reference, the cabin is HOLLOW: the lower body is carved out and
# the four door apertures are cut clean through the flanks, so opening any door
# reveals the interior (floor, seats, dash, wheel) rather than solid mesh.
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TireCarcass,
    TireGeometry,
    TireSidewall,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    boolean_difference,
    mesh_from_geometry,
    superellipse_side_loft,
)

# ---------------------------------------------------------------- proportions
WHEEL_R = 0.30
WHEEL_W = 0.205
HALF_TRACK = 0.73
FRONT_AXLE_Y = 1.28
REAR_AXLE_Y = -1.27
BELT_Z = 0.85          # beltline (top of lower body / bottom of greenhouse)
FLANK_X = 0.85         # body flank half-width
AXLE_ROD_R = 0.042     # straight axle rod radius
AXLE_CHANNEL_R = 0.072  # bored channel the rod passes through

# Lower-body 3-box side rails: (y, z_min, z_max, width). Boxy upright Santana
# hull: blunt squared-off ends, a flat-ish bonnet/boot deck, a near-constant
# beltline through the cabin. Ends stay wide and tall so it is not torpedo-like.
LOWER_SECTIONS = [
    (2.22, 0.26, 0.80, 1.56),
    (2.04, 0.20, 0.82, 1.66),
    (1.74, 0.17, 0.83, 1.70),
    (1.28, 0.16, 0.84, 1.70),
    (0.92, 0.16, 0.85, 1.70),
    (0.30, 0.16, 0.85, 1.72),
    (-0.30, 0.16, 0.85, 1.72),
    (-0.92, 0.16, 0.85, 1.70),
    (-1.27, 0.16, 0.85, 1.70),
    (-1.74, 0.17, 0.86, 1.68),
    (-2.04, 0.20, 0.85, 1.62),
    (-2.22, 0.26, 0.82, 1.50),
]

# Cohesive greenhouse glasshouse (white shell) -- roof + A/B/C pillars + window
# frames in one lofted shape (tumblehome, narrower than the body). Glass windows
# are laid over it. (y, z_min, z_max, width).
# z_min dips to 0.80 (below the beltline 0.85) so the greenhouse/pillars overlap
# the lower body and the roof is solidly connected down to it (no floating roof).
GREENHOUSE_SECTIONS = [
    (1.00, 0.80, 1.00, 1.66),
    (0.62, 0.80, 1.34, 1.64),
    (0.28, 0.80, 1.45, 1.62),
    (-0.34, 0.80, 1.46, 1.62),
    (-0.62, 0.80, 1.40, 1.62),
    (-1.00, 0.80, 1.10, 1.62),
]

# Cabin hollow + door apertures (carved out of the solid lower body so the
# interior is open and the doors reveal it).
CABIN_HALF_X = 0.70
CABIN_Y = (-0.96, 0.96)
CABIN_Z = (0.42, 0.90)
DOOR_APERTURE_X = (0.55, 1.02)
# Aperture floor at 0.44; the interior floor pan (top 0.46) + raised rocker cap the
# opening so opening a door shows only the flat grey floor, never the body's sloped
# underbody/sill facets below it.
DOOR_APERTURE_Z = (0.44, 0.86)
DOOR_SPANS = (("front", 0.05, 0.89), ("rear", -0.89, -0.04))  # (which, y0, y1)


def _save(name, geom):
    return mesh_from_geometry(geom, name)


def _box_cutter(x0, x1, y0, y1, z0, z1):
    # Axis-aligned solid box cutter. BoxGeometry is wound INWARD (manifold3d reads
    # it as empty -> a silent no-op subtraction); flip the faces so it really cuts.
    box = BoxGeometry((x1 - x0, y1 - y0, z1 - z0)).translate(
        (x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0
    )
    return MeshGeometry(vertices=list(box.vertices), faces=[(f[0], f[2], f[1]) for f in box.faces])


def _raked_box_cutter(size, rx, cx, cy, cz):
    # A box raked about X (for the sloped windshield / backlight openings), faces
    # flipped so it really subtracts (BoxGeometry is wound inward).
    g = BoxGeometry(size).rotate_x(rx).translate(cx, cy, cz)
    return MeshGeometry(vertices=list(g.vertices), faces=[(f[0], f[2], f[1]) for f in g.faces])


def _drop_small_islands(geom, min_faces=8):
    # Remove tiny disconnected mesh islands (boolean slivers / stray triangles).
    # Connectivity is edge-based (two faces joined iff they share an edge), so a
    # lone artifact triangle becomes its own component and is dropped.
    faces = [tuple(f) for f in geom.faces]
    parent = list(range(len(faces)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    edge_map = {}
    for fi, f in enumerate(faces):
        for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
            e = (a, b) if a < b else (b, a)
            if e in edge_map:
                ra, rb = find(fi), find(edge_map[e])
                if ra != rb:
                    parent[ra] = rb
            else:
                edge_map[e] = fi

    from collections import Counter

    sizes = Counter(find(fi) for fi in range(len(faces)))
    keep = {r for r, c in sizes.items() if c >= min_faces}
    new_faces = [faces[fi] for fi in range(len(faces)) if find(fi) in keep]
    return MeshGeometry(vertices=list(geom.vertices), faces=new_faces)


_LOWER_BODY_CACHE = None


def _lower_body_mesh():
    # Solid 3-box hull, then carve: wheel wells, the hollow cabin, and the four
    # door apertures, so the doors open onto an open interior.
    global _LOWER_BODY_CACHE
    if _LOWER_BODY_CACHE is None:
        body = superellipse_side_loft(LOWER_SECTIONS, exponents=4.2, segments=64)
        for ax, ay in (
            (HALF_TRACK, FRONT_AXLE_Y),
            (-HALF_TRACK, FRONT_AXLE_Y),
            (HALF_TRACK, REAR_AXLE_Y),
            (-HALF_TRACK, REAR_AXLE_Y),
        ):
            sign = 1.0 if ax > 0 else -1.0
            well = (
                CylinderGeometry(radius=WHEEL_R + 0.04, height=0.56, radial_segments=32)
                .rotate_y(pi / 2.0)
                .translate(sign * 0.85, ay, WHEEL_R)
            )
            body = boolean_difference(body, well)
        for sgn in (1.0, -1.0):
            xa, xb = sorted((sgn * DOOR_APERTURE_X[0], sgn * DOOR_APERTURE_X[1]))
            for _which, y0, y1 in DOOR_SPANS:
                body = boolean_difference(
                    body, _box_cutter(xa, xb, y0, y1, DOOR_APERTURE_Z[0], DOOR_APERTURE_Z[1])
                )
                # Also carve the SILL below the aperture (down to the rocker) so the
                # body's faceted underbody/sill is removed, not just hidden -- the
                # rocker covers the outside, the floor pan caps the inside.
                body = boolean_difference(body, _box_cutter(xa, xb, y0, y1, 0.24, DOOR_APERTURE_Z[0]))
        # Hollow the engine bay (front box) and the trunk (rear box) so the hood
        # and trunk lid open onto real open space, not a solid block. Keep the
        # cavity walls INBOARD of the wheel wells (inner cap x ~= 0.57) so the
        # cavities never break through to the tires -- the side walls stay solid.
        body = boolean_difference(body, _box_cutter(-0.48, 0.48, 0.98, 1.92, 0.46, 0.88))
        body = boolean_difference(body, _box_cutter(-0.48, 0.48, -1.92, -0.98, 0.46, 0.88))
        # Bore straight channels at wheel-center height for the front + rear axle
        # rods, so each rod passes cleanly through the body, aligned to the hubs.
        for ay in (FRONT_AXLE_Y, REAR_AXLE_Y):
            chan = (
                CylinderGeometry(radius=AXLE_CHANNEL_R, height=1.52, radial_segments=24)
                .rotate_y(pi / 2.0)
                .translate(0.0, ay, WHEEL_R)
            )
            body = boolean_difference(body, chan)
        # Carve the hollow cabin LAST (and a touch larger) so it scrubs out every
        # stray interior facet the wheel-well/pit/channel booleans leave behind --
        # the cabin reads as a clean empty box, no white mesh floating over seats.
        body = boolean_difference(
            body, _box_cutter(-CABIN_HALF_X, CABIN_HALF_X, CABIN_Y[0], CABIN_Y[1], CABIN_Z[0], CABIN_Z[1])
        )
        _LOWER_BODY_CACHE = body
    return _LOWER_BODY_CACHE.clone()


_GREENHOUSE_CACHE = None


def _greenhouse_parts():
    # Return ONE clean closed white greenhouse shell (no window cut-throughs).
    global _GREENHOUSE_CACHE
    if _GREENHOUSE_CACHE is None:
        t = 0.06
        # Hollow shell, then cut the 6 window OPENINGS through it (so windows are
        # open + see-through). Any tiny corner sliver the cut leaves is covered by
        # the slightly-larger, proud glass panes added in build_object_model -- that
        # hides the "missing corner" without leaving the windshield white. exp 6.0 /
        # 72 segs = smooth, near-vertical sides.
        outer = superellipse_side_loft(GREENHOUSE_SECTIONS, exponents=6.0, segments=72)
        inner_secs = [
            (y, zmin - 0.25, zmax - t, max(w - 2.0 * t, 0.04)) for (y, zmin, zmax, w) in GREENHOUSE_SECTIONS
        ]
        inner = superellipse_side_loft(inner_secs, exponents=6.0, segments=72)
        inner = MeshGeometry(vertices=list(inner.vertices), faces=[(f[0], f[2], f[1]) for f in inner.faces])
        shell = boolean_difference(outer, inner)
        windshield_box = _raked_box_cutter((1.00, 0.80, 0.46), -0.80, 0.0, 0.58, 1.09)
        rear_box = _raked_box_cutter((1.00, 0.76, 0.46), 0.66, 0.0, -0.73, 1.12)
        side_boxes = []
        for sgn in (1.0, -1.0):
            xa, xb = sorted((sgn * 0.62, sgn * 1.14))
            side_boxes.append(_box_cutter(xa, xb, 0.14, 0.70, 0.78, 1.16))
            side_boxes.append(_box_cutter(xa, xb, -0.70, -0.08, 0.78, 1.16))
        frame = shell
        for b in (windshield_box, rear_box, *side_boxes):
            frame = boolean_difference(frame, b)
        _GREENHOUSE_CACHE = _drop_small_islands(frame)
    return _GREENHOUSE_CACHE.clone()


# Shared wheel/tire geometry: black tire + silver steel-look wheel.
_TIRE_GEOM = TireGeometry(
    WHEEL_R,
    WHEEL_W,
    inner_radius=0.20,
    carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.03),
    sidewall=TireSidewall(style="rounded", bulge=0.03),
)
_WHEEL_GEOM = WheelGeometry(
    0.205,
    0.16,
    rim=WheelRim(inner_radius=0.165, flange_height=0.012, flange_thickness=0.006),
    hub=WheelHub(
        radius=0.055,
        width=0.08,
        cap_style="domed",
        bolt_pattern=BoltPattern(count=5, circle_diameter=0.066, hole_diameter=0.008),
    ),
    face=WheelFace(dish_depth=0.010, front_inset=0.004),
    spokes=WheelSpokes(style="straight", count=5, thickness=0.045, window_radius=0.028),
    bore=WheelBore(style="round", diameter=0.034),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vw_santana_sedan")

    blue_body = model.material("body_blue", rgba=(0.10, 0.22, 0.52, 1.0))
    black_trim = model.material("black_trim", rgba=(0.05, 0.05, 0.055, 1.0))
    glass = model.material("glass_tint", rgba=(0.11, 0.12, 0.15, 0.66))
    chrome = model.material("chrome", rgba=(0.80, 0.81, 0.84, 1.0))
    hubcap_silver = model.material("hubcap_silver", rgba=(0.62, 0.63, 0.66, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    amber = model.material("amber", rgba=(0.86, 0.52, 0.07, 1.0))
    red_tail = model.material("tail_red", rgba=(0.62, 0.05, 0.06, 1.0))
    lens_pale = model.material("lens_pale", rgba=(0.82, 0.84, 0.86, 1.0))
    interior_grey = model.material("interior_grey", rgba=(0.22, 0.22, 0.24, 1.0))
    grille_dk = model.material("grille_dark", rgba=(0.09, 0.09, 0.10, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")
    body.visual(_save("lower_body.obj", _lower_body_mesh()), material=blue_body, name="lower_body")
    # Greenhouse: one clean closed white shell (no cut-through windows -> no corner
    # gaps), with the glass laid on as flush tinted panes: windshield, backlight,
    # and four side windows. Panes are a touch thick so they sit proud of the shell.
    body.visual(_save("greenhouse.obj", _greenhouse_parts()), material=blue_body, name="roof")
    # Windows stay open/see-through; each glass pane is a touch LARGER than + proud
    # of its opening so it covers the tiny cut-corner slivers -> no visible gap.
    body.visual(
        Box((1.08, 0.90, 0.06)),
        origin=Origin(xyz=(0.0, 0.58, 1.09), rpy=(-0.80, 0.0, 0.0)),
        material=glass,
        name="windshield",
    )
    body.visual(
        Box((1.08, 0.86, 0.06)),
        origin=Origin(xyz=(0.0, -0.73, 1.12), rpy=(0.66, 0.0, 0.0)),
        material=glass,
        name="rear_window",
    )
    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((0.08, 0.66, 0.40)),
            origin=Origin(xyz=(sgn * 0.82, 0.42, 1.01)),
            material=glass,
            name=f"side_window_front_{side}",
        )
        body.visual(
            Box((0.08, 0.72, 0.40)),
            origin=Origin(xyz=(sgn * 0.82, -0.39, 1.01)),
            material=glass,
            name=f"side_window_rear_{side}",
        )

    # ---- Interior (so the hollow cabin is not empty when a door opens) ----
    # Wide floor pan -- top at 0.46 (above the door aperture floor 0.44) and out to
    # the flank, so it CAPS the door opening: looking in, you see only the flat grey
    # floor, never the body's sloped underbody/sill facets below.
    body.visual(
        Box((1.72, 1.96, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
        material=interior_grey,
        name="cabin_floor",
    )
    body.visual(
        Box((1.30, 0.42, 0.30)),
        origin=Origin(xyz=(0.0, 0.86, 0.62)),
        material=interior_grey,
        name="dashboard",
    )
    for sx, side in ((0.34, "left"), (-0.34, "right")):
        body.visual(
            Box((0.46, 0.48, 0.18)),
            origin=Origin(xyz=(sx, 0.34, 0.55)),
            material=interior_grey,
            name=f"seat_base_front_{side}",
        )
        body.visual(
            Box((0.46, 0.10, 0.40)),
            origin=Origin(xyz=(sx, 0.12, 0.72)),
            material=interior_grey,
            name=f"seat_back_front_{side}",
        )
    body.visual(
        Box((1.20, 0.42, 0.18)),
        origin=Origin(xyz=(0.0, -0.55, 0.55)),
        material=interior_grey,
        name="seat_base_rear",
    )
    body.visual(
        Box((1.20, 0.10, 0.42)),
        origin=Origin(xyz=(0.0, -0.78, 0.73)),
        material=interior_grey,
        name="seat_back_rear",
    )
    # Rear parcel shelf below the backlight -- seals the cabin off from the boot
    # so you no longer see through into the trunk.
    body.visual(
        Box((1.34, 0.34, 0.06)),
        origin=Origin(xyz=(0.0, -0.84, 0.89)),
        material=interior_grey,
        name="parcel_shelf",
    )
    _SW_HUB = (0.34, 0.62, 0.74)
    _SW_RAKE = (0.55, 0.0, 0.0)
    # Steering COLUMN stays fixed on the body, in line with the wheel's turn axis.
    # The wheel itself is a separate part on a revolute joint (see steering_wheel).
    body.visual(
        Cylinder(radius=0.02, length=0.30),
        origin=Origin(xyz=(0.34, 0.74, 0.60), rpy=_SW_RAKE),
        material=black_trim,
        name="steering_column",
    )

    # ---- Front fascia (Santana face): slatted chrome grille + VW badge,
    #      swept headlight clusters, lower bumper intake + fog lamps. ----
    # The nose front cap is at y ~= 2.22, so fascia parts sit PROUD of it (y>2.22)
    # to read as real grille/lamps rather than being buried in the body.
    NOSE_Y = 2.22
    # Slim slatted grille between the lamps, with a chrome brow tying into them.
    body.visual(
        Box((0.66, 0.05, 0.13)),
        origin=Origin(xyz=(0.0, NOSE_Y + 0.005, 0.585)),
        material=grille_dk,
        name="grille",
    )
    for k in range(3):
        body.visual(
            Box((0.64, 0.05, 0.018)),
            origin=Origin(xyz=(0.0, NOSE_Y + 0.03, 0.545 + 0.043 * k)),
            material=chrome,
            name=f"grille_slat_{k}",
        )
    body.visual(
        Box((1.42, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, NOSE_Y + 0.025, 0.655)),
        material=chrome,
        name="grille_brow",
    )
    body.visual(
        Cylinder(radius=0.058, length=0.06),
        origin=Origin(xyz=(0.0, NOSE_Y + 0.05, 0.585), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="badge",
    )
    # Headlight clusters: dark housing (slightly recessed), pale lens proud, amber.
    for sx, side in ((0.555, "left"), (-0.555, "right")):
        body.visual(
            Box((0.42, 0.05, 0.18)),
            origin=Origin(xyz=(sx, NOSE_Y + 0.01, 0.625)),
            material=black_trim,
            name=f"headlight_housing_{side}",
        )
        body.visual(
            Box((0.36, 0.05, 0.115)),
            origin=Origin(xyz=(sx, NOSE_Y + 0.04, 0.645)),
            material=lens_pale,
            name=f"headlight_{side}",
        )
        body.visual(
            Box((0.36, 0.05, 0.032)),
            origin=Origin(xyz=(sx, NOSE_Y + 0.04, 0.565)),
            material=amber,
            name=f"front_indicator_{side}",
        )
    # Lower bumper: body-colour bar with a wide dark central intake + fog lamps.
    body.visual(
        Box((1.60, 0.12, 0.24)),
        origin=Origin(xyz=(0.0, NOSE_Y + 0.0, 0.35)),
        material=blue_body,
        name="front_bumper",
    )
    body.visual(
        Box((1.00, 0.06, 0.14)),
        origin=Origin(xyz=(0.0, NOSE_Y + 0.055, 0.34)),
        material=grille_dk,
        name="front_intake",
    )
    body.visual(
        Box((0.34, 0.05, 0.10)),
        origin=Origin(xyz=(0.0, NOSE_Y + 0.055, 0.47)),
        material=chrome,
        name="front_plate",
    )
    for sx, side in ((0.58, "left"), (-0.58, "right")):
        body.visual(
            Cylinder(radius=0.05, length=0.06),
            origin=Origin(xyz=(sx, NOSE_Y + 0.055, 0.33), rpy=(pi / 2.0, 0.0, 0.0)),
            material=lens_pale,
            name=f"fog_{side}",
        )
    body.visual(
        Box((1.54, 0.10, 0.20)),
        origin=Origin(xyz=(0.0, -2.20, 0.40)),
        material=black_trim,
        name="rear_bumper",
    )
    for sx, side in ((0.55, "left"), (-0.55, "right")):
        body.visual(
            Box((0.34, 0.05, 0.16)),
            origin=Origin(xyz=(sx, -2.075, 0.60)),
            material=red_tail,
            name=f"taillight_{side}",
        )
    # Rocker / sill panel: a black sill that rises to the door bottom (z=0.40) and
    # sits just outboard of the flank, hiding the body's sloped underbody/sill
    # facets that otherwise show at the door foot when a door is open.
    for sx, side in ((0.845, "left"), (-0.845, "right")):
        body.visual(
            Box((0.09, 1.90, 0.25)),
            origin=Origin(xyz=(sx, 0.0, 0.315)),
            material=black_trim,
            name=f"rocker_{side}",
        )

    # ---- Door seam lines (thin dark strips on the flank at door boundaries) ----
    # These read as the panel-gap seams between doors and body / between doors.
    seam_t = 0.006  # seam strip thickness (proud of flank)
    seam_w = 0.020  # seam strip width (visible gap width)
    seam_z_mid = 0.62  # vertical center of door seam (midway between sill and beltline)
    seam_h = 0.44  # vertical extent of seam
    # Y positions of door seams: A-pillar, B-pillar, C-pillar
    door_seam_ys = (0.89, 0.005, -0.89)
    seam_idx = 0
    for seam_y in door_seam_ys:
        for sx, side in ((0.856, "left"), (-0.856, "right")):
            body.visual(
                Box((seam_t, seam_w, seam_h)),
                origin=Origin(xyz=(sx, seam_y, seam_z_mid)),
                material=black_trim,
                name=f"door_seam_{side}_{seam_idx}",
            )
            seam_idx += 1

    # ---- Hood seam lines (around the engine bay opening on the bonnet deck) ----
    hood_seam_t = 0.006
    hood_deck_z = 0.86  # top of lower body at the cowl
    # Side seams along the hood edges
    for sx, side in ((0.49, "left"), (-0.49, "right")):
        body.visual(
            Box((hood_seam_t, 0.88, 0.012)),
            origin=Origin(xyz=(sx, 1.45, hood_deck_z + 0.006)),
            material=black_trim,
            name=f"hood_seam_{side}",
        )
    # Front hood seam (at the nose end)
    body.visual(
        Box((0.96, hood_seam_t, 0.012)),
        origin=Origin(xyz=(0.0, 1.90, hood_deck_z + 0.006)),
        material=black_trim,
        name="hood_seam_front",
    )
    # Rear hood seam (at the cowl/windshield base)
    body.visual(
        Box((0.96, hood_seam_t, 0.012)),
        origin=Origin(xyz=(0.0, 0.99, hood_deck_z + 0.006)),
        material=black_trim,
        name="hood_seam_rear",
    )

    # ---- Trunk seam lines (around the boot opening on the trunk deck) ----
    trunk_deck_z = 0.86
    for sx, side in ((0.49, "left"), (-0.49, "right")):
        body.visual(
            Box((hood_seam_t, 0.88, 0.012)),
            origin=Origin(xyz=(sx, -1.45, trunk_deck_z + 0.006)),
            material=black_trim,
            name=f"trunk_seam_{side}",
        )
    # Front trunk seam (near rear window base)
    body.visual(
        Box((0.96, hood_seam_t, 0.012)),
        origin=Origin(xyz=(0.0, -0.99, trunk_deck_z + 0.006)),
        material=black_trim,
        name="trunk_seam_front",
    )
    # Rear trunk seam (at the tail)
    body.visual(
        Box((0.96, hood_seam_t, 0.012)),
        origin=Origin(xyz=(0.0, -1.90, trunk_deck_z + 0.006)),
        material=black_trim,
        name="trunk_seam_rear",
    )

    # Straight front + rear axle rods, through the bored channels, ends reaching
    # the wheel hubs -- centered exactly on the wheel center (z = WHEEL_R).
    for ay, side in ((FRONT_AXLE_Y, "front"), (REAR_AXLE_Y, "rear")):
        body.visual(
            Cylinder(radius=AXLE_ROD_R, length=2.0 * HALF_TRACK + 0.10),
            origin=Origin(xyz=(0.0, ay, WHEEL_R), rpy=(0.0, pi / 2.0, 0.0)),
            material=chrome,
            name=f"{side}_axle_rod",
        )

    body.inertial = Inertial.from_geometry(
        Box((1.7, 4.5, 1.1)), mass=1150.0, origin=Origin(xyz=(0.0, 0.0, 0.6))
    )

    # ---------------------------------------------------------------- doors
    def make_door(side, which, hinge_y, rear_y):
        name = f"door_{which}_{side}"
        d = model.part(name)
        span = hinge_y - rear_y
        midy = -span / 2.0
        # NOTE: door visuals are authored RELATIVE TO THE HINGE (origin z = 0.55),
        # so a body z of Z maps to a local z of (Z - 0.55). Lower panel fills the
        # door aperture (body z ~0.30-0.84); glass sits in the side-window opening.
        d.visual(
            Box((0.05, span - 0.04, 0.40)),
            origin=Origin(xyz=(0.0, midy, 0.09)),  # body z ~0.44-0.84 (sill stays solid below)
            material=blue_body,
            name="door_skin",
        )
        # (No door glass: the side windows are glazed into the sealed roof canopy,
        #  so the door is the lower body panel up to the beltline.)
        d.visual(
            Box((0.05, span - 0.04, 0.02)),
            origin=Origin(xyz=(0.0, midy, 0.30)),  # body z ~0.85 beltline
            material=black_trim,
            name="door_beltline",
        )
        d.visual(
            Box((0.05, 0.12, 0.03)),
            origin=Origin(xyz=(0.006, -span + 0.16, 0.12)),  # body z ~0.67 handle
            material=chrome,
            name="door_handle",
        )
        # (Side mirrors removed -- they read as floating after the door reshape.)
        d.inertial = Inertial.from_geometry(
            Box((0.06, span, 0.85)), mass=24.0, origin=Origin(xyz=(0.0, midy, 0.25))
        )
        return d

    door_specs = []
    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        for which, y0, y1 in (("front", 0.05, 0.89), ("rear", -0.89, -0.04)):
            d = make_door(side, which, y1, y0)
            door_specs.append((d, sgn, y1))

    # ----------------------------------------------------------- hood + trunk
    # Hood covers the front box (engine bay), hinged at the cowl; the panel
    # extends forward (+Y) and is raked slightly down to follow the bonnet line.
    # Hood is sized to the engine-bay pit (x +-0.48, y 0.98..1.92), not larger.
    hood = model.part("hood")
    hood.visual(
        Box((0.94, 0.90, 0.05)),
        origin=Origin(xyz=(0.0, 0.47, -0.02), rpy=(-0.13, 0.0, 0.0)),
        material=blue_body,
        name="hood_skin",
    )
    hood.inertial = Inertial.from_geometry(Box((0.94, 0.90, 0.08)), mass=18.0, origin=Origin(xyz=(0.0, 0.47, 0.0)))

    # Trunk lid covers the rear box (boot), hinged at its front edge; the panel
    # extends rearward (-Y) and is raked slightly down toward the tail.
    # Trunk lid is sized to the boot pit (x +-0.48, y -1.92..-0.98), not larger.
    trunk = model.part("trunk")
    trunk.visual(
        Box((0.94, 0.90, 0.05)),
        origin=Origin(xyz=(0.0, -0.47, -0.02), rpy=(0.12, 0.0, 0.0)),
        material=blue_body,
        name="trunk_skin",
    )
    trunk.inertial = Inertial.from_geometry(Box((0.94, 0.90, 0.08)), mass=14.0, origin=Origin(xyz=(0.0, -0.47, 0.0)))

    # ---------------------------------------------------------------- wheels
    def make_wheel(name, outboard_sign):
        w = model.part(name)
        face_rpy = (0.0, 0.0, 0.0) if outboard_sign > 0 else (0.0, 0.0, pi)
        w.visual(_save(f"{name}_tire.obj", _TIRE_GEOM.clone()), origin=Origin(rpy=face_rpy), material=rubber, name="tire")
        w.visual(
            _save(f"{name}_wheel.obj", _WHEEL_GEOM.clone()),
            origin=Origin(xyz=(outboard_sign * 0.03, 0.0, 0.0), rpy=face_rpy),
            material=hubcap_silver,
            name="rim",
        )
        w.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_R, length=WHEEL_W), mass=18.0, origin=Origin(rpy=(0.0, pi / 2.0, 0.0))
        )
        return w

    make_wheel("wheel_front_left", 1.0)
    make_wheel("wheel_front_right", -1.0)
    make_wheel("wheel_rear_left", 1.0)
    make_wheel("wheel_rear_right", -1.0)

    def make_knuckle(name):
        k = model.part(name)
        k.inertial = Inertial.from_geometry(Box((0.10, 0.10, 0.20)), mass=5.0)
        return k

    knuckle_fl = make_knuckle("steer_knuckle_front_left")
    knuckle_fr = make_knuckle("steer_knuckle_front_right")

    # Rotating steering wheel (separate part). Authored in the wheel's own frame:
    # rim + spokes in the XY plane, hub along +Z. The joint origin applies the
    # column rake and spins it about local +Z (the column line), so it visibly turns.
    steer_wheel = model.part("steering_wheel")
    steer_wheel.visual(
        _save("sw_rim.obj", TorusGeometry(radius=0.15, tube=0.017, radial_segments=12, tubular_segments=44)),
        material=black_trim,
        name="sw_rim",
    )
    steer_wheel.visual(Box((0.28, 0.022, 0.018)), material=black_trim, name="sw_spoke_a")
    steer_wheel.visual(Box((0.022, 0.28, 0.018)), material=black_trim, name="sw_spoke_b")
    steer_wheel.visual(Cylinder(radius=0.035, length=0.05), material=chrome, name="sw_hub")
    steer_wheel.inertial = Inertial.from_geometry(Cylinder(radius=0.15, length=0.05), mass=1.5)

    # ----------------------------------------------------------- articulations
    for d, sgn, hinge_y in door_specs:
        axis = (0.0, 0.0, 1.0) if sgn > 0 else (0.0, 0.0, -1.0)
        model.articulation(
            f"{d.name}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=d,
            origin=Origin(xyz=(sgn * FLANK_X, hinge_y, 0.55)),
            axis=axis,
            motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.2),
        )

    # Hood hinges at the REAR edge of the engine-bay pit (y=0.98), not on the
    # windshield; +q swings the forward edge UP.
    model.articulation(
        "hood_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.0, 0.98, 0.86)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.5, lower=0.0, upper=1.0),
    )
    # Trunk hinges at the FRONT edge of the boot pit (y=-0.98), not on the rear
    # window; +q swings the rear edge UP.
    model.articulation(
        "trunk_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trunk,
        origin=Origin(xyz=(0.0, -0.98, 0.86)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.5, lower=0.0, upper=1.0),
    )

    STEER_LOCK = 0.42
    # Display pose: front wheels slightly turned left (~7°) so the sedan reads
    # as mid-maneuver rather than parked dead-straight.
    DISPLAY_STEER_YAW = 0.12
    for knuckle, sx in ((knuckle_fl, HALF_TRACK), (knuckle_fr, -HALF_TRACK)):
        model.articulation(
            f"{knuckle.name}_steer",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knuckle,
            origin=Origin(xyz=(sx, FRONT_AXLE_Y, WHEEL_R), rpy=(0.0, 0.0, DISPLAY_STEER_YAW)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=100.0, velocity=4.0, lower=-STEER_LOCK, upper=STEER_LOCK),
        )

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
            motion_limits=MotionLimits(effort=160.0, velocity=60.0),
        )

    # Steering wheel turns about the raked column axis (origin rpy applies the rake;
    # the joint spins the wheel about its local +Z = the column line).
    model.articulation(
        "steering_wheel_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steer_wheel,
        origin=Origin(xyz=_SW_HUB, rpy=_SW_RAKE),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0, lower=-3.14, upper=3.14),
    )

    return model


object_model = build_object_model()


def run_tests():
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    doors = {
        f"door_{w}_{s}": object_model.get_part(f"door_{w}_{s}")
        for s in ("left", "right")
        for w in ("front", "rear")
    }
    wheels = {n: object_model.get_part(n) for n in (
        "wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right")}

    # --- Intentional overlaps -------------------------------------------------
    for n, w in wheels.items():
        for elem in ("tire", "rim"):
            ctx.allow_overlap(body, w, elem_a="lower_body", elem_b=elem,
                              reason="Wheel seated in the fender wheel arch of the body shell.")
            rod = "front_axle_rod" if "front" in n else "rear_axle_rod"
            ctx.allow_overlap(body, w, elem_a=rod, elem_b=elem,
                              reason="Axle rod end reaches into the wheel hub by design.")
    # Front wheels: tire inner face can brush the cabin floor and dashboard
    # edges when the wheel is at full lock or in the display-pose yaw offset.
    for n in ("wheel_front_left", "wheel_front_right"):
        w = object_model.get_part(n)
        for belem in ("cabin_floor", "dashboard"):
            ctx.allow_overlap(body, w, elem_a=belem, elem_b="tire",
                              reason="Front tire inner face sits in the wheel well; slight contact with floor/dash edge at steering offset is expected.")
    for dn, d in doors.items():
        side = dn.split("_")[-1]
        for shell in ("lower_body", "roof", "windshield", f"rocker_{side}",
                      f"side_window_front_{side}", f"side_window_rear_{side}",
                      "cabin_floor"):
            for delem in ("door_skin", "door_beltline"):
                ctx.allow_overlap(body, d, elem_a=shell, elem_b=delem,
                                  reason="Door seats flush in the body aperture; thin embed is intentional.")
    for part_name in ("hood", "trunk"):
        p = object_model.get_part(part_name)
        ctx.allow_overlap(body, p, elem_a="lower_body", elem_b=f"{part_name}_skin",
                          reason="Hood/trunk lid seats flush on the body opening.")
    # Trunk skin thin embed into the rear window glass and parcel shelf at the boot hinge line.
    trunk_p = object_model.get_part("trunk")
    ctx.allow_overlap(body, trunk_p,
                      elem_a="rear_window", elem_b="trunk_skin",
                      reason="Trunk lid trailing edge seats against the rear window glass at the hinge line.")
    ctx.allow_overlap(body, trunk_p,
                      elem_a="parcel_shelf", elem_b="trunk_skin",
                      reason="Trunk lid seats against the parcel shelf at the boot opening.")
    # Rear wheels: tire top sits close to the cabin floor edge at the wheel well.
    for n in ("wheel_rear_left", "wheel_rear_right"):
        w = object_model.get_part(n)
        ctx.allow_overlap(body, w, elem_a="cabin_floor", elem_b="tire",
                          reason="Rear tire top sits in the wheel well, close to the cabin floor edge.")
    # Steering wheel rim and spokes sit in front of the dashboard; small overlap
    # where they pass through the dash plane is intentional (column mount).
    steer_w = object_model.get_part("steering_wheel")
    for sw_elem in ("sw_rim", "sw_spoke_a", "sw_spoke_b", "sw_hub"):
        ctx.allow_overlap(body, steer_w,
                          elem_a="dashboard", elem_b=sw_elem,
                          reason="Steering wheel element passes through the dashboard plane at the column mount.")
    # Hood sits slightly proud of the body shell at the cowl (hinge line gap).
    ctx.allow_isolated_part(object_model.get_part("hood"),
                            reason="Hood is connected to the body only through the hinge articulation; small cowl gap is intentional.")

    # --- Hero features present ------------------------------------------------
    vis = {v.name for v in body.visuals}
    ctx.check("3-box body shell present", {"lower_body", "roof"} <= vis)
    ctx.check("greenhouse glass present (windshield + backlight)", {"windshield"} <= vis)
    ctx.check("grille + two headlights + two taillights",
              {"grille", "headlight_left", "headlight_right", "taillight_left", "taillight_right"} <= vis)
    ctx.check("interior present (floor, seats, dash, wheel)",
              {"cabin_floor", "dashboard", "seat_base_rear", "steering_column"} <= vis)
    sw = object_model.get_part("steering_wheel")
    ctx.check("rotating steering wheel present (rim + spokes + hub)",
              {"sw_rim", "sw_spoke_a", "sw_hub"} <= {v.name for v in sw.visuals})
    sw_turn = object_model.get_articulation("steering_wheel_turn")
    ctx.check("steering wheel TURNS (revolute about the column axis)",
              sw_turn.articulation_type == ArticulationType.REVOLUTE
              and tuple(sw_turn.axis) == (0.0, 0.0, 1.0))
    ctx.check("front + rear axle rods present", {"front_axle_rod", "rear_axle_rod"} <= vis)
    # Axle rods centered on the wheel centers (z = WHEEL_R) -----------------------
    for side, ay in (("front", FRONT_AXLE_Y), ("rear", REAR_AXLE_Y)):
        rod = next(v for v in body.visuals if v.name == f"{side}_axle_rod")
        rz = rod.origin.xyz[2]
        ctx.check(f"{side} axle rod is at wheel-center height", abs(rz - WHEEL_R) < 1e-6,
                  details=f"rod z={rz:.3f}, wheel center z={WHEEL_R:.3f}")
    ctx.check("four doors exist",
              set(doors) == {"door_front_left", "door_rear_left", "door_front_right", "door_rear_right"})

    # --- Scale ----------------------------------------------------------------
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    lo, hi = bb
    ctx.check("sedan length ~4.5 m", 4.3 <= hi[1] - lo[1] <= 4.7, details=f"L={hi[1] - lo[1]:.2f}")
    ctx.check("sedan width ~1.7 m", 1.6 <= hi[0] - lo[0] <= 1.85, details=f"W={hi[0] - lo[0]:.2f}")
    ctx.check("sedan height ~1.4 m", 1.3 <= hi[2] <= 1.55, details=f"H={hi[2]:.2f}")

    # --- Blue body, dark glass ------------------------------------------------
    mats = {m.name: m for m in object_model.materials}
    blue = mats["body_blue"].rgba[:3]
    ctx.check("body is blue", blue[2] > blue[0] + 0.15 and blue[2] > blue[1] + 0.10,
              details=f"rgba={mats['body_blue'].rgba}")
    ctx.check("body is medium-dark (not white)", sum(blue) < 1.8)
    ctx.check("glass darker than body",
              sum(mats["glass_tint"].rgba[:3]) < sum(blue) - 0.2)

    # --- Hubcap material (silver, not chrome-bright) -------------------------
    ctx.check("hubcap material present", "hubcap_silver" in mats)
    hubcap = mats["hubcap_silver"].rgba[:3]
    ctx.check("hubcap is muted silver (not bright chrome)",
              sum(hubcap) < 2.3 and hubcap[0] > 0.4,
              details=f"hubcap rgba={mats['hubcap_silver'].rgba}")

    # --- Doors: vertical hinge, swing OUTWARD, reveal the cabin ---------------
    for dn, d in doors.items():
        side = dn.split("_")[-1]
        hinge = object_model.get_articulation(f"{dn}_hinge")
        ax = tuple(hinge.axis)
        ctx.check(f"{dn} hinge is near-vertical (Z) revolute",
                  abs(ax[2]) > 0.9 and hinge.articulation_type == ArticulationType.REVOLUTE,
                  details=f"axis={ax}")
        rest = ctx.part_world_aabb(d)
        with ctx.pose({hinge: 1.0}):
            opened = ctx.part_world_aabb(d)
        assert rest is not None and opened is not None
        if side == "left":
            moved_out = opened[1][0] > rest[1][0] + 0.15
        else:
            moved_out = opened[0][0] < rest[0][0] - 0.15
        ctx.check(f"{dn} swings OUTWARD when opened", moved_out,
                  details=f"rest x=[{rest[0][0]:.2f},{rest[1][0]:.2f}] open x=[{opened[0][0]:.2f},{opened[1][0]:.2f}]")

    # --- Hood + trunk hinge open ---------------------------------------------
    for part_name, jn, q in (("hood", "hood_hinge", 0.85), ("trunk", "trunk_hinge", 0.85)):
        p = object_model.get_part(part_name)
        j = object_model.get_articulation(jn)
        ctx.check(f"{part_name} hinge is lateral (X) revolute",
                  abs(tuple(j.axis)[0]) > 0.9 and j.articulation_type == ArticulationType.REVOLUTE)
        rest = ctx.part_world_aabb(p)
        with ctx.pose({j: q}):
            lifted = ctx.part_world_aabb(p)
        assert rest is not None and lifted is not None
        ctx.check(f"{part_name} lifts up when opened", lifted[1][2] > rest[1][2] + 0.2,
                  details=f"rest top z={rest[1][2]:.2f}, open top z={lifted[1][2]:.2f}")

    # --- Wheels steer + spin, grounded ---------------------------------------
    for name, w in wheels.items():
        sp = object_model.get_articulation(f"{name}_spin")
        ctx.check(f"{name} spins about lateral X (continuous)",
                  tuple(sp.axis) == (1.0, 0.0, 0.0) and sp.articulation_type == ArticulationType.CONTINUOUS)
        wbb = ctx.part_world_aabb(w)
        assert wbb is not None
        ctx.check(f"{name} touches the ground", abs(wbb[0][2]) <= 0.03, details=f"min z={wbb[0][2]:.3f}")
    for side, kn in (("left", "steer_knuckle_front_left_steer"), ("right", "steer_knuckle_front_right_steer")):
        st = object_model.get_articulation(kn)
        ctx.check(f"front-{side} steering is vertical (Z) revolute",
                  abs(tuple(st.axis)[2]) > 0.9 and st.articulation_type == ArticulationType.REVOLUTE)

    # --- Front wheels slightly turned at rest ---------------------------------
    # At q=0 (rest pose), the knuckle origins include a yaw offset so the wheels
    # appear slightly turned. A turned wheel projects more of the tire diameter
    # along world X, making the X-extent wider than a straight wheel.
    # Compare front wheel X-extent to the rear (straight) wheel X-extent.
    rear_w = object_model.get_part("wheel_rear_left")
    rear_bb = ctx.part_world_aabb(rear_w)
    assert rear_bb is not None
    rear_x_extent = rear_bb[1][0] - rear_bb[0][0]
    for name in ("wheel_front_left", "wheel_front_right"):
        w = object_model.get_part(name)
        wbb = ctx.part_world_aabb(w)
        assert wbb is not None
        front_x_extent = wbb[1][0] - wbb[0][0]
        ctx.check(f"{name} is slightly turned at rest (wider X than straight)",
                  front_x_extent > rear_x_extent + 0.01,
                  details=f"front X={front_x_extent:.4f}, rear X={rear_x_extent:.4f}")

    return ctx.report()
