from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

VARIANT = "flattened_oval_flask"

# ── Base geometry constants (parent round bottle) ──────────────────────
_NECK_R = 0.014
_PRESS_TRAVEL = 0.018
_COLLAR_R = 0.0185

# ── Variant-specific height and position constants ─────────────────────
if VARIANT == "flattened_oval_flask":
    _ZLIFT = 0.038
    BODY_R = 0.040
    BODY_H = 0.180
    SHOULDER_TOP = 0.204
    NECK_R = _NECK_R
    NECK_TOP = 0.226
    COLLAR_R = _COLLAR_R
    COLLAR_Z0 = 0.208
    COLLAR_Z1 = 0.234
    PUMP_Z0 = 0.232
    PUMP_Z1 = 0.267
    PRESS_TRAVEL = _PRESS_TRAVEL
else:
    _ZLIFT = 0.0
    BODY_R = 0.033
    BODY_H = 0.142
    SHOULDER_TOP = 0.166
    NECK_R = _NECK_R
    NECK_TOP = 0.188
    COLLAR_R = _COLLAR_R
    COLLAR_Z0 = 0.170
    COLLAR_Z1 = 0.196
    PUMP_Z0 = 0.194
    PUMP_Z1 = 0.229
    PRESS_TRAVEL = _PRESS_TRAVEL

CAP_Z0 = COLLAR_Z1
CAP_Z1 = CAP_Z0 + 0.075
CAP_R = 0.036


# ── Shared helpers ─────────────────────────────────────────────────────

def _merge(geoms):
    base = None
    for geom in geoms:
        base = geom if base is None else base.merge(geom)
    return base


def _ring_mesh(name: str, r_outer: float, r_inner: float, z0: float, z1: float, segments: int = 56):
    geom = LatheGeometry(
        [(r_inner, z0), (r_outer, z0), (r_outer, z1), (r_inner, z1)],
        segments=segments,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


# ── Bottle body meshes ─────────────────────────────────────────────────

def _round_bottle_mesh():
    wall = 0.0022
    outer = [
        (0.0, 0.000),
        (BODY_R - 0.006, 0.000),
        (BODY_R, 0.008),
        (BODY_R, BODY_H),
        (BODY_R - 0.010, SHOULDER_TOP - 0.006),
        (NECK_R + 0.004, SHOULDER_TOP),
        (NECK_R, SHOULDER_TOP + 0.004),
        (NECK_R, NECK_TOP),
    ]
    inner = [
        (NECK_R - wall, NECK_TOP + 0.002),
        (NECK_R - wall, SHOULDER_TOP + 0.004),
        (BODY_R - wall - 0.010, SHOULDER_TOP - 0.008),
        (BODY_R - wall, BODY_H - 0.004),
        (BODY_R - wall, 0.014),
        (BODY_R - wall - 0.006, 0.006),
        (0.0, 0.006),
    ]
    geom = LatheGeometry(outer + inner, segments=72, closed=True)
    for z in (0.050, 0.083, 0.116):
        rib = TorusGeometry(BODY_R + 0.0004, 0.0009, radial_segments=8, tubular_segments=72)
        rib.translate(0.0, 0.0, z)
        geom.merge(rib)
    for z in (COLLAR_Z0 + 0.004, COLLAR_Z0 + 0.011, COLLAR_Z0 + 0.017):
        thread = TorusGeometry(NECK_R + 0.0003, 0.0010, radial_segments=8, tubular_segments=60)
        thread.translate(0.0, 0.0, z)
        geom.merge(thread)
    return mesh_from_geometry(geom, "bottle_shell")


def _rounded_rect_points(width: float, depth: float, radius: float, n: int = 6):
    pts = []
    centers = [
        (width / 2 - radius, depth / 2 - radius, 0.0),
        (-width / 2 + radius, depth / 2 - radius, math.pi / 2),
        (-width / 2 + radius, -depth / 2 + radius, math.pi),
        (width / 2 - radius, -depth / 2 + radius, 3 * math.pi / 2),
    ]
    for cx, cy, a0 in centers:
        for i in range(n + 1):
            a = a0 + (math.pi / 2) * i / n
            pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
    return pts


def _loft_sections(sections, name: str):
    geom = MeshGeometry()
    rings = []
    for z, pts in sections:
        ring = [geom.add_vertex(x, y, z) for x, y in pts]
        rings.append(ring)
    count = len(rings[0])
    for ring in rings:
        if len(ring) != count:
            raise ValueError("all sections must share a point count")
    for s in range(len(rings) - 1):
        a_ring, b_ring = rings[s], rings[s + 1]
        for i in range(count):
            j = (i + 1) % count
            geom.add_face(a_ring[i], a_ring[j], b_ring[j])
            geom.add_face(a_ring[i], b_ring[j], b_ring[i])
    bottom_center = geom.add_vertex(0.0, 0.0, sections[0][0])
    top_center = geom.add_vertex(0.0, 0.0, sections[-1][0])
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(bottom_center, rings[0][j], rings[0][i])
        geom.add_face(top_center, rings[-1][i], rings[-1][j])
    return mesh_from_geometry(geom, name)


def _square_bottle_mesh():
    base = _rounded_rect_points(0.066, 0.066, 0.010, n=5)
    shoulder = _rounded_rect_points(0.050, 0.050, 0.008, n=5)
    neck = [(NECK_R * math.cos(2 * math.pi * i / len(base)),
             NECK_R * math.sin(2 * math.pi * i / len(base))) for i in range(len(base))]
    return _loft_sections(
        [(0.0, base), (BODY_H, base), (SHOULDER_TOP, shoulder), (NECK_TOP, neck)],
        "bottle_shell",
    )


def _oval_bottle_mesh():
    count = 64
    def ellipse(rx, ry):
        return [(rx * math.cos(2 * math.pi * i / count),
                 ry * math.sin(2 * math.pi * i / count)) for i in range(count)]
    return _loft_sections(
        [
            (0.0, ellipse(0.030, 0.020)),
            (0.040, ellipse(0.037, 0.023)),
            (BODY_H, ellipse(0.032, 0.020)),
            (SHOULDER_TOP, ellipse(0.019, 0.014)),
            (NECK_TOP, ellipse(NECK_R, NECK_R)),
        ],
        "bottle_shell",
    )


def _flask_ellipse(rx: float, ry: float, count: int = 72):
    """Sample an ellipse with given half-widths into (x, y) points."""
    return [
        (rx * math.cos(2 * math.pi * i / count),
         ry * math.sin(2 * math.pi * i / count))
        for i in range(count)
    ]


def _flask_bottle_mesh():
    """Tall flattened oval flask with tapered waist and rounded shoulders.

    Cross-section aspect ratios range from ~3.2:1 to ~3.8:1 (width:depth),
    giving a clearly lens-shaped front silhouette and a very slim side view.
    The neck stays circular (NECK_R) to accept the threaded collar.
    """
    sections = [
        (0.000, _flask_ellipse(0.028, 0.009)),      # base
        (0.008, _flask_ellipse(0.036, 0.010)),      # base flare
        (0.030, _flask_ellipse(0.043, 0.012)),      # lower body expanding
        (0.055, _flask_ellipse(0.045, 0.012)),      # widest (90 mm × 24 mm)
        (0.085, _flask_ellipse(0.034, 0.009)),      # waist taper (68 mm × 18 mm)
        (0.115, _flask_ellipse(0.038, 0.010)),      # above-waist recovery
        (0.148, _flask_ellipse(0.043, 0.012)),      # upper body (86 mm × 24 mm)
        (BODY_H - 0.006, _flask_ellipse(0.030, 0.010)),   # shoulder start
        (SHOULDER_TOP, _flask_ellipse(0.017, 0.009)),      # shoulder taper
        (NECK_TOP, _flask_ellipse(NECK_R, NECK_R)),        # neck top (circular)
    ]
    return _loft_sections(sections, "bottle_shell")


def _bottle_mesh():
    if VARIANT == "flattened_oval_flask":
        return _flask_bottle_mesh()
    if VARIANT == "square_prism_body":
        return _square_bottle_mesh()
    if VARIANT == "tapered_oval_body":
        return _oval_bottle_mesh()
    return _round_bottle_mesh()


# ── Front label ────────────────────────────────────────────────────────

def _front_label_mesh():
    geom = MeshGeometry()
    if VARIANT == "flattened_oval_flask":
        w = 0.052
        y = 0.0135
        z0, z1 = 0.055, 0.135
    elif VARIANT == "square_prism_body":
        w = 0.043
        y = BODY_R + 0.001
        z0, z1 = 0.052, 0.116
    elif VARIANT == "tapered_oval_body":
        w = 0.036
        y = 0.0245
        z0, z1 = 0.052, 0.116
    else:
        w = 0.036
        y = BODY_R + 0.001
        z0, z1 = 0.052, 0.116
    t = 0.001
    verts = [
        geom.add_vertex(-w / 2, y, z0),
        geom.add_vertex( w / 2, y, z0),
        geom.add_vertex( w / 2, y, z1),
        geom.add_vertex(-w / 2, y, z1),
        geom.add_vertex(-w / 2, y + t, z0),
        geom.add_vertex( w / 2, y + t, z0),
        geom.add_vertex( w / 2, y + t, z1),
        geom.add_vertex(-w / 2, y + t, z1),
    ]
    faces = [
        (0, 1, 2), (0, 2, 3),
        (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1),
        (3, 2, 6), (3, 6, 7),
        (1, 5, 6), (1, 6, 2),
        (0, 3, 7), (0, 7, 4),
    ]
    for face in faces:
        geom.add_face(*face)
    return mesh_from_geometry(geom, "front_label")


# ── Collar ─────────────────────────────────────────────────────────────

def _collar_mesh():
    geom = LatheGeometry(
        [
            (NECK_R + 0.001, COLLAR_Z0),
            (COLLAR_R, COLLAR_Z0),
            (COLLAR_R, COLLAR_Z1),
            (NECK_R + 0.002, COLLAR_Z1),
        ],
        segments=64,
        closed=True,
    )
    for i in range(28):
        a = 2 * math.pi * i / 28
        rib = CylinderGeometry(0.0008, COLLAR_Z1 - COLLAR_Z0, radial_segments=5)
        rib.translate(COLLAR_R + 0.0004, 0.0, (COLLAR_Z0 + COLLAR_Z1) / 2)
        rib.rotate_z(a)
        geom.merge(rib)
    return mesh_from_geometry(geom, "collar_shell")


# ── Pump head ──────────────────────────────────────────────────────────

def _pump_head_mesh():
    if VARIANT == "long_trigger_foam_head":
        chamber = LatheGeometry(
            [(0.006, 0.194), (0.022, 0.194), (0.024, 0.202),
             (0.024, 0.262), (0.021, 0.270), (0.007, 0.270), (0.007, 0.194)],
            segments=56, closed=True,
        )
        top = CylinderGeometry(0.026, 0.010, radial_segments=48)
        top.translate(0.0, 0.0, 0.276)
        chamber.merge(top)
        nozzle = tube_from_spline_points(
            [(0.020, 0.0, 0.252), (0.038, 0.0, 0.251), (0.052, 0.0, 0.247)],
            radius=0.0055, samples_per_segment=8, radial_segments=14,
        )
        chamber.merge(nozzle)
        stem = CylinderGeometry(0.006, 0.040, radial_segments=24)
        stem.translate(0.0, 0.0, 0.174)
        chamber.merge(stem)
        return mesh_from_geometry(chamber, "pump_head_shell")

    z_shift = 0.052 if VARIANT == "detached_dip_tube_pump" else 0.0
    stem_bottom = 0.150 + _ZLIFT + z_shift
    geom = LatheGeometry(
        [
            (0.004, stem_bottom),
            (0.006, stem_bottom),
            (0.006, PUMP_Z0 + z_shift),
            (0.017, PUMP_Z0 + z_shift),
            (0.017, PUMP_Z1 + z_shift),
            (0.010, PUMP_Z1 + 0.006 + z_shift),
            (0.004, PUMP_Z1 + 0.006 + z_shift),
        ],
        segments=56,
        closed=True,
    )
    disk = CylinderGeometry(0.020, 0.007, radial_segments=48)
    disk.translate(0.0, 0.0, PUMP_Z1 + 0.0095 + z_shift)
    geom.merge(disk)
    if VARIANT == "lock_twist_collar":
        for x in (-0.015, 0.015):
            tab = CylinderGeometry(0.003, 0.018, radial_segments=8)
            tab.rotate_y(math.pi / 2)
            tab.translate(x, 0.0, PUMP_Z0 + 0.006)
            geom.merge(tab)
    return mesh_from_geometry(geom, "pump_head_shell")


# ── Spout ──────────────────────────────────────────────────────────────

def _spout_mesh():
    z_shift = 0.052 if VARIANT == "detached_dip_tube_pump" else 0.0
    if VARIANT == "long_trigger_foam_head":
        return mesh_from_geometry(CylinderGeometry(0.002, 0.002), "spout_shell")
    zl = _ZLIFT
    pts = [
        (0.013, 0.0, 0.218 + zl + z_shift),
        (0.030, 0.0, 0.220 + zl + z_shift),
        (0.047, 0.0, 0.211 + zl + z_shift),
    ]
    geom = tube_from_spline_points(
        pts, radius=0.0038, samples_per_segment=10, radial_segments=14,
    )
    tip = CylinderGeometry(0.0043, 0.008, radial_segments=16)
    tip.rotate_y(math.pi / 2)
    tip.translate(0.050, 0.0, 0.211 + zl + z_shift)
    geom.merge(tip)
    return mesh_from_geometry(geom, "spout_shell")


# ── Dip tube ───────────────────────────────────────────────────────────

def _dip_tube_mesh():
    if VARIANT == "detached_dip_tube_pump":
        z_top = 0.206
        mid_z = 0.115
    elif VARIANT == "flattened_oval_flask":
        z_top = PUMP_Z0 - 0.010
        mid_z = 0.153
    else:
        z_top = 0.158
        mid_z = 0.115
    pts = [(0.0, 0.0, z_top), (0.006, 0.0, mid_z), (0.004, 0.0, 0.020)]
    geom = tube_from_spline_points(
        pts, radius=0.0022, samples_per_segment=12, radial_segments=10,
    )
    return mesh_from_geometry(geom, "dip_tube")


# ── Liquid fill ────────────────────────────────────────────────────────

def _elliptical_liquid_mesh() -> object:
    """Elliptical cylinder liquid fill sized for the flask body interior."""
    segments = 48
    rx, ry = 0.028, 0.008
    z_base = 0.012
    height = 0.068
    geom = MeshGeometry()
    bottom = []
    top = []
    for i in range(segments):
        a = 2 * math.pi * i / segments
        x, y = rx * math.cos(a), ry * math.sin(a)
        bottom.append(geom.add_vertex(x, y, z_base))
        top.append(geom.add_vertex(x, y, z_base + height))
    bc = geom.add_vertex(0.0, 0.0, z_base)
    tc = geom.add_vertex(0.0, 0.0, z_base + height)
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])
        geom.add_face(bc, bottom[j], bottom[i])
        geom.add_face(tc, top[i], top[j])
    return mesh_from_geometry(geom, "liquid_fill")


# ── Object assembly ────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name=f"container_dispenser_{VARIANT}")
    clear = model.material("clear_plastic", rgba=(0.78, 0.88, 0.92, 0.34))
    liquid = model.material("pale_soap", rgba=(0.78, 0.90, 0.84, 0.62))
    label_mat = model.material("white_label", rgba=(0.96, 0.96, 0.91, 1.0))
    white = model.material("warm_white", rgba=(0.94, 0.93, 0.90, 1.0))
    grey = model.material("soft_grey", rgba=(0.70, 0.72, 0.72, 1.0))
    tube_mat = model.material("milky_tube", rgba=(0.88, 0.92, 0.92, 0.78))

    # ── Bottle (root) ──
    bottle = model.part("bottle")
    bottle.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    bottle.visual(_front_label_mesh(), material=label_mat, name="front_label")
    if VARIANT == "flattened_oval_flask":
        bottle.visual(_elliptical_liquid_mesh(), material=liquid, name="liquid_fill")
    else:
        liquid_geom = CylinderGeometry(
            0.026 if VARIANT != "tapered_oval_body" else 0.020,
            0.065, radial_segments=48,
        )
        liquid_geom.translate(0.0, 0.0, 0.0385)
        bottle.visual(
            mesh_from_geometry(liquid_geom, "liquid_fill"),
            material=liquid, name="liquid_fill",
        )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_H), mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2)),
    )

    # ── Collar (fixed to bottle neck) ──
    collar = model.part("collar")
    collar.visual(_collar_mesh(), material=white, name="collar_shell")
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_Z1 - COLLAR_Z0), mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, (COLLAR_Z0 + COLLAR_Z1) / 2)),
    )
    model.articulation(
        "bottle_to_collar", ArticulationType.FIXED,
        parent=bottle, child=collar, origin=Origin(),
    )

    parent_for_head = collar
    if VARIANT == "lock_twist_collar":
        lock_ring = model.part("lock_ring")
        lock_ring.visual(
            _ring_mesh("lock_ring_shell", 0.023, 0.017, 0.198, 0.207),
            material=grey, name="lock_ring_shell",
        )
        lock_ring.inertial = Inertial.from_geometry(
            Cylinder(0.023, 0.009), mass=0.006,
            origin=Origin(xyz=(0.0, 0.0, 0.2025)),
        )
        model.articulation(
            "lock_ring_twist", ArticulationType.REVOLUTE,
            parent=collar, child=lock_ring, origin=Origin(),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.0,
                                       lower=0.0, upper=math.pi / 2),
        )
        parent_for_head = lock_ring

    # ── Pump head (press-down prismatic) ──
    pump = model.part("pump_head")
    pump.visual(_pump_head_mesh(), material=white, name="pump_head_shell")
    if VARIANT == "long_trigger_foam_head":
        pump.inertial = Inertial.from_geometry(
            Cylinder(0.026, 0.082), mass=0.045,
            origin=Origin(xyz=(0.0, 0.0, 0.236)),
        )
        model.articulation(
            "bottle_to_pump", ArticulationType.FIXED,
            parent=parent_for_head, child=pump, origin=Origin(),
        )
    else:
        pump.inertial = Inertial.from_geometry(
            Cylinder(0.019, 0.080), mass=0.035,
            origin=Origin(xyz=(0.0, 0.0, (PUMP_Z0 + PUMP_Z1) / 2)),
        )
        lower = -PRESS_TRAVEL
        upper = 0.0
        if VARIANT == "detached_dip_tube_pump":
            lower, upper = -0.020, 0.025
        model.articulation(
            "pump_press", ArticulationType.PRISMATIC,
            parent=parent_for_head, child=pump, origin=Origin(),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=0.15,
                                       lower=lower, upper=upper),
        )

    # ── Dip tube (fixed to pump) ──
    dip_tube = model.part("dip_tube")
    dip_tube.visual(_dip_tube_mesh(), material=tube_mat, name="dip_tube")
    dip_tube.inertial = Inertial.from_geometry(
        Cylinder(0.003, 0.145 + _ZLIFT), mass=0.004,
        origin=Origin(xyz=(0.003, 0.0, (0.020 + PUMP_Z0) / 2)),
    )
    model.articulation(
        "pump_to_dip_tube", ArticulationType.FIXED,
        parent=pump, child=dip_tube, origin=Origin(),
    )

    # ── Spout (revolute swivel on pump) ──
    if VARIANT != "long_trigger_foam_head":
        spout = model.part("spout")
        spout.visual(_spout_mesh(), material=white, name="spout_shell")
        spout.inertial = Inertial.from_geometry(
            Cylinder(0.005, 0.050), mass=0.006,
            origin=Origin(xyz=(0.033, 0.0, 0.218 + _ZLIFT)),
        )
        model.articulation(
            "spout_swivel", ArticulationType.REVOLUTE,
            parent=pump, child=spout, origin=Origin(),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=3.0,
                                       lower=-math.pi, upper=math.pi),
        )

    if VARIANT == "long_trigger_foam_head":
        trigger = model.part("trigger")
        trigger.visual(_trigger_mesh(), material=grey, name="trigger_lever")
        trigger.inertial = Inertial.from_geometry(
            Box((0.052, 0.014, 0.012)), mass=0.008,
            origin=Origin(xyz=(0.043, 0.0, 0.231)),
        )
        model.articulation(
            "trigger_pivot", ArticulationType.REVOLUTE,
            parent=pump, child=trigger,
            origin=Origin(xyz=(0.024, 0.0, 0.244)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=3.0,
                                       lower=-0.55, upper=0.15),
        )

    return model


# ── Test helpers ───────────────────────────────────────────────────────

def _z_top(ctx: TestContext, part, elem: str) -> float:
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    return aabb[1][2] if aabb else -999.0


def _z_min(ctx: TestContext, part, elem: str) -> float:
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    return aabb[0][2] if aabb else 999.0


# ── Tests ──────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    pump = object_model.get_part("pump_head")
    dip_tube = object_model.get_part("dip_tube")

    # ── Intentional overlap allowances ──
    ctx.allow_overlap(
        collar, bottle, elem_a="collar_shell", elem_b="bottle_shell",
        reason="The threaded collar screws over the bottle neck.",
    )
    ctx.allow_overlap(
        pump, collar, elem_a="pump_head_shell", elem_b="collar_shell",
        reason="The pump stem passes through the collar bore.",
    )
    ctx.allow_overlap(
        dip_tube, bottle, elem_a="dip_tube", elem_b="bottle_shell",
        reason="The suction dip tube runs inside the transparent bottle.",
    )
    ctx.allow_overlap(
        dip_tube, pump, elem_a="dip_tube", elem_b="pump_head_shell",
        reason="The dip tube plugs into the pump stem.",
    )
    ctx.allow_overlap(
        bottle, bottle, elem_a="liquid_fill", elem_b="bottle_shell",
        reason="The visible liquid volume is contained by the transparent bottle wall.",
    )
    ctx.allow_overlap(
        pump, bottle, elem_a="pump_head_shell", elem_b="bottle_shell",
        reason="The pump stem passes through the bottle neck and shoulder interior.",
    )

    # ── Shared assertions ──
    ctx.expect_overlap(collar, bottle, axes="xy", min_overlap=0.020,
                       name="collar centered on neck")
    ctx.check("transparent bottle material",
              bottle.get_visual("bottle_shell").material.rgba[3] < 0.5)
    ctx.check("dip tube descends below label",
              _z_min(ctx, dip_tube, "dip_tube") < 0.030)

    # ── Pump-press and spout-swivel assertions ──
    pump_spout_variants = {
        "parent", "square_prism_body", "tapered_oval_body",
        "lock_twist_collar", "flattened_oval_flask",
    }
    if VARIANT in pump_spout_variants:
        spout = object_model.get_part("spout")
        press = object_model.get_articulation("pump_press")
        swivel = object_model.get_articulation("spout_swivel")
        ctx.allow_overlap(
            spout, pump, elem_a="spout_shell", elem_b="pump_head_shell",
            reason="The swivel spout plugs into the pump head socket.",
        )
        rest_z = ctx.part_world_position(pump)[2]
        with ctx.pose({press: -PRESS_TRAVEL}):
            pressed_z = ctx.part_world_position(pump)[2]
        ctx.check(
            "pump head presses downward",
            pressed_z < rest_z - 0.010,
            details=f"rest={rest_z}, pressed={pressed_z}",
        )
        ext0 = ctx.part_element_world_aabb(spout, elem="spout_shell")
        with ctx.pose({swivel: math.pi / 2}):
            ext90 = ctx.part_element_world_aabb(spout, elem="spout_shell")
        ctx.check(
            "spout visibly swivels",
            ext0 is not None and ext90 is not None
            and (ext0[1][0] - ext0[0][0]) > (ext90[1][0] - ext90[0][0]) + 0.010,
        )
        ctx.check(
            "pump spout remains exposed like the reference",
            _z_top(ctx, spout, "spout_shell") > _z_top(ctx, pump, "pump_head_shell") - 0.030,
        )

    # ── Variant-specific body shape assertions ──
    if VARIANT == "square_prism_body":
        aabb = ctx.part_element_world_aabb(bottle, elem="bottle_shell")
        ctx.check(
            "square prism body has broad flat footprint",
            aabb is not None
            and abs((aabb[1][0] - aabb[0][0]) - (aabb[1][1] - aabb[0][1])) < 0.004,
        )
    elif VARIANT == "tapered_oval_body":
        aabb = ctx.part_element_world_aabb(bottle, elem="bottle_shell")
        ctx.check(
            "oval body is flattened front-to-back",
            aabb is not None
            and (aabb[1][0] - aabb[0][0]) > (aabb[1][1] - aabb[0][1]) * 1.35,
        )
    elif VARIANT == "flattened_oval_flask":
        aabb = ctx.part_element_world_aabb(bottle, elem="bottle_shell")
        width = aabb[1][0] - aabb[0][0]
        depth = aabb[1][1] - aabb[0][1]
        height = aabb[1][2] - aabb[0][2]
        ctx.check(
            "flask body is very flat oval (width:depth >= 3:1)",
            aabb is not None and width > depth * 3.0,
            details=f"width={width:.4f}, depth={depth:.4f}",
        )
        ctx.check(
            "flask body is visibly taller than parent round bottle",
            height > 0.170,
            details=f"height={height:.4f}",
        )
        ctx.check(
            "flask has wide lens-shaped front silhouette",
            width > 0.065,
            details=f"width={width:.4f}",
        )
        pump_top = _z_top(ctx, pump, "pump_head_shell")
        ctx.check(
            "exposed pump head sits well above bottle shoulder",
            pump_top > SHOULDER_TOP + 0.020,
            details=f"pump_top={pump_top:.4f}, shoulder={SHOULDER_TOP}",
        )
        ctx.expect_overlap(
            pump, bottle, axes="z",
            elem_a="pump_head_shell", elem_b="bottle_shell",
            min_overlap=0.010,
            name="pump stem inserted into bottle neck",
        )
        # The pump stem (r≈0.006) passes through the neck bore (r=NECK_R=0.014);
        # the collar sits between them and the stem is captured inside.
        ctx.expect_gap(
            pump, bottle, axis="z",
            positive_elem="pump_head_shell", negative_elem="bottle_shell",
            max_penetration=0.040,
            name="pump stem does not extend below bottle body",
        )

    # ── Variant-specific mechanism assertions ──
    if VARIANT == "lock_twist_collar":
        ring = object_model.get_part("lock_ring")
        twist = object_model.get_articulation("lock_ring_twist")
        ctx.allow_overlap(
            ring, collar, elem_a="lock_ring_shell", elem_b="collar_shell",
            reason="The lock ring rides around the collar throat.",
        )
        ctx.allow_overlap(
            pump, ring, elem_a="pump_head_shell", elem_b="lock_ring_shell",
            reason="Pump lock tabs engage the rotating lock ring.",
        )
        ctx.check("lock ring twists a quarter turn",
                  twist.motion_limits.upper >= math.pi / 2 - 0.001)
    elif VARIANT == "long_trigger_foam_head":
        trigger = object_model.get_part("trigger")
        pivot = object_model.get_articulation("trigger_pivot")
        ctx.allow_overlap(
            trigger, pump, elem_a="trigger_lever", elem_b="pump_head_shell",
            reason="The trigger pin is captured in the foaming pump head.",
        )
        ctx.check("foaming head is taller than parent pump",
                  _z_top(ctx, pump, "pump_head_shell") > 0.270)
        rest = ctx.part_world_position(trigger)
        with ctx.pose({pivot: -0.45}):
            pulled = ctx.part_world_position(trigger)
        ctx.check(
            "trigger lever pivots",
            rest is not None and pulled is not None
            and abs(pulled[2] - rest[2]) > 0.002,
        )
    elif VARIANT == "detached_dip_tube_pump":
        spout = object_model.get_part("spout")
        lift = object_model.get_articulation("pump_press")
        ctx.allow_overlap(
            spout, pump, elem_a="spout_shell", elem_b="pump_head_shell",
            reason="The spout plugs into the lifted pump head socket.",
        )
        ctx.check("pump assembly is visibly lifted above collar",
                  _z_min(ctx, pump, "pump_head_shell") > COLLAR_Z1 + 0.004)
        ctx.check("lifted pump still has long tube entering bottle",
                  _z_min(ctx, pump, "dip_tube") < 0.030)
        rest_z = ctx.part_world_position(pump)[2]
        with ctx.pose({lift: 0.020}):
            lifted_z = ctx.part_world_position(pump)[2]
        ctx.check("detached pump can lift farther upward",
                  lifted_z > rest_z + 0.015)

    return ctx.report()


object_model = build_object_model()
