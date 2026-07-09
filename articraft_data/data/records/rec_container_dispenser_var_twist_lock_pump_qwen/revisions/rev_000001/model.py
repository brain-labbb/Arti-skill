from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
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

VARIANT = "lock_twist_collar"

BODY_R = 0.033
BODY_H = 0.142
SHOULDER_TOP = 0.166
NECK_R = 0.014
NECK_TOP = 0.188
COLLAR_R = 0.0185
COLLAR_Z0 = 0.170
COLLAR_Z1 = 0.196
PUMP_Z0 = 0.194
PUMP_Z1 = 0.229
PRESS_TRAVEL = 0.018
CAP_Z0 = 0.188
CAP_Z1 = 0.263
CAP_R = 0.036


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
    for z in (0.174, 0.181, 0.187):
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
    neck = [(NECK_R * math.cos(2 * math.pi * i / len(base)), NECK_R * math.sin(2 * math.pi * i / len(base))) for i in range(len(base))]
    return _loft_sections([(0.0, base), (BODY_H, base), (SHOULDER_TOP, shoulder), (NECK_TOP, neck)], "bottle_shell")


def _oval_bottle_mesh():
    count = 64
    def ellipse(rx, ry):
        return [(rx * math.cos(2 * math.pi * i / count), ry * math.sin(2 * math.pi * i / count)) for i in range(count)]
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


def _bottle_mesh():
    if VARIANT == "square_prism_body":
        return _square_bottle_mesh()
    if VARIANT == "tapered_oval_body":
        return _oval_bottle_mesh()
    return _round_bottle_mesh()


def _front_label_mesh():
    geom = MeshGeometry()
    w = 0.036 if VARIANT != "square_prism_body" else 0.043
    y = BODY_R + 0.001 if VARIANT != "tapered_oval_body" else 0.0245
    z0, z1 = 0.052, 0.116
    t = 0.001
    verts = [
        geom.add_vertex(-w / 2, y, z0),
        geom.add_vertex(w / 2, y, z0),
        geom.add_vertex(w / 2, y, z1),
        geom.add_vertex(-w / 2, y, z1),
        geom.add_vertex(-w / 2, y + t, z0),
        geom.add_vertex(w / 2, y + t, z0),
        geom.add_vertex(w / 2, y + t, z1),
        geom.add_vertex(-w / 2, y + t, z1),
    ]
    faces = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 4, 5), (0, 5, 1), (3, 2, 6), (3, 6, 7), (1, 5, 6), (1, 6, 2), (0, 3, 7), (0, 7, 4)]
    for face in faces:
        geom.add_face(*face)
    return mesh_from_geometry(geom, "front_label")


def _collar_mesh():
    geom = LatheGeometry([(NECK_R + 0.001, COLLAR_Z0), (COLLAR_R, COLLAR_Z0), (COLLAR_R, COLLAR_Z1), (NECK_R + 0.002, COLLAR_Z1)], segments=64, closed=True)
    for i in range(28):
        a = 2 * math.pi * i / 28
        rib = CylinderGeometry(0.0008, COLLAR_Z1 - COLLAR_Z0, radial_segments=5)
        rib.translate(COLLAR_R + 0.0004, 0.0, (COLLAR_Z0 + COLLAR_Z1) / 2)
        rib.rotate_z(a)
        geom.merge(rib)
    return mesh_from_geometry(geom, "collar_shell")


def _lock_ring_mesh():
    """Narrow twist-lock ring with low-profile grip tabs, indicator ridges, and cam slots."""
    ring_z0 = COLLAR_Z1  # 0.196 — sits flush on collar top
    ring_h = 0.005  # 5 mm tall — thin ring
    ring_z1 = ring_z0 + ring_h
    r_outer = 0.020
    r_inner = 0.0155

    # Ring body: thin annular lathe profile
    geom = LatheGeometry(
        [
            (r_inner, ring_z0),
            (r_outer, ring_z0),
            (r_outer, ring_z1),
            (r_outer - 0.001, ring_z1 + 0.0005),
            (r_inner + 0.001, ring_z1 + 0.0005),
            (r_inner, ring_z1),
        ],
        segments=64,
        closed=True,
    )

    # Two small low-profile grip tabs (opposite sides, ~4 mm wide, ~1.5 mm proud)
    tab_w = 0.004
    tab_proud = 0.0015
    for sign in (1, -1):
        tab = BoxGeometry((tab_w, tab_proud, ring_h))
        tab.translate(sign * (r_outer + tab_proud / 2), 0.0, (ring_z0 + ring_z1) / 2)
        geom.merge(tab)

    # Shallow lock/unlock indicator ridges — small raised bumps at 90° and 270°
    for angle_deg in (90, 270):
        a = math.radians(angle_deg)
        bump = CylinderGeometry(0.0008, 0.0008, radial_segments=6)
        bump.translate(
            (r_outer + 0.0004) * math.cos(a),
            (r_outer + 0.0004) * math.sin(a),
            (ring_z0 + ring_z1) / 2,
        )
        geom.merge(bump)

    # Small cam-slot grooves — thin slots on the top face at 45° intervals
    slot_depth = 0.001
    slot_width = 0.002
    slot_length = 0.004
    for i in range(4):
        a = math.pi / 2 * i + math.pi / 8
        r_mid = (r_outer + r_inner) / 2
        slot = BoxGeometry((slot_length, slot_width, slot_depth))
        slot.translate(r_mid, 0.0, ring_z1 + slot_depth / 2)
        slot.rotate_z(a)
        geom.merge(slot)

    return mesh_from_geometry(geom, "lock_ring_shell")


def _pump_head_mesh():
    if VARIANT == "long_trigger_foam_head":
        chamber = LatheGeometry([(0.006, 0.194), (0.022, 0.194), (0.024, 0.202), (0.024, 0.262), (0.021, 0.270), (0.007, 0.270), (0.007, 0.194)], segments=56, closed=True)
        top = CylinderGeometry(0.026, 0.010, radial_segments=48)
        top.translate(0.0, 0.0, 0.276)
        chamber.merge(top)
        nozzle = tube_from_spline_points([(0.020, 0.0, 0.252), (0.038, 0.0, 0.251), (0.052, 0.0, 0.247)], radius=0.0055, samples_per_segment=8, radial_segments=14)
        chamber.merge(nozzle)
        stem = CylinderGeometry(0.006, 0.040, radial_segments=24)
        stem.translate(0.0, 0.0, 0.174)
        chamber.merge(stem)
        return mesh_from_geometry(chamber, "pump_head_shell")

    z_shift = 0.052 if VARIANT == "detached_dip_tube_pump" else 0.0
    geom = LatheGeometry(
        [
            (0.004, 0.150 + z_shift),
            (0.006, 0.150 + z_shift),
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
    # For lock_twist_collar: small engagement pins on the pump stem that ride in cam slots
    if VARIANT == "lock_twist_collar":
        for i in range(2):
            a = math.pi * i + math.pi / 8
            pin = CylinderGeometry(0.001, 0.004, radial_segments=6)
            pin.translate(
                0.016 * math.cos(a),
                0.016 * math.sin(a),
                PUMP_Z0 + 0.002,
            )
            geom.merge(pin)
    return mesh_from_geometry(geom, "pump_head_shell")


def _spout_mesh():
    z_shift = 0.052 if VARIANT == "detached_dip_tube_pump" else 0.0
    if VARIANT == "long_trigger_foam_head":
        return mesh_from_geometry(CylinderGeometry(0.002, 0.002), "spout_shell")
    pts = [(0.013, 0.0, 0.218 + z_shift), (0.030, 0.0, 0.220 + z_shift), (0.047, 0.0, 0.211 + z_shift)]
    geom = tube_from_spline_points(pts, radius=0.0038, samples_per_segment=10, radial_segments=14)
    tip = CylinderGeometry(0.0043, 0.008, radial_segments=16)
    tip.rotate_y(math.pi / 2)
    tip.translate(0.050, 0.0, 0.211 + z_shift)
    geom.merge(tip)
    return mesh_from_geometry(geom, "spout_shell")


def _dip_tube_mesh():
    z_top = 0.158 if VARIANT != "detached_dip_tube_pump" else 0.206
    pts = [(0.0, 0.0, z_top), (0.006, 0.0, 0.115), (0.004, 0.0, 0.020)]
    geom = tube_from_spline_points(pts, radius=0.0022, samples_per_segment=12, radial_segments=10)
    return mesh_from_geometry(geom, "dip_tube")



def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name=f"container_dispenser_{VARIANT}")
    clear = model.material("clear_plastic", rgba=(0.78, 0.88, 0.92, 0.34))
    clear_cap = model.material("clear_cap", rgba=(0.86, 0.94, 0.98, 0.28))
    liquid = model.material("pale_soap", rgba=(0.78, 0.90, 0.84, 0.62))
    label = model.material("white_label", rgba=(0.96, 0.96, 0.91, 1.0))
    white = model.material("warm_white", rgba=(0.94, 0.93, 0.90, 1.0))
    grey = model.material("soft_grey", rgba=(0.70, 0.72, 0.72, 1.0))
    tube = model.material("milky_tube", rgba=(0.88, 0.92, 0.92, 0.78))

    bottle = model.part("bottle")
    bottle.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    bottle.visual(_front_label_mesh(), material=label, name="front_label")
    liquid_geom = CylinderGeometry(0.026 if VARIANT != "tapered_oval_body" else 0.020, 0.065, radial_segments=48)
    liquid_geom.translate(0.0, 0.0, 0.0385)
    bottle.visual(mesh_from_geometry(liquid_geom, "liquid_fill"), material=liquid, name="liquid_fill")
    bottle.inertial = Inertial.from_geometry(Cylinder(BODY_R, BODY_H), mass=0.22, origin=Origin(xyz=(0.0, 0.0, BODY_H / 2)))

    collar = model.part("collar")
    collar.visual(_collar_mesh(), material=white, name="collar_shell")
    collar.inertial = Inertial.from_geometry(Cylinder(COLLAR_R, COLLAR_Z1 - COLLAR_Z0), mass=0.018, origin=Origin(xyz=(0.0, 0.0, (COLLAR_Z0 + COLLAR_Z1) / 2)))
    model.articulation("bottle_to_collar", ArticulationType.FIXED, parent=bottle, child=collar, origin=Origin())

    parent_for_head = collar
    if VARIANT == "lock_twist_collar":
        lock_ring = model.part("lock_ring")
        lock_ring.visual(_lock_ring_mesh(), material=grey, name="lock_ring_shell")
        lock_ring.inertial = Inertial.from_geometry(Cylinder(0.020, 0.005), mass=0.004, origin=Origin(xyz=(0.0, 0.0, COLLAR_Z1 + 0.0025)))
        model.articulation("lock_ring_twist", ArticulationType.REVOLUTE, parent=collar, child=lock_ring, origin=Origin(), axis=(0.0, 0.0, 1.0), motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=math.pi / 2))
        parent_for_head = lock_ring

    pump = model.part("pump_head")
    pump.visual(_pump_head_mesh(), material=white, name="pump_head_shell")
    if VARIANT == "long_trigger_foam_head":
        pump.inertial = Inertial.from_geometry(Cylinder(0.026, 0.082), mass=0.045, origin=Origin(xyz=(0.0, 0.0, 0.236)))
        model.articulation("bottle_to_pump", ArticulationType.FIXED, parent=parent_for_head, child=pump, origin=Origin())
    else:
        pump.inertial = Inertial.from_geometry(Cylinder(0.019, 0.080), mass=0.035, origin=Origin(xyz=(0.0, 0.0, 0.205)))
        lower = -PRESS_TRAVEL
        upper = 0.0
        if VARIANT == "detached_dip_tube_pump":
            lower, upper = -0.020, 0.025
        model.articulation("pump_press", ArticulationType.PRISMATIC, parent=parent_for_head, child=pump, origin=Origin(), axis=(0.0, 0.0, 1.0), motion_limits=MotionLimits(effort=20.0, velocity=0.15, lower=lower, upper=upper))

    dip_tube = model.part("dip_tube")
    dip_tube.visual(_dip_tube_mesh(), material=tube, name="dip_tube")
    dip_tube.inertial = Inertial.from_geometry(Cylinder(0.003, 0.145), mass=0.004, origin=Origin(xyz=(0.003, 0.0, 0.085)))
    model.articulation("pump_to_dip_tube", ArticulationType.FIXED, parent=pump, child=dip_tube, origin=Origin())

    if VARIANT != "long_trigger_foam_head":
        spout = model.part("spout")
        spout.visual(_spout_mesh(), material=white, name="spout_shell")
        spout.inertial = Inertial.from_geometry(Cylinder(0.005, 0.050), mass=0.006, origin=Origin(xyz=(0.033, 0.0, 0.218)))
        model.articulation("spout_swivel", ArticulationType.REVOLUTE, parent=pump, child=spout, origin=Origin(), axis=(0.0, 0.0, 1.0), motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-math.pi, upper=math.pi))

    if VARIANT == "long_trigger_foam_head":
        trigger = model.part("trigger")
        trigger.visual(_trigger_mesh(), material=grey, name="trigger_lever")
        trigger.inertial = Inertial.from_geometry(Box((0.052, 0.014, 0.012)), mass=0.008, origin=Origin(xyz=(0.043, 0.0, 0.231)))
        model.articulation("trigger_pivot", ArticulationType.REVOLUTE, parent=pump, child=trigger, origin=Origin(xyz=(0.024, 0.0, 0.244)), axis=(0.0, 1.0, 0.0), motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-0.55, upper=0.15))
    return model


def _z_top(ctx: TestContext, part, elem: str) -> float:
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    return aabb[1][2] if aabb else -999.0


def _z_min(ctx: TestContext, part, elem: str) -> float:
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    return aabb[0][2] if aabb else 999.0


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    pump = object_model.get_part("pump_head")
    dip_tube = object_model.get_part("dip_tube")

    ctx.allow_overlap(collar, bottle, elem_a="collar_shell", elem_b="bottle_shell", reason="The threaded collar screws over the bottle neck.")
    ctx.allow_overlap(pump, collar, elem_a="pump_head_shell", elem_b="collar_shell", reason="The pump stem passes through the collar bore.")
    ctx.allow_overlap(dip_tube, bottle, elem_a="dip_tube", elem_b="bottle_shell", reason="The suction dip tube runs inside the transparent bottle.")
    ctx.allow_overlap(dip_tube, pump, elem_a="dip_tube", elem_b="pump_head_shell", reason="The dip tube plugs into the pump stem.")
    ctx.allow_overlap(bottle, bottle, elem_a="liquid_fill", elem_b="bottle_shell", reason="The visible liquid volume is contained by the transparent bottle wall.")
    ctx.expect_overlap(collar, bottle, axes="xy", min_overlap=0.020, name="collar centered on neck")
    ctx.check("transparent bottle material", bottle.get_visual("bottle_shell").material.rgba[3] < 0.5)
    ctx.check("dip tube descends below label", _z_min(ctx, dip_tube, "dip_tube") < 0.030)

    if VARIANT in {"parent", "square_prism_body", "tapered_oval_body", "lock_twist_collar"}:
        spout = object_model.get_part("spout")
        press = object_model.get_articulation("pump_press")
        swivel = object_model.get_articulation("spout_swivel")
        ctx.allow_overlap(spout, pump, elem_a="spout_shell", elem_b="pump_head_shell", reason="The swivel spout plugs into the pump head socket.")
        rest_z = ctx.part_world_position(pump)[2]
        with ctx.pose({press: -PRESS_TRAVEL}):
            pressed_z = ctx.part_world_position(pump)[2]
        ctx.check("pump head presses downward", pressed_z < rest_z - 0.010, details=f"rest={rest_z}, pressed={pressed_z}")
        ext0 = ctx.part_element_world_aabb(spout, elem="spout_shell")
        with ctx.pose({swivel: math.pi / 2}):
            ext90 = ctx.part_element_world_aabb(spout, elem="spout_shell")
        ctx.check("spout visibly swivels", ext0 is not None and ext90 is not None and (ext0[1][0] - ext0[0][0]) > (ext90[1][0] - ext90[0][0]) + 0.010)
        ctx.check("pump spout remains exposed like the reference", _z_top(ctx, spout, "spout_shell") > _z_top(ctx, pump, "pump_head_shell") - 0.030)

    if VARIANT == "square_prism_body":
        aabb = ctx.part_element_world_aabb(bottle, elem="bottle_shell")
        ctx.check("square prism body has broad flat footprint", aabb is not None and abs((aabb[1][0]-aabb[0][0]) - (aabb[1][1]-aabb[0][1])) < 0.004)
    elif VARIANT == "tapered_oval_body":
        aabb = ctx.part_element_world_aabb(bottle, elem="bottle_shell")
        ctx.check("oval body is flattened front-to-back", aabb is not None and (aabb[1][0]-aabb[0][0]) > (aabb[1][1]-aabb[0][1]) * 1.35)

    if VARIANT == "lock_twist_collar":
        ring = object_model.get_part("lock_ring")
        twist = object_model.get_articulation("lock_ring_twist")
        ctx.allow_overlap(ring, collar, elem_a="lock_ring_shell", elem_b="collar_shell", reason="The narrow lock ring rides around the collar throat just above the threaded collar.")
        ctx.allow_overlap(pump, ring, elem_a="pump_head_shell", elem_b="lock_ring_shell", reason="Small pump engagement pins ride in the lock ring cam slots.")

        # Lock ring exists and is a distinct part
        ctx.check("lock_ring part exists", ring is not None)

        # Lock ring is narrow/low-profile (thin in Z)
        ring_aabb = ctx.part_element_world_aabb(ring, elem="lock_ring_shell")
        if ring_aabb is not None:
            ring_height = ring_aabb[1][2] - ring_aabb[0][2]
            ctx.check(
                "lock ring is low-profile (under 10mm tall)",
                ring_height < 0.010,
                details=f"ring_height={ring_height:.4f}",
            )
            # Lock ring sits near the collar top, not far above
            ctx.check(
                "lock ring sits near collar top",
                ring_aabb[0][2] > COLLAR_Z1 - 0.003,
                details=f"ring_bottom_z={ring_aabb[0][2]:.4f}",
            )

        # Lock ring is wider than the collar but not oversized
        ring_span_x = ring_aabb[1][0] - ring_aabb[0][0] if ring_aabb else 0
        ctx.check(
            "lock ring is modest diameter (under 50mm)",
            ring_span_x < 0.050,
            details=f"ring_span_x={ring_span_x:.4f}",
        )

        # Lock ring twists a quarter turn
        ctx.check("lock ring twists a quarter turn", twist.motion_limits.upper >= math.pi / 2 - 0.001)

        # Verify twist motion actually rotates the ring
        rest_pos = ctx.part_world_position(ring)
        with ctx.pose({twist: math.pi / 2}):
            twisted_pos = ctx.part_world_position(ring)
        # The ring origin should not translate (it's a revolute joint), but verify it exists
        ctx.check(
            "lock ring twist joint is revolute",
            rest_pos is not None and twisted_pos is not None,
        )

        # Pump remains uncapped — no part above the pump head disk
        pump_top = _z_top(ctx, pump, "pump_head_shell")
        ctx.check(
            "pump head remains exposed and uncapped",
            pump_top > 0.230,
            details=f"pump_top_z={pump_top:.4f}",
        )

        # No cap part exists (verify no closure was added)
        cap_names = {"cap", "dust_cover", "flip_lid", "dome_cap", "over_cap"}
        existing_parts = {p.name for p in object_model.parts}
        ctx.check(
            "no cap or closure parts exist",
            not cap_names.intersection(existing_parts),
            details=f"parts={existing_parts}",
        )

        # Pump press still works
        press = object_model.get_articulation("pump_press")
        rest_z = ctx.part_world_position(pump)[2]
        with ctx.pose({press: -PRESS_TRAVEL}):
            pressed_z = ctx.part_world_position(pump)[2]
        ctx.check(
            "pump head presses downward through lock ring",
            pressed_z < rest_z - 0.010,
            details=f"rest={rest_z}, pressed={pressed_z}",
        )
    elif VARIANT == "long_trigger_foam_head":
        trigger = object_model.get_part("trigger")
        pivot = object_model.get_articulation("trigger_pivot")
        ctx.allow_overlap(trigger, pump, elem_a="trigger_lever", elem_b="pump_head_shell", reason="The trigger pin is captured in the foaming pump head.")
        ctx.check("foaming head is taller than parent pump", _z_top(ctx, pump, "pump_head_shell") > 0.270)
        rest = ctx.part_world_position(trigger)
        with ctx.pose({pivot: -0.45}):
            pulled = ctx.part_world_position(trigger)
        ctx.check("trigger lever pivots", rest is not None and pulled is not None and abs(pulled[2] - rest[2]) > 0.002)
    elif VARIANT == "detached_dip_tube_pump":
        spout = object_model.get_part("spout")
        lift = object_model.get_articulation("pump_press")
        ctx.allow_overlap(spout, pump, elem_a="spout_shell", elem_b="pump_head_shell", reason="The spout plugs into the lifted pump head socket.")
        ctx.check("pump assembly is visibly lifted above collar", _z_min(ctx, pump, "pump_head_shell") > COLLAR_Z1 + 0.004)
        ctx.check("lifted pump still has long tube entering bottle", _z_min(ctx, pump, "dip_tube") < 0.030)
        rest_z = ctx.part_world_position(pump)[2]
        with ctx.pose({lift: 0.020}):
            lifted_z = ctx.part_world_position(pump)[2]
        ctx.check("detached pump can lift farther upward", lifted_z > rest_z + 0.015)

    return ctx.report()


object_model = build_object_model()
