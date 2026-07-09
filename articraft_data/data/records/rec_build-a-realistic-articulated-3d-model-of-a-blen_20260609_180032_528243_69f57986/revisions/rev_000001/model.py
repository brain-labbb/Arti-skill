from __future__ import annotations

# Countertop blender: bulbous matte-black plastic base on rubber feet with a
# graphite front control panel carrying a lever-style speed knob, a fluted
# clear glass jar with a side handle seated on the base coupling, and a dark
# lid with a red accent ring and a removable center measuring cap.
#
# Articulated parts (all clearly visible):
#   - jar_lid:    PRISMATIC vertical lift — the lid (and its center cap) lifts
#                 straight up off the jar rim, the dominant motion of a real
#                 blender.
#   - lid_cap:    REVOLUTE twist of the center measuring cap (lugged, so the
#                 rotation is visible).
#   - speed_dial: REVOLUTE lever knob on the front panel, -150..150 deg, with a
#                 D-flat silhouette, a raised lever bar and an off-axis light
#                 pointer strip so the sweep reads.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Key dimensions (meters)
# ----------------------------------------------------------------------------
FOOT_LIFT = 0.010  # rubber feet raise the housing this much off the ground
BASE_BELLY_R = 0.108  # widest radius of the bulbous body
BASE_BOTTOM_R = 0.082  # radius at the very bottom (foot ring)
BASE_TOP_R = 0.080  # radius of the upper neck deck
BASE_TOP_Z = 0.200  # height of the top coupling deck of the base (local)
BELLY_Z = 0.075  # height of the widest belly point (local)

COUPLING_R = 0.058  # radius of the jar-seating coupling ring on the base
COUPLING_Z = 0.012  # height of that coupling ring above the deck

JAR_BOTTOM_R = 0.052  # glass jar radius at its base (sits in the coupling)
JAR_TOP_R = 0.072  # glass jar radius at the rim (wider, flared)
JAR_BASE_Z = FOOT_LIFT + BASE_TOP_Z + COUPLING_Z  # world z of the jar bottom
JAR_H = 0.190  # glass body height

LID_R = 0.084  # lid skirt radius (covers the flared jar lip, r=0.076)
LID_H = 0.028  # lid body height
LID_SEAT_DROP = 0.010  # how far the skirt drops below the jar rim when seated
LID_LIFT = 0.045  # prismatic lift travel (lid fully clears the jar rim)

CAP_PLUG_R = 0.017  # measuring-cap plug radius (lid hole is r=0.018)
CAP_DISC_R = 0.022
CAP_LUG_TIP = 0.029  # lug tips reach this radius (non-axisymmetric)

DIAL_R = 0.026  # rotary speed knob radius
DIAL_DEPTH = 0.014  # knob boss depth off the panel
DIAL_SWEEP = math.radians(150.0)  # +/- sweep of the speed lever
PANEL_Z = FOOT_LIFT + 0.150  # world z of the panel / dial hub center
DIAL_HUB_Y = -0.100  # world y of the knob seat plane (proud of the panel)


def _base_housing_mesh() -> object:
    """Bulbous tapered base body, revolved from a 2D profile."""
    profile = [
        (BASE_BOTTOM_R, 0.0),
        (BASE_BOTTOM_R + 0.006, 0.006),
        (BASE_BELLY_R - 0.004, 0.040),
        (BASE_BELLY_R, BELLY_Z),  # belly
        (BASE_BELLY_R - 0.006, 0.105),
        (0.099, 0.135),  # shoulder begins to taper in
        (0.090, 0.165),
        (BASE_TOP_R, BASE_TOP_Z),  # top deck rim
    ]
    pts = [(0.0, 0.0)]
    pts.extend(profile)
    pts.append((0.0, BASE_TOP_Z))
    body = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    )
    # Recess the top deck so the jar coupling reads as a seated socket.
    body = body.faces(">Z").workplane().hole(2.0 * COUPLING_R + 0.004, 0.010)
    return body


def _coupling_mesh() -> object:
    """Raised coupling ring on the base top deck that the jar seats into."""
    ring = (
        cq.Workplane("XY")
        .circle(COUPLING_R + 0.005)
        .circle(COUPLING_R - 0.012)
        .extrude(COUPLING_Z)
        .edges(">Z")
        .fillet(0.002)
    )
    return ring


def _trim_ring_mesh() -> object:
    """Thin bright seam ring around the belly (visible in the photo)."""
    return (
        cq.Workplane("XY")
        .circle(BASE_BELLY_R + 0.0008)
        .circle(BASE_BELLY_R - 0.006)
        .extrude(0.003)
    )


def _foot_mesh() -> object:
    """A single soft rubber foot stub."""
    return (
        cq.Workplane("XY")
        .circle(0.012)
        .extrude(0.012)
        .edges("<Z")
        .fillet(0.004)
    )


def _control_panel_mesh() -> object:
    """Graphite control-panel patch facing -Y on the base shoulder."""
    panel = (
        cq.Workplane("XZ")
        .rect(0.072, 0.050)
        .extrude(0.014)
        .edges("|Y")
        .fillet(0.008)
    )
    return panel


def _jar_glass_mesh() -> object:
    """Fluted, hollow glass jar with a flared rim and vertical flute ribs."""
    outer = [
        (JAR_BOTTOM_R, 0.0),
        (JAR_BOTTOM_R + 0.002, 0.006),
        (JAR_BOTTOM_R + 0.012, 0.045),
        (JAR_TOP_R - 0.010, 0.130),
        (JAR_TOP_R, JAR_H - 0.012),
        (JAR_TOP_R + 0.004, JAR_H),  # flared lip
    ]
    outer_pts = [(0.0, 0.0)] + outer + [(0.0, JAR_H)]
    glass = (
        cq.Workplane("XZ")
        .polyline(outer_pts)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    )
    cavity = [
        (0.0, 0.010),
        (JAR_BOTTOM_R - 0.006, 0.014),
        (JAR_BOTTOM_R + 0.006, 0.045),
        (JAR_TOP_R - 0.014, 0.130),
        (JAR_TOP_R - 0.005, JAR_H + 0.002),
        (0.0, JAR_H + 0.002),
    ]
    cav = (
        cq.Workplane("XZ")
        .polyline(cavity)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    )
    glass = glass.cut(cav)

    # Vertical flute ribs: shallow rounded cuts around the lower body.
    n_flutes = 10
    for i in range(n_flutes):
        ang = 2.0 * math.pi * i / n_flutes
        cx = (JAR_TOP_R + 0.004) * math.cos(ang)
        cy = (JAR_TOP_R + 0.004) * math.sin(ang)
        flute = (
            cq.Workplane("XY")
            .center(cx, cy)
            .circle(0.006)
            .extrude(0.135)
            .translate((0.0, 0.0, 0.016))
        )
        glass = glass.cut(flute)
    return glass


def _jar_collar_mesh() -> object:
    """Dark plastic collar/base ring at the bottom of the jar (blade housing)."""
    collar = (
        cq.Workplane("XY")
        .circle(JAR_BOTTOM_R + 0.004)
        .extrude(0.018)
        .edges(">Z")
        .fillet(0.003)
    )
    return collar


def _jar_handle_mesh() -> object:
    """C-shaped side handle swept along a path on the +X side of the jar."""
    z0 = 0.050
    z1 = 0.150
    x_out = JAR_TOP_R + 0.040
    path_pts = [
        (JAR_BOTTOM_R + 0.010, 0.0, z0),
        (x_out - 0.006, 0.0, z0 + 0.010),
        (x_out, 0.0, (z0 + z1) / 2.0),
        (x_out - 0.006, 0.0, z1 - 0.010),
        (JAR_TOP_R - 0.002, 0.0, z1),
    ]
    path = cq.Workplane("XZ").spline([(p[0], p[2]) for p in path_pts])
    handle = (
        cq.Workplane("XY")
        .center(path_pts[0][0], 0.0)
        .workplane(offset=path_pts[0][2])
        .ellipse(0.011, 0.008)
        .sweep(path)
    )
    return handle


def _lid_mesh() -> object:
    """Dark lid: stepped skirt + low dome, hollow underside, center cap hole."""
    skirt = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(0.012)
        .faces(">Z")
        .workplane()
        .circle(LID_R - 0.010)
        .extrude(0.010)
    )
    dome = (
        cq.Workplane("XY")
        .workplane(offset=0.022)
        .circle(LID_R - 0.018)
        .extrude(0.006)
    )
    lid = skirt.union(dome)
    # Hollow underside so the skirt caps the jar opening (jar lip r=0.076
    # nests inside this r=0.078 pocket without touching the wall).
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(LID_R - 0.006)
        .extrude(0.012)
    )
    lid = lid.cut(cavity)
    # Center filler opening for the measuring cap (cap plug r=0.017).
    hole = (
        cq.Workplane("XY")
        .workplane(offset=0.009)
        .circle(0.018)
        .extrude(0.025)
    )
    lid = lid.cut(hole)
    return lid


def _lid_accent_ring_mesh() -> object:
    """Red accent ring on the lid top, around the measuring cap (photo)."""
    return (
        cq.Workplane("XY")
        .circle(0.042)
        .circle(0.032)
        .extrude(0.003)
    )


def _cap_mesh() -> object:
    """Measuring cap: plug + disc + two grip lugs + top ridge bar.

    Authored with the seat plane at z=0 (plug hangs below into the lid hole).
    The lugs and ridge make the cap non-axisymmetric so its twist is visible.
    """
    plug = (
        cq.Workplane("XY")
        .workplane(offset=-0.010)
        .circle(CAP_PLUG_R)
        .extrude(0.010)
    )
    disc = (
        cq.Workplane("XY")
        .circle(CAP_DISC_R)
        .extrude(0.006)
        .edges(">Z")
        .fillet(0.0015)
    )
    cap = plug.union(disc)
    for sx in (-1.0, 1.0):
        lug = (
            cq.Workplane("XY")
            .center(sx * 0.024, 0.0)
            .rect(0.010, 0.012)
            .extrude(0.005)
        )
        cap = cap.union(lug)
    ridge = (
        cq.Workplane("XY")
        .workplane(offset=0.006)
        .rect(0.036, 0.008)
        .extrude(0.004)
    )
    cap = cap.union(ridge)
    return cap


def _dial_mesh() -> object:
    """Lever-style speed knob, authored spinning about local +Z.

    Round boss with a D-flat cut on the local -Y side and a raised lever bar
    across the face, so the silhouette is clearly non-circular.
    """
    knob = (
        cq.Workplane("XY")
        .circle(DIAL_R)
        .extrude(DIAL_DEPTH)
        .edges(">Z")
        .fillet(0.003)
    )
    # D-flat: shave a chord off the local -Y side.
    flat = (
        cq.Workplane("XY")
        .center(0.0, -DIAL_R - 0.001)
        .rect(0.080, 0.014)
        .extrude(DIAL_DEPTH + 0.010)
    )
    knob = knob.cut(flat)
    # Raised lever bar across the face (grip + visual lever).
    bar = (
        cq.Workplane("XY")
        .workplane(offset=DIAL_DEPTH - 0.001)
        .rect(0.011, 0.044)
        .extrude(0.007)
        .edges(">Z")
        .fillet(0.002)
    )
    knob = knob.union(bar)
    return knob


def _pointer_strip_mesh() -> object:
    """Light pointer strip on the outer (local +Y) half of the lever bar."""
    return (
        cq.Workplane("XY")
        .center(0.0, 0.016)
        .rect(0.005, 0.018)
        .extrude(0.0018)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_blender")

    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    panel_graphite = model.material("panel_graphite", rgba=(0.30, 0.30, 0.33, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.05, 0.05, 0.05, 1.0))
    glass_clear = model.material("glass_clear", rgba=(0.85, 0.88, 0.92, 0.25))
    dial_dark = model.material("dial_dark", rgba=(0.17, 0.17, 0.18, 1.0))
    pointer_light = model.material("pointer_light", rgba=(0.88, 0.88, 0.90, 1.0))
    accent_red = model.material("accent_red", rgba=(0.60, 0.08, 0.08, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.72, 0.72, 0.75, 1.0))
    cap_gray = model.material("cap_gray", rgba=(0.24, 0.24, 0.26, 1.0))

    # ------------------------------------------------------------------ base
    base = model.part("base_housing")
    base.visual(
        mesh_from_cadquery(_base_housing_mesh(), "base_body"),
        origin=Origin(xyz=(0.0, 0.0, FOOT_LIFT)),
        material=plastic_black,
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(_coupling_mesh(), "base_coupling"),
        # Embed the coupling ring 3 mm into the top deck recess.
        origin=Origin(xyz=(0.0, 0.0, FOOT_LIFT + BASE_TOP_Z - 0.003)),
        material=plastic_black,
        name="base_coupling",
    )
    base.visual(
        mesh_from_cadquery(_trim_ring_mesh(), "belly_trim"),
        origin=Origin(xyz=(0.0, 0.0, FOOT_LIFT + BELLY_Z - 0.0015)),
        material=trim_silver,
        name="belly_trim",
    )
    base.visual(
        mesh_from_cadquery(_control_panel_mesh(), "control_panel"),
        # Panel face spans world y in [-0.102, -0.088]; embeds into the body.
        origin=Origin(xyz=(0.0, -0.088, PANEL_Z)),
        material=panel_graphite,
        name="control_panel",
    )
    # Four rubber feet under the belly, resting on the ground plane (z=0).
    foot_mesh = mesh_from_cadquery(_foot_mesh(), "foot")
    for i in range(4):
        ang = math.radians(45.0 + i * 90.0)
        fx = 0.062 * math.cos(ang)
        fy = 0.062 * math.sin(ang)
        base.visual(
            foot_mesh,
            origin=Origin(xyz=(fx, fy, 0.0)),
            material=rubber_dark,
            name=f"foot_{i}",
        )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_BELLY_R, length=BASE_TOP_Z),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, FOOT_LIFT + BASE_TOP_Z / 2.0)),
    )

    # ------------------------------------------------------------- speed dial
    dial = model.part("speed_dial")
    dial.visual(
        mesh_from_cadquery(_dial_mesh(), "dial_body"),
        material=dial_dark,
        name="dial_body",
    )
    dial.visual(
        mesh_from_cadquery(_pointer_strip_mesh(), "dial_pointer"),
        origin=Origin(xyz=(0.0, 0.0, DIAL_DEPTH + 0.006)),
        material=pointer_light,
        name="dial_pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=DIAL_R, length=DIAL_DEPTH), mass=0.03
    )

    # Knob authored along +Z; rpy=(+90deg,0,0) maps local +Z -> world -Y so the
    # knob stands proud of the front panel; local +Y (the pointer) -> world +Z.
    model.articulation(
        "dial_spin",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial,
        origin=Origin(
            xyz=(0.0, DIAL_HUB_Y, PANEL_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-DIAL_SWEEP,
            upper=DIAL_SWEEP,
        ),
    )

    # -------------------------------------------------------------- glass jar
    jar = model.part("glass_jar")
    jar.visual(
        mesh_from_cadquery(_jar_collar_mesh(), "jar_collar"),
        material=plastic_black,
        name="jar_collar",
    )
    jar.visual(
        mesh_from_cadquery(_jar_glass_mesh(), "jar_glass"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=glass_clear,
        name="jar_glass",
    )
    jar.visual(
        mesh_from_cadquery(_jar_handle_mesh(), "jar_handle"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=plastic_black,
        name="jar_handle",
    )
    jar.inertial = Inertial.from_geometry(
        Cylinder(radius=JAR_TOP_R, length=JAR_H), mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, JAR_H / 2.0)),
    )

    # Jar seats fixed into the base coupling.
    model.articulation(
        "base_to_jar",
        ArticulationType.FIXED,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, JAR_BASE_Z - 0.004)),
    )

    # ----------------------------------------------------------------- lid
    lid = model.part("jar_lid")
    lid.visual(
        mesh_from_cadquery(_lid_mesh(), "lid_body"),
        material=plastic_black,
        name="lid_body",
    )
    lid.visual(
        mesh_from_cadquery(_lid_accent_ring_mesh(), "lid_accent_ring"),
        origin=Origin(xyz=(0.0, 0.0, LID_H)),
        material=accent_red,
        name="lid_accent_ring",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=LID_R, length=LID_H), mass=0.08
    )

    # The lid LIFTS straight off the jar: PRISMATIC along vertical Z.
    # Jar local rim z = 0.014 (collar offset) + JAR_H; the skirt seats
    # LID_SEAT_DROP below the rim.
    lid_seat_z = 0.014 + JAR_H - LID_SEAT_DROP
    model.articulation(
        "lid_lift",
        ArticulationType.PRISMATIC,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, lid_seat_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.2,
            lower=0.0,
            upper=LID_LIFT,
        ),
    )

    # ------------------------------------------------------- measuring cap
    cap = model.part("lid_cap")
    cap.visual(
        mesh_from_cadquery(_cap_mesh(), "cap_body"),
        material=cap_gray,
        name="cap_body",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_DISC_R, length=0.016), mass=0.02
    )

    # Cap twist-releases in the lid filler opening: REVOLUTE about vertical Z.
    model.articulation(
        "cap_twist",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, LID_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(60.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_housing")
    dial = object_model.get_part("speed_dial")
    jar = object_model.get_part("glass_jar")
    lid = object_model.get_part("jar_lid")
    cap = object_model.get_part("lid_cap")

    dial_spin = object_model.get_articulation("dial_spin")
    base_to_jar = object_model.get_articulation("base_to_jar")
    lid_lift = object_model.get_articulation("lid_lift")
    cap_twist = object_model.get_articulation("cap_twist")

    # --- Intentional seated fits.
    ctx.allow_overlap(
        base,
        dial,
        reason="The speed knob seats into / stands proud of the front panel.",
    )
    ctx.allow_overlap(
        jar,
        lid,
        reason="The lid skirt intentionally nests over and seats on the jar rim.",
    )
    ctx.allow_overlap(
        lid,
        cap,
        reason="The measuring-cap plug seats inside the lid filler opening.",
    )

    # --- Single root: the base housing carries everything.
    ctx.check(
        "base is the single root",
        len(object_model.root_parts()) == 1
        and object_model.root_parts()[0].name == "base_housing",
        details=f"roots={[p.name for p in object_model.root_parts()]}",
    )

    # --- Joint types/axes/limits.
    ctx.check(
        "lid lift joint is prismatic along vertical Z",
        lid_lift.joint_type == "prismatic"
        and tuple(round(a, 3) for a in lid_lift.axis) == (0.0, 0.0, 1.0),
        details=f"type={lid_lift.joint_type}, axis={lid_lift.axis}",
    )
    ctx.check(
        "lid lift travel is generous (>= 1x lid height)",
        lid_lift.motion_limits is not None
        and lid_lift.motion_limits.upper >= LID_H,
        details=f"upper={lid_lift.motion_limits.upper if lid_lift.motion_limits else None}",
    )
    ctx.check(
        "dial joint is revolute with a bounded +/-150 deg sweep",
        dial_spin.joint_type == "revolute"
        and dial_spin.motion_limits is not None
        and abs(dial_spin.motion_limits.lower + DIAL_SWEEP) < 1e-6
        and abs(dial_spin.motion_limits.upper - DIAL_SWEEP) < 1e-6,
        details=f"type={dial_spin.joint_type}, limits=({dial_spin.motion_limits.lower if dial_spin.motion_limits else None}, {dial_spin.motion_limits.upper if dial_spin.motion_limits else None})",
    )
    ctx.check(
        "cap twist joint is revolute about vertical Z",
        cap_twist.joint_type == "revolute"
        and tuple(round(a, 3) for a in cap_twist.axis) == (0.0, 0.0, 1.0),
        details=f"type={cap_twist.joint_type}, axis={cap_twist.axis}",
    )
    ctx.check(
        "jar mounts fixed to base",
        base_to_jar.joint_type == "fixed",
        details=str(base_to_jar.joint_type),
    )

    # --- Layout / silhouette / grounding.
    base_aabb = ctx.part_world_aabb(base)
    jar_aabb = ctx.part_world_aabb(jar)
    lid_aabb = ctx.part_world_aabb(lid)
    cap_aabb = ctx.part_world_aabb(cap)
    assert base_aabb is not None and jar_aabb is not None
    assert lid_aabb is not None and cap_aabb is not None

    ctx.check(
        "model rests on the ground plane (z-min ~ 0)",
        abs(base_aabb[0][2]) < 0.003,
        details=f"base_min_z={base_aabb[0][2]:.4f}",
    )
    ctx.check(
        "jar sits above base top deck",
        jar_aabb[0][2] > FOOT_LIFT + BASE_TOP_Z - 0.02,
        details=f"jar_min_z={jar_aabb[0][2]:.3f}",
    )
    ctx.check(
        "lid is seated at the jar rim",
        lid_aabb[0][2] > jar_aabb[1][2] - 0.02,
        details=f"lid_min_z={lid_aabb[0][2]:.3f}, jar_max_z={jar_aabb[1][2]:.3f}",
    )
    ctx.check(
        "cap rides on top of the lid",
        cap_aabb[1][2] > lid_aabb[1][2],
        details=f"cap_max_z={cap_aabb[1][2]:.3f}, lid_max_z={lid_aabb[1][2]:.3f}",
    )
    ctx.check(
        "base belly is the widest footprint",
        (base_aabb[1][0] - base_aabb[0][0]) > 0.19,
        details=f"base_x_span={(base_aabb[1][0] - base_aabb[0][0]):.3f}",
    )

    # --- Seating contacts.
    ctx.expect_contact(jar, base, contact_tol=0.006, name="jar seats on base coupling")
    ctx.expect_contact(dial, base, contact_tol=0.004, name="dial contacts the front panel")
    ctx.expect_overlap(lid, jar, axes="xy", min_overlap=0.04, name="lid covers jar rim")

    # --- LID LIFT: at full travel the lid (and cap) must clear the jar rim.
    jar_rim_top = jar_aabb[1][2]
    with ctx.pose({lid_lift: LID_LIFT}):
        lifted_lid = ctx.part_world_aabb(lid)
        lifted_cap = ctx.part_world_aabb(cap)
    assert lifted_lid is not None and lifted_cap is not None
    ctx.check(
        "lid lifts fully clear of the jar rim",
        lifted_lid[0][2] > jar_rim_top + 0.005,
        details=f"lifted_lid_min_z={lifted_lid[0][2]:.3f}, jar_rim_top={jar_rim_top:.3f}",
    )
    ctx.check(
        "cap rides up with the lifted lid",
        lifted_cap[0][2] > jar_rim_top + 0.005,
        details=f"lifted_cap_min_z={lifted_cap[0][2]:.3f}, jar_rim_top={jar_rim_top:.3f}",
    )

    # --- DIAL: the light pointer strip is clearly off the spin axis, and FK at
    # both limits swings it to visibly different positions.
    def _centroid(aabb):
        return tuple(0.5 * (aabb[0][i] + aabb[1][i]) for i in range(3))

    rest_ptr = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    assert rest_ptr is not None
    rc = _centroid(rest_ptr)
    # Spin axis is the world -Y line through (0, *, PANEL_Z).
    off_axis = math.hypot(rc[0], rc[2] - PANEL_Z)
    ctx.check(
        "dial pointer is off-axis by at least half the knob radius",
        off_axis >= 0.5 * DIAL_R,
        details=f"off_axis={off_axis:.4f}, half_knob_r={0.5 * DIAL_R:.4f}",
    )
    with ctx.pose({dial_spin: -DIAL_SWEEP}):
        lo_ptr = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({dial_spin: DIAL_SWEEP}):
        hi_ptr = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    assert lo_ptr is not None and hi_ptr is not None
    lo_c, hi_c = _centroid(lo_ptr), _centroid(hi_ptr)
    swing = math.dist(lo_c, hi_c)
    moved_from_rest = min(math.dist(rc, lo_c), math.dist(rc, hi_c))
    ctx.check(
        "dial pointer visibly swings between the two limits",
        swing > 0.01 and moved_from_rest > 0.01,
        details=f"limit_to_limit={swing:.4f}, rest_to_limit={moved_from_rest:.4f}",
    )

    # --- CAP TWIST: the lugged cap is non-axisymmetric, so its world AABB
    # changes measurably when twisted.
    rest_cap = ctx.part_world_aabb(cap)
    with ctx.pose({cap_twist: math.radians(60.0)}):
        turned_cap = ctx.part_world_aabb(cap)
    assert rest_cap is not None and turned_cap is not None
    dx = abs((rest_cap[1][0] - rest_cap[0][0]) - (turned_cap[1][0] - turned_cap[0][0]))
    dy = abs((rest_cap[1][1] - rest_cap[0][1]) - (turned_cap[1][1] - turned_cap[0][1]))
    ctx.check(
        "cap twist visibly rotates the lugged cap (AABB extents change)",
        max(dx, dy) > 0.003,
        details=f"dx_span={dx:.4f}, dy_span={dy:.4f}",
    )
    ctx.check(
        "cap stays seated while twisting (z constant)",
        abs(turned_cap[0][2] - rest_cap[0][2]) < 0.001,
        details=f"rest_z={rest_cap[0][2]:.4f}, turned_z={turned_cap[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
