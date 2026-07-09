from __future__ import annotations

# Retro vintage portable transistor radio (bronze/copper cabinet).
# Frame:
#   - X: left(-) / right(+), cabinet width ~0.19 m, centered at x=0.
#   - Y: depth; front face (grille/dial/knobs) at +Y, rear at -Y, depth ~0.085 m.
#   - Z: up; cabinet height ~0.12 m, centered at z=0.
# Layout (matches the reference photo):
#   - ribbed speaker GRILLE on the LEFT half of the front face
#   - horizontal tuning DIAL window with frequency scale + needle on the RIGHT
#   - two large control KNOBS + one small center knob below the dial (front face)
#   - chunky black folding CARRY HANDLE on top (two pivots)
#   - telescoping metal ANTENNA at the rear-right (extend + base swivel)
# Articulations:
#   - volume / tuning / center knobs: CONTINUOUS spin about each knob axis (+Y),
#     each with an off-axis pointer so the spin is detectable.
#   - carry handle: REVOLUTE swing about its two top pivots (upright -> folded).
#   - antenna: PRISMATIC telescoping extend along its length + REVOLUTE base swivel.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- cabinet master dimensions ----
CAB_W = 0.190  # X
CAB_D = 0.085  # Y
CAB_H = 0.120  # Z
FRONT_Y = CAB_D / 2.0  # +0.0425 front face plane
CORNER_R = 0.018

# Front-face feature centers.
GRILLE_CX = -0.050  # left half
DIAL_CX = 0.035  # right half, shifted as a group with the knobs
DIAL_CZ = 0.018  # dial sits in the upper-right
KNOB_CZ = -0.024  # knobs below the dial, raised as a group

# Knob front-face Y (knob caps stand proud of the front).
KNOB_FACE_Y = FRONT_Y - 0.004


def _rounded_box(w: float, d: float, h: float, r: float) -> cq.Workplane:
    # Rounded rectangular cabinet: box with vertical edges filleted, then the
    # front/back top edges softened for the period rounded-corner look.
    box = (
        cq.Workplane("XY")
        .box(w, d, h)
        .edges("|Z")
        .fillet(r)
    )
    # Soften the top and bottom long edges a touch for a pillowed silhouette.
    box = box.edges("|X and >Z").fillet(0.010)
    box = box.edges("|X and <Z").fillet(0.008)
    return box


def _cabinet_solid() -> cq.Workplane:
    body = _rounded_box(CAB_W, CAB_D, CAB_H, CORNER_R)

    # Recess the front face slightly to seat the grille panel and dial bezel:
    # cut a shallow pocket over the grille region.
    grille_pocket = (
        cq.Workplane("XY")
        .center(GRILLE_CX, FRONT_Y - 0.004)
        .box(0.072, 0.012, 0.082)
    )
    body = body.cut(grille_pocket)

    # Dial window recess on the right.
    dial_pocket = (
        cq.Workplane("XY")
        .workplane(offset=DIAL_CZ)
        .center(DIAL_CX, FRONT_Y - 0.003)
        .box(0.080, 0.010, 0.034)
    )
    body = body.cut(dial_pocket)
    return body


def _grille_mesh():
    # Ribbed / slatted speaker grille: a slotted panel sitting in the front pocket.
    # Panel lies in local XY and extends along Z (slats); we rotate it so the
    # slot face points out along +Y and slats run vertically across the grille.
    # Horizontal ribbed slats (match the reference photo): slots run along panel
    # X, stacked along panel Y. After rotating the panel up to face +Y, panel X
    # stays world X (horizontal) and panel Y maps to world Z, so the ribs stack
    # vertically as horizontal slats.
    panel = SlotPatternPanelGeometry(
        (0.066, 0.078),
        0.006,
        slot_size=(0.052, 0.0055),
        pitch=(0.066, 0.0105),
        frame=0.006,
        corner_radius=0.006,
        slot_angle_deg=0.0,
        center=True,
    )
    # Face the panel toward +Y: rotate the XY panel about X by +90 deg so its
    # thickness (was +Z) now points along +Y.
    panel.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(panel, "grille_panel")


def _dial_face_mesh():
    # Cream dial face plate that sits at the back of the dial recess.
    plate = BoxGeometry((0.078, 0.004, 0.032))
    return mesh_from_geometry(plate, "dial_face")


def _dial_scale_mesh():
    # Frequency scale: thin tick marks + two baseline rails across the dial face.
    g = BoxGeometry((0.072, 0.0015, 0.0016))  # upper rail
    g.translate(0.0, 0.0, 0.010)
    lower = BoxGeometry((0.072, 0.0015, 0.0016))
    lower.translate(0.0, 0.0, -0.010)
    g.merge(lower)
    # Tick marks.
    n = 13
    for i in range(n):
        tx = -0.034 + i * (0.068 / (n - 1))
        tall = 0.010 if i % 3 == 0 else 0.006
        tick = BoxGeometry((0.0012, 0.0015, tall))
        tick.translate(tx, 0.0, 0.0)
        g.merge(tick)
    return mesh_from_geometry(g, "dial_scale")


def _needle_mesh():
    # Vertical red pointer/needle riding across the dial scale.
    n = BoxGeometry((0.0018, 0.0028, 0.026))
    return mesh_from_geometry(n, "needle")


def _bezel_ring_mesh():
    # Dark trim bezel framing the dial window.
    ring = BoxGeometry((0.086, 0.006, 0.040))
    cut = BoxGeometry((0.078, 0.010, 0.032))
    ring = ring  # outer
    # Build a frame by subtracting via cadquery for clean inner opening.
    outer = cq.Workplane("XZ").box(0.086, 0.040, 0.006)
    inner = cq.Workplane("XZ").box(0.078, 0.032, 0.012)
    frame = outer.cut(inner)
    return mesh_from_cadquery(frame, "dial_bezel")


def _knob_mesh(name: str, diameter: float, height: float):
    # Knurled rotary knob aligned to local +Z, with a pointer on the rim so the
    # spin is detectable off-axis.
    knob = KnobGeometry(
        diameter,
        height,
        body_style="skirted",
        top_diameter=diameter * 0.82,
        skirt=KnobSkirt(diameter * 1.08, height * 0.30, flare=0.06, chamfer=0.0010),
        grip=KnobGrip(style="knurled", count=28, depth=0.0010),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,  # mounting face on z=0
    )
    return mesh_from_geometry(knob, name)


def _knob_marker_mesh(diameter: float, height: float):
    # Off-axis marker peg so rotation is unambiguous in tests/visuals. Spans from
    # inside the cap body up past the top so it stays a connected island and
    # reads as a raised pointer/dot near the rim.
    peg = BoxGeometry((0.0028, 0.0028, height * 1.1))
    peg.translate(diameter * 0.16, 0.0, height * 0.55)
    return mesh_from_geometry(peg, "knob_marker")


def _handle_mesh():
    # Chunky black folding carry handle: an arched bar swept as a rounded square
    # tube following a low arch, with two short feet at the ends that carry the
    # pivots. Modeled in the handle-local frame (pivot axis along X at z=0).
    # The arch spans in X and rises in +Z.
    half = 0.058  # half span between pivots in X
    pts = [
        (-half, 0.0, 0.004),
        (-half + 0.012, 0.0, 0.028),
        (-half + 0.030, 0.0, 0.046),
        (0.0, 0.0, 0.052),
        (half - 0.030, 0.0, 0.046),
        (half - 0.012, 0.0, 0.028),
        (half, 0.0, 0.004),
    ]
    path = cq.Workplane("XZ").spline([(p[0], p[2]) for p in pts])
    # Sweep a rounded rectangular profile along the arch.
    prof = (
        cq.Workplane("XY")
        .workplane(offset=-half)  # start at the left end plane
    )
    # Build via sweep: profile on a plane normal to the path start.
    grip = (
        cq.Workplane("YZ")
        .workplane(offset=-half)
        .rect(0.020, 0.016)
        .sweep(path)
    )
    try:
        grip = grip.edges("|Y").fillet(0.004)
    except Exception:
        pass
    # Two pivot feet (knuckles) at the ends, cylinders about the X pivot axis.
    foot_l = (
        cq.Workplane("YZ")
        .workplane(offset=-half - 0.006)
        .circle(0.012)
        .extrude(0.012)
    )
    foot_r = (
        cq.Workplane("YZ")
        .workplane(offset=half - 0.006)
        .circle(0.012)
        .extrude(0.012)
    )
    handle = grip.union(foot_l).union(foot_r)
    return mesh_from_cadquery(handle, "carry_handle")


def _handle_pivot_mount_mesh(side_x: float):
    # Raised black pivot boss on the cabinet top that the handle foot sits in.
    boss = CylinderGeometry(0.013, 0.014, radial_segments=24).rotate_y(math.pi / 2.0)
    boss.translate(side_x, 0.0, 0.0)
    return mesh_from_geometry(boss, f"handle_boss_{'l' if side_x < 0 else 'r'}")


def _antenna_base_mesh():
    # Swivel base block + socket for the antenna at the rear-right of the top.
    block = BoxGeometry((0.018, 0.018, 0.014))
    socket = CylinderGeometry(0.006, 0.016)
    socket.translate(0.0, 0.0, 0.012)
    block.merge(socket)
    return mesh_from_geometry(block, "antenna_base")


def _antenna_lower_mesh():
    # Lower (thicker) telescoping tube segment; local axis along +Z.
    g = CylinderGeometry(0.0050, 0.130, radial_segments=20)
    g.translate(0.0, 0.0, 0.065)
    return mesh_from_geometry(g, "antenna_lower")


def _antenna_upper_mesh():
    # Upper (thinner) telescoping segment + ball tip; local axis along +Z.
    g = CylinderGeometry(0.0034, 0.150, radial_segments=18)
    g.translate(0.0, 0.0, 0.075)
    tip = CylinderGeometry(0.0050, 0.0050, radial_segments=16)
    tip.translate(0.0, 0.0, 0.150)
    g.merge(tip)
    return mesh_from_geometry(g, "antenna_upper")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="transistor_radio")

    bronze = model.material("bronze", rgba=(0.62, 0.40, 0.16, 1.0))
    dark_brown = model.material("dark_brown", rgba=(0.20, 0.12, 0.07, 1.0))
    black = model.material("black_plastic", rgba=(0.07, 0.07, 0.08, 1.0))
    metal = model.material("chrome_metal", rgba=(0.78, 0.79, 0.82, 1.0))
    cream = model.material("dial_cream", rgba=(0.90, 0.86, 0.74, 1.0))
    red = model.material("needle_red", rgba=(0.80, 0.12, 0.10, 1.0))

    # ---- cabinet (root) ----
    body = model.part("cabinet")
    body.visual(
        mesh_from_cadquery(_cabinet_solid(), "cabinet_shell"),
        material=bronze,
        name="cabinet_shell",
    )

    # Dark brown trim band framing the front face (thin proud border).
    trim = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.0, FRONT_Y - 0.0015)
        .box(CAB_W - 0.006, 0.006, CAB_H - 0.006)
    )
    inner_cut = (
        cq.Workplane("XY")
        .center(0.0, FRONT_Y)
        .box(CAB_W - 0.030, 0.020, CAB_H - 0.026)
    )
    trim = trim.cut(inner_cut)
    body.visual(mesh_from_cadquery(trim, "front_trim"), material=dark_brown, name="front_trim")

    # Grille panel seated in the left front pocket.
    body.visual(
        _grille_mesh(),
        origin=Origin(xyz=(GRILLE_CX, FRONT_Y - 0.006, 0.0)),
        material=dark_brown,
        name="grille_panel",
    )

    # Dial assembly (static face/scale/bezel; the needle is its own moving part is
    # overkill — keep needle fixed as part of the dial face here).
    body.visual(
        _dial_face_mesh(),
        origin=Origin(xyz=(DIAL_CX, FRONT_Y - 0.006, DIAL_CZ)),
        material=cream,
        name="dial_face",
    )
    body.visual(
        _dial_scale_mesh(),
        origin=Origin(xyz=(DIAL_CX, FRONT_Y - 0.0035, DIAL_CZ)),
        material=dark_brown,
        name="dial_scale",
    )
    body.visual(
        _needle_mesh(),
        origin=Origin(xyz=(DIAL_CX + 0.012, FRONT_Y - 0.0028, DIAL_CZ)),
        material=red,
        name="needle",
    )
    body.visual(
        _bezel_ring_mesh(),
        origin=Origin(xyz=(DIAL_CX, FRONT_Y - 0.004, DIAL_CZ)),
        material=dark_brown,
        name="dial_bezel",
    )

    # Handle pivot bosses on the cabinet top.
    body.visual(
        _handle_pivot_mount_mesh(-0.058),
        origin=Origin(xyz=(0.0, 0.0, CAB_H / 2.0 - 0.002)),
        material=black,
        name="handle_boss_l",
    )
    body.visual(
        _handle_pivot_mount_mesh(0.058),
        origin=Origin(xyz=(0.0, 0.0, CAB_H / 2.0 - 0.002)),
        material=black,
        name="handle_boss_r",
    )

    # Antenna swivel base mounted at the rear-right of the cabinet top.
    ANT_BASE = (0.072, -0.026, CAB_H / 2.0 + 0.004)
    body.visual(
        _antenna_base_mesh(),
        origin=Origin(xyz=ANT_BASE),
        material=black,
        name="antenna_base",
    )

    body.inertial = Inertial.from_geometry(
        Box((CAB_W, CAB_D, CAB_H)), mass=0.85, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # ---- three rotary knobs on the front face, below the dial ----
    # (volume left-large, tuning right-large, small center knob)
    knob_specs = [
        ("volume_knob", DIAL_CX - 0.026, KNOB_CZ - 0.004, 0.030, 0.018),
        ("tuning_knob", DIAL_CX + 0.026, KNOB_CZ - 0.004, 0.030, 0.018),
        ("center_knob", DIAL_CX, KNOB_CZ + 0.006, 0.016, 0.012),
    ]
    for kname, kx, kz, kdia, kh in knob_specs:
        knob = model.part(kname)
        # Knob local +Z is the spin axis; we mount it pointing along +Y.
        knob.visual(
            _knob_mesh(f"{kname}_cap", kdia, kh),
            material=black,
            name=f"{kname}_cap",
        )
        knob.visual(
            _knob_marker_mesh(kdia, kh),
            material=metal,
            name="knob_marker",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(kdia / 2.0, kh), mass=0.02,
            origin=Origin(xyz=(0.0, 0.0, kh / 2.0)),
        )
        # Articulation frame: rotate so child local +Z points to world +Y, then
        # the spin axis is +Z (which maps to +Y / out the front face).
        model.articulation(
            f"cabinet_to_{kname}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(kx, KNOB_FACE_Y, kz), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.4, velocity=8.0),
        )

    # ---- carry handle: revolute swing about the two top pivots (axis along X) ----
    handle = model.part("carry_handle")
    handle.visual(_handle_mesh(), material=black, name="carry_handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.130, 0.020, 0.052)), mass=0.06, origin=Origin(xyz=(0.0, 0.0, 0.026))
    )
    # Pivot located on the cabinet top centerline, between the two bosses.
    model.articulation(
        "cabinet_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, CAB_H / 2.0 - 0.002)),
        axis=(1.0, 0.0, 0.0),
        # Upright at 0; folds down toward the sides up to ~90 deg.
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )

    # ---- antenna: base swivel (revolute) + telescoping extend (prismatic) ----
    ANT_PIVOT = (ANT_BASE[0], ANT_BASE[1], ANT_BASE[2] + 0.012)

    antenna_lower = model.part("antenna_lower")
    antenna_lower.visual(_antenna_lower_mesh(), material=metal, name="antenna_lower")
    antenna_lower.inertial = Inertial.from_geometry(
        Cylinder(0.0050, 0.130), mass=0.012, origin=Origin(xyz=(0.0, 0.0, 0.065))
    )
    # Swivel about the base: tilt the antenna mast. Axis along X so it can rake
    # forward/back from vertical.
    model.articulation(
        "base_to_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child=antenna_lower,
        origin=Origin(xyz=ANT_PIVOT),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-1.2, upper=1.2),
    )

    antenna_upper = model.part("antenna_upper")
    antenna_upper.visual(_antenna_upper_mesh(), material=metal, name="antenna_upper")
    antenna_upper.inertial = Inertial.from_geometry(
        Cylinder(0.0034, 0.150), mass=0.006, origin=Origin(xyz=(0.0, 0.0, 0.075))
    )
    # Telescoping extend along the mast (+Z of the lower segment frame).
    model.articulation(
        "antenna_telescope",
        ArticulationType.PRISMATIC,
        parent=antenna_lower,
        child=antenna_upper,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.5, lower=0.0, upper=0.090),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("cabinet")
    volume = object_model.get_part("volume_knob")
    tuning = object_model.get_part("tuning_knob")
    center = object_model.get_part("center_knob")
    handle = object_model.get_part("carry_handle")
    ant_lower = object_model.get_part("antenna_lower")
    ant_upper = object_model.get_part("antenna_upper")

    vol_joint = object_model.get_articulation("cabinet_to_volume_knob")
    handle_joint = object_model.get_articulation("cabinet_to_handle")
    swivel_joint = object_model.get_articulation("base_to_antenna")
    tele_joint = object_model.get_articulation("antenna_telescope")

    # ---- layout: grille on the LEFT, dial on the RIGHT ----
    grille_aabb = ctx.part_element_world_aabb(body, elem="grille_panel")
    dial_aabb = ctx.part_element_world_aabb(body, elem="dial_face")
    grille_cx = 0.5 * (grille_aabb[0][0] + grille_aabb[1][0])
    dial_cx = 0.5 * (dial_aabb[0][0] + dial_aabb[1][0])
    ctx.check(
        "speaker grille is on the left, dial on the right",
        grille_cx < -0.01 and dial_cx > 0.01,
        details=f"grille_cx={grille_cx}, dial_cx={dial_cx}",
    )
    # Both face features sit on the front (+Y).
    ctx.check(
        "grille and dial are on the front face",
        grille_aabb[1][1] > 0.02 and dial_aabb[1][1] > 0.02,
        details=f"grille_ymax={grille_aabb[1][1]}, dial_ymax={dial_aabb[1][1]}",
    )

    # ---- knobs sit proud of the front face and below the dial ----
    for knob in (volume, tuning, center):
        kpos = ctx.part_world_position(knob)
        ctx.check(
            f"{knob.name} mounted on the front face below the dial",
            kpos is not None and kpos[1] > 0.02 and kpos[2] < 0.0,
            details=f"{knob.name} pos={kpos}",
        )
        ctx.expect_contact(knob, body, name=f"{knob.name} seated on cabinet")

    # ---- each knob spins about its axis: the off-axis marker moves ----
    def marker_pos(part):
        a = ctx.part_element_world_aabb(part, elem="knob_marker")
        return (0.5 * (a[0][0] + a[1][0]), 0.5 * (a[0][2] + a[1][2]))

    rest_mx, rest_mz = marker_pos(volume)
    with ctx.pose({vol_joint: math.pi / 2.0}):
        spin_mx, spin_mz = marker_pos(volume)
    ctx.check(
        "volume knob spin moves the off-axis marker",
        abs(spin_mx - rest_mx) > 0.004 or abs(spin_mz - rest_mz) > 0.004,
        details=f"rest=({rest_mx},{rest_mz}) spun=({spin_mx},{spin_mz})",
    )

    # ---- carry handle folds down about the top pivots ----
    rest_h = _ext(ctx.part_world_aabb(handle))
    rest_top = ctx.part_world_aabb(handle)[1][2]
    with ctx.pose({handle_joint: math.pi / 2.0}):
        folded_top = ctx.part_world_aabb(handle)[1][2]
        folded_h = _ext(ctx.part_world_aabb(handle))
    ctx.check(
        "carry handle folds down (top drops, footprint spreads sideways)",
        folded_top < rest_top - 0.02 and folded_h[1] > rest_h[1] + 0.01,
        details=f"rest_top={rest_top}, folded_top={folded_top}, "
        f"rest_y={rest_h[1]}, folded_y={folded_h[1]}",
    )
    ctx.expect_contact(handle, body, name="handle pivots rest on cabinet top")

    # ---- antenna telescoping extend: upper segment grows in length/height ----
    ctx.expect_within(
        ant_upper,
        ant_lower,
        axes="xy",
        name="antenna upper stays centered in the lower mast",
    )
    ctx.expect_overlap(
        ant_upper,
        ant_lower,
        axes="z",
        min_overlap=0.08,
        name="collapsed antenna stays inserted in the mast",
    )
    rest_uppos = ctx.part_world_position(ant_upper)
    rest_tip = ctx.part_world_aabb(ant_upper)[1][2]
    with ctx.pose({tele_joint: 0.090}):
        ext_uppos = ctx.part_world_position(ant_upper)
        ext_tip = ctx.part_world_aabb(ant_upper)[1][2]
        ctx.expect_within(
            ant_upper,
            ant_lower,
            axes="xy",
            name="extended antenna stays centered in the mast",
        )
        ctx.expect_overlap(
            ant_upper,
            ant_lower,
            axes="z",
            min_overlap=0.01,
            name="extended antenna retains insertion in the mast",
        )
    ctx.check(
        "antenna telescopes upward (tip rises with extension)",
        ext_tip > rest_tip + 0.05 and ext_uppos[2] > rest_uppos[2] + 0.05,
        details=f"rest_tip={rest_tip}, ext_tip={ext_tip}, "
        f"rest_z={rest_uppos[2]}, ext_z={ext_uppos[2]}",
    )

    # ---- antenna swivels at its base ----
    rest_swivel = ctx.part_world_aabb(ant_lower)
    rest_topz = rest_swivel[1][2]
    rest_yspan = rest_swivel[1][1] - rest_swivel[0][1]
    with ctx.pose({swivel_joint: 1.0}):
        swung = ctx.part_world_aabb(ant_lower)
        swung_topz = swung[1][2]
        swung_yspan = swung[1][1] - swung[0][1]
    ctx.check(
        "antenna swivels at its base (rakes off vertical)",
        swung_topz < rest_topz - 0.01 and swung_yspan > rest_yspan + 0.02,
        details=f"rest_topz={rest_topz}, swung_topz={swung_topz}, "
        f"rest_yspan={rest_yspan}, swung_yspan={swung_yspan}",
    )

    # ---- intentional seated overlaps ----
    # Knob skirts/caps intentionally press into the front face + trim band
    # (seated controls).
    for knob in (volume, tuning, center):
        ctx.allow_overlap(
            knob,
            body,
            elem_a=f"{knob.name}_cap",
            elem_b="cabinet_shell",
            reason="Knob skirt is intentionally seated against the cabinet front face.",
        )
        ctx.allow_overlap(
            knob,
            body,
            elem_a=f"{knob.name}_cap",
            elem_b="front_trim",
            reason="Knob skirt is intentionally seated against the front trim band.",
        )
    # Handle feet/knuckles nest over the pivot bosses and into the cabinet top.
    ctx.allow_overlap(
        handle,
        body,
        elem_a="carry_handle",
        elem_b="cabinet_shell",
        reason="Handle pivot feet/knuckles intentionally seat down into the cabinet top.",
    )
    ctx.allow_overlap(
        handle,
        body,
        elem_a="carry_handle",
        elem_b="handle_boss_l",
        reason="Handle pivot knuckle is intentionally seated over the left pivot boss.",
    )
    ctx.allow_overlap(
        handle,
        body,
        elem_a="carry_handle",
        elem_b="handle_boss_r",
        reason="Handle pivot knuckle is intentionally seated over the right pivot boss.",
    )
    # Antenna lower base intentionally enters its socket on the cabinet base.
    ctx.allow_overlap(
        ant_lower,
        body,
        elem_a="antenna_lower",
        elem_b="antenna_base",
        reason="Antenna mast base is intentionally seated inside the swivel socket.",
    )
    # Telescoping upper member nests inside the lower mast proxy.
    ctx.allow_overlap(
        ant_upper,
        ant_lower,
        elem_a="antenna_upper",
        elem_b="antenna_lower",
        reason="Upper antenna segment is intentionally represented sliding inside the lower mast.",
    )

    return ctx.report()


object_model = build_object_model()
