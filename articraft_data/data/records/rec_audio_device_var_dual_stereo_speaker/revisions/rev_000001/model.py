from __future__ import annotations

# Silver portable CD RADIO BOOMBOX (rounded-square slab body, flat front).
#
# Frame:
#   +X = width (right +X, left -X); body ~0.30 m wide, centered on x=0.
#   +Y = depth; the FRONT face is at -Y, the rear (antenna) at +Y.
#   +Z = up; the body sits on the ground with its flat foot at z=0.
#
# Construction:
#   - Body: a rounded-square slab — a box with generously filleted vertical
#     corners and softened top/bottom edges. The FRONT face is a true plane so
#     the speaker and the transport keys mount flush; one watertight CadQuery
#     solid with a real cylindrical speaker bore, a recessed transport tray and
#     a circular CD well cut in.
#   - Dual speakers: perforated dark grille disc + surround ring + bezel ring +
#     dust cap recessed into a bore on each side of the front face (static body
#     detail), symmetric about center.
#   - CD lid: a smoke-translucent low oval dome on the top-left, hinged at the
#     rear on an X-axis REVOLUTE; a silvery CD disc + hub sit under it.
#   - Control deck (top-right): two blue domed knobs (CONTINUOUS), a green LCD,
#     and two small function buttons (PRISMATIC), all surface-mounted facing up.
#   - Transport buttons: a row of five cream playback keys (PRISMATIC) seated in
#     a dark recessed tray centered on the flat front face between the speakers.
#   - Antenna (rear-right): a swivel knuckle + lower mast (REVOLUTE) and a thin
#     telescoping upper rod with a ball tip (PRISMATIC); shown extended at rest.
#   - Four dark rubber feet pads under the base.
#
# Primary articulations: cd_lid hinge, antenna swivel + telescope, the two rotary
# knobs, the five transport keys, and two function keys.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    place_on_surface,
)

# ---- body dimensions ----
BODY_W = 0.300  # x
BODY_D = 0.210  # y
BODY_H = 0.150  # z (flat top deck plane)
Z_TOP = BODY_H
CORNER_R = 0.032  # vertical corner fillet -> soft rounded-square footprint
TOP_R = 0.013  # softened top edge
BOT_R = 0.005  # softened bottom edge
FRONT_Y = -BODY_D / 2.0  # the flat front plane

# ---- dual stereo speakers (symmetric, one per side) ----
SPK_X_OFFSET = 0.088  # ± from center
SPK_Z = 0.075
SPK_BORE_R = 0.048
SPK_BORE_DEPTH = 0.020

# ---- transport tray (front-center recessed key panel, between speakers) ----
TRAY_W = 0.076
TRAY_H = 0.028
TRAY_DEPTH = 0.004
TRAY_CX = 0.0
TRAY_CZ = 0.056
TRAY_FLOOR_Y = FRONT_Y + TRAY_DEPTH

# ---- CD dome (top-left) ----
DOME_CX = -0.040
DOME_RX = 0.064
DOME_RY = 0.062
DOME_RZ = 0.040
DOME_REAR_Y = 0.060  # hinge line (rear edge of dome footprint)
CD_WELL_R = 0.055
CD_WELL_DEPTH = 0.013

FRONT_N = (0.0, -1.0, 0.0)  # outward normal of the flat front face


def _rpy_from_z(n: tuple[float, float, float], up=(0.0, 0.0, 1.0)) -> tuple[float, float, float]:
    """Roll/pitch/yaw (URDF ZYX) for a frame whose local +Z points along n."""
    nx, ny, nz = n
    m = math.sqrt(nx * nx + ny * ny + nz * nz)
    nx, ny, nz = nx / m, ny / m, nz / m
    ux, uy, uz = up
    if abs(ux * nx + uy * ny + uz * nz) > 0.95:
        ux, uy, uz = 1.0, 0.0, 0.0
    # u = normalize(up x n)
    ax, ay, az = uy * nz - uz * ny, uz * nx - ux * nz, ux * ny - uy * nx
    am = math.sqrt(ax * ax + ay * ay + az * az)
    ux2, uy2, uz2 = ax / am, ay / am, az / am
    # v = n x u
    vx, vy, vz = ny * uz2 - nz * uy2, nz * ux2 - nx * uz2, nx * uy2 - ny * ux2
    # R columns = [u, v, n]
    r00, r10, r20 = ux2, uy2, uz2
    r21, r22 = vz, nz
    pitch = math.atan2(-r20, math.hypot(r00, r10))
    yaw = math.atan2(r10, r00)
    roll = math.atan2(r21, r22)
    return (roll, pitch, yaw)


def _body_solid() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(CORNER_R)
        .edges(">Z")
        .fillet(TOP_R)
        .edges("<Z")
        .fillet(BOT_R)
    )
    # recessed cylindrical bores for the dual stereo speakers, one per side
    for sx in (-SPK_X_OFFSET, SPK_X_OFFSET):
        spk_plane = cq.Plane(origin=(sx, FRONT_Y - 0.012, SPK_Z), normal=FRONT_N)
        body = body.cut(
            cq.Workplane(spk_plane).circle(SPK_BORE_R).extrude(-(SPK_BORE_DEPTH + 0.012))
        )
    # shallow rectangular tray for the transport keys (front-center, between speakers)
    tray = (
        cq.Workplane("XZ")
        .workplane(offset=-(FRONT_Y - 0.010))
        .center(TRAY_CX, TRAY_CZ)
        .rect(TRAY_W, TRAY_H)
        .extrude(-(TRAY_DEPTH + 0.010))
    )
    body = body.cut(tray)
    # shallow circular CD well in the top deck, under the dome
    well = (
        cq.Workplane("XY")
        .workplane(offset=Z_TOP - CD_WELL_DEPTH)
        .center(DOME_CX, 0.0)
        .circle(CD_WELL_R)
        .extrude(CD_WELL_DEPTH + 0.01)
    )
    body = body.cut(well)
    return body


def _front_origin(x: float, z: float, off: float) -> Origin:
    """Frame on the flat front face; local +Z points out of the face (-Y). off>0 is proud."""
    return Origin(xyz=(x, FRONT_Y - off, z), rpy=_rpy_from_z(FRONT_N))


def _speaker_origin(sx: float, off: float) -> Origin:
    return _front_origin(sx, SPK_Z, off)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cd_radio_boombox")

    silver = model.material("silver", rgba=(0.80, 0.82, 0.85, 1.0))
    dark = model.material("dark_gray", rgba=(0.13, 0.14, 0.16, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    smoke = model.material("dome_smoke", rgba=(0.10, 0.13, 0.22, 0.45))
    blue = model.material("accent_blue", rgba=(0.18, 0.42, 0.88, 1.0))
    lcd_green = model.material("lcd_green", rgba=(0.35, 0.95, 0.50, 1.0))
    lcd_black = model.material("lcd_black", rgba=(0.05, 0.06, 0.07, 1.0))
    chrome = model.material("chrome", rgba=(0.74, 0.76, 0.80, 1.0))
    cream = model.material("button_cream", rgba=(0.87, 0.87, 0.85, 1.0))
    logo_blue = model.material("logo_blue", rgba=(0.13, 0.32, 0.72, 1.0))
    cd_silver = model.material("cd_silver", rgba=(0.86, 0.87, 0.90, 1.0))
    rubber = model.material("foot_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    # =====================================================================
    # BODY (root)
    # =====================================================================
    body = model.part("body")
    shell = body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell", tolerance=0.00045, angular_tolerance=0.2),
        material=silver,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)), mass=2.3, origin=Origin(xyz=(0.0, 0.0, 0.07))
    )

    # ---- rubber feet pads under the base ----
    for i, (fx, fy) in enumerate(((-0.105, -0.062), (0.105, -0.062), (-0.105, 0.062), (0.105, 0.062))):
        foot = CylinderGeometry(0.013, 0.006, radial_segments=28)
        body.visual(mesh_from_geometry(foot, f"foot_{i}"), origin=Origin(xyz=(fx, fy, 0.001)),
                    material=rubber, name=f"foot_{i}")

    # ---- dual stereo speakers (one per side): basket + grille + surround + bezel + cap ----
    # Each speaker basket sits deep in its bore, fusing the grille assembly to the body floor.
    for i, sx in enumerate((-SPK_X_OFFSET, SPK_X_OFFSET)):
        can = CylinderGeometry(0.041, 0.022, radial_segments=40)
        body.visual(mesh_from_geometry(can, f"speaker_basket_{i}"),
                    origin=_speaker_origin(sx, -0.016),
                    material=grille_dark, name=f"speaker_basket_{i}")
        grille = PerforatedPanelGeometry(
            (0.090, 0.090),
            0.004,
            hole_diameter=0.0050,
            pitch=(0.0090, 0.0090),
            frame=0.003,
            stagger=True,
        )
        body.visual(mesh_from_geometry(grille, f"speaker_grille_{i}"),
                    origin=_speaker_origin(sx, -0.006),
                    material=grille_dark, name=f"speaker_grille_{i}")
        # inner surround ring (cone detail visible through the grille plane)
        surround = TorusGeometry(radius=0.030, tube=0.004, radial_segments=14, tubular_segments=48)
        body.visual(mesh_from_geometry(surround, f"speaker_surround_{i}"),
                    origin=_speaker_origin(sx, -0.004),
                    material=dark, name=f"speaker_surround_{i}")
        ring = TorusGeometry(radius=SPK_BORE_R, tube=0.005, radial_segments=18, tubular_segments=72)
        body.visual(mesh_from_geometry(ring, f"speaker_bezel_{i}"),
                    origin=_speaker_origin(sx, -0.002),
                    material=dark, name=f"speaker_bezel_{i}")
        cap = DomeGeometry(0.013, radial_segments=24, height_segments=10)
        cap.scale(1.0, 1.0, 0.6)
        body.visual(mesh_from_geometry(cap, f"speaker_cap_{i}"),
                    origin=_speaker_origin(sx, -0.006),
                    material=dark, name=f"speaker_cap_{i}")

    # ---- dark tray plate lining the recessed transport-key tray ----
    tray_plate = BoxGeometry((TRAY_W - 0.002, 0.003, TRAY_H - 0.002))
    body.visual(
        mesh_from_geometry(tray_plate, "transport_tray"),
        origin=Origin(xyz=(TRAY_CX, TRAY_FLOOR_Y - 0.0005, TRAY_CZ)),
        material=dark,
        name="transport_tray",
    )

    # ---- CD disc + hub seated in the well under the dome (visible through the smoke lid) ----
    well_floor = Z_TOP - CD_WELL_DEPTH
    disc = CylinderGeometry(0.052, 0.003, radial_segments=48)
    body.visual(mesh_from_geometry(disc, "cd_disc"), origin=Origin(xyz=(DOME_CX, 0.0, well_floor + 0.0015)),
                material=cd_silver, name="cd_disc")
    hub = CylinderGeometry(0.012, 0.008, radial_segments=24)
    body.visual(mesh_from_geometry(hub, "cd_hub"), origin=Origin(xyz=(DOME_CX, 0.0, well_floor + 0.005)),
                material=dark, name="cd_hub")
    # dome seam ring (where the lid rim seats)
    seam = TorusGeometry(radius=DOME_RX + 0.003, tube=0.0045, radial_segments=14, tubular_segments=72)
    seam_geo = mesh_from_geometry(seam, "dome_seam")
    body.visual(seam_geo, origin=Origin(xyz=(DOME_CX, 0.0, Z_TOP)), material=dark, name="dome_seam")

    # ---- ROBERTS logo plate, centered on the flat front face below the transport tray ----
    body.visual(
        mesh_from_geometry(BoxGeometry((0.050, 0.012, 0.005)), "roberts_logo"),
        origin=_front_origin(0.0, 0.026, -0.0012),
        material=logo_blue,
        name="roberts_logo",
    )

    # ---- antenna mounting boss on the rear-right of the top deck ----
    boss_x, boss_y = 0.064, 0.052
    boss = CylinderGeometry(0.013, 0.018, radial_segments=28)
    body.visual(mesh_from_geometry(boss, "antenna_boss"), origin=Origin(xyz=(boss_x, boss_y, Z_TOP + 0.004)),
                material=dark, name="antenna_boss")
    BOSS_TOP_Z = Z_TOP + 0.013

    # =====================================================================
    # CD LID (smoke dome) -- REVOLUTE about the rear hinge (X axis)
    # =====================================================================
    lid = model.part("cd_lid")
    # Smoked dome cover as an open-bottomed shell (so the CD disc in the well below shows
    # through it). Squashed slightly oval and shifted forward of the rear pivot.
    dome = DomeGeometry(1.0, radial_segments=56, height_segments=26, closed=False)
    dome.scale(DOME_RX, DOME_RY, DOME_RZ)
    dome.translate(0.0, -DOME_REAR_Y, 0.0)  # forward of the rear pivot
    lid.visual(mesh_from_geometry(dome, "lid_dome"), material=smoke, name="lid_dome")
    barrel = CylinderGeometry(0.005, 2.0 * DOME_RX * 0.92, radial_segments=20).rotate_y(math.pi / 2.0)
    lid.visual(mesh_from_geometry(barrel, "lid_hinge"), material=dark, name="lid_hinge")
    lid.inertial = Inertial.from_geometry(
        Box((2 * DOME_RX, 2 * DOME_RY, DOME_RZ)), mass=0.16, origin=Origin(xyz=(0.0, -DOME_REAR_Y, 0.012))
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(DOME_CX, DOME_REAR_Y, Z_TOP)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=math.radians(72.0)),
    )

    # =====================================================================
    # TOP DECK CONTROLS -- knobs (CONTINUOUS) + function keys (PRISMATIC)
    # =====================================================================
    def _knob(name: str, x: float, y: float) -> None:
        knob = model.part(name)
        kg = KnobGeometry(
            0.028,
            0.020,
            body_style="domed",
            grip=KnobGrip(style="fluted", count=20, depth=0.0010),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        )
        knob.visual(mesh_from_geometry(kg, f"{name}_cap"), material=blue, name=f"{name}_cap")
        # off-axis marker so the spin is detectable
        marker = BoxGeometry((0.004, 0.013, 0.006))
        marker.translate(0.0, 0.009, 0.019)
        knob.visual(mesh_from_geometry(marker, f"{name}_marker"), material=chrome, name=f"{name}_marker")
        knob.inertial = Inertial.from_geometry(Cylinder(radius=0.014, length=0.02), mass=0.012,
                                               origin=Origin(xyz=(0.0, 0.0, 0.01)))
        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=place_on_surface(knob, shell, point_hint=(x, y, 0.168), child_axis="+z"),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=6.0),
        )

    _knob("volume_knob", 0.043, 0.026)
    _knob("tuning_knob", 0.043, -0.028)

    def _deck_button(name: str, x: float, y: float, size: tuple[float, float]) -> None:
        btn = model.part(name)
        cap_geo = BoxGeometry((size[0], size[1], 0.006))
        cap_geo.translate(0.0, 0.0, 0.003)
        btn.visual(mesh_from_geometry(cap_geo, f"{name}_cap"), material=cream, name=f"{name}_cap")
        btn.inertial = Inertial.from_geometry(Box((size[0], size[1], 0.006)), mass=0.003,
                                              origin=Origin(xyz=(0.0, 0.0, 0.003)))
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=btn,
            origin=place_on_surface(btn, shell, point_hint=(x, y, 0.168), child_axis="+z"),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.0016),
        )

    _deck_button("function_button_0", 0.086, -0.034, (0.015, 0.009))
    _deck_button("function_button_1", 0.066, -0.052, (0.015, 0.009))

    # ---- LCD display (FIXED, surface-mounted on the deck) ----
    lcd = model.part("lcd_display")
    bez = BoxGeometry((0.042, 0.026, 0.008))
    bez.translate(0.0, 0.0, 0.004)
    lcd.visual(mesh_from_geometry(bez, "lcd_bezel"), material=lcd_black, name="lcd_bezel")
    face = BoxGeometry((0.034, 0.018, 0.003))
    face.translate(0.0, 0.0, 0.0085)
    lcd.visual(mesh_from_geometry(face, "lcd_face"), material=lcd_green, name="lcd_face")
    lcd.inertial = Inertial.from_geometry(Box((0.042, 0.026, 0.009)), mass=0.02,
                                          origin=Origin(xyz=(0.0, 0.0, 0.004)))
    model.articulation(
        "lcd_mount",
        ArticulationType.FIXED,
        parent=body,
        child=lcd,
        origin=place_on_surface(lcd, shell, point_hint=(0.086, 0.010, 0.168), child_axis="+z"),
    )

    # =====================================================================
    # TRANSPORT BUTTONS -- five cream keys seated in the front tray
    # =====================================================================
    transport_x = [-0.030, -0.015, 0.0, 0.015, 0.030]
    transport_z = TRAY_CZ
    for i, bx in enumerate(transport_x):
        name = f"transport_button_{i}"
        btn = model.part(name)
        cap_geo = BoxGeometry((0.011, 0.011, 0.008))
        cap_geo.translate(0.0, 0.0, 0.004)
        btn.visual(mesh_from_geometry(cap_geo, f"{name}_cap"), material=cream, name=f"{name}_cap")
        btn.inertial = Inertial.from_geometry(Box((0.011, 0.011, 0.008)), mass=0.003,
                                              origin=Origin(xyz=(0.0, 0.0, 0.004)))
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=btn,
            # explicit mount on the tray floor: place_on_surface would snap to the
            # tray rim/side walls of the recess instead of its flat floor.
            origin=Origin(xyz=(bx, TRAY_FLOOR_Y, transport_z), rpy=_rpy_from_z(FRONT_N)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.0016),
        )

    # =====================================================================
    # ANTENNA -- swivel knuckle + mast (REVOLUTE), telescoping rod (PRISMATIC)
    # =====================================================================
    ant_base = model.part("antenna_base")
    knuckle = CylinderGeometry(0.009, 0.016, radial_segments=24)
    knuckle.translate(0.0, 0.0, 0.008)
    ant_base.visual(mesh_from_geometry(knuckle, "antenna_knuckle"), material=dark, name="antenna_knuckle")
    # HOLLOW mast tube: thin-wall annular section, open mouth at the top so the
    # rod really slides into a bore (outer r 6 mm, bore r 4.5 mm vs rod r 3.6 mm).
    # The tube foot is buried 2 mm into the knuckle so the pieces fuse.
    mast_tube = (
        cq.Workplane("XY")
        .workplane(offset=0.014)
        .circle(0.006)
        .circle(0.0045)
        .extrude(0.132)
    )
    ant_base.visual(
        mesh_from_cadquery(mast_tube, "antenna_mast", tolerance=0.0002, angular_tolerance=0.25),
        material=chrome,
        name="antenna_mast",
    )
    ant_base.inertial = Inertial.from_geometry(Box((0.018, 0.018, 0.10)), mass=0.03,
                                               origin=Origin(xyz=(0.0, 0.0, 0.05)))
    model.articulation(
        "antenna_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=ant_base,
        origin=Origin(xyz=(boss_x, boss_y, BOSS_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=math.radians(85.0)),
    )

    ant_rod = model.part("antenna_rod")
    rod = CylinderGeometry(0.0036, 0.135, radial_segments=16)
    rod.translate(0.0, 0.0, 0.0675)
    ant_rod.visual(mesh_from_geometry(rod, "antenna_rod"), material=chrome, name="antenna_rod")
    # guide collar at the rod foot: rides in the mast bore (slightly over bore radius
    # so the sliding fit registers as a declared axisymmetric bearing contact).
    collar = CylinderGeometry(0.00465, 0.010, radial_segments=18)
    collar.translate(0.0, 0.0, 0.005)
    ant_rod.visual(mesh_from_geometry(collar, "antenna_rod_collar"), material=chrome, name="antenna_rod_collar")
    tip = SphereGeometry(0.006, width_segments=18, height_segments=12)
    tip.translate(0.0, 0.0, 0.135)
    ant_rod.visual(mesh_from_geometry(tip, "antenna_tip"), material=dark, name="antenna_tip")
    ant_rod.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.135)), mass=0.015,
                                              origin=Origin(xyz=(0.0, 0.0, 0.0675)))
    # rest pose: extended, with the rod foot 20 mm inside the mast top for real overlap.
    # Travel is derived from what the mast can swallow: the foot may descend only to
    # 4 mm above the mast bottom, so the retracted rod nests fully inside the mast and
    # just the ball tip seats on the mast mouth (nothing ever pokes out below).
    model.articulation(
        "antenna_extend",
        ArticulationType.PRISMATIC,
        parent=ant_base,
        child=ant_rod,
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.2, lower=0.0, upper=0.106),
    )

    return model


def _ctr(aabb, axis):
    return 0.5 * (aabb[0][axis] + aabb[1][axis])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("cd_lid")
    ant_base = object_model.get_part("antenna_base")
    ant_rod = object_model.get_part("antenna_rod")
    volume = object_model.get_part("volume_knob")
    tuning = object_model.get_part("tuning_knob")

    lid_hinge = object_model.get_articulation("lid_hinge")
    ant_swivel = object_model.get_articulation("antenna_swivel")
    ant_extend = object_model.get_articulation("antenna_extend")

    # ---- overall proportions: wider than deep, deep than tall, sits on the ground ----
    aabb = ctx.part_world_aabb(body)
    dx, dy, dz = (aabb[1][i] - aabb[0][i] for i in range(3))
    ctx.check("body is a wide rounded-square slab (width > depth > height)", dx > dy > dz,
              details=f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f}")
    ctx.check("body sits on the ground", abs(aabb[0][2]) < 0.01, details=f"zmin={aabb[0][2]:.4f}")

    # ---- dual stereo speakers: one recessed grille on each side of the front face ----
    for i, (expected_sign, label) in enumerate(((-1, "left"), (1, "right"))):
        spk = ctx.part_element_world_aabb(body, elem=f"speaker_grille_{i}")
        ctx.check(f"{label} speaker faces the front",
                  spk is not None and _ctr(spk, 1) < -0.05,
                  details=f"{label} spk y-center={None if spk is None else round(_ctr(spk,1),3)}")
        ctx.check(f"{label} speaker is on the {label} side",
                  spk is not None and (_ctr(spk, 0) * expected_sign) > 0.03,
                  details=f"{label} spk x-center={None if spk is None else round(_ctr(spk,0),3)}")
        ctx.check(f"{label} speaker grille sits just behind the flat front plane",
                  spk is not None and spk[0][1] > FRONT_Y - 0.001,
                  details=f"{label} grille ymin={None if spk is None else round(spk[0][1],4)} front={FRONT_Y:.4f}")
    # speakers are symmetric about center
    spk0 = ctx.part_element_world_aabb(body, elem="speaker_grille_0")
    spk1 = ctx.part_element_world_aabb(body, elem="speaker_grille_1")
    ctx.check("dual speakers are symmetric about body center",
              spk0 is not None and spk1 is not None
              and abs(_ctr(spk0, 0) + _ctr(spk1, 0)) < 0.005,
              details=f"left x={None if spk0 is None else round(_ctr(spk0,0),3)} "
                      f"right x={None if spk1 is None else round(_ctr(spk1,0),3)}")

    # ---- CD lid seats over the deck and opens about the rear hinge ----
    ctx.allow_overlap(lid, body, elem_a="lid_dome", elem_b="body_shell",
                      reason="The dome lid rim seats onto the CD deck / seam ring on the body top.")
    ctx.allow_overlap(lid, body, elem_a="lid_dome", elem_b="dome_seam",
                      reason="The dome lid rim seats onto the seam ring.")
    ctx.allow_overlap(lid, body, elem_a="lid_hinge", elem_b="body_shell",
                      reason="The hinge barrel is captured at the rear top edge of the body.")
    ctx.allow_overlap(lid, body, elem_a="lid_hinge", elem_b="dome_seam",
                      reason="The hinge barrel runs along the dome's rear seam ring.")
    rest = ctx.part_element_world_aabb(lid, elem="lid_dome")
    rest_front_y, rest_top_z = rest[0][1], rest[1][2]
    with ctx.pose({lid_hinge: math.radians(65.0)}):
        opened = ctx.part_element_world_aabb(lid, elem="lid_dome")
        open_front_y, open_top_z = opened[0][1], opened[1][2]
    ctx.check("cd lid front edge swings back when opened", open_front_y > rest_front_y + 0.04,
              details=f"rest={rest_front_y:.3f} open={open_front_y:.3f}")
    ctx.check("cd lid rises as it opens", open_top_z > rest_top_z + 0.02,
              details=f"rest={rest_top_z:.3f} open={open_top_z:.3f}")

    # ---- knobs spin: off-axis marker swings around the spin axis ----
    for knob, label in ((volume, "volume"), (tuning, "tuning")):
        spin = object_model.get_articulation(f"{label}_knob_spin")
        ctx.allow_overlap(knob, body, elem_a=f"{label}_knob_cap", elem_b="body_shell",
                          reason="The knob base is seated onto the deck surface.")
        mk = f"{label}_knob_marker"
        r = ctx.part_element_world_aabb(knob, elem=mk)
        rc = (_ctr(r, 0), _ctr(r, 1))
        with ctx.pose({spin: math.pi}):
            s = ctx.part_element_world_aabb(knob, elem=mk)
            sc = (_ctr(s, 0), _ctr(s, 1))
        ctx.check(f"{label} knob marker swings when spun",
                  abs(sc[0] - rc[0]) > 0.004 or abs(sc[1] - rc[1]) > 0.004,
                  details=f"rest={tuple(round(v,3) for v in rc)} spun={tuple(round(v,3) for v in sc)}")

    # ---- deck function buttons + transport buttons press inward (into the body) ----
    for name in ("function_button_0", "function_button_1"):
        b = object_model.get_part(name)
        ctx.allow_overlap(b, body, elem_a=f"{name}_cap", elem_b="body_shell",
                          reason="Button cap is seated into the deck so it can press inward.")
    for i in range(5):
        name = f"transport_button_{i}"
        b = object_model.get_part(name)
        ctx.allow_overlap(b, body, elem_a=f"{name}_cap", elem_b="body_shell",
                          reason="Transport key is seated into the front tray so it can press inward.")
        ctx.allow_overlap(b, body, elem_a=f"{name}_cap", elem_b="transport_tray",
                          reason="Transport key passes through the dark tray lining plate.")
    tb = object_model.get_part("transport_button_2")
    tj = object_model.get_articulation("transport_button_2_press")
    rest_c = ctx.part_world_position(tb)
    with ctx.pose({tj: 0.0016}):
        press_c = ctx.part_world_position(tb)
    moved = math.dist(rest_c, press_c)
    ctx.check("transport button travels when pressed", moved > 0.0010,
              details=f"travel={moved:.4f}")

    # ---- the five transport keys all seat on one flat plane (flat front face) ----
    ys = []
    for i in range(5):
        b = object_model.get_part(f"transport_button_{i}")
        ys.append(ctx.part_world_position(b)[1])
    ctx.check("transport keys sit on a single flat front plane", max(ys) - min(ys) < 0.002,
              details=f"y spread={max(ys) - min(ys):.5f}")

    # ---- antenna: rear-mounted, telescopes, and swivels ----
    ctx.allow_overlap(ant_rod, ant_base, elem_a="antenna_rod", elem_b="antenna_mast",
                      reason="The thin upper rod telescopes inside the hollow mast tube.")
    ctx.allow_overlap(ant_rod, ant_base, elem_a="antenna_rod_collar", elem_b="antenna_mast",
                      reason="The rod's guide collar rides in the mast bore as a sliding bearing.")
    ctx.allow_overlap(ant_base, body, elem_a="antenna_knuckle", elem_b="antenna_boss",
                      reason="The swivel knuckle seats on the rear mounting boss.")
    base_pos = ctx.part_world_position(ant_base)
    ctx.check("antenna mounted toward the rear-right", base_pos[1] > 0.02 and base_pos[0] > 0.03,
              details=f"antenna base x={base_pos[0]:.3f} y={base_pos[1]:.3f}")
    rest_tip = ctx.part_world_aabb(ant_rod)[1][2]
    mast_aabb = ctx.part_element_world_aabb(ant_base, elem="antenna_mast")
    with ctx.pose({ant_extend: 0.106}):
        coll_tip = ctx.part_world_aabb(ant_rod)[1][2]
        coll_bottom = ctx.part_element_world_aabb(ant_rod, elem="antenna_rod")[0][2]
    ctx.check("antenna telescopes (tip height changes with the joint)", rest_tip - coll_tip > 0.08,
              details=f"extended_tip={rest_tip:.3f} collapsed_tip={coll_tip:.3f}")
    ctx.check("retracted rod stays nested inside the mast (never pokes out below)",
              coll_bottom > mast_aabb[0][2] - 0.001,
              details=f"rod bottom={coll_bottom:.4f} mast bottom={mast_aabb[0][2]:.4f}")
    rest_pos = ctx.part_world_position(ant_rod)
    with ctx.pose({ant_swivel: math.radians(75.0)}):
        sw_pos = ctx.part_world_position(ant_rod)
    ctx.check("antenna swivels at the base", abs(sw_pos[1] - rest_pos[1]) > 0.03,
              details=f"rest={tuple(round(v,3) for v in rest_pos)} swiveled={tuple(round(v,3) for v in sw_pos)}")

    return ctx.report()


object_model = build_object_model()
