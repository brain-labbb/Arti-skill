from __future__ import annotations

# CH-47 Chinook-style tandem-rotor military transport helicopter.
#
# Convention (real meters, Z-up): nose = +X, tail = -X, width along Y.
# Wheels touch the ground at z = 0.
#
# Key true-scale numbers:
#   fuselage ~16.2 m long (x: -8.0 .. +8.15), ~3.74 m wide over the sponsons,
#   slab cabin top at z ~3.05, rear rotor hub at z ~5.63, each 3-blade rotor
#   ~17.9 m in diameter.
#
# Articulations:
#   front_rotor_spin : CONTINUOUS, near-vertical axis tilted 9 deg forward,
#                      on the low front pylon above the cockpit.
#   rear_rotor_spin  : CONTINUOUS, near-vertical axis tilted 4 deg forward,
#                      atop the tall rear pylon, hub ~2.05 m higher than front.
#   ramp_hinge       : REVOLUTE about the lateral (-Y) axis at the bottom rear
#                      edge of the upswept tail. 0 = closed flush with the
#                      tail underside.  Upper limit 0.64 rad is where the ramp
#                      lip reaches ground level; with a real ~0.6 m hinge
#                      height and a ~3 m ramp a 60 deg swing would bury the
#                      ramp deep underground, so the physical ground-contact
#                      angle wins over the nominal "about 60 deg" plan.
from math import asin, cos, pi, radians, sin

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftSection,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    section_loft,
)

# ---------------------------------------------------------------------------
# Key dimensions
# ---------------------------------------------------------------------------
# Raise the whole airframe by Z_LIFT so it stands taller on its gear: the body
# (hull/shell/canopy/pylons via the section helpers, plus the directly-placed
# rotors/ramp/sponsons/engines/windows/cabin floor) all shift up, while the
# wheels stay on the ground (z=0) and the struts lengthen to span the gap.
Z_LIFT = 0.30
NOSE_X = 8.15
TAIL_X = -8.0
CABIN_TOP = 3.05
BELLY_Z = 0.62
CABIN_W = 2.30

# Tail upsweep (ramp opening) plane: from belly break to the tail bottom edge.
UPSWEEP_X0 = -5.0          # belly break
TAIL_BOTTOM_Z = 2.15       # hull bottom at the very tail
UPSWEEP_ANG = 0.47169      # atan((2.15-0.62)/3.0) -> ~27 deg

# Cargo ramp
RAMP_HINGE = (-4.95, 0.0, 0.585 + Z_LIFT)
RAMP_LEN = 2.95
RAMP_W = 1.85
RAMP_T = 0.07
RAMP_CLEAR = 0.02          # skin clearance off the hull upsweep plane
# Ground-contact opening angle (ramp lip touches z=0).
RAMP_OPEN = 0.74

# Rotors
ROTOR_R = 8.95             # blade tip radius -> ~17.9 m disk
FRONT_TILT = radians(9.0)  # shaft tilt toward the nose
REAR_TILT = radians(4.0)
FRONT_MOUNT = (5.90, 0.0, 3.28 + Z_LIFT)    # shaft exit on the front pylon top
REAR_MOUNT = (-6.45, 0.0, 5.33 + Z_LIFT)    # shaft exit on the rear pylon top
HUB_STANDOFF = 0.30                # hub center along shaft axis above mount
HUB_R = 0.32
HUB_H = 0.36

# Landing gear
WHEEL_R = 0.28
WHEEL_W = 0.18
GEAR_Y = 1.42
FRONT_GEAR_X = 3.45
REAR_GEAR_X = -4.25

SPONSON_R = 0.45
SPONSON_X0 = -4.6
SPONSON_LEN = 8.6
SPONSON_Y = 1.42
SPONSON_Z = 1.00 + Z_LIFT


def _axis_dir(tilt: float) -> tuple[float, float, float]:
    """Unit direction of a rotor shaft tilted `tilt` rad toward +X."""
    return (sin(tilt), 0.0, cos(tilt))


def _hub_center(mount: tuple[float, float, float], tilt: float) -> tuple[float, float, float]:
    a = _axis_dir(tilt)
    return (
        mount[0] + HUB_STANDOFF * a[0],
        mount[1] + HUB_STANDOFF * a[1],
        mount[2] + HUB_STANDOFF * a[2],
    )


FRONT_HUB = _hub_center(FRONT_MOUNT, FRONT_TILT)
REAR_HUB = _hub_center(REAR_MOUNT, REAR_TILT)


# ---------------------------------------------------------------------------
# Section helpers (rounded-rectangle loops with consistent correspondence)
# ---------------------------------------------------------------------------
def _rrect_yz(x: float, w: float, zb: float, zt: float, r: float, n: int = 7) -> LoftSection:
    """Rounded-rect loop in the YZ plane at station x (fuselage cross-section)."""
    zb += Z_LIFT
    zt += Z_LIFT
    r = min(r, 0.49 * w, 0.49 * (zt - zb))
    cy = w / 2.0 - r
    corners = (
        (cy, zt - r, 0.0),
        (-cy, zt - r, 90.0),
        (-cy, zb + r, 180.0),
        (cy, zb + r, 270.0),
    )
    pts = []
    for ccy, ccz, a0 in corners:
        for k in range(n):
            a = radians(a0 + 90.0 * k / n)
            pts.append((x, ccy + r * cos(a), ccz + r * sin(a)))
    return LoftSection(points=tuple(pts))


def _rrect_xy(z: float, cx: float, lx: float, wy: float, r: float, n: int = 7) -> LoftSection:
    """Rounded-rect loop in the XY plane at height z (pylon cross-section)."""
    z += Z_LIFT
    r = min(r, 0.49 * lx, 0.49 * wy)
    hx = lx / 2.0 - r
    hy = wy / 2.0 - r
    corners = (
        (cx + hx, hy, 0.0),
        (cx - hx, hy, 90.0),
        (cx - hx, -hy, 180.0),
        (cx + hx, -hy, 270.0),
    )
    pts = []
    for ccx, ccy, a0 in corners:
        for k in range(n):
            a = radians(a0 + 90.0 * k / n)
            pts.append((ccx + r * cos(a), ccy + r * sin(a), z))
    return LoftSection(points=tuple(pts))


def _capsule_x(radius: float, length: float, end_r: float) -> cq.Workplane:
    """Cylinder along +X (from x=0) with rounded rims."""
    return cq.Workplane("YZ").circle(radius).extrude(length).edges().fillet(end_r)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ch47_tandem_rotor_helicopter")

    tan = model.material("desert_tan", rgba=(0.78, 0.66, 0.44, 1.0))
    tan_dark = model.material("dark_tan", rgba=(0.57, 0.46, 0.30, 1.0))
    glass = model.material("glass_black", rgba=(0.05, 0.05, 0.07, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.07, 0.07, 0.08, 1.0))
    tip_green = model.material("tip_yellow_green", rgba=(0.72, 0.82, 0.28, 1.0))
    metal = model.material("dark_metal", rgba=(0.22, 0.22, 0.24, 1.0))
    rubber = model.material("tire_black", rgba=(0.05, 0.05, 0.05, 1.0))
    cabin_floor_mat = model.material("cabin_floor", rgba=(0.16, 0.16, 0.18, 1.0))
    seat_mat = model.material("troop_seat", rgba=(0.30, 0.34, 0.22, 1.0))
    seat_frame_mat = model.material("seat_frame", rgba=(0.18, 0.18, 0.19, 1.0))

    # ------------------------------------------------------------------
    # Fuselage (root): lofted hull, sponsons, pylons, engines, gear ...
    # ------------------------------------------------------------------
    fus = model.part("fuselage")

    hull_sections = (
        _rrect_yz(TAIL_X, 1.75, TAIL_BOTTOM_Z, CABIN_TOP, 0.30),
        _rrect_yz(-6.5, 2.15, 1.385, CABIN_TOP, 0.30),
        _rrect_yz(UPSWEEP_X0, CABIN_W, BELLY_Z, CABIN_TOP, 0.30),
        _rrect_yz(-2.0, CABIN_W, BELLY_Z, CABIN_TOP, 0.30),
        _rrect_yz(2.0, CABIN_W, BELLY_Z, CABIN_TOP, 0.30),
        _rrect_yz(6.3, CABIN_W, BELLY_Z, CABIN_TOP, 0.30),
        _rrect_yz(7.3, 2.05, 0.78, 2.85, 0.45),
        _rrect_yz(7.9, 1.45, 1.02, 2.38, 0.50),
        _rrect_yz(NOSE_X, 0.80, 1.30, 1.95, 0.32),
    )
    # Hull built as a CadQuery solid loft so we can cut a real rear CARGO
    # DOORWAY. Without it the upswept tail underside is a closed skin and the
    # open ramp just reveals a sealed wall; the cut removes the lower-aft skin
    # (below the tail boom) so the cabin interior shows through the open ramp.
    def _hull_wire(sec):
        pts = [cq.Vector(float(px), float(py), float(pz)) for (px, py, pz) in sec.points]
        return cq.Wire.makePolygon(pts + [pts[0]])

    hull_solid = cq.Solid.makeLoft([_hull_wire(s) for s in hull_sections], ruled=True)
    # Hollow the cabin into a single cavity and open its rear, so the cabin reads
    # as an open cargo hold through the ramp instead of a solid block:
    #   - cavity: inset interior over the cabin (x -4.6..6.0, y +/-1.02,
    #     z 0.72..2.88) -> the solid centre is removed; thin walls / roof / belly
    #     stay, giving a real hollow hold.
    #   - doorway: full-width lower-aft opening (x -7.6..-4.2, z 0.45..2.45) that
    #     drops the rear face toward the ramp. The tail boom + rear pylon
    #     (z > 2.45 / 2.88) stay intact so the rear rotor remains supported.
    # Cavity hollows the FLAT cabin only (x -5.0..6.0, y +/-1.0, z 0.72..2.88).
    # It stops at the belly break (x=-5) and its floor (z=0.72) stays above the
    # belly (z=0.62), so it never breaches the outer skin — the upswept tail
    # (x < -5) stays fully wrapped; only the ramp doorway opens it.
    # Hollow into a closed SHELL: subtract an inset inner loft (~0.10 m wall)
    # that follows the exact hull shape, so the WHOLE interior (cabin + upswept
    # tail) is hollow while every outer surface stays wrapped.
    T = 0.10
    inner_sections = (
        _rrect_yz(TAIL_X, 1.75 - 2 * T, TAIL_BOTTOM_Z + T, CABIN_TOP - T, 0.20),
        _rrect_yz(-6.5, 2.15 - 2 * T, 1.385 + T, CABIN_TOP - T, 0.20),
        _rrect_yz(UPSWEEP_X0, CABIN_W - 2 * T, BELLY_Z + T, CABIN_TOP - T, 0.20),
        _rrect_yz(-2.0, CABIN_W - 2 * T, BELLY_Z + T, CABIN_TOP - T, 0.20),
        _rrect_yz(2.0, CABIN_W - 2 * T, BELLY_Z + T, CABIN_TOP - T, 0.20),
        _rrect_yz(6.3, CABIN_W - 2 * T, BELLY_Z + T, CABIN_TOP - T, 0.20),
        _rrect_yz(7.3, 2.05 - 2 * T, 0.78 + T, 2.85 - T, 0.35),
        _rrect_yz(7.9, 1.45 - 2 * T, 1.02 + T, 2.38 - T, 0.40),
        _rrect_yz(NOSE_X, 0.80 - 2 * T, 1.30 + T, 1.95 - T, 0.22),
    )
    inner_solid = cq.Solid.makeLoft([_hull_wire(s) for s in inner_sections], ruled=True)
    hull_solid = hull_solid.cut(inner_solid)
    # Doorway = a RAMP-SHAPED hole in the upswept underside only (not full width),
    # oriented along the ramp and slightly smaller than it. This keeps the rear
    # SIDE WALLS wrapped, leaves the hinge lip (so the ramp stays carried), and
    # lets the closed ramp overlap the frame all around to seal the cabin.
    ct, st = cos(UPSWEEP_ANG), sin(UPSWEEP_ANG)
    du = (-ct, 0.0, st)                      # up-the-tail along the ramp
    mid = RAMP_LEN / 2.0
    foot_c = (RAMP_HINGE[0] + mid * du[0], 0.0, RAMP_HINGE[2] + mid * du[2])
    od_l, od_w, od_t = RAMP_LEN - 0.30, RAMP_W - 0.12, 0.35
    doorway = cq.Solid.makeBox(od_l, od_w, od_t).translate(
        (-od_l / 2.0, -od_w / 2.0, -od_t / 2.0)
    )
    doorway = doorway.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), UPSWEEP_ANG * 180.0 / pi)
    doorway = doorway.translate(cq.Vector(*foot_c))
    hull_solid = hull_solid.cut(doorway)
    fus.visual(
        mesh_from_cadquery(cq.Workplane("XY").add(hull_solid), "hull"),
        material=tan,
        name="hull",
    )

    # Glazed cockpit canopy: a slightly-proud black band lofted over the nose.
    canopy_sections = (
        _rrect_yz(6.90, 2.21, 1.65, 2.99, 0.40),
        _rrect_yz(7.30, 2.11, 1.55, 2.91, 0.45),
        _rrect_yz(7.90, 1.51, 1.30, 2.44, 0.45),
    )
    fus.visual(
        mesh_from_geometry(
            section_loft(SectionLoftSpec(sections=canopy_sections, ruled=True)),
            "cockpit_canopy",
        ),
        material=glass,
        name="cockpit_canopy",
    )

    # Drive-shaft tunnel spine along the cabin top, linking the two pylons.
    fus.visual(
        Cylinder(0.30, 11.0),
        origin=Origin(xyz=(0.2, 0.0, 3.00 + Z_LIFT), rpy=(0.0, pi / 2.0, 0.0)),
        material=tan,
        name="spine_fairing",
    )

    # Front pylon: low fairing above the cockpit.
    front_pylon_sections = (
        _rrect_xy(2.95, 5.90, 1.45, 1.50, 0.35),
        _rrect_xy(3.30, 5.90, 1.05, 1.00, 0.30),
    )
    fus.visual(
        mesh_from_geometry(
            section_loft(SectionLoftSpec(sections=front_pylon_sections, ruled=True)),
            "front_pylon",
        ),
        material=tan,
        name="front_pylon",
    )

    # Rear pylon: tall tapered tower at the tail.
    rear_pylon_sections = (
        _rrect_xy(2.90, -6.55, 2.90, 1.70, 0.40),
        _rrect_xy(4.20, -6.75, 2.30, 1.35, 0.40),
        _rrect_xy(5.35, -6.45, 1.60, 1.05, 0.35),
    )
    fus.visual(
        mesh_from_geometry(
            section_loft(SectionLoftSpec(sections=rear_pylon_sections, ruled=True)),
            "rear_pylon",
        ),
        material=tan,
        name="rear_pylon",
    )

    # Twin engine nacelles flanking the rear pylon base.
    engine = _capsule_x(0.42, 2.70, 0.25)
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        fus.visual(
            mesh_from_cadquery(
                engine.translate((-7.65, sign * 1.22, 3.25 + Z_LIFT)),
                f"{side}_engine_nacelle",
            ),
            material=tan_dark,
            name=f"{side}_engine_nacelle",
        )

    # Rear exhaust: a cover plate over the square port above the cargo doorway,
    # with two larger bored holes that two hollow exhaust nozzles plug snugly and
    # protrude aft (bores open all the way through). Position constants are easy
    # to nudge: EXH_CX (fore/aft), EXH_CZ (up/down), EXH_DY (nozzle spacing).
    exh_mat = model.material("exhaust_metal", rgba=(0.17, 0.18, 0.19, 1.0))
    # Rounded-rect cover plate matching the port outline, sitting OUT on the tail
    # surface (EXH_CX more aft = more -X), with two bored holes that two hollow
    # nozzles plug snugly and protrude aft (bores open through).
    EXH_CX, EXH_CZ, EXH_DY, EXH_R = -7.95, 2.85, 0.40, 0.20
    EXH_PLATE_W, EXH_PLATE_H = 1.60, 0.70
    cover = (
        cq.Workplane("YZ")
        .rect(EXH_PLATE_W, EXH_PLATE_H)
        .extrude(0.10)
        .edges("|X")
        .fillet(0.12)
        .translate((EXH_CX - 0.05, 0.0, EXH_CZ))
    )
    for hy in (EXH_DY, -EXH_DY):
        cover = cover.cut(
            cq.Workplane("YZ").circle(EXH_R).extrude(0.60).translate((EXH_CX - 0.30, hy, EXH_CZ))
        )
    for hy in (EXH_DY, -EXH_DY):
        cover = cover.union(
            cq.Workplane("YZ")
            .circle(EXH_R)
            .circle(EXH_R - 0.07)
            .extrude(-0.45)
            .translate((EXH_CX + 0.05, hy, EXH_CZ))
        )
    fus.visual(mesh_from_cadquery(cover, "rear_exhaust"), material=exh_mat, name="rear_exhaust")

    # Side fuel sponsons: long half-rounded pods low on each flank.
    sponson = _capsule_x(SPONSON_R, SPONSON_LEN, 0.30)
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        fus.visual(
            mesh_from_cadquery(
                sponson.translate((SPONSON_X0, sign * SPONSON_Y, SPONSON_Z)),
                f"{side}_sponson",
            ),
            material=tan,
            name=f"{side}_sponson",
        )

    # Row of small round cabin windows on each flank.
    win_xs = (-3.4, -2.2, -1.0, 0.2, 1.4, 2.6)
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        for i, wx in enumerate(win_xs):
            fus.visual(
                Cylinder(0.16, 0.06),
                origin=Origin(xyz=(wx, sign * 1.13, 2.25 + Z_LIFT), rpy=(pi / 2.0, 0.0, 0.0)),
                material=glass,
                name=f"{side}_window_{i}",
            )
        # Cockpit side window.
        fus.visual(
            Box((0.80, 0.06, 0.60)),
            origin=Origin(xyz=(6.65, sign * 1.12, 2.25 + Z_LIFT)),
            material=glass,
            name=f"{side}_cockpit_window",
        )

    # (Camouflage patch panels removed per request — they read as stuck-on strips.)

    # ------------------------------------------------------------------
    # Cargo cabin interior (visible through the open rear ramp): a deck
    # floor + fold-down troop bench seats down both side walls, like a real
    # CH-47 troop transport. Non-moving, so fuselage visuals (not a part).
    # All elements touch the hull / each other so they stay one connected
    # island; they sit inside the hull skin and read through the rear opening.
    # ------------------------------------------------------------------
    CAB_X0, CAB_X1 = -4.3, 3.3
    floor_z = BELLY_Z + 0.05 + Z_LIFT
    fus.visual(
        Box((CAB_X1 - CAB_X0, CABIN_W, 0.10)),
        origin=Origin(xyz=((CAB_X0 + CAB_X1) / 2.0, 0.0, floor_z)),
        material=cabin_floor_mat,
        name="cabin_floor",
    )
    pan_z = floor_z + 0.43
    wall_y = CABIN_W / 2.0 - 0.15      # backrest against the hollowed inner wall
    seat_xs = [-3.7 + 0.72 * i for i in range(9)]   # -3.7 .. +2.06
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        for i, sx in enumerate(seat_xs):
            fus.visual(
                Box((0.50, 0.06, 0.52)),
                origin=Origin(xyz=(sx, sign * wall_y, pan_z + 0.28)),
                material=seat_mat,
                name=f"{side}_seat_back_{i}",
            )
            fus.visual(
                Box((0.50, 0.46, 0.06)),
                origin=Origin(xyz=(sx, sign * (wall_y - 0.24), pan_z)),
                material=seat_mat,
                name=f"{side}_seat_pan_{i}",
            )
            fus.visual(
                Box((0.05, 0.05, pan_z - floor_z)),
                origin=Origin(
                    xyz=(sx, sign * (wall_y - 0.44), floor_z + (pan_z - floor_z) / 2.0)
                ),
                material=seat_frame_mat,
                name=f"{side}_seat_leg_{i}",
            )

    # ------------------------------------------------------------------
    # Internal cockpit/cabin bulkhead: a transverse partition wall at the
    # forward end of the troop cabin (ahead of the seats) with a passage
    # doorway, dividing the cockpit from the seated cabin. A hinged door
    # fills the doorway and swings forward into the cockpit. Both are visible
    # through the open cargo ramp / cockpit.
    # ------------------------------------------------------------------
    BHD_X = 3.0                        # bulkhead station (fwd of last seat ~2.06)
    BHD_T = 0.06                       # wall thickness (X)
    BHD_Y = 1.00                       # half-width (inside the hollow inner wall)
    BHD_Z1 = 3.05                      # top (near the cabin ceiling)
    DOOR_HW = 0.42                     # doorway half-width
    DOOR_TOP = 2.35                    # doorway lintel height
    wall = cq.Solid.makeBox(BHD_T, 2 * BHD_Y, BHD_Z1 - floor_z).translate(
        (BHD_X - BHD_T / 2.0, -BHD_Y, floor_z)
    )
    wall = wall.cut(
        cq.Solid.makeBox(BHD_T + 0.04, 2 * DOOR_HW, DOOR_TOP - floor_z).translate(
            (BHD_X - BHD_T / 2.0 - 0.02, -DOOR_HW, floor_z)
        )
    )
    fus.visual(
        mesh_from_cadquery(cq.Workplane("XY").add(wall), "cabin_bulkhead"),
        material=tan_dark,
        name="cabin_bulkhead",
    )

    # Bulkhead passage door (hinged on the +Y jamb, swings forward into cockpit).
    bdoor = model.part("bulkhead_door")
    BH = (BHD_X, DOOR_HW, floor_z)     # hinge: +Y vertical edge at the floor
    DBOT, DT = floor_z + 0.05, DOOR_TOP - 0.03
    panel = cq.Solid.makeBox(0.05, 2 * DOOR_HW - 0.05, DT - DBOT).translate(
        (BHD_X - 0.025, -(DOOR_HW - 0.025), DBOT)
    )
    panel = panel.cut(
        cq.Solid.makeBox(0.12, 0.34, 0.34).translate((BHD_X - 0.06, -0.27, DOOR_TOP - 0.62))
    )
    panel = panel.translate((-BH[0], -BH[1], -BH[2]))
    bdoor.visual(
        mesh_from_cadquery(cq.Workplane("XY").add(panel), "door_panel"),
        material=tan,
        name="door_panel",
    )
    bdoor.visual(
        Box((0.03, 0.34, 0.34)),
        origin=Origin(xyz=(BHD_X - BH[0], -0.10 - BH[1], DOOR_TOP - 0.45 - BH[2])),
        material=glass,
        name="door_vision_window",
    )
    # Lever handle (rose + lever) near the free (-Y) edge.
    bdoor.visual(
        Cylinder(0.045, 0.02),
        origin=Origin(xyz=(BHD_X + 0.035 - BH[0], -0.30 - BH[1], DBOT + 0.85 - BH[2]), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="door_handle_rose",
    )
    bdoor.visual(
        Box((0.03, 0.17, 0.025)),
        origin=Origin(xyz=(BHD_X + 0.06 - BH[0], -0.355 - BH[1], DBOT + 0.85 - BH[2])),
        material=metal,
        name="door_handle",
    )
    # Hinge knuckle on the +Y jamb (seats into the bulkhead post; carried).
    # Bottom kept above the cabin-cavity floor so it nests in the jamb, not the
    # hull belly shell.
    bdoor.visual(
        Cylinder(0.03, DT - DBOT - 0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (DT - DBOT) + 0.06)),
        material=metal,
        name="hinge_knuckle",
    )
    model.articulation(
        "bulkhead_door_hinge",
        ArticulationType.REVOLUTE,
        parent=fus,
        child=bdoor,
        origin=Origin(xyz=(BHD_X, DOOR_HW, floor_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=1.40),
    )

    # Landing gear: four visible struts, each carrying a SPINNING wheel. The
    # wheel is its own part with a CONTINUOUS spin joint (rolls about Y); the
    # vertical oleo strut + lateral axle stay on the fuselage so the link from
    # wheel to airframe is clearly visible.
    gear_pos = (
        (FRONT_GEAR_X, GEAR_Y, "front_left"),
        (FRONT_GEAR_X, -GEAR_Y, "front_right"),
        (REAR_GEAR_X, GEAR_Y, "rear_left"),
        (REAR_GEAR_X, -GEAR_Y, "rear_right"),
    )
    for gx, gy, gname in gear_pos:
        sgn = 1.0 if gy > 0 else -1.0
        fus.visual(
            Cylinder(0.06, 0.62),
            origin=Origin(xyz=(gx, gy - sgn * 0.22, 0.62)),
            material=metal,
            name=f"{gname}_gear_strut",
        )
        fus.visual(
            Cylinder(0.035, 0.30),
            origin=Origin(xyz=(gx, gy - sgn * 0.13, WHEEL_R), rpy=(pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"{gname}_axle",
        )
        wheel = model.part(f"{gname}_wheel")
        wheel.visual(
            Cylinder(WHEEL_R, WHEEL_W),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(0.10, WHEEL_W + 0.02),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="hub",
        )
        model.articulation(
            f"{gname}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=fus,
            child=wheel,
            origin=Origin(xyz=(gx, gy, WHEEL_R)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=200.0, velocity=40.0),
        )

    # ------------------------------------------------------------------
    # Rotors: dark hub + three tapered black blades with yellow-green tips
    # ------------------------------------------------------------------
    def _build_rotor(part_name: str, base_angle: float) -> object:
        rotor = model.part(part_name)
        # Rotating mast: coaxial with the spin axis, seats down through the
        # pylon-top bearing (intentional local overlap with the pylon loft).
        rotor.visual(
            Cylinder(0.085, 0.40),
            origin=Origin(xyz=(0.0, 0.0, -HUB_STANDOFF)),
            material=metal,
            name="mast",
        )
        rotor.visual(Cylinder(HUB_R, HUB_H), material=metal, name="hub")
        rotor.visual(
            Cylinder(0.16, 0.14),
            origin=Origin(xyz=(0.0, 0.0, 0.24)),
            material=metal,
            name="hub_cap",
        )
        blade_geom = section_loft(
            [
                [(0.20, -0.30, -0.050), (0.20, 0.30, -0.050),
                 (0.20, 0.30, 0.050), (0.20, -0.30, 0.050)],
                [(ROTOR_R, -0.16, -0.026), (ROTOR_R, 0.16, -0.026),
                 (ROTOR_R, 0.16, 0.026), (ROTOR_R, -0.16, 0.026)],
            ]
        )
        blade_mesh = mesh_from_geometry(blade_geom, f"{part_name}_blade")
        tip_r = 8.575
        for i in range(3):
            ang = base_angle + i * 2.0 * pi / 3.0
            rotor.visual(
                blade_mesh,
                origin=Origin(rpy=(0.0, 0.0, ang)),
                material=rotor_black,
                name=f"blade_{i}",
            )
            rotor.visual(
                Box((0.75, 0.34, 0.058)),
                origin=Origin(
                    xyz=(tip_r * cos(ang), tip_r * sin(ang), 0.0),
                    rpy=(0.0, 0.0, ang),
                ),
                material=tip_green,
                name=f"blade_tip_{i}",
            )
        return rotor

    front_rotor = _build_rotor("front_rotor", 0.0)
    rear_rotor = _build_rotor("rear_rotor", pi / 3.0)

    model.articulation(
        "front_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fus,
        child=front_rotor,
        origin=Origin(xyz=FRONT_HUB, rpy=(0.0, FRONT_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=30.0),
    )
    model.articulation(
        "rear_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fus,
        child=rear_rotor,
        origin=Origin(xyz=REAR_HUB, rpy=(0.0, REAR_TILT, 0.0)),
        # Counter-rotating pair: positive q spins the rear rotor the opposite
        # hand of the front rotor about the world-up direction.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=30.0),
    )

    # ------------------------------------------------------------------
    # Rear cargo ramp: skin panel hugging the upswept tail underside
    # ------------------------------------------------------------------
    ramp = model.part("cargo_ramp")
    # Hinge barrel on the swing axis: a lateral knuckle tube that embeds a
    # few millimetres into the hull belly edge so the ramp is physically
    # carried by the airframe (rotation-invariant intentional overlap).
    ramp.visual(
        Cylinder(0.06, 1.60),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )
    ct, st = cos(UPSWEEP_ANG), sin(UPSWEEP_ANG)
    d = (-ct, 0.0, st)        # closed ramp direction (up the tail)
    nrm = (-st, 0.0, -ct)     # outward (down/back) normal of the ramp plane
    plate_off = RAMP_T / 2.0 + RAMP_CLEAR
    plate_c = (
        RAMP_LEN / 2.0 * d[0] + plate_off * nrm[0],
        0.0,
        RAMP_LEN / 2.0 * d[2] + plate_off * nrm[2],
    )
    ramp.visual(
        Box((RAMP_LEN, RAMP_W, RAMP_T)),
        origin=Origin(xyz=plate_c, rpy=(0.0, UPSWEEP_ANG, 0.0)),
        material=tan,
        name="ramp_plate",
    )
    # Lip edge beam at the free end of the ramp (also the ground-contact
    # reference used by the open-pose test).
    lip_c = (
        2.90 * d[0] + plate_off * nrm[0],
        0.0,
        2.90 * d[2] + plate_off * nrm[2],
    )
    ramp.visual(
        Box((0.12, RAMP_W, 0.08)),
        origin=Origin(xyz=lip_c, rpy=(0.0, UPSWEEP_ANG, 0.0)),
        material=tan_dark,
        name="ramp_lip",
    )
    # Longitudinal stiffener ribs on the outer skin.
    rib_off = plate_off + RAMP_T / 2.0 + 0.020
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        rib_c = (
            RAMP_LEN / 2.0 * d[0] + rib_off * nrm[0],
            sign * 0.55,
            RAMP_LEN / 2.0 * d[2] + rib_off * nrm[2],
        )
        ramp.visual(
            Box((2.60, 0.08, 0.05)),
            origin=Origin(xyz=rib_c, rpy=(0.0, UPSWEEP_ANG, 0.0)),
            material=tan_dark,
            name=f"{side}_ramp_rib",
        )

    model.articulation(
        "ramp_hinge",
        ArticulationType.REVOLUTE,
        parent=fus,
        child=ramp,
        origin=Origin(xyz=RAMP_HINGE),
        # Closed ramp extends up/back from the hinge; -Y makes positive q
        # swing the free edge down toward the ground.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=0.8, lower=0.0, upper=RAMP_OPEN),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fus = object_model.get_part("fuselage")
    front_rotor = object_model.get_part("front_rotor")
    rear_rotor = object_model.get_part("rear_rotor")
    ramp = object_model.get_part("cargo_ramp")
    front_spin = object_model.get_articulation("front_rotor_spin")
    rear_spin = object_model.get_articulation("rear_rotor_spin")
    ramp_hinge = object_model.get_articulation("ramp_hinge")

    # --- intentional seated overlaps (declared + proven) ----------------
    # Rotor masts seat through the pylon-top bearings; the overlap is
    # coaxial with the spin axis so it is identical at every rotor angle.
    ctx.allow_overlap(
        front_rotor,
        fus,
        elem_a="mast",
        elem_b="front_pylon",
        reason="Rotating mast seats through the front pylon top bearing; coaxial with the spin axis.",
    )
    ctx.allow_overlap(
        front_rotor,
        fus,
        elem_a="mast",
        elem_b="hull",
        reason="Front mast root passes the cabin-top skin into the transmission; coaxial with the spin axis.",
    )
    ctx.allow_overlap(
        rear_rotor,
        fus,
        elem_a="mast",
        elem_b="rear_pylon",
        reason="Rotating mast seats through the rear pylon top bearing; coaxial with the spin axis.",
    )
    ctx.expect_overlap(
        front_rotor,
        fus,
        axes="z",
        elem_a="mast",
        elem_b="front_pylon",
        min_overlap=0.08,
        name="front mast retained in the pylon bearing",
    )
    ctx.expect_overlap(
        rear_rotor,
        fus,
        axes="z",
        elem_a="mast",
        elem_b="rear_pylon",
        min_overlap=0.08,
        name="rear mast retained in the pylon bearing",
    )
    # Ramp hinge barrel embeds a few mm into the hull belly edge; the
    # barrel is on the swing axis so the embed is pose-invariant.
    ctx.allow_overlap(
        ramp,
        fus,
        elem_a="hinge_barrel",
        elem_b="hull",
        reason="Hinge knuckle tube embeds into the hull lower tail edge to carry the ramp; on-axis, pose-invariant.",
    )
    ctx.expect_contact(
        ramp,
        fus,
        elem_a="hinge_barrel",
        elem_b="hull",
        name="hinge barrel seated against the hull",
    )

    # Wheel hubs rotate around the fixed axles; the axle shaft is intentionally
    # nested inside the hub bore on all four gear legs (pose-invariant, coaxial).
    for g in ("front_left", "front_right", "rear_left", "rear_right"):
        wp = object_model.get_part(f"{g}_wheel")
        ctx.allow_overlap(
            wp, fus, elem_a="hub", elem_b=f"{g}_axle",
            reason=f"Axle shaft passes through the {g} wheel hub bore; coaxial with the spin axis.",
        )
        ctx.allow_overlap(
            wp, fus, elem_a="tire", elem_b=f"{g}_axle",
            reason=f"Axle shaft passes through the {g} tire inner bore; coaxial with the spin axis.",
        )
        ctx.expect_overlap(
            wp, fus, axes="y", elem_a="hub", elem_b=f"{g}_axle",
            min_overlap=0.05, name=f"{g} hub retained on axle",
        )

    # --- bulkhead passage door -----------------------------------------
    bdoor = object_model.get_part("bulkhead_door")
    bdoor_hinge = object_model.get_articulation("bulkhead_door_hinge")
    ctx.check("bulkhead_door_part", bdoor is not None, "missing bulkhead_door part")
    ctx.check(
        "bulkhead_door_revolute",
        bdoor_hinge is not None
        and bdoor_hinge.articulation_type == ArticulationType.REVOLUTE
        and bdoor_hinge.motion_limits is not None
        and abs(bdoor_hinge.motion_limits.lower) < 1e-9
        and bdoor_hinge.motion_limits.upper >= 1.0,
        "Bulkhead door must be a revolute that opens from flush (q=0).",
    )
    ctx.check(
        "bulkhead_door_axis_vertical",
        bdoor_hinge is not None and abs(bdoor_hinge.axis[2]) > 0.99,
        "Bulkhead door hinge must swing on a vertical (Z) axis.",
    )
    ctx.check(
        "visual_cabin_bulkhead",
        fus.get_visual("cabin_bulkhead") is not None,
        "missing cabin_bulkhead",
    )
    # The bulkhead door is an INTERNAL partition that sits entirely within the
    # fuselage envelope, so the global QC (which treats the closed hull as a
    # solid) sees the whole door volume as "inside" the hull. Allow that
    # containment and prove it: the door stays within the fuselage, and its
    # hinge knuckle seats into the bulkhead jamb (carried, not floating).
    ctx.allow_overlap(
        bdoor, fus,
        reason="Internal cockpit/cabin partition door sits wholly inside the fuselage envelope; hinge knuckle nests in the bulkhead jamb.",
    )
    ctx.expect_within(
        bdoor, fus, axes=("x", "y", "z"),
        name="bulkhead door contained within the fuselage",
    )
    ctx.expect_contact(
        bdoor, fus, elem_a="hinge_knuckle", elem_b="cabin_bulkhead",
        name="bulkhead door hinge seated in the jamb",
    )
    # The door really swings open (free edge moves forward, +X).
    closed_d = ctx.part_world_aabb(bdoor)
    if closed_d is not None and bdoor_hinge is not None:
        with ctx.pose({bdoor_hinge: bdoor_hinge.motion_limits.upper}):
            open_d = ctx.part_world_aabb(bdoor)
            ctx.check(
                "bulkhead_door_opens",
                open_d is not None and (float(open_d[1][0]) - float(closed_d[1][0])) > 0.30,
                "Opening the hinge must swing the door forward into the cockpit (+X).",
            )

    # --- joint types and limits --------------------------------------
    ctx.check(
        "rotor_joints_continuous",
        front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS,
        "Both rotors must spin on continuous joints.",
    )
    ctx.check(
        "ramp_joint_revolute",
        ramp_hinge.articulation_type == ArticulationType.REVOLUTE
        and ramp_hinge.motion_limits is not None
        and abs(ramp_hinge.motion_limits.lower) < 1e-9
        and 0.55 <= ramp_hinge.motion_limits.upper <= 0.80,
        "Ramp must be revolute, closed at q=0, opening to the ground-contact angle.",
    )
    ctx.check(
        "ramp_axis_lateral",
        abs(ramp_hinge.axis[1]) > 0.99,
        "Ramp hinge must be a horizontal lateral (Y) axis.",
    )

    # --- rotor geometry: tandem layout, heights, tilt, 18 m disks -----
    fz = front_spin.origin.xyz[2]
    rz = rear_spin.origin.xyz[2]
    ctx.check("rear_hub_height", 5.3 <= rz <= 6.0, f"rear hub z={rz:.2f}")
    ctx.check(
        "rear_hub_above_front",
        1.8 <= (rz - fz) <= 2.6,
        f"rear-front hub height delta={rz - fz:.2f}",
    )
    ctx.check(
        "tandem_hub_separation",
        11.0 <= (front_spin.origin.xyz[0] - rear_spin.origin.xyz[0]) <= 13.5,
        "Hubs must sit at opposite ends of the fuselage.",
    )
    ctx.check(
        "front_axis_tilted_forward",
        0.05 <= front_spin.origin.rpy[1] <= 0.25,
        "Front shaft should tilt slightly toward the nose.",
    )

    fr_aabb = ctx.part_world_aabb(front_rotor)
    ctx.check("front_rotor_aabb", fr_aabb is not None, "front rotor AABB missing")
    if fr_aabb is not None:
        mins, maxs = fr_aabb
        y_span = float(maxs[1] - mins[1])
        # Two blades at +/-120 deg give a chord of 2*R*sin(60 deg) ~ 15.5 m.
        ctx.check("front_rotor_disk_scale", 15.0 <= y_span <= 16.2, f"y_span={y_span:.2f}")
        ctx.check(
            "front_blade_reaches_past_nose",
            float(maxs[0]) >= 14.0,
            f"x_max={float(maxs[0]):.2f}; blade 0 points past the nose at q=0",
        )

    # Off-axis spin proof: rotating the rotor 60 deg swings a blade through
    # the -X half plane and drags the AABB with it.
    if fr_aabb is not None:
        with ctx.pose({front_spin: pi / 3.0}):
            posed = ctx.part_world_aabb(front_rotor)
            ctx.check(
                "front_rotor_spins_blades",
                posed is not None and (fr_aabb[0][0] - posed[0][0]) > 2.0,
                "Posing the continuous joint must move the off-axis blades.",
            )

    # --- overall airframe scale and grounding -------------------------
    f_aabb = ctx.part_world_aabb(fus)
    ctx.check("fuselage_aabb", f_aabb is not None, "fuselage AABB missing")
    if f_aabb is not None:
        mins, maxs = f_aabb
        length = float(maxs[0] - mins[0])
        width = float(maxs[1] - mins[1])
        ctx.check("fuselage_length", 15.5 <= length <= 16.8, f"length={length:.2f}")
        ctx.check("fuselage_width_over_sponsons", 3.5 <= width <= 4.0, f"width={width:.2f}")
        ctx.check(
            "fuselage_belly_clears_ground",
            float(mins[2]) > 0.12,
            f"belly/strut zmin={float(mins[2]):.3f} (spinning wheel parts now carry the airframe)",
        )
        ctx.check(
            "rear_pylon_height",
            5.1 + Z_LIFT <= float(maxs[2]) <= 5.6 + Z_LIFT,
            f"zmax={float(maxs[2]):.2f} (rear pylon top)",
        )

    # --- hero visuals present -----------------------------------------
    for vname in (
        "cockpit_canopy",
        "left_sponson",
        "right_sponson",
        "left_engine_nacelle",
        "left_window_3",
        "front_left_gear_strut",
        "rear_pylon",
    ):
        ctx.check(f"visual_{vname}", fus.get_visual(vname) is not None, f"missing {vname}")

    # --- landing gear: four spinning wheel parts resting on the ground --------
    wheel_zs = []
    for g in ("front_left", "front_right", "rear_left", "rear_right"):
        wp = object_model.get_part(f"{g}_wheel")
        ctx.check(f"{g}_wheel_part", wp is not None, f"missing {g}_wheel part")
        spin = object_model.get_articulation(f"{g}_wheel_spin")
        ctx.check(
            f"{g}_wheel_spins",
            spin is not None and spin.articulation_type == ArticulationType.CONTINUOUS,
            f"{g}_wheel needs a CONTINUOUS spin joint",
        )
        wa = ctx.part_world_aabb(wp)
        if wa is not None:
            wheel_zs.append(float(wa[0][2]))
    ctx.check(
        "wheels_touch_ground",
        len(wheel_zs) == 4 and max(abs(z) for z in wheel_zs) <= 0.03,
        f"wheel zmins={[round(z, 3) for z in wheel_zs]}; the four wheels must rest at z=0",
    )
    ctx.check(
        "blade_tips_marked",
        front_rotor.get_visual("blade_tip_2") is not None
        and rear_rotor.get_visual("blade_tip_0") is not None,
        "Each blade needs a yellow-green tip cap.",
    )
    ctx.check(
        "rotor_masts_present",
        front_rotor.get_visual("mast") is not None
        and rear_rotor.get_visual("mast") is not None,
        "Each rotor needs a rotating mast that carries the hub.",
    )

    # --- ramp behaviour -------------------------------------------------
    r_aabb = ctx.part_world_aabb(ramp)
    ctx.check("ramp_aabb", r_aabb is not None, "ramp AABB missing")
    if r_aabb is not None:
        ctx.check(
            "ramp_closed_hugs_tail",
            float(r_aabb[0][2]) >= 0.35,
            f"closed ramp zmin={float(r_aabb[0][2]):.2f}; must lie up on the tail",
        )
        ctx.expect_overlap(ramp, fus, axes="xy", min_overlap=0.5)
        with ctx.pose({ramp_hinge: RAMP_OPEN}):
            # part_world_aabb is conservative under chained rotations
            # (AABB-of-AABB), so check the small lip element instead of
            # the whole posed part.
            lip_aabb = ctx.part_element_world_aabb(ramp, elem="ramp_lip")
            ctx.check(
                "ramp_opens_to_ground",
                lip_aabb is not None and -0.22 <= float(lip_aabb[0][2]) <= 0.15,
                "Fully open, the ramp lip must reach ground level.",
            )
            ctx.check(
                "ramp_swings_aft_and_down",
                lip_aabb is not None and float(lip_aabb[0][0]) <= -7.2,
                "Open lip must land aft of the tail, behind the hinge.",
            )

    # Sanity: opening angle really is the ground-contact angle for this
    # hinge height and ramp length.
    q_ground = UPSWEEP_ANG + asin((RAMP_HINGE[2] - 0.09) / RAMP_LEN)
    ctx.check(
        "ramp_limit_matches_geometry",
        abs(RAMP_OPEN - q_ground) < 0.05,
        f"upper={RAMP_OPEN:.2f} vs geometric ground contact {q_ground:.2f}",
    )

    return ctx.report()


object_model = build_object_model()
