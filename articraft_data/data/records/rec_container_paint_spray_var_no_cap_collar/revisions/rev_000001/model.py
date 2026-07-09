from __future__ import annotations

# Aerosol SPRAY PAINT CAN — capless safety-collar variant.
# Frame: can axis along +Z. Base rim sits at z=0, can body rises in +Z, the
# crimped dome and spray nozzle are at the top. ~0.20 m tall, ~0.065 m dia.
# NO overcap/dust cap. A low retaining collar sits below the nozzle button
# with grip ridges and a sliding spray-lock tab. The tab slides under the
# button lip to block downward travel without intersecting the button mesh.
# Articulations (two INDEPENDENT prismatic joints):
#   - spray nozzle button: PRISMATIC press straight DOWN (-Z, ~0.004 m).
#   - lock tab: PRISMATIC slide laterally (-Y, ~0.014 m) to lock/unlock button.

import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
CAN_R = 0.0325            # body radius (~0.065 m diameter)
BODY_BOTTOM = 0.0         # base plane
BODY_TOP = 0.150          # where the straight wall ends and the dome begins
DOME_TOP = 0.176          # top of the crimped dome (mounting cup rim)
VALVE_TOP = 0.182         # top of the central valve stem the nozzle seats on
NOZZLE_SEAT_Z = 0.176     # z where the nozzle button base sits
NOZZLE_TOP_Z = 0.196      # top of the nozzle button

# Collar dimensions
COLLAR_INNER_R = 0.0145   # inner radius clears the valve boss and spray button
COLLAR_OUTER_R = 0.0190   # outer guide ring for the safety slider
COLLAR_HEIGHT = 0.0055    # collar ring thickness
COLLAR_BASE_Z = DOME_TOP - 0.0090  # low enough to stay below the button mesh

# Lock tab dimensions
TAB_WIDTH = 0.011         # along X, broad enough to read as a real lock
TAB_LENGTH = 0.014        # along Y (slide direction)
TAB_HEIGHT = 0.0023       # thin shelf below nozzle bottom
TAB_SLIDE = 0.0165        # prismatic travel, stops before crossing center
# Tab rests on the collar's +Y guide mouth; slides toward -Y to lock under
# the nozzle lip. At full travel it enters the nozzle XY footprint while
# remaining below the nozzle's z-min.
TAB_ORIGIN_Y = COLLAR_OUTER_R + TAB_LENGTH / 2.0 + 0.002
TAB_TOP_Z = NOZZLE_SEAT_Z - 0.0005
TAB_BASE_Z = TAB_TOP_Z - TAB_HEIGHT


# ---- shared geometry helpers ----

def _can_body_solid() -> cq.Workplane:
    """Tall cylindrical can: bottom rim, straight wall, crimped shoulder dome,
    and central valve stem boss."""
    # Bottom rim (slightly larger ring for the rolled seam).
    body = (
        cq.Workplane("XY")
        .circle(CAN_R + 0.0012)
        .extrude(0.006)
    )
    # Main straight wall.
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .circle(CAN_R)
        .extrude(BODY_TOP - 0.005)
    )
    # Crimped top dome: loft from the wall up and inward to the mounting cup rim.
    dome = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .circle(CAN_R)
        .workplane(offset=0.006)
        .circle(CAN_R - 0.001)
        .workplane(offset=0.004)
        .circle(CAN_R - 0.006)
        .workplane(offset=0.017)
        .circle(0.014)
        .loft(ruled=False)
    )
    body = body.union(dome)
    # Central valve stem boss the nozzle seats over.
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=DOME_TOP - 0.001)
        .circle(0.006)
        .extrude(VALVE_TOP - DOME_TOP)
    )
    return body


def _nozzle_solid() -> cq.Workplane:
    """Press-down spray button with a spray hole on the front face."""
    btn = (
        cq.Workplane("XY")
        .box(0.018, 0.016, 0.014, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
    )
    # Tiny spray hole on the +X front face.
    hole = (
        cq.Workplane("YZ")
        .workplane(offset=0.009)
        .center(0.0, 0.008)
        .circle(0.0014)
        .extrude(-0.010)
    )
    btn = btn.cut(hole)
    return btn


def _collar_ring_solid() -> cq.Workplane:
    """Low retaining collar annulus with a shallow slider mouth on +Y."""
    ring = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_R)
        .circle(COLLAR_INNER_R)
        .extrude(COLLAR_HEIGHT)
    )
    # A flat guide block on the +Y side makes the lock path explicit while
    # staying below the nozzle button.
    guide = (
        cq.Workplane("XY")
        .center(0, COLLAR_OUTER_R + 0.003)
        .box(TAB_WIDTH + 0.005, 0.012, COLLAR_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.001)
    )
    ring = ring.union(guide)
    return ring


def _grip_ridge_solid() -> cq.Workplane:
    """Small raised bump on the collar outer surface for finger grip.
    Width (tangential) x depth (radial) x height."""
    ridge = (
        cq.Workplane("XY")
        .box(0.003, 0.002, COLLAR_HEIGHT * 0.85, centered=(True, True, False))
    )
    return ridge


def _lock_tab_solid() -> cq.Workplane:
    """Sliding lock tab: a low shelf plus an outside thumb nub.

    The shelf enters below the nozzle button at the locked pose. The raised
    thumb nub stays on the outside of the collar so it never collides with the
    nozzle button even when the shelf is under the button footprint.
    """
    tab = (
        cq.Workplane("XY")
        .box(TAB_WIDTH, TAB_LENGTH, TAB_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.001)
    )
    # Grip nub on the outside half only.
    nub = (
        cq.Workplane("XY")
        .workplane(offset=TAB_HEIGHT - 0.0003)
        .center(0, TAB_LENGTH * 0.30)
        .box(TAB_WIDTH * 0.65, TAB_LENGTH * 0.28, 0.002, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0008)
    )
    tab = tab.union(nub)
    # Capture tongue runs in the guide/collar groove below the shelf.
    tongue = (
        cq.Workplane("XY")
        .workplane(offset=-0.0025)
        .center(0, -TAB_LENGTH * 0.18)
        .box(TAB_WIDTH * 0.55, TAB_LENGTH * 0.55, 0.0025, centered=(True, True, False))
    )
    tab = tab.union(tongue)
    return tab


def _place_grip_ridge(index: int, total: int) -> cq.Workplane:
    """Place a grip ridge at the correct angular position on the collar."""
    angle = 2.0 * math.pi * index / total
    # Ridge center on the collar outer surface, slightly embedded for contact.
    r_center = COLLAR_OUTER_R + 0.0005
    gx = r_center * math.cos(angle)
    gy = r_center * math.sin(angle)
    ridge = _grip_ridge_solid()
    ridge = ridge.rotate((0, 0, 0), (0, 0, 1), math.degrees(angle))
    ridge = ridge.translate((gx, gy, COLLAR_BASE_Z))
    return ridge


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spray_paint_can")

    label = model.material("label_splatter", rgba=(0.90, 0.90, 0.92, 1.0))
    metal = model.material("metal", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_collar = model.material("dark_collar", rgba=(0.22, 0.22, 0.23, 1.0))
    dark_nozzle = model.material("dark_nozzle", rgba=(0.16, 0.16, 0.17, 1.0))
    red_tab = model.material("red_lock_tab", rgba=(0.80, 0.15, 0.12, 1.0))

    # ---- can body (root) ----
    body = model.part("can_body")
    body.visual(
        mesh_from_cadquery(_can_body_solid(), "can_body"),
        material=metal,
        name="can_body",
    )
    # Splatter-graphic label band wrapped on the straight wall.
    label_band = CylinderGeometry(CAN_R + 0.0004, 0.105, radial_segments=64)
    label_band.translate(0.0, 0.0, 0.012 + 0.105 / 2.0)
    body.visual(mesh_from_geometry(label_band, "label_band"), material=label, name="label_band")

    # Collar ring: inlined as parent visual (non-moving decoration on body).
    collar_ring = _collar_ring_solid().translate((0, 0, COLLAR_BASE_Z))
    body.visual(
        mesh_from_cadquery(collar_ring, "collar_ring"),
        material=dark_collar,
        name="collar_ring",
    )

    # Grip ridges around the collar: repeated sub-parts via for-i-in-range loop.
    n_grips = 6
    for i in range(n_grips):
        ridge = _place_grip_ridge(i, n_grips)
        body.visual(
            mesh_from_cadquery(ridge, f"grip_ridge_{i}"),
            material=dark_collar,
            name=f"grip_ridge_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Cylinder(CAN_R, 0.176),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
    )

    # ---- spray nozzle button: child of the can body, presses straight down ----
    nozzle = model.part("spray_nozzle")
    nozzle.visual(
        mesh_from_cadquery(_nozzle_solid(), "spray_nozzle"),
        material=dark_nozzle,
        name="spray_nozzle",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.018, 0.016, 0.014)),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )
    model.articulation(
        "nozzle_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=nozzle,
        origin=Origin(xyz=(0.0, 0.0, NOZZLE_SEAT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.05, lower=0.0, upper=0.004),
    )

    # ---- lock tab: child of the can body, slides laterally to lock/unlock ----
    lock_tab = model.part("lock_tab")
    lock_tab.visual(
        mesh_from_cadquery(_lock_tab_solid(), "lock_tab"),
        material=red_tab,
        name="lock_tab",
    )
    lock_tab.inertial = Inertial.from_geometry(
        Box((TAB_WIDTH, TAB_LENGTH, TAB_HEIGHT + 0.002)),
        mass=0.002,
        origin=Origin(xyz=(0.0, 0.0, (TAB_HEIGHT + 0.002) / 2.0)),
    )
    # Joint origin at the collar top surface where the tab contacts.
    # Positive q slides the tab toward -Y (inward) to lock the nozzle.
    model.articulation(
        "tab_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lock_tab,
        origin=Origin(xyz=(0.0, TAB_ORIGIN_Y, TAB_BASE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=TAB_SLIDE),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("can_body")
    nozzle = object_model.get_part("spray_nozzle")
    lock_tab = object_model.get_part("lock_tab")
    nozzle_press = object_model.get_articulation("nozzle_press")
    tab_slide = object_model.get_articulation("tab_slide")

    # --- can is a tall cylinder ---
    bmn, bmx = ctx.part_world_aabb(body)
    height = bmx[2] - bmn[2]
    dia_x = bmx[0] - bmn[0]
    dia_y = bmx[1] - bmn[1]
    ctx.check(
        "can body is a tall cylinder (~0.18 m, ~0.065 m dia)",
        height > 0.16 and 0.05 < dia_x < 0.08 and 0.05 < dia_y < 0.08 and height > dia_x * 2.0,
        details=f"height={height:.3f}, dia_x={dia_x:.3f}, dia_y={dia_y:.3f}",
    )

    # --- nozzle sits on the can top, near the central axis ---
    npos = ctx.part_world_position(nozzle)
    ctx.check(
        "nozzle button is mounted on the can top",
        npos is not None and npos[2] > 0.16 and abs(npos[0]) < 0.01 and abs(npos[1]) < 0.01,
        details=f"nozzle origin={npos}",
    )

    # --- nozzle seats on the valve stem (intentional local overlap) ---
    ctx.allow_overlap(
        nozzle,
        body,
        elem_a="spray_nozzle",
        elem_b="can_body",
        reason="The spray button is intentionally seated over the central valve stem.",
    )
    ctx.expect_contact(nozzle, body, name="nozzle seated on the valve stem")

    # --- nozzle presses straight DOWN ---
    nz_rest = ctx.part_world_position(nozzle)[2]
    with ctx.pose({nozzle_press: 0.004}):
        nz_down = ctx.part_world_position(nozzle)[2]
    ctx.check(
        "nozzle button presses straight down",
        nz_down < nz_rest - 0.0035,
        details=f"rest_z={nz_rest:.4f}, pressed_z={nz_down:.4f}",
    )

    # --- no dust cap part exists (capless design) ---
    part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no dust cap part (capless safety-collar design)",
        "dust_cap" not in part_names,
        details=f"parts: {part_names}",
    )

    # --- collar ring is present on can body (inline visual) ---
    body_visual_names = [v.name for v in body.visuals]
    ctx.check(
        "collar ring is present as inline visual on can body",
        "collar_ring" in body_visual_names,
        details=f"body visuals: {body_visual_names}",
    )

    # --- 6 grip ridges are present (repeated sub-parts) ---
    grip_names = [f"grip_ridge_{i}" for i in range(6)]
    found_grips = [n for n in grip_names if n in body_visual_names]
    ctx.check(
        "6 grip ridges present on collar",
        len(found_grips) == 6,
        details=f"found {len(found_grips)} of 6 grip ridges",
    )

    # --- lock tab sits at the collar level on the can top ---
    tab_pos = ctx.part_world_position(lock_tab)
    ctx.check(
        "lock tab is mounted at the collar level",
        tab_pos is not None
        and tab_pos[2] > COLLAR_BASE_Z - 0.002
        and tab_pos[2] < COLLAR_BASE_Z + COLLAR_HEIGHT + 0.010,
        details=f"tab z={tab_pos[2]:.4f}",
    )

    # --- lock tab capture tongue is embedded in the collar (intentional overlap) ---
    ctx.allow_overlap(
        lock_tab,
        body,
        elem_a="lock_tab",
        elem_b="collar_ring",
        reason="Lock tab capture tongue intentionally slides inside the collar annulus groove.",
    )
    ctx.expect_contact(
        lock_tab, body,
        elem_a="lock_tab",
        elem_b="collar_ring",
        name="lock tab capture tongue contacts the collar ring",
    )

    # --- lock tab slides laterally (not vertically) ---
    tab_rest_pos = ctx.part_world_position(lock_tab)
    with ctx.pose({tab_slide: TAB_SLIDE}):
        tab_locked_pos = ctx.part_world_position(lock_tab)
    ctx.check(
        "lock tab slides laterally (Y axis, not Z)",
        abs(tab_locked_pos[2] - tab_rest_pos[2]) < 0.001
        and abs(tab_locked_pos[1] - tab_rest_pos[1]) > 0.010,
        details=f"rest=({tab_rest_pos[0]:.4f},{tab_rest_pos[1]:.4f},{tab_rest_pos[2]:.4f}), "
                f"locked=({tab_locked_pos[0]:.4f},{tab_locked_pos[1]:.4f},{tab_locked_pos[2]:.4f})",
    )

    # --- at rest (unlocked), tab clears the nozzle XY footprint ---
    with ctx.pose({tab_slide: 0.0}):
        nmn, nmx = ctx.part_world_aabb(nozzle)
        tmn, tmx = ctx.part_world_aabb(lock_tab)
    overlap_x = min(nmx[0], tmx[0]) - max(nmn[0], tmn[0])
    overlap_y = min(nmx[1], tmx[1]) - max(nmn[1], tmn[1])
    no_xy_overlap = overlap_x <= 0 or overlap_y <= 0
    ctx.check(
        "unlocked tab clears the nozzle XY footprint",
        no_xy_overlap,
        details=f"overlap_x={overlap_x:.4f}, overlap_y={overlap_y:.4f}",
    )

    # --- at locked pose, tab enters under the nozzle footprint without 3-D collision ---
    ctx.allow_overlap(
        lock_tab,
        body,
        elem_a="lock_tab",
        elem_b="collar_ring",
        reason="Lock tab capture tongue remains inside the collar guide at full lock travel.",
    )
    with ctx.pose({tab_slide: TAB_SLIDE}):
        nmn, nmx = ctx.part_world_aabb(nozzle)
        tmn, tmx = ctx.part_world_aabb(lock_tab)
        overlap_x = min(nmx[0], tmx[0]) - max(nmn[0], tmn[0])
        overlap_y = min(nmx[1], tmx[1]) - max(nmn[1], tmn[1])
        z_clear = TAB_TOP_Z <= nmn[2] + 0.0008
        ctx.check(
            "locked tab sits below the nozzle button, not through it",
            overlap_x > 0.003 and overlap_y > 0.003 and z_clear,
            details=f"xy_overlap=({overlap_x:.4f},{overlap_y:.4f}), "
                    f"shelf_top_z={TAB_TOP_Z:.4f}, nozzle_z_min={nmn[2]:.4f}",
        )
        ctx.expect_overlap(
            lock_tab, nozzle,
            axes="xy",
            elem_a="lock_tab",
            elem_b="spray_nozzle",
            min_overlap=0.003,
            name="locked tab overlaps nozzle XY footprint (blocks button press)",
        )

    # --- lock tab has at least one non-fixed joint ---
    ctx.check(
        "lock tab has a non-fixed articulation",
        tab_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"tab_slide type={tab_slide.articulation_type}",
    )

    return ctx.report()
