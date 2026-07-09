from __future__ import annotations

# Wall-mounted rotary-selector power switch, forked from
# picture/Equipment/Power switch/001.png.
#
# Identity (from the reference):
#   A vertical rounded-rectangular faceplate with a raised perimeter border and a
#   recessed, chamfered-corner inner field. Near the top sits a raised louver pad
#   (four horizontal vent slots) flanked by two corner mounting screws. The centre
#   now carries the requested fork-variant rotary selector: a round fixed escutcheon
#   on the plate with a skirted knob/handle and raised pointer skirt turning about
#   the panel-normal Z axis. A recessed chamfered nameplate sits at the bottom.
#
# Coordinate convention (meters):
#   X = plate width (horizontal), Y = plate height (+Y up),
#   Z = depth out of the wall toward the viewer (+Z faces the room).
#
# Articulation:
#   ONE revolute joint = the selector knob turning on the faceplate about +Z.
#   q=0 points up; positive q rotates the pointer counter-clockwise on the wall.

import cadquery as cq
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
PLATE_W = 0.090          # plate width  (X)
PLATE_H = 0.130          # plate height (Y)
PLATE_T = 0.008          # plate thickness (Z); front face at Z=PLATE_T
PLATE_CORNER = 0.012     # rounded outer corner radius

# Recessed chamfered-corner inner field (leaves a raised perimeter border).
BORDER = 0.008           # border width in from the plate edge
FIELD_CHAMFER = 0.016    # chamfer of the inner field corners (octagonal look)
FIELD_DEPTH = 0.0018     # how deep the field is recessed into the front face

# Top louver / vent pad (a raised sub-panel carrying horizontal slots).
PAD_CY = PLATE_H / 2.0 - 0.024
PAD_W = 0.052
PAD_H = 0.028
PAD_PROUD = 0.0016                   # how far the pad stands above the field
LOUVER_W = 0.040
LOUVER_SLOT_H = 0.0016
LOUVER_SLOT_GAP = 0.0024
LOUVER_COUNT = 4
LOUVER_DEPTH = 0.0014

# Top mounting screws (flanking the louver pad).
SCREW_R = 0.0034
SCREW_HEAD_Z = 0.0012
SCREW_OFF_X = PLATE_W / 2.0 - 0.011
SCREW_OFF_Y = PLATE_H / 2.0 - 0.012

# Central rotary selector hardware (fixed escutcheon plus moving skirted knob).
SELECTOR_CY = 0.000
MOUNT_R = 0.024                      # fixed circular escutcheon radius
MOUNT_PROUD = 0.0035                 # escutcheon front above plate front
MOUNT_RING_R = 0.022
MOUNT_RING_W = 0.0028
KNOB_SEAT_Z = PLATE_T + MOUNT_PROUD  # real contact plane for the rotating child

KNOB_SKIRT_R = 0.0195
KNOB_SKIRT_T = 0.0042
KNOB_CAP_R = 0.0120
KNOB_CAP_H = 0.0080
KNOB_HANDLE_W = 0.010
KNOB_HANDLE_L = 0.031
KNOB_HANDLE_H = 0.0055
POINTER_W = 0.007
POINTER_LEN = 0.020
POINTER_T = 0.0014

# Bottom recessed nameplate (chamfered rectangle).
NAME_W = 0.044
NAME_H = 0.024
NAME_CY = -PLATE_H / 2.0 + 0.028
NAME_CHAMFER = 0.006
NAME_DEPTH = 0.0016

# Selector throw: q=0 points up; positive turn is counter-clockwise on the wall.
SELECT_LOWER = -0.85
SELECT_UPPER = 0.85


# ---------------------------------------------------------------------------
# Profile helper
# ---------------------------------------------------------------------------
def _chamfer_rect(w: float, h: float, c: float) -> list[tuple[float, float]]:
    """Centered rectangle with cut (chamfered) corners -> octagon profile."""
    hx, hy = w / 2.0, h / 2.0
    return [
        (-hx + c, -hy), (hx - c, -hy), (hx, -hy + c), (hx, hy - c),
        (hx - c, hy), (-hx + c, hy), (-hx, hy - c), (-hx, -hy + c),
    ]


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
def _register_materials(model: ArticulatedObject) -> None:
    model.material("plate_metal", color=(0.60, 0.62, 0.64))
    model.material("field_metal", color=(0.54, 0.56, 0.58))
    model.material("selector_mount", color=(0.50, 0.52, 0.54))
    model.material("rivet_steel", color=(0.44, 0.46, 0.49))
    model.material("screw_zinc", color=(0.78, 0.80, 0.82))
    model.material("knob_dark", color=(0.13, 0.14, 0.15))
    model.material("pointer_light", color=(0.82, 0.83, 0.80))


# ---------------------------------------------------------------------------
# Faceplate (fixed root)
# ---------------------------------------------------------------------------
def _build_faceplate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_W, PLATE_H, PLATE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(PLATE_CORNER)
    )

    # Recessed chamfered inner field (leaves a raised perimeter border).
    field = _chamfer_rect(PLATE_W - 2.0 * BORDER, PLATE_H - 2.0 * BORDER, FIELD_CHAMFER)
    pocket = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_T - FIELD_DEPTH)
        .polyline(field)
        .close()
        .extrude(FIELD_DEPTH + 0.001)
    )
    plate = plate.cut(pocket)

    # Bottom recessed nameplate (chamfered rectangle), cut deeper into the field,
    # leaving a raised inner border by cutting a smaller inner pocket deeper still.
    name = _chamfer_rect(NAME_W, NAME_H, NAME_CHAMFER)
    name_cut = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_T - FIELD_DEPTH - NAME_DEPTH)
        .center(0.0, NAME_CY)
        .polyline(name)
        .close()
        .extrude(NAME_DEPTH + 0.001)
    )
    plate = plate.cut(name_cut)
    inner = _chamfer_rect(NAME_W - 0.010, NAME_H - 0.008, NAME_CHAMFER - 0.002)
    inner_cut = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_T - FIELD_DEPTH - NAME_DEPTH - 0.0012)
        .center(0.0, NAME_CY)
        .polyline(inner)
        .close()
        .extrude(0.0014)
    )
    plate = plate.cut(inner_cut)

    # Four corner rivets on the nameplate border.
    name_face_z = PLATE_T - FIELD_DEPTH - NAME_DEPTH
    for sx in (-1, 1):
        for sy in (-1, 1):
            rivet = (
                cq.Workplane("XY")
                .workplane(offset=name_face_z - 0.0004)
                .center(sx * (NAME_W / 2.0 - 0.004), NAME_CY + sy * (NAME_H / 2.0 - 0.004))
                .circle(0.0015)
                .extrude(0.0008)
            )
            plate = plate.union(rivet)
    return plate


def _build_louver_pad() -> cq.Workplane:
    field_front = PLATE_T - FIELD_DEPTH
    pad = (
        cq.Workplane("XY")
        .workplane(offset=field_front)
        .center(0.0, PAD_CY)
        .box(PAD_W, PAD_H, FIELD_DEPTH + PAD_PROUD, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    pad_front = field_front + FIELD_DEPTH + PAD_PROUD
    band = LOUVER_COUNT * LOUVER_SLOT_H + (LOUVER_COUNT - 1) * LOUVER_SLOT_GAP
    y0 = PAD_CY + band / 2.0 - LOUVER_SLOT_H / 2.0
    for i in range(LOUVER_COUNT):
        cy = y0 - i * (LOUVER_SLOT_H + LOUVER_SLOT_GAP)
        slot = (
            cq.Workplane("XY")
            .workplane(offset=pad_front - LOUVER_DEPTH)
            .center(0.0, cy)
            .box(LOUVER_W, LOUVER_SLOT_H, LOUVER_DEPTH + 0.001, centered=(True, True, False))
        )
        pad = pad.cut(slot)
    return pad


def _build_rotary_mount() -> cq.Workplane:
    """Fixed circular escutcheon on the recessed field, ending at KNOB_SEAT_Z."""
    field_front = PLATE_T - FIELD_DEPTH
    base = (
        cq.Workplane("XY")
        .workplane(offset=field_front)
        .center(0.0, SELECTOR_CY)
        .circle(MOUNT_R)
        .extrude(FIELD_DEPTH + MOUNT_PROUD)
    )

    # Raised ring detail around the seated rotary skirt. It sits on the same
    # front plane as the knob seat so the child is visibly mounted, not floating.
    ring_outer = (
        cq.Workplane("XY")
        .workplane(offset=KNOB_SEAT_Z - 0.0008)
        .center(0.0, SELECTOR_CY)
        .circle(MOUNT_RING_R)
        .extrude(0.0008)
    )
    ring_inner = (
        cq.Workplane("XY")
        .workplane(offset=KNOB_SEAT_Z - 0.0010)
        .center(0.0, SELECTOR_CY)
        .circle(MOUNT_RING_R - MOUNT_RING_W)
        .extrude(0.0012)
    )
    base = base.union(ring_outer.cut(ring_inner))

    # Four small visible escutcheon screws/rivets, emitted with regular angular
    # placement. They are outside the moving knob skirt and overlap the mount face.
    screw_r = 0.00155
    screw_circle = MOUNT_R - 0.0020
    for i in range(4):
        angle = 0.7853981633974483 + i * 1.5707963267948966
        sx = screw_circle * math.cos(angle)
        sy = SELECTOR_CY + screw_circle * math.sin(angle)
        screw = (
            cq.Workplane("XY")
            .workplane(offset=KNOB_SEAT_Z - 0.00065)
            .center(sx, sy)
            .circle(screw_r)
            .extrude(0.00055)
        )
        base = base.union(screw)
    return base


def _build_screws() -> cq.Workplane:
    screws = None
    for sx in (-1, 1):
        head = (
            cq.Workplane("XY")
            .workplane(offset=PLATE_T - 0.0006)
            .center(sx * SCREW_OFF_X, SCREW_OFF_Y)
            .circle(SCREW_R)
            .extrude(SCREW_HEAD_Z + 0.0006)
        )
        slot_a = (
            cq.Workplane("XY")
            .workplane(offset=PLATE_T + SCREW_HEAD_Z - 0.0004)
            .center(sx * SCREW_OFF_X, SCREW_OFF_Y)
            .box(SCREW_R * 1.6, 0.0008, 0.001, centered=(True, True, False))
        )
        head = head.cut(slot_a)
        screws = head if screws is None else screws.union(head)
    return screws


# ---------------------------------------------------------------------------
# Rotary selector knob (moving child, authored in the contact-surface frame)
# ---------------------------------------------------------------------------
def _build_knob_body() -> cq.Workplane:
    # Local frame: origin is the real seating plane on the front of the escutcheon.
    # All geometry extends in +Z toward the viewer, so the joint origin is exactly
    # at the visible parent/child contact face.
    skirt = (
        cq.Workplane("XY")
        .circle(KNOB_SKIRT_R)
        .extrude(KNOB_SKIRT_T)
    )
    cap = (
        cq.Workplane("XY")
        .workplane(offset=KNOB_SKIRT_T - 0.00025)
        .circle(KNOB_CAP_R)
        .extrude(KNOB_CAP_H + 0.00025)
    )
    # Rounded raised handle, aligned with the pointer direction at q=0.
    handle = (
        cq.Workplane("XY")
        .workplane(offset=KNOB_SKIRT_T + KNOB_CAP_H - 0.00035)
        .center(0.0, KNOB_HANDLE_L / 2.0 - 0.004)
        .box(KNOB_HANDLE_W, KNOB_HANDLE_L, KNOB_HANDLE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0020)
    )
    return skirt.union(cap).union(handle)


def _build_pointer_skirt() -> cq.Workplane:
    # Light raised triangular pointer riding on the dark selector handle/skirt.
    y_base = 0.000
    y_tip = POINTER_LEN
    pts = [
        (-POINTER_W / 2.0, y_base),
        (POINTER_W / 2.0, y_base),
        (0.0, y_tip),
    ]
    return (
        cq.Workplane("XY")
        .workplane(offset=KNOB_SKIRT_T + KNOB_CAP_H + KNOB_HANDLE_H - 0.00045)
        .polyline(pts)
        .close()
        .extrude(POINTER_T)
    )


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_rotary_power_switch")
    _register_materials(model)

    faceplate = model.part("faceplate")
    faceplate.visual(
        mesh_from_cadquery(_build_faceplate(), "faceplate_shell"),
        origin=Origin(),
        material="plate_metal",
        name="faceplate_shell",
    )
    faceplate.visual(
        mesh_from_cadquery(_build_louver_pad(), "louver_pad"),
        origin=Origin(),
        material="field_metal",
        name="louver_pad",
    )
    faceplate.visual(
        mesh_from_cadquery(_build_rotary_mount(), "rotary_mount"),
        origin=Origin(),
        material="selector_mount",
        name="rotary_mount",
    )
    faceplate.visual(
        mesh_from_cadquery(_build_screws(), "mount_screws"),
        origin=Origin(),
        material="screw_zinc",
        name="mount_screws",
    )

    selector = model.part("selector_knob")
    selector.visual(
        mesh_from_cadquery(_build_knob_body(), "knob_skirt"),
        origin=Origin(),
        material="knob_dark",
        name="knob_skirt",
    )
    selector.visual(
        mesh_from_cadquery(_build_pointer_skirt(), "pointer_skirt"),
        origin=Origin(),
        material="pointer_light",
        name="pointer_skirt",
    )

    model.articulation(
        "plate_to_selector",
        ArticulationType.REVOLUTE,
        parent=faceplate,
        child=selector,
        origin=Origin(xyz=(0.0, SELECTOR_CY, KNOB_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=3.0, lower=SELECT_LOWER, upper=SELECT_UPPER
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    faceplate = object_model.get_part("faceplate")
    selector = object_model.get_part("selector_knob")
    joint = object_model.get_articulation("plate_to_selector")

    ctx.check(
        "faceplate is the single root",
        faceplate in object_model.root_parts() and len(object_model.root_parts()) == 1,
        details=f"roots={[p.name for p in object_model.root_parts()]}",
    )
    ctx.check(
        "selector is rotary child",
        joint.child == "selector_knob" and joint.parent == "faceplate",
        details=f"parent={joint.parent}, child={joint.child}",
    )
    ctx.check(
        "joint type is revolute",
        str(joint.articulation_type).lower().endswith("revolute"),
        details=f"type={joint.articulation_type}",
    )
    ax = tuple(float(a) for a in joint.axis)
    ctx.check(
        "selector axis is panel-normal Z",
        abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6 and abs(ax[2]) > 0.99,
        details=f"axis={ax}",
    )

    # Faceplate reads as a vertical wall plate.
    plate_aabb = ctx.part_element_world_aabb(faceplate, elem="faceplate_shell")
    ctx.check("faceplate aabb present", plate_aabb is not None, details=str(plate_aabb))
    if plate_aabb is not None:
        pmin, pmax = plate_aabb
        pw = float(pmax[0] - pmin[0])
        ph = float(pmax[1] - pmin[1])
        pt = float(pmax[2] - pmin[2])
        ctx.check("plate width matches", abs(pw - PLATE_W) < 0.002, details=f"w={pw}")
        ctx.check("plate height matches", abs(ph - PLATE_H) < 0.002, details=f"h={ph}")
        ctx.check("plate is taller than wide", ph > pw + 0.02, details=f"w={pw}, h={ph}")
        ctx.check("plate is thin slab", pt < 0.013, details=f"t={pt}")

    # The fixed circular escutcheon and the dark selector knob sit on the plate.
    mount_aabb = ctx.part_element_world_aabb(faceplate, elem="rotary_mount")
    knob_aabb = ctx.part_element_world_aabb(selector, elem="knob_skirt")
    pointer_aabb = ctx.part_element_world_aabb(selector, elem="pointer_skirt")
    ctx.check("rotary mount present", mount_aabb is not None, details=str(mount_aabb))
    ctx.check("knob skirt present", knob_aabb is not None, details=str(knob_aabb))
    ctx.check("pointer skirt present", pointer_aabb is not None, details=str(pointer_aabb))
    if mount_aabb is not None:
        mmin, mmax = mount_aabb
        mdx = float(mmax[0] - mmin[0])
        mdy = float(mmax[1] - mmin[1])
        ctx.check(
            "escutcheon reads circular",
            abs(mdx - mdy) < 0.003 and mdx > 0.040,
            details=f"dx={mdx}, dy={mdy}",
        )
    if knob_aabb is not None:
        kmin, kmax = knob_aabb
        kdx = float(kmax[0] - kmin[0])
        kdy = float(kmax[1] - kmin[1])
        ctx.check(
            "knob stands proud of selector mount",
            float(kmax[2]) > KNOB_SEAT_Z + 0.010,
            details=f"knob_front_z={float(kmax[2])}, seat_z={KNOB_SEAT_Z}",
        )
        ctx.check(
            "knob is skirted round selector",
            abs(kdx - 2.0 * KNOB_SKIRT_R) < 0.004 and kdy > kdx + 0.006,
            details=f"dx={kdx}, dy={kdy}",
        )

    # The selector is mounted on the plate footprint and seated on the real visible
    # escutcheon face; the gap check proves it is not floating off the faceplate.
    ctx.expect_overlap(
        selector,
        faceplate,
        axes="xy",
        elem_a="knob_skirt",
        elem_b="rotary_mount",
        min_overlap=0.010,
        name="selector overlaps escutcheon footprint",
    )
    ctx.expect_gap(
        selector,
        faceplate,
        axis="z",
        positive_elem="knob_skirt",
        negative_elem="rotary_mount",
        max_gap=0.001,
        max_penetration=0.0,
        name="selector skirt seats on escutcheon face",
    )

    # Mechanism: turning the selector rotates the pointer about the Z axis on the
    # wall faceplate. Track the pointer's AABB at two poses -- a no-op joint cannot
    # fake the lateral sweep.
    def _pointer():
        box = ctx.part_element_world_aabb(selector, elem="pointer_skirt")
        if box is None:
            return None
        mn, mx = box
        return (float((mn[0] + mx[0]) / 2.0), float((mn[1] + mx[1]) / 2.0))

    rest = _pointer()
    ctx.check("pointer present", rest is not None, details=str(rest))

    with ctx.pose({joint: SELECT_UPPER}):
        turned_x, turned_y = _pointer()
    with ctx.pose({joint: 0.0}):
        up_x, up_y = _pointer()

    swept = abs(up_x - turned_x) > 0.006 or abs(up_y - turned_y) > 0.006
    ctx.check(
        "turning selector actually sweeps pointer",
        swept,
        details=f"pointer center rest=({up_x}, {up_y}) turned=({turned_x}, {turned_y})",
    )
    ctx.check(
        "positive turn rotates pointer counter-clockwise",
        turned_x < up_x - 0.006 and turned_y < up_y,
        details=f"pointer center rest=({up_x}, {up_y}) turned=({turned_x}, {turned_y})",
    )

    return ctx.report()


object_model = build_object_model()
