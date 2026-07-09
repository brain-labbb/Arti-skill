from __future__ import annotations

# Wall-mounted toggle / draw-latch power switch, modeled from
# picture/Equipment/Power switch/001.png.
#
# Identity (from the reference):
#   A vertical rounded-rectangular faceplate with a raised perimeter border and a
#   recessed, chamfered-corner inner field. Near the top sits a raised louver pad
#   (four horizontal vent slots) flanked by two corner mounting screws. The centre
#   carries two side-by-side draw-latch / over-centre toggle units: each has a
#   horizontal cylindrical ROLLER bar held by two side arms that pivot on two
#   side bolts, swinging independently over its own fixed keeper block. A
#   recessed chamfered nameplate sits at the bottom of the shared plate.
#
# Coordinate convention (meters):
#   X = plate width (horizontal), Y = plate height (+Y up),
#   Z = depth out of the wall toward the viewer (+Z faces the room).
#
# Articulation:
#   TWO revolute joints = one roller bail unit each (roller + two arms + pivot
#   hubs) pivoting about that unit's horizontal side-bolt axis (X). q=0 is the
#   raised/latched pose shown in the reference; positive q throws that unit's
#   roller down and forward to release while the other unit can remain latched.

import cadquery as cq

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
PLATE_W = 0.154          # wider two-gang plate width  (X)
PLATE_H = 0.130          # plate height (Y)
PLATE_T = 0.008          # plate thickness (Z); front face at Z=PLATE_T
PLATE_CORNER = 0.012     # rounded outer corner radius

# Recessed chamfered-corner inner field (leaves a raised perimeter border).
BORDER = 0.008           # border width in from the plate edge
FIELD_CHAMFER = 0.016    # chamfer of the inner field corners (octagonal look)
FIELD_DEPTH = 0.0018     # how deep the field is recessed into the front face

# Top louver / vent pad (a raised sub-panel carrying horizontal slots).
PAD_CY = PLATE_H / 2.0 - 0.024
PAD_W = 0.086
PAD_H = 0.028
PAD_PROUD = 0.0016                   # how far the pad stands above the field
LOUVER_W = 0.072
LOUVER_SLOT_H = 0.0016
LOUVER_SLOT_GAP = 0.0024
LOUVER_COUNT = 4
LOUVER_DEPTH = 0.0014

# Top mounting screws (flanking the louver pad).
SCREW_R = 0.0034
SCREW_HEAD_Z = 0.0012
SCREW_OFF_X = PLATE_W / 2.0 - 0.011
SCREW_OFF_Y = PLATE_H / 2.0 - 0.012

# Side-by-side bail units on the shared plate.
UNIT_COUNT = 2
UNIT_PITCH = 0.066       # equal X pitch between the two independent units

# Central keeper block for each unit (fixed raised block the bail latches over).
KEEP_W = 0.030
KEEP_H = 0.032
KEEP_CY = 0.000
KEEP_PROUD = 0.007                   # stands proud of the field front

# Pivot (side-bolt) axis: horizontal X line just above the keeper centre.
PIVOT_Y = 0.004
PIVOT_Z = PLATE_T + 0.006
HUB_R = 0.0058                       # pivot-hub / bolt-head radius
HUB_X = 0.022                        # |X| of each arm / hub
HUB_LEN = 0.006                      # hub thickness along X

# Roller bail (moving): two arms rising to a horizontal roller bar.
ARM_RISE = 0.024                     # roller centre sits this far above the pivot
ARM_W = 0.006                        # arm width  (X)
ARM_T = 0.006                        # arm thickness (Z)
ROLLER_R = 0.0052
ROLLER_LEN = 0.052                   # spans past both arms

# Bottom recessed nameplate (chamfered rectangle).
NAME_W = 0.082
NAME_H = 0.024
NAME_CY = -PLATE_H / 2.0 + 0.028
NAME_CHAMFER = 0.006
NAME_DEPTH = 0.0016

# Roller throw: q=0 raised/latched; swing down/forward to release.
ROCK_LOWER = -0.20
ROCK_UPPER = 1.30


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


def _unit_x(i: int) -> float:
    """Equal-pitch X center for a side-by-side bail unit."""
    return (i - (UNIT_COUNT - 1) / 2.0) * UNIT_PITCH


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
def _register_materials(model: ArticulatedObject) -> None:
    model.material("plate_metal", color=(0.60, 0.62, 0.64))
    model.material("field_metal", color=(0.54, 0.56, 0.58))
    model.material("keeper_steel", color=(0.56, 0.58, 0.61))
    model.material("rivet_steel", color=(0.44, 0.46, 0.49))
    model.material("screw_zinc", color=(0.78, 0.80, 0.82))
    model.material("arm_metal", color=(0.60, 0.62, 0.65))
    model.material("roller_dark", color=(0.16, 0.17, 0.19))
    model.material("bolt_steel", color=(0.46, 0.48, 0.51))


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


def _build_keeper(unit_x: float = 0.0) -> cq.Workplane:
    field_front = PLATE_T - FIELD_DEPTH
    keep = (
        cq.Workplane("XY")
        .workplane(offset=field_front)
        .center(unit_x, KEEP_CY)
        .box(KEEP_W, KEEP_H, FIELD_DEPTH + KEEP_PROUD, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    # Two small rivets on the keeper face (above and below the bail pivot).
    keep_front = field_front + FIELD_DEPTH + KEEP_PROUD
    for sy in (-1, 1):
        rivet = (
            cq.Workplane("XY")
            .workplane(offset=keep_front - 0.0004)
            .center(unit_x, KEEP_CY + sy * 0.010)
            .circle(0.0020)
            .extrude(0.0008)
        )
        keep = keep.union(rivet)
    return keep


def _build_side_bolts(unit_x: float = 0.0) -> cq.Workplane:
    # Two fixed bolt heads on the pivot axis, just outboard of the bail arms.
    bolts = None
    for sx in (-1, 1):
        bx = unit_x + sx * (HUB_X + 0.004)
        bolt = (
            cq.Workplane("YZ")
            .workplane(offset=bx - HUB_LEN / 2.0)
            .center(PIVOT_Y, PIVOT_Z)
            .circle(0.0046)
            .extrude(HUB_LEN)
        )
        # Stub embedding the bolt into the keeper so it is not a floating island.
        stub_start_x = unit_x + sx * (KEEP_W / 2.0 - 0.002)
        stub = (
            cq.Workplane("YZ")
            .workplane(offset=stub_start_x)
            .center(PIVOT_Y, PIVOT_Z)
            .circle(0.0030)
            .extrude(bx - stub_start_x)
        )
        part = bolt.union(stub)
        bolts = part if bolts is None else bolts.union(part)
    return bolts


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
# Roller bail (moving child, authored in the pivot frame)
# ---------------------------------------------------------------------------
def _build_bail_arms() -> cq.Workplane:
    # Local frame: origin on the pivot axis (x=0). Local +Y up, +Z toward viewer.
    # Two pivot hubs + two arms rising to meet the roller (a stub at the top wraps
    # the roller so the arms and roller bond into one rigid bail).
    asm = None
    for sx in (-1, 1):
        cx = sx * HUB_X
        # Pivot hub (reads as the bail eye wrapping the side bolt).
        hub = (
            cq.Workplane("YZ")
            .workplane(offset=cx - HUB_LEN / 2.0)
            .center(0.0, 0.0)
            .circle(HUB_R)
            .extrude(HUB_LEN)
        )
        # Arm rising from the hub up to the roller.
        arm = (
            cq.Workplane("XY")
            .center(cx, ARM_RISE / 2.0)
            .box(ARM_W, ARM_RISE, ARM_T, centered=(True, True, True))
            .edges("|Z")
            .fillet(0.0015)
        )
        # Short stub wrapping the roller end so the bail is one connected solid.
        stub = (
            cq.Workplane("YZ")
            .workplane(offset=cx - HUB_LEN / 2.0)
            .center(ARM_RISE, 0.0)
            .circle(ROLLER_R + 0.0010)
            .extrude(HUB_LEN)
        )
        part = hub.union(arm).union(stub)
        asm = part if asm is None else asm.union(part)
    return asm


def _build_roller() -> cq.Workplane:
    # Horizontal dark roller bar joining the two arm tops.
    return (
        cq.Workplane("YZ")
        .workplane(offset=-ROLLER_LEN / 2.0)
        .center(ARM_RISE, 0.0)
        .circle(ROLLER_R)
        .extrude(ROLLER_LEN)
    )


def _add_bail_unit_visuals(bail_part, i: int) -> None:
    """Shared visible geometry for every independently pivoting bail_i unit."""
    bail_part.visual(
        mesh_from_cadquery(_build_bail_arms(), f"bail_{i}_arms"),
        origin=Origin(),
        material="arm_metal",
        name="bail_arms",
    )
    bail_part.visual(
        mesh_from_cadquery(_build_roller(), f"bail_{i}_roller"),
        origin=Origin(),
        material="roller_dark",
        name="roller",
    )


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_bail_wall_power_switch")
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
    for i in range(UNIT_COUNT):
        unit_x = _unit_x(i)
        faceplate.visual(
            mesh_from_cadquery(_build_keeper(unit_x), f"keeper_block_{i}"),
            origin=Origin(),
            material="keeper_steel",
            name=f"keeper_block_{i}",
        )
        faceplate.visual(
            mesh_from_cadquery(_build_side_bolts(unit_x), f"side_bolts_{i}"),
            origin=Origin(),
            material="bolt_steel",
            name=f"side_bolts_{i}",
        )
    faceplate.visual(
        mesh_from_cadquery(_build_screws(), "mount_screws"),
        origin=Origin(),
        material="screw_zinc",
        name="mount_screws",
    )

    for i in range(UNIT_COUNT):
        unit_x = _unit_x(i)
        bail = model.part(f"bail_{i}")
        _add_bail_unit_visuals(bail, i)

        model.articulation(
            f"plate_to_bail_{i}",
            ArticulationType.REVOLUTE,
            parent=faceplate,
            child=bail,
            origin=Origin(xyz=(unit_x, PIVOT_Y, PIVOT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=4.0, lower=ROCK_LOWER, upper=ROCK_UPPER
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    faceplate = object_model.get_part("faceplate")
    bails = [object_model.get_part(f"bail_{i}") for i in range(UNIT_COUNT)]
    hinges = [object_model.get_articulation(f"plate_to_bail_{i}") for i in range(UNIT_COUNT)]

    ctx.check(
        "faceplate is the single root",
        faceplate in object_model.root_parts() and len(object_model.root_parts()) == 1,
        details=f"roots={[p.name for p in object_model.root_parts()]}",
    )
    ctx.check(
        "two independent bail parts exist",
        len(bails) == 2 and all(b is not None for b in bails),
        details=f"bails={[getattr(b, 'name', None) for b in bails]}",
    )
    ctx.check(
        "two independent bail joints exist",
        len(hinges) == 2 and all(h is not None for h in hinges),
        details=f"hinges={[getattr(h, 'name', None) for h in hinges]}",
    )
    for i, hinge in enumerate(hinges):
        bail = bails[i]
        ctx.check(
            f"bail_{i} is hinge child",
            hinge.child == f"bail_{i}" and hinge.parent == "faceplate",
            details=f"parent={hinge.parent}, child={hinge.child}",
        )
        ctx.check(
            f"bail_{i} joint type is revolute",
            str(hinge.articulation_type).lower().endswith("revolute"),
            details=f"type={hinge.articulation_type}",
        )
        ax = tuple(float(a) for a in hinge.axis)
        ctx.check(
            f"bail_{i} hinge axis is horizontal X",
            abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
            details=f"axis={ax}",
        )

    # Faceplate reads as one shared two-unit wall plate.
    plate_aabb = ctx.part_element_world_aabb(faceplate, elem="faceplate_shell")
    ctx.check("faceplate aabb present", plate_aabb is not None, details=str(plate_aabb))
    if plate_aabb is not None:
        pmin, pmax = plate_aabb
        pw = float(pmax[0] - pmin[0])
        ph = float(pmax[1] - pmin[1])
        pt = float(pmax[2] - pmin[2])
        ctx.check("plate width matches", abs(pw - PLATE_W) < 0.002, details=f"w={pw}")
        ctx.check("plate height matches", abs(ph - PLATE_H) < 0.002, details=f"h={ph}")
        ctx.check("two-unit plate is wider than tall", pw > ph + 0.015, details=f"w={pw}, h={ph}")
        ctx.check("plate is thin slab", pt < 0.013, details=f"t={pt}")

    # Equal-pitch placement: the two child origins sit on the two pivot axes.
    pos = [ctx.part_world_position(b) for b in bails]
    ctx.check("bail origins present", all(p is not None for p in pos), details=f"pos={pos}")
    if all(p is not None for p in pos):
        pitch = float(pos[1][0] - pos[0][0])
        ctx.check(
            "bail units are equally pitched on X",
            abs(pitch - UNIT_PITCH) < 0.002,
            details=f"pitch={pitch}, expected={UNIT_PITCH}",
        )
        for i, p in enumerate(pos):
            ctx.check(
                f"bail_{i} origin is on visible side-bolt axis",
                abs(float(p[1]) - PIVOT_Y) < 0.001 and abs(float(p[2]) - PIVOT_Z) < 0.001,
                details=f"pos={p}, pivot_y={PIVOT_Y}, pivot_z={PIVOT_Z}",
            )

    def _roller(bail_part):
        box = ctx.part_element_world_aabb(bail_part, elem="roller")
        if box is None:
            return None
        mn, mx = box
        return {
            "min": (float(mn[0]), float(mn[1]), float(mn[2])),
            "max": (float(mx[0]), float(mx[1]), float(mx[2])),
            "top_y": float(mx[1]),
            "front_z": float(mx[2]),
            "center_x": (float(mn[0]) + float(mx[0])) / 2.0,
            "width": float(mx[0] - mn[0]),
        }

    for i, bail in enumerate(bails):
        # The dark roller bar stands proud of the plate front and runs horizontally.
        roller = _roller(bail)
        ctx.check(f"bail_{i} roller present", roller is not None, details=str(roller))
        if roller is not None:
            ctx.check(
                f"bail_{i} roller stands proud of plate front",
                roller["front_z"] > PLATE_T + 0.004,
                details=f"roller_front_z={roller['front_z']}",
            )
            ctx.check(
                f"bail_{i} roller spans horizontally",
                roller["width"] > 0.030,
                details=f"roller_width={roller['width']}",
            )
            ctx.check(
                f"bail_{i} roller centered on its bay",
                abs(roller["center_x"] - _unit_x(i)) < 0.002,
                details=f"center_x={roller['center_x']}, expected={_unit_x(i)}",
            )

        # The pivot hubs wrap the fixed side bolts: an intentional captured-pin nest.
        ctx.allow_overlap(
            faceplate,
            bail,
            elem_a=f"side_bolts_{i}",
            elem_b="bail_arms",
            reason=f"bail_{i} pivot hubs are captured around its fixed side bolts.",
        )
        ctx.expect_overlap(
            faceplate,
            bail,
            axes="xyz",
            elem_a=f"side_bolts_{i}",
            elem_b="bail_arms",
            min_overlap=0.002,
            name=f"bail_{i} hubs overlap captured side bolts",
        )

        # Each bail sits in front of the shared plate and overlaps its own bay footprint.
        ctx.expect_overlap(
            bail,
            faceplate,
            axes="xy",
            elem_a="bail_arms",
            elem_b="faceplate_shell",
            min_overlap=0.010,
            name=f"bail_{i} overlaps plate footprint",
        )

    # Mechanism: throwing the latch swings the roller about the X axis. Track the
    # roller at the two extreme poses -- a no-op joint cannot fake it.
    for i, (bail, hinge) in enumerate(zip(bails, hinges)):
        up = _roller(bail)
        with ctx.pose({hinge: ROCK_UPPER}):
            thrown = _roller(bail)

        ctx.check(
            f"bail_{i} roller positions present",
            up is not None and thrown is not None,
            details=f"up={up}, thrown={thrown}",
        )
        if up is not None and thrown is not None:
            swung = (
                abs(up["top_y"] - thrown["top_y"]) > 0.006
                or abs(up["front_z"] - thrown["front_z"]) > 0.006
            )
            ctx.check(
                f"bail_{i} throw swings its roller",
                swung,
                details=(
                    f"top_y up={up['top_y']} thrown={thrown['top_y']}; "
                    f"front_z up={up['front_z']} thrown={thrown['front_z']}"
                ),
            )
            ctx.check(
                f"bail_{i} positive throw lowers the roller",
                thrown["top_y"] < up["top_y"],
                details=f"top_y up={up['top_y']} thrown={thrown['top_y']}",
            )

    # Independence: posing one revolute joint must not carry the other bail unit.
    other_rest = _roller(bails[1])
    with ctx.pose({hinges[0]: ROCK_UPPER, hinges[1]: 0.0}):
        moved_first = _roller(bails[0])
        still_second = _roller(bails[1])
    ctx.check(
        "bail_0 can move while bail_1 stays latched",
        (
            other_rest is not None
            and moved_first is not None
            and still_second is not None
            and abs(moved_first["top_y"] - other_rest["top_y"]) > 0.006
            and abs(still_second["top_y"] - other_rest["top_y"]) < 0.001
            and abs(still_second["front_z"] - other_rest["front_z"]) < 0.001
        ),
        details=f"bail_0={moved_first}, bail_1_rest={other_rest}, bail_1_posed={still_second}",
    )

    return ctx.report()


object_model = build_object_model()
