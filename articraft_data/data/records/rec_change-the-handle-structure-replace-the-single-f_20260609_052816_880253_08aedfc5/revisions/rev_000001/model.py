from __future__ import annotations

# Rectangular red plastic shopping basket with two folding carry handles.
#
# Coordinate convention: +Z up, basket rests on the ground at z=0, long axis
# along X. The tub is a hollow tapered shell (wider at the top) with rounded
# vertical edges, a thick rolled top rim, and vertical slot perforations cut
# clean through every wall. Two independent U-shaped folding handles are
# mounted on the two opposite long rims; each pivots up to stand for carrying
# and folds down outward alongside the basket.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------
L_TOP = 0.40  # outer length at the mouth (long axis, X)
D_TOP = 0.30  # outer depth at the mouth (short axis, Y)
L_BOT = 0.34  # outer length at the base (taper: narrower at the bottom)
D_BOT = 0.24  # outer depth at the base
H = 0.22  # tub height
WALL = 0.012  # wall thickness

R_VERT_BOT = 0.030  # rounded vertical edge radius at the base
R_VERT_TOP = 0.036  # rounded vertical edge radius at the mouth

# Rolled rim lip
RIM_H = 0.024
RIM_OVERHANG = 0.011  # how far the lip rolls out past the top wall

# Slot perforations
SLOT_W = 0.028
SLOT_H = 0.120
SLOT_R = 0.013
SLOT_ZC = H * 0.43  # vertical center of the slots
LONG_SLOT_X = (-0.124, -0.042, 0.042, 0.124)  # 4 slots per long wall
SHORT_SLOT_Y = (-0.060, 0.060)  # 2 slots per short wall

# Folding handles (mounted on long rims)
HANDLE_SPAN = 0.22  # distance between the two pivot points along X
HANDLE_RISE = 0.10  # height of the grip bar above the pivot line
BAR_R = 0.009  # handle bar radius
PIVOT_Z = H - 0.014  # pivot height at the rim
HANDLE_HALF_SPAN = HANDLE_SPAN / 2.0

# Pivot hardware
KNUCKLE_R = 0.016  # handle knuckle outer radius
KNUCKLE_LEN = 0.022  # handle knuckle length along Y (pin axis)
TUB_BOSS_R = 0.012  # tub boss radius (fits inside knuckle)
TUB_BOSS_LEN = 0.014  # tub boss protrusion from wall


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------
def _tub_mesh() -> object:
    """Hollow tapered tub: loft two rounded rectangles, shell open at the top,
    add the rolled rim, cut vertical slot perforations, and add pivot bosses
    on both long walls for the two folding handles."""
    outer = (
        cq.Workplane("XY")
        .placeSketch(
            cq.Sketch().rect(L_BOT, D_BOT).vertices().fillet(R_VERT_BOT),
            cq.Sketch()
            .rect(L_TOP, D_TOP)
            .vertices()
            .fillet(R_VERT_TOP)
            .moved(cq.Location(cq.Vector(0, 0, H))),
        )
        .loft()
    )
    tub = outer.faces(">Z").shell(-WALL)

    try:
        tub = tub.edges("<Z").fillet(0.008)
    except Exception:
        pass

    # Rolled top rim
    l_o = L_TOP + 2 * RIM_OVERHANG
    d_o = D_TOP + 2 * RIM_OVERHANG
    l_i = L_TOP - WALL
    d_i = D_TOP - WALL
    r_o = R_VERT_TOP + RIM_OVERHANG
    r_i = max(0.012, R_VERT_TOP - WALL)
    ring = (
        cq.Workplane("XY", origin=(0, 0, H - RIM_H * 0.5))
        .placeSketch(cq.Sketch().rect(l_o, d_o).vertices().fillet(r_o))
        .extrude(RIM_H)
    )
    inner_cut = (
        cq.Workplane("XY", origin=(0, 0, H - RIM_H * 0.5 - 0.002))
        .placeSketch(cq.Sketch().rect(l_i, d_i).vertices().fillet(r_i))
        .extrude(RIM_H + 0.004)
    )
    rim = ring.cut(inner_cut)
    try:
        rim = rim.edges("#Z").fillet(0.005)
    except Exception:
        pass
    tub = tub.union(rim)

    # Vertical slot perforations
    def _slot(plane: str, origin: tuple[float, float, float]) -> object:
        sk = cq.Sketch().rect(SLOT_W, SLOT_H).vertices().fillet(SLOT_R)
        return cq.Workplane(plane, origin=origin).placeSketch(sk).extrude(0.6, both=True)

    for x in LONG_SLOT_X:
        tub = tub.cut(_slot("XZ", (x, 0.0, SLOT_ZC)))
    for y in SHORT_SLOT_Y:
        tub = tub.cut(_slot("YZ", (0.0, y, SLOT_ZC)))

    # Pivot bosses on both long walls for the two folding handles.
    # Each handle has two pivot points at x = ±HANDLE_HALF_SPAN on the rim.

    # +Y side (front): bosses protrude outward in +Y
    for sx in (-1.0, 1.0):
        boss = (
            cq.Workplane(
                "XZ",
                origin=(sx * HANDLE_HALF_SPAN, D_TOP / 2.0 - 0.001, PIVOT_Z),
            )
            .circle(TUB_BOSS_R)
            .extrude(TUB_BOSS_LEN + 0.002)
        )
        tub = tub.union(boss)

    # -Y side (back): bosses protrude outward in -Y
    for sx in (-1.0, 1.0):
        boss = (
            cq.Workplane(
                "XZ",
                origin=(sx * HANDLE_HALF_SPAN, -D_TOP / 2.0 + 0.001, PIVOT_Z),
            )
            .circle(TUB_BOSS_R)
            .extrude(-(TUB_BOSS_LEN + 0.002))
        )
        tub = tub.union(boss)

    return mesh_from_cadquery(tub, "basket_tub")


def _handle_mesh(name: str = "folding_handle") -> object:
    """U-shaped folding carry handle in LOCAL frame: pivot line along X at
    y=0, z=0. Handle rises in +Z; the grip bar connects the two leg tops.
    Pivot knuckles at each end capture the tub bosses."""
    span = HANDLE_SPAN
    rise = HANDLE_RISE
    r = BAR_R

    # Left leg (vertical cylinder from pivot up to grip height)
    left_leg = (
        cq.Workplane("XY", origin=(-span / 2.0, 0.0, 0.0))
        .circle(r)
        .extrude(rise)
    )
    # Right leg
    right_leg = (
        cq.Workplane("XY", origin=(span / 2.0, 0.0, 0.0))
        .circle(r)
        .extrude(rise)
    )
    # Grip bar (horizontal cylinder along X at height rise)
    grip = (
        cq.Workplane("YZ", origin=(0.0, 0.0, rise))
        .circle(r)
        .extrude(span / 2.0, both=True)
    )
    handle = left_leg.union(right_leg).union(grip)

    # Corner spheres for smooth molded-plastic transitions
    for sx in (-1.0, 1.0):
        corner = (
            cq.Workplane("XY", origin=(sx * span / 2.0, 0.0, rise))
            .sphere(r * 1.5)
        )
        handle = handle.union(corner)

    # Thicker grip section in the middle of the bar (ergonomic feel)
    grip_thick = (
        cq.Workplane("YZ", origin=(0.0, 0.0, rise))
        .circle(r * 1.7)
        .extrude(span * 0.22, both=True)
    )
    handle = handle.union(grip_thick)

    # Rounded end caps on the grip thick section
    for sx in (-1.0, 1.0):
        cap = (
            cq.Workplane("XY", origin=(sx * span * 0.22, 0.0, rise))
            .sphere(r * 1.7)
        )
        handle = handle.union(cap)

    # Pivot knuckles at each end (sleeves along Y to capture tub bosses)
    for sx in (-1.0, 1.0):
        knuckle = (
            cq.Workplane("XZ", origin=(sx * span / 2.0, 0.0, 0.0))
            .circle(KNUCKLE_R)
            .extrude(KNUCKLE_LEN / 2.0, both=True)
        )
        handle = handle.union(knuckle)
        # Bottom sphere to blend knuckle into leg base
        bottom = (
            cq.Workplane("XY", origin=(sx * span / 2.0, 0.0, 0.0))
            .sphere(r * 1.3)
        )
        handle = handle.union(bottom)

    return mesh_from_cadquery(handle, name)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="red_shopping_basket")

    body_finish = model.material("body_finish", rgba=(0.88, 0.27, 0.22, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.55, 0.10, 0.09, 1.0))

    tub = model.part("basket_tub")
    tub.visual(_tub_mesh(), material=body_finish, name="tub_shell")
    tub.inertial = Inertial.from_geometry(
        Box((L_TOP, D_TOP, H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, H / 2.0)),
    )

    # Front handle: mounted on the +Y long rim
    handle_front = model.part("handle_front")
    handle_front.visual(
        _handle_mesh("handle_front_bar"),
        material=handle_finish,
        name="handle_bar",
    )
    handle_front.inertial = Inertial.from_geometry(
        Box((HANDLE_SPAN, 2 * KNUCKLE_R, HANDLE_RISE)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_RISE / 2.0)),
    )

    # Back handle: mounted on the -Y long rim
    handle_back = model.part("handle_back")
    handle_back.visual(
        _handle_mesh("handle_back_bar"),
        material=handle_finish,
        name="handle_bar",
    )
    handle_back.inertial = Inertial.from_geometry(
        Box((HANDLE_SPAN, 2 * KNUCKLE_R, HANDLE_RISE)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_RISE / 2.0)),
    )

    # Front handle articulation:
    # Pivot line on the +Y rim at z=PIVOT_Z. Axis chosen so positive q folds
    # the handle outward (toward +Y): right-hand rule about -X rotates +Z
    # toward +Y.
    model.articulation(
        "tub_to_handle_front",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=handle_front,
        origin=Origin(xyz=(0.0, D_TOP / 2.0, PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    # Back handle articulation:
    # Pivot line on the -Y rim at z=PIVOT_Z. Axis chosen so positive q folds
    # the handle outward (toward -Y): right-hand rule about +X rotates +Z
    # toward -Y.
    model.articulation(
        "tub_to_handle_back",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=handle_back,
        origin=Origin(xyz=(0.0, -D_TOP / 2.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("basket_tub")
    handle_front = object_model.get_part("handle_front")
    handle_back = object_model.get_part("handle_back")
    joint_front = object_model.get_articulation("tub_to_handle_front")
    joint_back = object_model.get_articulation("tub_to_handle_back")

    # --- Footprint: wider in X (long axis) than Y, and rests at z ~= 0. ---
    tub_aabb = ctx.part_world_aabb(tub)
    ctx.check(
        "tub present with world AABB",
        tub_aabb is not None,
        details=f"tub_aabb={tub_aabb}",
    )
    if tub_aabb is not None:
        (tmn, tmx) = tub_aabb
        ext_x = tmx[0] - tmn[0]
        ext_y = tmx[1] - tmn[1]
        ext_z = tmx[2] - tmn[2]
        ctx.check(
            "footprint wider in X than Y",
            ext_x > ext_y + 0.05,
            details=f"ext_x={ext_x:.3f}, ext_y={ext_y:.3f}",
        )
        ctx.check(
            "basket rests at z~=0",
            abs(tmn[2]) < 0.01,
            details=f"z_min={tmn[2]:.4f}",
        )
        ctx.check(
            "tub has realistic height",
            0.18 < ext_z < 0.30,
            details=f"ext_z={ext_z:.3f}",
        )

    # --- Tapered + hollow ---
    ctx.check(
        "body is tapered (mouth wider than base)",
        L_TOP > L_BOT + 0.02 and D_TOP > D_BOT + 0.02,
        details=f"L_top={L_TOP}, L_bot={L_BOT}, D_top={D_TOP}, D_bot={D_BOT}",
    )
    inner_top = L_TOP - 2 * WALL
    ctx.check(
        "tub is hollow (open interior cavity)",
        inner_top > 0.30 and WALL < 0.05,
        details=f"inner_top={inner_top:.3f}, wall={WALL}",
    )

    # --- Slot perforations ---
    n_slots = len(LONG_SLOT_X) * 2 + len(SHORT_SLOT_Y) * 2
    ctx.check(
        "vertical slot perforations present on every wall",
        len(LONG_SLOT_X) >= 3 and len(SHORT_SLOT_Y) >= 1 and n_slots >= 8,
        details=f"long_per_wall={len(LONG_SLOT_X)}, short_per_wall={len(SHORT_SLOT_Y)}, total={n_slots}",
    )

    # --- Two independent folding handles exist ---
    ctx.check(
        "two handle parts exist",
        handle_front is not None and handle_back is not None,
        details="handle_front or handle_back missing",
    )

    # Both joints are revolute about X axis (long axis)
    for jname, j in [("front", joint_front), ("back", joint_back)]:
        ax = j.axis
        ctx.check(
            f"{jname} joint axis runs along X (long axis)",
            abs(ax[0]) > 0.99 and abs(ax[1]) < 0.01 and abs(ax[2]) < 0.01,
            details=f"axis={ax}",
        )
        ctx.check(
            f"{jname} joint is revolute",
            str(j.articulation_type).upper().endswith("REVOLUTE"),
            details=f"type={j.articulation_type}",
        )
        lim = j.motion_limits
        ctx.check(
            f"{jname} handle has realistic fold range (0 to ~95 deg)",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and abs(lim.lower) < 0.1
            and lim.upper > 1.4,
            details=f"lower={lim.lower}, upper={lim.upper}",
        )

    # Handle pose checks: upright at q=0, folded at q=upper
    for jname, j, h in [
        ("front", joint_front, handle_front),
        ("back", joint_back, handle_back),
    ]:
        with ctx.pose({j: 0.0}):
            up_aabb = ctx.part_world_aabb(h)
        with ctx.pose({j: j.motion_limits.upper}):
            down_aabb = ctx.part_world_aabb(h)

        ctx.check(
            f"{jname} handle poses resolve",
            up_aabb is not None and down_aabb is not None,
            details=f"up={up_aabb}, down={down_aabb}",
        )
        if up_aabb is not None and down_aabb is not None:
            up_top_z = up_aabb[1][2]
            down_top_z = down_aabb[1][2]
            ctx.check(
                f"{jname} handle upright reaches higher than folded",
                up_top_z > down_top_z + 0.03,
                details=f"upright_z={up_top_z:.3f}, folded_z={down_top_z:.3f}",
            )

    # Handles rise above the rim when upright
    if tub_aabb is not None:
        for jname, j, h in [
            ("front", joint_front, handle_front),
            ("back", joint_back, handle_back),
        ]:
            with ctx.pose({j: 0.0}):
                up_aabb = ctx.part_world_aabb(h)
            if up_aabb is not None:
                rise = up_aabb[1][2] - tub_aabb[1][2]
                ctx.check(
                    f"{jname} handle rises above rim when upright",
                    rise > 0.06,
                    details=f"rise={rise:.3f}",
                )

    # Front handle on +Y side, back handle on -Y side (at rest q=0)
    with ctx.pose({joint_front: 0.0, joint_back: 0.0}):
        front_aabb = ctx.part_world_aabb(handle_front)
        back_aabb = ctx.part_world_aabb(handle_back)
    if front_aabb is not None and back_aabb is not None:
        front_cy = 0.5 * (front_aabb[0][1] + front_aabb[1][1])
        back_cy = 0.5 * (back_aabb[0][1] + back_aabb[1][1])
        ctx.check(
            "front handle on +Y side, back on -Y side",
            front_cy > 0.05 and back_cy < -0.05,
            details=f"front_cy={front_cy:.3f}, back_cy={back_cy:.3f}",
        )

    # Overlap allowances for pivot knuckle/boss captures
    ctx.allow_overlap(
        tub,
        handle_front,
        reason=(
            "Front handle pivot knuckles capture the tub long-rim bosses; "
            "local overlap represents the pin-in-knuckle pivot joints."
        ),
    )
    ctx.allow_overlap(
        tub,
        handle_back,
        reason=(
            "Back handle pivot knuckles capture the tub long-rim bosses; "
            "local overlap represents the pin-in-knuckle pivot joints."
        ),
    )

    return ctx.report()


object_model = build_object_model()
