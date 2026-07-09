from __future__ import annotations

# Yellow construction tower crane (flat-top / horizontal-jib type).
# Z-up world. The square lattice MAST rises along +Z from a concrete base at
# z = 0. The slewing UPPERWORKS (machinery deck + operator cab + horizontal
# lattice JIB toward +X + COUNTER-JIB with concrete counterweights toward -X)
# rotates about the vertical mast axis. A TROLLEY travels horizontally along the
# jib (prismatic, +X). A HOOK BLOCK raises/lowers below the trolley
# (prismatic, vertical Z).
#
# Articulations:
#   slew         : REVOLUTE about +Z at the mast top  (PRIMARY)
#   trolley_travel: PRISMATIC along +X on the jib
#   hook_hoist   : PRISMATIC along -Z below the trolley

# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Real-world dimensions (meters) for a medium tower crane.
# ----------------------------------------------------------------------------
MAST_W = 2.0           # square mast width
MAST_BOTTOM_Z = 1.0    # top of the concrete base
MAST_TOP_Z = 26.0      # mast top (slewing ring height)
JIB_LEN = 24.0         # working-jib reach (+X)
JIB_HALF_W = 0.9       # jib truss half-width (Y)
JIB_BOTTOM_Z = 0.6     # jib bottom chord height above the slew deck
CJIB_LEN = 9.0         # counter-jib length (-X)
TROLLEY_MIN = 4.0      # nearest trolley position from mast center
TROLLEY_MAX = 22.0     # farthest trolley position
HOOK_DROP = 18.0       # max hook drop below the trolley


def _save(name, geom):
    return mesh_from_geometry(geom, name)


def _midpoint(a, b):
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a, b):
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _member(part, a, b, radius, material, name):
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _square_mast(part, *, width, bottom_z, top_z, panels, chord_r, brace_r, material):
    half = width * 0.5
    corners = [(half, half), (half, -half), (-half, -half), (-half, half)]
    levels = [bottom_z + (top_z - bottom_z) * i / panels for i in range(panels + 1)]
    # vertical chords
    for ci, (x, y) in enumerate(corners):
        _member(part, (x, y, bottom_z), (x, y, top_z), chord_r, material, f"mast_chord_{ci}")
    # horizontal rings + diagonal braces per panel
    for li, z in enumerate(levels):
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            _member(part, (x0, y0, z), (x1, y1, z), brace_r, material, f"mast_ring_{li}_{i}")
    for p in range(panels):
        z0, z1 = levels[p], levels[p + 1]
        for j in range(4):
            x0, y0 = corners[j]
            x1, y1 = corners[(j + 1) % 4]
            _member(part, (x0, y0, z0), (x1, y1, z1), brace_r, material, f"mast_diag_{p}_{j}")


def _triangular_jib(part, *, prefix, x_start, x_end, bottom_z, half_w, root_top_z,
                    tip_top_z, panels, chord_r, brace_r, material):
    # Apex-up triangular truss: two lower chords (±Y) + one upper chord (centre).
    xs = [x_start + (x_end - x_start) * i / panels for i in range(panels + 1)]
    span = x_end - x_start

    def top_z(x):
        t = 0.0 if abs(span) < 1e-9 else (x - x_start) / span
        return root_top_z + (tip_top_z - root_top_z) * t

    lower_a = [(x, -half_w, bottom_z) for x in xs]
    lower_b = [(x, half_w, bottom_z) for x in xs]
    upper = [(x, 0.0, top_z(x)) for x in xs]

    for i in range(panels):
        _member(part, lower_a[i], lower_a[i + 1], chord_r, material, f"{prefix}_la_{i}")
        _member(part, lower_b[i], lower_b[i + 1], chord_r, material, f"{prefix}_lb_{i}")
        _member(part, upper[i], upper[i + 1], chord_r, material, f"{prefix}_up_{i}")
    for i in range(panels + 1):
        _member(part, lower_a[i], lower_b[i], brace_r, material, f"{prefix}_bot_{i}")
        _member(part, lower_a[i], upper[i], brace_r, material, f"{prefix}_va_{i}")
        _member(part, lower_b[i], upper[i], brace_r, material, f"{prefix}_vb_{i}")
    for i in range(panels):
        if i % 2 == 0:
            _member(part, lower_a[i], upper[i + 1], brace_r, material, f"{prefix}_da_{i}")
            _member(part, lower_b[i], upper[i + 1], brace_r, material, f"{prefix}_db_{i}")
        else:
            _member(part, upper[i], lower_a[i + 1], brace_r, material, f"{prefix}_da_{i}")
            _member(part, upper[i], lower_b[i + 1], brace_r, material, f"{prefix}_db_{i}")
    return {"lower_a": lower_a, "lower_b": lower_b, "upper": upper}


def _hook_mesh(name):
    geom = tube_from_spline_points(
        [
            (0.0, 0.0, -0.10),
            (0.30, 0.0, -0.55),
            (0.55, 0.0, -1.05),
            (0.45, 0.0, -1.55),
            (0.0, 0.0, -1.78),
            (-0.45, 0.0, -1.60),
            (-0.55, 0.0, -1.15),
        ],
        radius=0.11,
        samples_per_segment=12,
        radial_segments=14,
        up_hint=(0.0, 1.0, 0.0),
    )
    return _save(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.92, 0.74, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.22, 0.24, 0.27, 1.0))
    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.63, 1.0))
    ballast = model.material("ballast", rgba=(0.52, 0.52, 0.50, 1.0))
    cable = model.material("cable", rgba=(0.14, 0.14, 0.16, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.55, 0.74, 0.86, 0.40))

    # ------------------------------------------------------------- tower base
    tower_base = model.part("tower_base")
    # concrete foundation pad at ground level
    tower_base.visual(
        Box((5.0, 5.0, 0.6)), origin=Origin(xyz=(0.0, 0.0, 0.3)), material=concrete, name="foundation_pad"
    )
    tower_base.visual(
        Box((3.0, 3.0, 0.4)), origin=Origin(xyz=(0.0, 0.0, 0.8)), material=concrete, name="footing"
    )
    # anchor stubs
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower_base.visual(
                Cylinder(radius=0.07, length=0.5),
                origin=Origin(xyz=(sx, sy, 0.85)),
                material=steel,
                name=f"anchor_{0 if sx < 0 else 1}_{0 if sy < 0 else 1}",
            )
    # square lattice mast
    _square_mast(
        tower_base,
        width=MAST_W,
        bottom_z=MAST_BOTTOM_Z,
        top_z=MAST_TOP_Z,
        panels=12,
        chord_r=0.075,
        brace_r=0.045,
        material=crane_yellow,
    )
    # mast cap / slew bearing seat
    tower_base.visual(
        Cylinder(radius=1.1, length=0.3),
        origin=Origin(xyz=(0.0, 0.0, MAST_TOP_Z + 0.15)),
        material=dark_grey,
        name="slew_bearing_seat",
    )
    tower_base.inertial = Inertial.from_geometry(
        Box((5.0, 5.0, MAST_TOP_Z + 0.5)),
        mass=22000.0,
        origin=Origin(xyz=(0.0, 0.0, MAST_TOP_Z * 0.45)),
    )

    # ------------------------------------------------------------- upperworks
    # All upperworks geometry is authored in the slew frame, whose origin sits
    # at the mast top (z = MAST_TOP_Z) so it lands flush at slew q = 0.
    upper = model.part("upperworks")
    deck_z = 0.0  # local z at the slew ring top
    # slewing ring
    upper.visual(
        Cylinder(radius=1.05, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, deck_z + 0.175)),
        material=dark_grey,
        name="slew_ring",
    )
    # machinery / turntable deck
    upper.visual(
        Box((3.4, 2.0, 0.5)),
        origin=Origin(xyz=(0.0, 0.0, deck_z + 0.55)),
        material=crane_yellow,
        name="turntable_deck",
    )
    deck_top = deck_z + 0.80

    # operator cab with glass on the +X (jib) side of the deck
    upper.visual(
        Box((1.5, 1.5, 1.6)),
        origin=Origin(xyz=(1.4, 0.0, deck_top + 0.8)),
        material=crane_yellow,
        name="operator_cab",
    )
    upper.visual(
        Box((1.42, 1.42, 1.1)),
        origin=Origin(xyz=(1.46, 0.0, deck_top + 0.95)),
        material=cab_glass,
        name="cab_glass",
    )
    # machinery house on the -X (counter-jib) side
    upper.visual(
        Box((2.2, 1.7, 1.3)),
        origin=Origin(xyz=(-1.3, 0.0, deck_top + 0.65)),
        material=dark_grey,
        name="machinery_house",
    )

    # horizontal working jib (triangular lattice) toward +X
    jib_bottom = deck_top + JIB_BOTTOM_Z
    jib = _triangular_jib(
        upper,
        prefix="jib",
        x_start=1.0,
        x_end=1.0 + JIB_LEN,
        bottom_z=jib_bottom,
        half_w=JIB_HALF_W,
        root_top_z=jib_bottom + 1.6,
        tip_top_z=jib_bottom + 0.7,
        panels=12,
        chord_r=0.06,
        brace_r=0.035,
        material=crane_yellow,
    )
    # trolley running rails along the jib bottom chords
    for sy, tag in ((-0.55, "a"), (0.55, "b")):
        _member(
            upper,
            (1.0, sy, jib_bottom + 0.05),
            (1.0 + JIB_LEN, sy, jib_bottom + 0.05),
            0.035,
            steel,
            f"jib_rail_{tag}",
        )

    # counter-jib (triangular lattice) toward -X
    cjib_bottom = deck_top + JIB_BOTTOM_Z
    cjib = _triangular_jib(
        upper,
        prefix="cjib",
        x_start=-1.0,
        x_end=-1.0 - CJIB_LEN,
        bottom_z=cjib_bottom,
        half_w=JIB_HALF_W * 0.85,
        root_top_z=cjib_bottom + 1.4,
        tip_top_z=cjib_bottom + 0.7,
        panels=5,
        chord_r=0.06,
        brace_r=0.035,
        material=crane_yellow,
    )
    # concrete counterweight blocks hung under the counter-jib tip
    for i in range(3):
        upper.visual(
            Box((1.0, 1.6, 1.2)),
            origin=Origin(xyz=(-1.0 - CJIB_LEN + 0.8 + i * 1.05, 0.0, cjib_bottom - 0.3)),
            material=ballast,
            name=f"counterweight_{i}",
        )

    # A-frame apex above the slew with tie-bars (pendant cables) to both jibs
    apex = (0.2, 0.0, deck_top + 5.4)
    _member(upper, (0.9, -0.7, deck_top + 0.4), apex, 0.07, crane_yellow, "apex_leg_a")
    _member(upper, (0.9, 0.7, deck_top + 0.4), apex, 0.07, crane_yellow, "apex_leg_b")
    _member(upper, (-0.9, -0.7, deck_top + 0.4), apex, 0.07, crane_yellow, "apex_leg_c")
    _member(upper, (-0.9, 0.7, deck_top + 0.4), apex, 0.07, crane_yellow, "apex_leg_d")
    upper.visual(
        Box((0.4, 0.4, 0.4)), origin=Origin(xyz=apex), material=crane_yellow, name="apex_cap"
    )
    # forestay tie-bars to the jib upper chord
    _member(upper, apex, jib["upper"][6], 0.03, steel, "forestay_mid")
    _member(upper, apex, jib["upper"][-1], 0.03, steel, "forestay_tip")
    # backstay tie-bars to the counter-jib upper chord
    _member(upper, apex, cjib["upper"][-1], 0.03, steel, "backstay")

    upper.inertial = Inertial.from_geometry(
        Box((JIB_LEN + CJIB_LEN + 2.0, 2.0, 6.0)),
        mass=9000.0,
        origin=Origin(xyz=(JIB_LEN * 0.3, 0.0, 2.0)),
    )

    # ---------------------------------------------------------------- trolley
    # Authored in its own local frame; the prismatic joint slides it along +X.
    trolley = model.part("trolley")
    trolley.visual(Box((1.0, 1.3, 0.25)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark_grey, name="trolley_frame")
    # four flanged wheels riding the rails
    for sx in (-0.4, 0.4):
        for sy in (-0.55, 0.55):
            trolley.visual(
                Cylinder(radius=0.12, length=0.14, ),
                origin=Origin(xyz=(sx, sy, 0.14), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=f"trolley_wheel_{0 if sx < 0 else 1}_{0 if sy < 0 else 1}",
            )
    # hoist sheave block under the trolley
    trolley.visual(Box((0.5, 0.4, 0.3)), origin=Origin(xyz=(0.0, 0.0, -0.2)), material=steel, name="sheave_block")
    trolley.visual(
        Cylinder(radius=0.18, length=0.1),
        origin=Origin(xyz=(0.0, 0.0, -0.3), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="sheave",
    )
    trolley.inertial = Inertial.from_geometry(
        Box((1.0, 1.3, 0.7)), mass=900.0, origin=Origin(xyz=(0.0, 0.0, -0.1))
    )

    # ------------------------------------------------------------- hook block
    # Authored in its own local frame; the prismatic joint lowers it along -Z.
    hook = model.part("hook_block")
    # two hoist cables running from the trolley sheave down to the hook block
    for sy in (-0.12, 0.12):
        hook.visual(
            Cylinder(radius=0.018, length=1.6),
            origin=Origin(xyz=(0.0, sy, 0.8)),
            material=cable,
            name=f"hoist_cable_{0 if sy < 0 else 1}",
        )
    # hook block housing + sheaves
    hook.visual(Box((0.5, 0.35, 0.45)), origin=Origin(xyz=(0.0, 0.0, -0.1)), material=crane_yellow, name="hook_housing")
    hook.visual(
        Cylinder(radius=0.16, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, -0.1), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="hook_sheave",
    )
    # swivel + the hook itself
    hook.visual(
        Cylinder(radius=0.08, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, -0.42)),
        material=steel,
        name="hook_swivel",
    )
    hook.visual(_hook_mesh("crane_hook.obj"), origin=Origin(xyz=(0.0, 0.0, -0.45)), material=steel, name="hook")
    hook.inertial = Inertial.from_geometry(
        Box((0.6, 0.4, 2.4)), mass=600.0, origin=Origin(xyz=(0.0, 0.0, -0.4))
    )

    # ----------------------------------------------------------- articulations
    # PRIMARY: the upperworks slews about the vertical mast axis.
    model.articulation(
        "slew",
        ArticulationType.REVOLUTE,
        parent=tower_base,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, MAST_TOP_Z + 0.3)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=400000.0, velocity=0.12, lower=-6.2832, upper=6.2832),
    )
    # The trolley travels horizontally along the jib (+X). Joint origin at the
    # nearest trolley position on the jib bottom chord.
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=upper,
        child=trolley,
        origin=Origin(xyz=(1.0 + TROLLEY_MIN, 0.0, jib_bottom - 0.28)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120000.0, velocity=0.6, lower=0.0, upper=TROLLEY_MAX - TROLLEY_MIN),
    )
    # The hook block raises/lowers below the trolley. At q = 0 the hook hangs
    # just below the trolley sheaves (cables visible); positive q lowers it
    # along -Z.
    model.articulation(
        "hook_hoist",
        ArticulationType.PRISMATIC,
        parent=trolley,
        child=hook,
        origin=Origin(xyz=(0.0, 0.0, -0.65)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120000.0, velocity=1.5, lower=0.0, upper=HOOK_DROP),
    )

    return model


# >>> USER_CODE_END

object_model = build_object_model()


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)

    base = object_model.get_part("tower_base")
    upper = object_model.get_part("upperworks")
    trolley = object_model.get_part("trolley")
    hook = object_model.get_part("hook_block")
    slew = object_model.get_articulation("slew")
    travel = object_model.get_articulation("trolley_travel")
    hoist = object_model.get_articulation("hook_hoist")

    # --- Intentional mechanical-contact overlaps ----------------------------
    # The trolley flanged wheels ride directly on the jib running rails.
    for sx in (0, 1):
        for sy in (0, 1):
            ctx.allow_overlap(
                trolley,
                upper,
                elem_a=f"trolley_wheel_{sx}_{sy}",
                elem_b="jib_rail_a" if sy == 0 else "jib_rail_b",
                reason="Trolley wheel rolls on the jib running rail; rolling contact overlap is intentional.",
            )
    # The hoist cables reeve up through the trolley frame and over the trolley
    # sheave inside the sheave block.
    for ci in (0, 1):
        for member in ("trolley_frame", "sheave_block", "sheave"):
            ctx.allow_overlap(
                hook,
                trolley,
                elem_a=f"hoist_cable_{ci}",
                elem_b=member,
                reason="Hoist cable reeves up through the trolley frame and over the trolley sheave; overlap is intentional.",
            )
    # The hoist cables run vertically past the jib bottom bracing.
    for ci, member in ((0, "jib_bot_2"), (1, "jib_bot_2")):
        ctx.allow_overlap(
            hook,
            upper,
            elem_a=f"hoist_cable_{ci}",
            elem_b=member,
            reason="Hoist cables pass alongside the jib bottom bracing; thin crossing overlap is intentional.",
        )

    # --- Base sits at ground level (z ~ 0) ----------------------------------
    bb = ctx.part_world_aabb(base)
    assert bb is not None
    lo, hi = bb
    ctx.check("foundation sits at ground z~0", abs(lo[2]) <= 0.02, details=f"base min z={lo[2]:.4f}")
    ctx.check("mast rises to a realistic height (>20 m)", hi[2] > 20.0, details=f"base top z={hi[2]:.3f}")

    # --- Lattice mast is a real truss (many members, not a solid beam) -------
    mast_members = [v for v in base.visuals if v.name.startswith("mast_")]
    ctx.check(
        "mast is a lattice truss with many members",
        len(mast_members) > 60,
        details=f"mast member count={len(mast_members)}",
    )

    # --- Jib + counter-jib are real lattice trusses -------------------------
    jib_members = [v for v in upper.visuals if v.name.startswith("jib_")]
    cjib_members = [v for v in upper.visuals if v.name.startswith("cjib_")]
    ctx.check("working jib is a lattice truss", len(jib_members) > 40, details=f"jib members={len(jib_members)}")
    ctx.check("counter-jib is a lattice truss", len(cjib_members) > 20, details=f"cjib members={len(cjib_members)}")

    # --- Counterweights present on the counter-jib --------------------------
    cw = [v for v in upper.visuals if v.name.startswith("counterweight_")]
    ctx.check("concrete counterweights present", len(cw) == 3, details=f"counterweights={len(cw)}")

    # --- Operator cab + glass present ---------------------------------------
    ctx.check(
        "operator cab with glass present",
        any(v.name == "operator_cab" for v in upper.visuals)
        and any(v.name == "cab_glass" for v in upper.visuals),
        details="missing cab/glass",
    )

    # --- Jib extends far in +X, counter-jib in -X (correct layout) ----------
    ub = ctx.part_world_aabb(upper)
    assert ub is not None
    ctx.check("jib reaches well past the mast (+X)", ub[1][0] > 20.0, details=f"upper max x={ub[1][0]:.3f}")
    ctx.check("counter-jib extends behind the mast (-X)", ub[0][0] < -8.0, details=f"upper min x={ub[0][0]:.3f}")

    # --- PRIMARY articulation: slew rotates the upperworks about vertical Z ---
    ctx.check("slew axis is vertical (Z)", tuple(slew.axis) == (0.0, 0.0, 1.0), details=f"axis={slew.axis}")
    jib_tip_rest = None
    for v in upper.visuals:
        if v.name == "jib_up_11":
            jib_tip_rest = ctx.part_element_world_aabb(upper, elem=v.name)
    rest_y = None
    if jib_tip_rest is not None:
        rest_y = 0.5 * (jib_tip_rest[0][1] + jib_tip_rest[1][1])
    with ctx.pose({slew: math.pi / 2.0}):
        ub_rot = ctx.part_world_aabb(upper)
        assert ub_rot is not None
        # after a 90 deg slew the jib that pointed +X now points +Y
        ctx.check(
            "slewing 90deg swings the jib into +Y",
            ub_rot[1][1] > 20.0,
            details=f"rotated upper max y={ub_rot[1][1]:.3f}",
        )

    # --- Trolley travels horizontally along the jib (+X) --------------------
    ctx.check("trolley axis is horizontal (X)", tuple(travel.axis) == (1.0, 0.0, 0.0), details=f"axis={travel.axis}")
    rest_tx = ctx.part_world_position(trolley)
    with ctx.pose({travel: TROLLEY_MAX - TROLLEY_MIN}):
        far_tx = ctx.part_world_position(trolley)
    assert rest_tx is not None and far_tx is not None
    ctx.check(
        "trolley moves outward along the jib",
        far_tx[0] > rest_tx[0] + 10.0 and abs(far_tx[2] - rest_tx[2]) < 0.05,
        details=f"rest={rest_tx}, far={far_tx}",
    )

    # --- Hook block raises/lowers vertically --------------------------------
    rest_h = ctx.part_world_position(hook)
    with ctx.pose({hoist: HOOK_DROP}):
        low_h = ctx.part_world_position(hook)
    assert rest_h is not None and low_h is not None
    ctx.check(
        "hook lowers vertically (downward)",
        low_h[2] < rest_h[2] - 12.0 and abs(low_h[0] - rest_h[0]) < 0.05,
        details=f"rest={rest_h}, low={low_h}",
    )

    # --- Connectivity: trolley rides on the jib (near the rails) ------------
    ctx.expect_contact(upper, trolley, contact_tol=0.2, name="trolley is seated on the jib rails")
    # hook hangs from the trolley via the hoist cables
    ctx.expect_contact(trolley, hook, contact_tol=0.2, name="hook block is suspended from the trolley")

    return ctx.report()
