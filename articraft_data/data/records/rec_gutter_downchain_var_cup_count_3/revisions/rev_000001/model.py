from __future__ import annotations

# Gutter downpipe RAIN CHAIN (kusari-doi, "cup" style).
#
# This is the decorative downspout alternative seen on the reference image: a
# series of small tapered metal cups (open funnels, apex pointing DOWN) strung
# vertically beneath a gutter outlet. Rainwater spills from the gutter into the
# top cup, brims over and drips into the cup below, and so on down the chain,
# guiding the water visibly to the ground.
#
# Real mechanism / articulation:
#   - A fixed gutter-outlet collar/bracket mounts under the gutter and carries a
#     small hanging eye. It is the ROOT (does not move).
#   - Each cup hangs from a short twisted-wire link. The cup is free to SWING
#     like a pendulum about the horizontal pivot where its link hooks onto the
#     eye of the cup (or bracket) above it. That swing is the primary real
#     motion of a rain chain in wind/water, and it is modeled as one REVOLUTE
#     joint per cup, rocking about a horizontal (Y) axis through the upper link
#     eye.
#   - The chain therefore reads as: bracket -> cup_1 -> cup_2 -> ... each a
#     pendulum hung off the link of the one above. Posing a cup's joint swings
#     that cup (and everything below it) sideways, exactly like the real chain.
#
# Coordinate convention:
#   - +Z is up. The gutter bracket sits near z=0 (top); the chain hangs DOWN
#     into -Z. Each cup's local frame origin is its TOP RIM CENTER, so the cup
#     body and its hanging wire loop extend downward into -Z from that origin,
#     and the next link's pivot eye sits at the cup bottom.
#   - Each cup is a hollow thin-wall tapered shell (wide open mouth up, small
#     apex down), weathered galvanized zinc with a brighter rolled rim, matching
#     the greenish-grey cups with pale rims in the photo.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# --- cup dimensions (meters) ---
CUP_TOP_R = 0.052  # outer radius of the open mouth
CUP_BOT_R = 0.018  # outer radius at the small bottom (apex region)
CUP_H = 0.085  # height of the cup body (rim down to bottom)
WALL = 0.0014  # sheet-metal wall thickness

RIM_TUBE = 0.0040  # rolled-rim tube (minor) radius
RIM_CENTER_R = CUP_TOP_R - RIM_TUBE * 0.4  # torus center radius at the mouth edge

# --- hanging link / pivot eyes ---
# A small loop of twisted wire hangs each cup from the eye of the part above.
#
# Cup-frame convention used below: the cup PART FRAME ORIGIN is the top PIVOT /
# hook point (where this cup's link hooks the eye of the part above). All cup
# geometry hangs DOWN from there, so a pendulum swing about the part frame is
# physically correct (it pivots at the top hook, not at the rim).
WIRE_R = 0.0018  # link wire radius
EYE_R = 0.0085  # radius of the small hanging eye loop on each cup bottom
LINK_LEN = 0.022  # vertical drop of the hanging link from the upper eye to the rim

PIVOT_Z = 0.0  # cup part-frame origin = the top hook/pivot point
RIM_Z = PIVOT_Z - LINK_LEN  # rolled-rim mouth, hung below the pivot
BOT_Z = RIM_Z - CUP_H  # narrow cup bottom
BOT_EYE_Z = BOT_Z - 0.013  # this cup's own hanging eye (pivot for the next cup)

N_CUPS = 3

# --- gutter outlet bracket (root) ---
GUTTER_R = 0.034  # outlet spout radius
GUTTER_LEN = 0.040  # short outlet spout length
BRACKET_EYE_Z = -GUTTER_LEN - 0.010  # eye under the outlet where cup_1 hooks on


def _cup_shell_mesh(name: str):
    """Hollow thin-wall tapered cup shell.

    Open wide mouth at z=0 (rolled rim), tapering down to a small near-apex
    bottom at z=-CUP_H. Profiles are (radius, z); the inner wall is offset inward
    by WALL so the cup reads as real sheet metal, hollow inside.
    """
    outer = [
        (CUP_BOT_R, BOT_Z),
        (CUP_BOT_R + (CUP_TOP_R - CUP_BOT_R) * 0.5, BOT_Z + CUP_H * 0.5),
        (CUP_TOP_R, RIM_Z),
    ]
    inner = [
        (max(CUP_BOT_R - WALL, 0.0008), BOT_Z + WALL * 1.5),
        (CUP_BOT_R + (CUP_TOP_R - CUP_BOT_R) * 0.5 - WALL, BOT_Z + CUP_H * 0.5),
        (CUP_TOP_R - WALL, RIM_Z),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=64)
    return mesh_from_geometry(geom, name)


def _vertical_eye_loop(center_z: float, name: str):
    """A small near-vertical wire loop (hanging eye) lying in the X-Z plane.

    The loop is a closed circle of radius EYE_R in the X-Z plane, centered on the
    cup axis at the given z. Built as a torus rotated to stand vertically.
    """
    geom = TorusGeometry(radius=EYE_R, tube=WIRE_R, radial_segments=12,
                         tubular_segments=28)
    geom.rotate_x(math.pi / 2.0)  # lay the torus into the X-Z plane (axis -> Y)
    geom.translate(0.0, 0.0, center_z)
    return mesh_from_geometry(geom, name)


def _link_bail(pivot_z: float, rim_z: float, rim_edge_r: float, name: str):
    """The bail wire that hangs the cup: it hooks both opposite rim edges and
    rises to a single hook point at the pivot.

    Runs from the +X rim edge up to the pivot point on the cup axis (where it
    hooks the eye of the part above), then back down to the -X rim edge. This is
    the classic two-point bail of a hanging cup, so the wire is physically tied
    to the rim at both ends and to the hook above at its apex.
    """
    pts = [
        (rim_edge_r, 0.0, rim_z),  # seats on +X rim edge
        (rim_edge_r * 0.55, 0.0, rim_z + (pivot_z - rim_z) * 0.55),
        (0.0, 0.0, pivot_z),  # hook apex on the axis at the pivot
        (-rim_edge_r * 0.55, 0.0, rim_z + (pivot_z - rim_z) * 0.55),
        (-rim_edge_r, 0.0, rim_z),  # seats on -X rim edge
    ]
    geom = tube_from_spline_points(
        pts, radius=WIRE_R, samples_per_segment=16, radial_segments=12,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gutter_rain_chain")

    zinc = model.material("zinc", rgba=(0.55, 0.58, 0.52, 1.0))  # weathered galvanized
    rim_metal = model.material("rim_metal", rgba=(0.80, 0.82, 0.84, 1.0))  # bright rim
    wire_steel = model.material("wire_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    bracket_metal = model.material("bracket_metal", rgba=(0.68, 0.70, 0.72, 1.0))

    # ------------------------------------------------------------------
    # ROOT: gutter outlet bracket. A short outlet spout under the gutter with a
    # mounting flange and a fixed hanging eye that carries the whole chain.
    # ------------------------------------------------------------------
    bracket = model.part("gutter_bracket")

    # Mounting flange / collar under the gutter (the part that bolts to the gutter
    # outlet hole).
    bracket.visual(
        Cylinder(radius=GUTTER_R + 0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=bracket_metal,
        name="bracket_flange",
    )
    # Short outlet spout pointing down out of the gutter.
    bracket.visual(
        Cylinder(radius=GUTTER_R, length=GUTTER_LEN),
        origin=Origin(xyz=(0.0, 0.0, -GUTTER_LEN / 2.0 - 0.008)),
        material=bracket_metal,
        name="outlet_spout",
    )
    # A small cross-bar / hanger lug under the spout that the top link hooks onto.
    bracket.visual(
        Box((0.020, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_EYE_Z + 0.006)),
        material=bracket_metal,
        name="hanger_lug",
    )
    bracket.visual(
        _vertical_eye_loop(BRACKET_EYE_Z, "bracket_eye_mesh"),
        material=wire_steel,
        name="bracket_eye",
    )
    bracket.inertial = Inertial.from_geometry(
        Cylinder(radius=GUTTER_R, length=GUTTER_LEN), mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -GUTTER_LEN / 2.0)),
    )

    # ------------------------------------------------------------------
    # CUPS: a vertical chain of identical tapered cups, each a REVOLUTE pendulum
    # hung from the eye of the part above it.
    # ------------------------------------------------------------------
    prev_part = bracket
    # The pivot z (in the PARENT's frame) where the current cup's link grabs the
    # eye above. For cup_1 the parent is the bracket eye; for later cups it is the
    # previous cup's bottom eye.
    parent_pivot_z = BRACKET_EYE_Z

    for i in range(1, N_CUPS + 1):
        cup = model.part(f"cup_{i}")

        # Cup body (hollow tapered shell).
        cup.visual(
            _cup_shell_mesh(f"cup_{i}_shell"),
            material=zinc,
            name="cup_shell",
        )
        # Bright rolled rim at the open mouth.
        rim_geom = TorusGeometry(radius=RIM_CENTER_R, tube=RIM_TUBE,
                                 radial_segments=16, tubular_segments=48)
        cup.visual(
            mesh_from_geometry(rim_geom, f"cup_{i}_rim"),
            origin=Origin(xyz=(0.0, 0.0, RIM_Z)),
            material=rim_metal,
            name="cup_rim",
        )
        # Bail wire: hooks both opposite rim edges and rises to the pivot/hook on
        # the cup axis (where it grabs the eye of the part above). Seating it on
        # the rim edges ties the whole hanging assembly to the cup body.
        cup.visual(
            _link_bail(PIVOT_Z, RIM_Z, RIM_CENTER_R, f"cup_{i}_bail"),
            material=wire_steel,
            name="link_wire",
        )
        # Closed boss disc across the cup bottom: closes the narrow apex and gives
        # the bottom stem/eye a solid metal anchor connected to the shell walls.
        cup.visual(
            Cylinder(radius=CUP_BOT_R - WALL * 0.5, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, BOT_Z + 0.002)),
            material=zinc,
            name="bottom_boss",
        )
        # Short stem dropping from the bottom boss down to this cup's hanging eye.
        stem_top = BOT_Z + 0.004
        stem_bot = BOT_EYE_Z
        cup.visual(
            Cylinder(radius=0.0026, length=(stem_top - stem_bot)),
            origin=Origin(xyz=(0.0, 0.0, (stem_top + stem_bot) / 2.0)),
            material=wire_steel,
            name="bottom_stem",
        )
        # This cup's own hanging eye at the bottom, carrying the next cup.
        cup.visual(
            _vertical_eye_loop(BOT_EYE_Z, f"cup_{i}_bottom_eye"),
            material=wire_steel,
            name="bottom_eye",
        )
        cup.inertial = Inertial.from_geometry(
            Cylinder(radius=CUP_TOP_R, length=CUP_H), mass=0.18,
            origin=Origin(xyz=(0.0, 0.0, -CUP_H * 0.45)),
        )

        # REVOLUTE pendulum: the cup part frame origin IS the top pivot/hook
        # point. The joint origin is placed at the PARENT's hanging-eye pivot, so
        # at q=0 the cup's hook coincides with that eye and the cup hangs straight
        # down. Positive q rocks the cup (and everything below it) about the
        # horizontal Y axis through the pivot, like a real pendulum.
        model.articulation(
            f"swing_{i}",
            ArticulationType.REVOLUTE,
            parent=prev_part,
            child=cup,
            origin=Origin(xyz=(0.0, 0.0, parent_pivot_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.0,
                lower=math.radians(-35.0),
                upper=math.radians(35.0),
            ),
        )

        # The cup's own bottom eye becomes the pivot for the next cup, expressed
        # in THIS cup's frame at BOT_EYE_Z.
        prev_part = cup
        parent_pivot_z = BOT_EYE_Z

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def _center(aabb):
    mn, mx = aabb
    return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, (mn[2] + mx[2]) / 2.0)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bracket = object_model.get_part("gutter_bracket")
    cups = [object_model.get_part(f"cup_{i}") for i in range(1, N_CUPS + 1)]
    joints = [object_model.get_articulation(f"swing_{i}") for i in range(1, N_CUPS + 1)]

    # ------------------------------------------------------------------
    # Intentional overlaps: each cup's bail hooks the eye above and seats onto
    # the rim edges; the bottom boss/stem weld into the cup bottom and carry the
    # hanging eye that the next cup grabs.
    # ------------------------------------------------------------------
    ctx.allow_overlap(cups[0], bracket, elem_a="link_wire", elem_b="bracket_eye",
                      reason="Top cup's bail hook passes through the fixed bracket eye.")
    for i, cup in enumerate(cups):
        ctx.allow_overlap(cup, cup, elem_a="link_wire", elem_b="cup_rim",
                          reason="Bail wire seats onto the rolled rim edges of its cup.")
        ctx.allow_overlap(cup, cup, elem_a="link_wire", elem_b="cup_shell",
                          reason="Bail wire ends seat at the rim where the shell wall is.")
        ctx.allow_overlap(cup, cup, elem_a="bottom_boss", elem_b="cup_shell",
                          reason="Closing boss disc is seamed into the cup-bottom wall.")
        ctx.allow_overlap(cup, cup, elem_a="bottom_stem", elem_b="bottom_boss",
                          reason="Bottom stem is fixed into the closing boss.")
        ctx.allow_overlap(cup, cup, elem_a="bottom_stem", elem_b="bottom_eye",
                          reason="Bottom stem carries (passes into) the hanging eye.")
        if i + 1 < len(cups):
            nxt = cups[i + 1]
            ctx.allow_overlap(nxt, cup, elem_a="link_wire", elem_b="bottom_eye",
                              reason="Each cup's bail hook passes through the bottom "
                              "eye of the cup above it.")

    # ------------------------------------------------------------------
    # Hero geometry: it is a vertical chain of N tapered hollow cups.
    # ------------------------------------------------------------------
    ctx.check(
        "chain has the expected number of cups",
        len(cups) == N_CUPS and N_CUPS == 3,
        details=f"n_cups={len(cups)}",
    )

    # Cup shape: wide open mouth on top, narrow bottom (a tapered funnel cup).
    ctx.check(
        "cup tapers from a wide mouth to a small bottom",
        CUP_BOT_R < CUP_TOP_R * 0.5,
        details=f"top_r={CUP_TOP_R}, bot_r={CUP_BOT_R}",
    )
    # Cup is hollow (thin wall).
    ctx.check(
        "cup body modeled hollow (thin sheet-metal wall)",
        WALL < CUP_TOP_R * 0.05,
        details=f"wall={WALL}, top_r={CUP_TOP_R}",
    )

    # Each cup's rim (open mouth) is ABOVE its bottom -> funnel points down.
    for i, cup in enumerate(cups, start=1):
        rim_c = _center(ctx.part_element_world_aabb(cup, elem="cup_rim"))
        shell_aabb = ctx.part_element_world_aabb(cup, elem="cup_shell")
        ctx.check(
            f"cup_{i}: open mouth (rim) sits above the narrow bottom",
            rim_c[2] > _center(shell_aabb)[2],
            details=f"rim_z={rim_c[2]}, shell_center_z={_center(shell_aabb)[2]}",
        )

    # Cups descend in a vertical stack: each cup is below the previous one.
    rim_zs = [_center(ctx.part_element_world_aabb(c, elem="cup_rim"))[2] for c in cups]
    descending = all(rim_zs[k + 1] < rim_zs[k] - 0.02 for k in range(len(rim_zs) - 1))
    ctx.check(
        "cups hang in a descending vertical chain",
        descending,
        details=f"rim_zs={rim_zs}",
    )
    # Chain hangs below the gutter bracket.
    brk_z = _center(ctx.part_element_world_aabb(bracket, elem="outlet_spout"))[2]
    ctx.check(
        "the whole chain hangs below the gutter outlet bracket",
        rim_zs[0] < brk_z,
        details=f"top_cup_rim_z={rim_zs[0]}, bracket_z={brk_z}",
    )

    # ------------------------------------------------------------------
    # Each cup hangs from the part above with a real link (no floating parts):
    # the cup's link wire reaches up to the parent's eye.
    # ------------------------------------------------------------------
    ctx.expect_contact(cups[0], bracket, elem_a="link_wire", elem_b="bracket_eye",
                       contact_tol=0.006, name="top cup link meets bracket eye")
    for i in range(1, len(cups)):
        ctx.expect_contact(cups[i], cups[i - 1], elem_a="link_wire",
                           elem_b="bottom_eye", contact_tol=0.006,
                           name=f"cup_{i + 1} link meets cup_{i} bottom eye")

    # ------------------------------------------------------------------
    # Articulation: each cup swings as a REVOLUTE pendulum about a horizontal Y
    # axis through the upper pivot, hanging straight down at q=0.
    # ------------------------------------------------------------------
    for i, joint in enumerate(joints, start=1):
        ctx.check(
            f"swing_{i} is a revolute joint",
            str(joint.articulation_type).upper().endswith("REVOLUTE"),
            details=f"type={joint.articulation_type}",
        )
        ax = joint.axis
        ctx.check(
            f"swing_{i} pivots about a horizontal axis",
            abs(ax[1]) > 0.99 and abs(ax[0]) < 0.02 and abs(ax[2]) < 0.02,
            details=f"axis={ax}",
        )
        lo, hi = joint.motion_limits.lower, joint.motion_limits.upper
        ctx.check(
            f"swing_{i} allows a real pendulum swing both ways",
            lo < math.radians(-15.0) and hi > math.radians(15.0),
            details=f"lower={lo}, upper={hi}",
        )

    # Decisive pose check: swinging the TOP cup's joint moves that cup (and the
    # whole chain below) sideways, like a pendulum.
    rest_c = _center(ctx.part_world_aabb(cups[0]))
    last_rest = _center(ctx.part_world_aabb(cups[-1]))
    with ctx.pose({joints[0]: math.radians(30.0)}):
        swung_c = _center(ctx.part_world_aabb(cups[0]))
        last_swung = _center(ctx.part_world_aabb(cups[-1]))
    ctx.check(
        "swinging the top joint moves the top cup sideways",
        abs(swung_c[0] - rest_c[0]) > 0.01,
        details=f"rest_x={rest_c[0]}, swung_x={swung_c[0]}",
    )
    ctx.check(
        "swinging the top joint carries the whole chain (bottom cup moves too)",
        abs(last_swung[0] - last_rest[0]) > 0.02,
        details=f"rest_x={last_rest[0]}, swung_x={last_swung[0]}",
    )

    # ------------------------------------------------------------------
    # Materials / colors: weathered zinc cups, bright rims, steel wire.
    # ------------------------------------------------------------------
    shell_rgba = cups[0].get_visual("cup_shell").material.rgba
    rim_rgba = cups[0].get_visual("cup_rim").material.rgba
    wire_rgba = cups[0].get_visual("link_wire").material.rgba
    ctx.check(
        "cup body is a muted weathered metal (greyish, not saturated)",
        max(shell_rgba[:3]) - min(shell_rgba[:3]) < 0.12 and 0.4 < shell_rgba[0] < 0.75,
        details=f"shell_rgba={shell_rgba}",
    )
    ctx.check(
        "rim reads brighter than the cup body",
        sum(rim_rgba[:3]) > sum(shell_rgba[:3]),
        details=f"rim_rgba={rim_rgba}, shell_rgba={shell_rgba}",
    )
    ctx.check(
        "hanging wire is steel-grey",
        min(wire_rgba[:3]) > 0.6 and max(wire_rgba[:3]) - min(wire_rgba[:3]) < 0.15,
        details=f"wire_rgba={wire_rgba}",
    )

    return ctx.report()


object_model = build_object_model()
