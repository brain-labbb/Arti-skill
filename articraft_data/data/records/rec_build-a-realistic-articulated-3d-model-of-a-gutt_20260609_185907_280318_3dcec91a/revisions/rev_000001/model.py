from __future__ import annotations

# Gutter downpipe rain chain (kusari-doi), variant 2.
#
# Reference image: a vertical run of square, downward-tapering metal cup links
# (galvanized/aluminium grey). Each cup is a hollow funnel: a wide square top
# mouth and a narrower square bottom with a central drain hole, so rainwater
# cascades from cup to cup. A small wire bail loops up out of the top edge of
# each cup; the cup below hangs from that bail, with its bail reaching up
# through the drain throat of the cup above. At the very top a V-shaped wire
# hanger with two hook ends straddles the gutter outlet and carries the chain.
#
# Real mechanism: the chain is articulated. Each cup hangs from the bail of the
# cup above and is free to swing (pendulum) about that link. The faithful
# articulation is therefore a stack of REVOLUTE swing joints, one per cup, plus
# a swing joint where the first cup hangs from the hanger. The hanger is the
# fixed root that grips the gutter.
#
# Coordinate convention (per cup, "mouth frame"):
#   - the wide top mouth lies at z = 0
#   - the funnel body and bottom drain descend into -z (bottom at z = -CUP_H)
#   - the wire bail arches up to its apex at z = +BAIL_RISE, where the next cup
#     down hooks on
#   - so the lower cup's mouth must sit at z = -(CUP_H + BAIL_RISE) in the upper
#     cup's mouth frame for its bail apex to reach the upper cup's drain.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
TOP_HALF = 0.0380        # half-width of the square top mouth of a cup
BOT_HALF = 0.0230        # half-width of the square bottom of a cup
CUP_H = 0.0640           # cup wall height (top mouth down to bottom)
WALL = 0.0018            # sheet-metal wall thickness
DRAIN_R = 0.0115         # radius of the central drain hole in the cup bottom

BAIL_R = 0.0022          # wire radius of the connecting bail loop
BAIL_RISE = 0.0240       # how far the bail loop apex rises above the cup mouth
BAIL_SPAN = 2 * (TOP_HALF - WALL)  # legs anchor on the mouth rim walls (+/- y)

# Transverse hanger bar spanning the drain at the cup bottom. The lower cup's
# vertical bail loop hooks over this bar, like a chain link over an axle.
BAR_R = 0.0024           # bottom hanger-bar wire radius
BAR_HALF = DRAIN_R + 0.004  # bar half-length (overhangs the drain so the loop
#                             cannot slip off sideways)

N_CUPS = 5

# Drop of each cup's mouth below the parent cup's mouth. The lower cup's bail
# loop apex (at +BAIL_RISE in the lower frame) must reach up and hook over the
# parent cup's bottom hanger bar (at -CUP_H in the parent frame). We seat the
# loop apex a touch above the bar so the loop's inner-top rests on the bar.
CUP_DROP = CUP_H + BAIL_RISE - 0.003  # loop apex laps ~3 mm over the bar

galv = (0.66, 0.68, 0.70, 1.0)        # galvanized / brushed aluminium grey
galv_dark = (0.55, 0.57, 0.59, 1.0)   # slightly darker rim lip
wire_grey = (0.60, 0.62, 0.64, 1.0)   # zinc wire bail / hanger


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------
def _square_funnel_shell() -> cq.Workplane:
    """Hollow, downward-tapering square cup shell.

    A lofted square frustum, shelled to sheet-metal thickness with the wide top
    mouth left open, and a central drain hole punched through the bottom so
    water (and the bail of the cup below) passes through. Local frame: top mouth
    plane at z=0, body descending to z=-CUP_H.
    """
    top = TOP_HALF
    bot = BOT_HALF
    h = CUP_H

    outer = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .rect(2 * top, 2 * top)
        .workplane(offset=-h)
        .rect(2 * bot, 2 * bot)
        .loft(combine=True)
    )

    # Shell out through the top mouth (+z face) -> hollow cup with a flat bottom.
    cup = outer.faces(">Z").shell(-WALL)

    # Punch the central drain hole through the bottom plate.
    cup = (
        cup.faces("<Z")
        .workplane(centerOption="CenterOfMass")
        .hole(2 * DRAIN_R)
    )

    # Transverse hanger bar across the drain (along Y), fused to the bottom plate
    # on both sides of the hole. The cup below hooks its bail loop over this bar.
    bar = (
        cq.Workplane("XY")
        .transformed(
            offset=(0.0, 0.0, -CUP_H + WALL * 0.5),
            rotate=(90.0, 0.0, 0.0),
        )
        .circle(BAR_R)
        .extrude(BAR_HALF, both=True)
    )
    cup = cup.union(bar)
    return cup


def _cup_rim_ring() -> cq.Workplane:
    """A thin square lip ring around the open mouth so the edge reads as real
    folded sheet metal and gives the bail anchors material to sit on."""
    outer = TOP_HALF + 0.0015
    inner = TOP_HALF - 2 * WALL
    return (
        cq.Workplane("XY")
        .rect(2 * outer, 2 * outer)
        .rect(2 * inner, 2 * inner)
        .extrude(WALL)
    )


def _bail_loop() -> object:
    """Wire bail: a vertical loop in the X-Z plane rising above the cup mouth.

    Two legs anchor on the +X and -X top edges of the mouth (biting down into the
    shell wall so the bail is fused to the cup), arch up, and close into an oval
    loop whose apex is at z=+BAIL_RISE. Because the loop lies in the X-Z plane it
    encircles a Y-direction bar, so it hooks over the bottom hanger bar of the
    cup above exactly like a chain link over an axle."""
    s = BAIL_SPAN / 2.0          # leg anchor half-spacing across the mouth (in X)
    rise = BAIL_RISE
    foot_z = -0.012              # feet bite down into the shell wall
    loop_cx = 0.0
    loop_w = s * 0.85            # loop half-width in X near the apex
    loop_top = rise
    loop_bot = rise - 0.014      # vertical extent of the closed loop eye

    # Ordered closed loop of points in the X-Z plane (y=0), going:
    # left foot -> up -> over the top -> down -> right foot -> across the bottom.
    pts = [
        (-s, 0.0, foot_z),
        (-s, 0.0, rise * 0.45),
        (-loop_w, 0.0, loop_bot),
        (-loop_w, 0.0, loop_top),
        (loop_cx, 0.0, loop_top + 0.003),
        (loop_w, 0.0, loop_top),
        (loop_w, 0.0, loop_bot),
        (s, 0.0, rise * 0.45),
        (s, 0.0, foot_z),
    ]
    return tube_from_spline_points(
        pts,
        radius=BAIL_R,
        samples_per_segment=12,
        radial_segments=10,
        cap_ends=True,
    )


def _hanger_bracket() -> object:
    """V-shaped wire hanger that straddles the gutter outlet. Two hook ends curl
    over the gutter lip; the wire dips to a low apex where cup 1 hangs. Local
    frame: apex (lowest point, where cup 1 hangs) at the origin; hooks rise +z."""
    hook_z = 0.090
    spread = 0.075
    curl = 0.012
    pts = [
        (-spread - curl, 0.0, hook_z - curl),  # left hook tip curled inboard
        (-spread, 0.0, hook_z),
        (-spread, 0.0, hook_z - 0.010),
        (-spread * 0.45, 0.0, hook_z * 0.40),
        (0.0, 0.0, 0.0),                        # apex - cup 1 hangs here
        (spread * 0.45, 0.0, hook_z * 0.40),
        (spread, 0.0, hook_z - 0.010),
        (spread, 0.0, hook_z),
        (spread + curl, 0.0, hook_z - curl),    # right hook tip
    ]
    wire = tube_from_spline_points(
        pts,
        radius=0.0026,
        samples_per_segment=16,
        radial_segments=12,
        cap_ends=True,
    )

    # Transverse axle bar at the apex (along Y) that cup 1's bail loop hooks over.
    bar = tube_from_spline_points(
        [
            (0.0, -BAR_HALF, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, BAR_HALF, 0.0),
        ],
        radius=BAR_R,
        samples_per_segment=6,
        radial_segments=10,
        cap_ends=True,
    )
    return wire.merge(bar)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gutter_rain_chain")

    m_galv = model.material("galv", rgba=galv)
    m_galv_dark = model.material("galv_dark", rgba=galv_dark)
    m_wire = model.material("wire", rgba=wire_grey)

    # --- Root: the gutter hanger bracket ---------------------------------
    hanger = model.part("gutter_hanger")
    hanger.visual(
        mesh_from_geometry(_hanger_bracket(), "hanger_wire"),
        material=m_wire,
        name="hanger_wire",
    )

    # Shared meshes (identical cups reuse the same managed asset).
    cup_shell_mesh = mesh_from_cadquery(_square_funnel_shell(), "cup_shell")
    cup_rim_mesh = mesh_from_cadquery(_cup_rim_ring(), "cup_rim")
    bail_mesh = mesh_from_geometry(_bail_loop(), "cup_bail")

    prev = "gutter_hanger"
    for i in range(1, N_CUPS + 1):
        cup = model.part(f"cup_{i}")
        cup.visual(cup_shell_mesh, material=m_galv, name="shell")
        cup.visual(cup_rim_mesh, material=m_galv_dark, name="rim")
        cup.visual(bail_mesh, material=m_wire, name="bail")

        if i == 1:
            # Cup 1 hangs from the hanger apex bar (hanger origin = apex, bar at
            # z=0). Drop cup 1's mouth so its bail loop apex (+BAIL_RISE) hooks
            # over the bar, lapping ~3 mm. Swing about the gutter run (Y).
            joint_origin = Origin(xyz=(0.0, 0.0, -(BAIL_RISE - 0.003)))
        else:
            # The lower cup's mouth drops CUP_DROP below the parent mouth so its
            # bail loop apex hooks over the parent cup's bottom hanger bar.
            joint_origin = Origin(xyz=(0.0, 0.0, -CUP_DROP))

        model.articulation(
            f"swing_{i}",
            ArticulationType.REVOLUTE,
            parent=prev,
            child=cup,
            origin=joint_origin,
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=2.0, lower=-0.7, upper=0.7
            ),
        )
        prev = f"cup_{i}"

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hanger = object_model.get_part("gutter_hanger")
    cups = [object_model.get_part(f"cup_{i}") for i in range(1, N_CUPS + 1)]
    swings = [object_model.get_articulation(f"swing_{i}") for i in range(1, N_CUPS + 1)]

    # Each cup's bail loop hooks over the bottom hanger bar of the cup above, so
    # the loop and the bar (part of the upper shell) intentionally overlap at the
    # captured chain link. Allow that scoped capture and prove it with contact.
    for upper, lower in zip(cups, cups[1:]):
        ctx.allow_overlap(
            upper,
            lower,
            elem_a="shell",
            elem_b="bail",
            reason=(
                "The lower cup's bail loop intentionally hooks over the upper "
                "cup's bottom hanger bar to form the captured chain link."
            ),
        )
        ctx.expect_contact(
            upper,
            lower,
            elem_a="shell",
            elem_b="bail",
            contact_tol=1e-4,
            name=f"{lower.name} loop is captured on {upper.name} bar",
        )
    # Cup 1's bail loop hooks over the hanger apex bar.
    ctx.allow_overlap(
        hanger,
        cups[0],
        elem_a="hanger_wire",
        elem_b="bail",
        reason="Cup 1's bail loop is captured over the hanger apex bar.",
    )
    ctx.expect_contact(
        hanger,
        cups[0],
        elem_a="hanger_wire",
        elem_b="bail",
        contact_tol=1e-4,
        name="cup_1 loop is captured on the hanger bar",
    )

    # --- Mechanical claims: every link is a Y-axis revolute swing ----------
    for sw in swings:
        ctx.check(
            f"{sw.name} is revolute",
            str(sw.articulation_type).endswith("REVOLUTE"),
            details=f"{sw.name} type={sw.articulation_type}",
        )
        ax = sw.axis
        ctx.check(
            f"{sw.name} swings about Y",
            abs(ax[1]) > 0.9 and abs(ax[0]) < 0.1 and abs(ax[2]) < 0.1,
            details=f"axis={ax}",
        )

    # --- Hero geometry present --------------------------------------------
    ctx.check("five cups present", len(cups) == N_CUPS, details=f"{len(cups)} cups")
    for cup in cups:
        names = {v.name for v in cup.visuals}
        ctx.check(
            f"{cup.name} has funnel shell + bail",
            {"shell", "bail"}.issubset(names),
            details=str(sorted(names)),
        )

    # --- The chain hangs downward as a vertical stack (rest pose) ----------
    z_centers = []
    for cup in cups:
        aabb = ctx.part_world_aabb(cup)
        assert aabb is not None
        z_centers.append(0.5 * (aabb[0][2] + aabb[1][2]))
    for upper_z, lower_z in zip(z_centers, z_centers[1:]):
        ctx.check(
            "cups descend in a stack",
            lower_z < upper_z - 0.03,
            details=f"upper_z={upper_z:.4f} lower_z={lower_z:.4f}",
        )

    # The hanger sits at the very top above cup 1.
    hanger_aabb = ctx.part_world_aabb(hanger)
    cup1_aabb = ctx.part_world_aabb(cups[0])
    assert hanger_aabb is not None and cup1_aabb is not None
    ctx.check(
        "hanger is above the first cup",
        hanger_aabb[1][2] > cup1_aabb[1][2],
        details=f"hanger_top={hanger_aabb[1][2]:.4f} cup1_top={cup1_aabb[1][2]:.4f}",
    )

    # --- Cups are real square funnels: top mouth wider than the bottom -----
    shell = cups[0].get_visual("shell")
    sb = ctx.part_element_world_aabb(cups[0], elem=shell)
    assert sb is not None
    width_x = sb[1][0] - sb[0][0]
    height_z = sb[1][2] - sb[0][2]
    ctx.check(
        "cup is a square funnel of the right size",
        abs(width_x - 2 * TOP_HALF) < 0.01 and height_z > 0.04,
        details=f"width_x={width_x:.4f} height_z={height_z:.4f}",
    )

    # --- The bail rises above the cup mouth (so the next cup can hook on) ---
    bail = cups[0].get_visual("bail")
    bb = ctx.part_element_world_aabb(cups[0], elem=bail)
    assert bb is not None
    cup1_top = cup1_aabb[1][2]
    ctx.check(
        "bail arches above the cup mouth",
        bb[1][2] > cup1_top - 0.001,
        details=f"bail_top={bb[1][2]:.4f} cup_top={cup1_top:.4f}",
    )

    # --- Swing actually moves a cup sideways ------------------------------
    # The bottom cup's funnel body hangs below its pivot (its mouth), so swinging
    # about Y must displace the body horizontally in X. Measure the shell element
    # center, which sits well below the pivot, not the part origin (on the axis).
    bottom_shell = cups[-1].get_visual("shell")

    def _shell_center_x() -> float:
        a = ctx.part_element_world_aabb(cups[-1], elem=bottom_shell)
        assert a is not None
        return 0.5 * (a[0][0] + a[1][0])

    rest_x = _shell_center_x()
    with ctx.pose({swings[-1]: 0.7}):
        swung_x = _shell_center_x()
        ctx.check(
            "swinging the bottom cup displaces it horizontally",
            abs(swung_x - rest_x) > 0.015,
            details=f"rest_x={rest_x:.4f} swung_x={swung_x:.4f}",
        )

    return ctx.report()
