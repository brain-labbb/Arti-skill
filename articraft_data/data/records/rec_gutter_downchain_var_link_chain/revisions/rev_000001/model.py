from __future__ import annotations

# Gutter downpipe RAIN CHAIN – chain-link variant (kusari-doi, "link" style).
#
# A classic chain-link rain chain: a vertical run of repeated oval (elliptical)
# metal links hung beneath a gutter outlet bracket. Each link passes through the
# one above it, alternating orientation by 90° like a real chain. Rainwater
# runs down the links from the gutter to the ground.
#
# Real mechanism / articulation:
#   - A fixed gutter-outlet collar/bracket mounts under the gutter and carries
#     a hanging ring. It is the ROOT (does not move).
#   - Each link hangs from the link (or bracket ring) above it. The link is free
#     to SWING like a pendulum about the horizontal axis through the contact
#     point where it passes through the parent link. That swing is the primary
#     real motion of a rain chain in wind, and it is modeled as one REVOLUTE
#     joint per link.
#   - Adjacent links alternate orientation by 90° (odd links lie in the XZ
#     plane, even links in the YZ plane). Odd links swing about Y; even links
#     swing about X.
#
# Coordinate convention:
#   - +Z is up. The gutter bracket sits near z=0 (top); the chain hangs DOWN
#     into -Z. Each link's local frame origin is its TOP pivot (the contact
#     point with the parent link), and the link body extends downward.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# --- chain-link dimensions (meters) ---
LINK_HALF_LEN = 0.032   # half of the long (vertical) axis of the oval
LINK_HALF_WID = 0.014   # half of the short (horizontal) axis
LINK_WIRE_R = 0.003     # wire/tube radius

# How far the next link's pivot sits below this link's pivot.
# In a real chain, adjacent links interlock: the child's top arc passes through
# the parent's bottom arc. The pitch is slightly less than the full link length.
LINK_PITCH = 2.0 * LINK_HALF_LEN - 2.0 * LINK_WIRE_R

# Position of the child pivot in the parent link's local frame (at the bottom
# of the parent oval where the child link passes through).
BOTTOM_PIVOT_Z = -(2.0 * LINK_HALF_LEN - 2.0 * LINK_WIRE_R)

N_LINKS = 8

# --- gutter outlet bracket (root) ---
GUTTER_R = 0.034
GUTTER_LEN = 0.040
# The bracket ring hangs below the spout; first link hooks into it.
BRACKET_RING_Z = -GUTTER_LEN - 0.014


def _oval_link_mesh(in_yz_plane: bool, name: str):
    """An oval (elliptical) chain link as a closed tube swept along an
    elliptical path.

    The oval is centred at z = -LINK_HALF_LEN (below the part-frame origin) so
    the top of the oval touches z = 0 (the pivot) and the bottom reaches
    z = -2 * LINK_HALF_LEN.

    When ``in_yz_plane`` is True the short axis of the oval is along Y instead
    of X (the link is rotated 90° about Z, like alternating links in a real
    chain).
    """
    n_pts = 36
    pts = []
    for j in range(n_pts):
        t = 2.0 * math.pi * j / n_pts
        hw = LINK_HALF_WID * math.cos(t)
        hl = LINK_HALF_LEN * math.sin(t)
        if in_yz_plane:
            pts.append((0.0, hw, -LINK_HALF_LEN + hl))
        else:
            pts.append((hw, 0.0, -LINK_HALF_LEN + hl))
    geom = tube_from_spline_points(
        pts,
        radius=LINK_WIRE_R,
        samples_per_segment=10,
        closed_spline=True,
        radial_segments=14,
        cap_ends=False,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gutter_chain_link_chain")

    # --- materials ---
    copper = model.material("copper", rgba=(0.62, 0.41, 0.26, 1.0))
    copper_dark = model.material("copper_dark", rgba=(0.48, 0.32, 0.22, 1.0))
    bracket_metal = model.material("bracket_metal", rgba=(0.50, 0.52, 0.54, 1.0))

    # ------------------------------------------------------------------
    # ROOT: gutter outlet bracket. A short outlet spout under the gutter with
    # a mounting flange and a horizontal ring that the first chain link passes
    # through.
    # ------------------------------------------------------------------
    bracket = model.part("gutter_bracket")

    # Mounting flange / collar under the gutter.
    bracket.visual(
        Cylinder(radius=GUTTER_R + 0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=bracket_metal,
        name="bracket_flange",
    )
    # Short outlet spout pointing down.
    bracket.visual(
        Cylinder(radius=GUTTER_R, length=GUTTER_LEN),
        origin=Origin(xyz=(0.0, 0.0, -GUTTER_LEN / 2.0 - 0.008)),
        material=bracket_metal,
        name="outlet_spout",
    )
    # Short vertical bar connecting the spout bottom to the hanging ring.
    bar_top = -GUTTER_LEN - 0.008
    bar_bot = BRACKET_RING_Z
    bracket.visual(
        Cylinder(radius=0.004, length=(bar_top - bar_bot)),
        origin=Origin(xyz=(0.0, 0.0, (bar_top + bar_bot) / 2.0)),
        material=bracket_metal,
        name="hanger_bar",
    )
    # Horizontal ring (lies in XY plane) at the bottom – the first link passes
    # through this ring.
    ring_r = LINK_HALF_WID + LINK_WIRE_R * 1.8  # inner clearance for link wire
    ring_tube = LINK_WIRE_R * 0.9
    ring_geom = TorusGeometry(
        radius=ring_r, tube=ring_tube,
        radial_segments=14, tubular_segments=36,
    )
    bracket.visual(
        mesh_from_geometry(ring_geom, "bracket_ring_mesh"),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_RING_Z)),
        material=bracket_metal,
        name="bracket_ring",
    )
    # Four short radial spokes connecting the hanger bar to the ring wire so the
    # ring reads as physically mounted on the bar.
    spoke_inner_r = 0.004   # matches hanger bar outer radius
    spoke_outer_r = ring_r  # meets the ring tube centre
    for j, angle in enumerate([0, 90, 180, 270]):
        a = math.radians(angle)
        ca, sa = math.cos(a), math.sin(a)
        spoke_pts = [
            (spoke_inner_r * ca, spoke_inner_r * sa, BRACKET_RING_Z),
            (spoke_outer_r * ca, spoke_outer_r * sa, BRACKET_RING_Z),
        ]
        spoke_geom = tube_from_spline_points(
            spoke_pts, radius=0.0018, samples_per_segment=4,
            radial_segments=8, cap_ends=True,
        )
        bracket.visual(
            mesh_from_geometry(spoke_geom, f"ring_spoke_{j}_mesh"),
            material=bracket_metal,
            name=f"ring_spoke_{j}",
        )
    bracket.inertial = Inertial.from_geometry(
        Cylinder(radius=GUTTER_R, length=GUTTER_LEN), mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -GUTTER_LEN / 2.0)),
    )

    # ------------------------------------------------------------------
    # CHAIN LINKS: a vertical sequence of oval links, each a REVOLUTE pendulum
    # hung from the link (or bracket) above. Adjacent links alternate plane.
    # ------------------------------------------------------------------
    prev_part = bracket
    parent_pivot_z = BRACKET_RING_Z  # first link hooks into the bracket ring

    for i in range(N_LINKS):
        link = model.part(f"link_{i}")

        # Alternate plane: even indices in XZ, odd in YZ.
        in_yz = (i % 2 == 1)
        mat = copper if i % 2 == 0 else copper_dark

        # Oval link body (the main visible geometry).
        link.visual(
            _oval_link_mesh(in_yz, f"link_{i}_oval"),
            material=mat,
            name="oval_body",
        )
        link.inertial = Inertial.from_geometry(
            Cylinder(radius=LINK_HALF_WID, length=2.0 * LINK_HALF_LEN),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.0, -LINK_HALF_LEN)),
        )

        # REVOLUTE pendulum: the link part-frame origin is the top pivot (where
        # it passes through the parent). Positive q swings the link about a
        # horizontal axis through that pivot.
        # Odd links (YZ plane) swing about X; even links (XZ plane) about Y.
        swing_axis = (1.0, 0.0, 0.0) if in_yz else (0.0, 1.0, 0.0)

        model.articulation(
            f"swing_{i}",
            ArticulationType.REVOLUTE,
            parent=prev_part,
            child=link,
            origin=Origin(xyz=(0.0, 0.0, parent_pivot_z)),
            axis=swing_axis,
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=2.0,
                lower=math.radians(-35.0),
                upper=math.radians(35.0),
            ),
        )

        prev_part = link
        parent_pivot_z = BOTTOM_PIVOT_Z  # child pivot in parent's local frame

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
    links = [object_model.get_part(f"link_{i}") for i in range(N_LINKS)]
    joints = [object_model.get_articulation(f"swing_{i}") for i in range(N_LINKS)]

    # ------------------------------------------------------------------
    # Intentional overlaps: adjacent links interlock (each link passes through
    # the one above it, like a real chain).
    # ------------------------------------------------------------------
    # Spoke–bar and spoke–ring junctions within the bracket (welded mount).
    for j in range(4):
        ctx.allow_overlap(
            bracket, bracket,
            elem_a=f"ring_spoke_{j}", elem_b="hanger_bar",
            reason=f"Spoke {j} is welded to the hanger bar.",
        )
        ctx.allow_overlap(
            bracket, bracket,
            elem_a=f"ring_spoke_{j}", elem_b="bracket_ring",
            reason=f"Spoke {j} is welded to the hanging ring.",
        )

    ctx.allow_overlap(
        links[0], bracket,
        elem_a="oval_body", elem_b="bracket_ring",
        reason="Top chain link passes through the bracket hanging ring.",
    )
    for j in range(4):
        ctx.allow_overlap(
            links[0], bracket,
            elem_a="oval_body", elem_b=f"ring_spoke_{j}",
            reason="Top chain link passes through the ring area where spokes are.",
        )
    for i in range(1, N_LINKS):
        ctx.allow_overlap(
            links[i], links[i - 1],
            elem_a="oval_body", elem_b="oval_body",
            reason=f"link_{i} interlocks with link_{i - 1} (chain links pass "
                   "through each other).",
        )

    # ------------------------------------------------------------------
    # Hero geometry: a vertical chain of N oval links.
    # ------------------------------------------------------------------
    ctx.check(
        "chain has the expected number of links",
        len(links) == N_LINKS and N_LINKS >= 6,
        details=f"n_links={len(links)}",
    )

    # Each link is oval: the long axis should be clearly larger than the short
    # axis.
    ctx.check(
        "oval link is elongated (long axis > short axis)",
        LINK_HALF_LEN > LINK_HALF_WID * 1.5,
        details=f"half_len={LINK_HALF_LEN}, half_wid={LINK_HALF_WID}",
    )

    # Links descend vertically: each link is below the previous one.
    link_zs = [_center(ctx.part_world_aabb(lk))[2] for lk in links]
    descending = all(
        link_zs[k + 1] < link_zs[k] - 0.02 for k in range(len(link_zs) - 1)
    )
    ctx.check(
        "links hang in a descending vertical chain",
        descending,
        details=f"link_zs={link_zs}",
    )

    # Chain hangs below the gutter bracket.
    brk_z = _center(ctx.part_element_world_aabb(bracket, elem="outlet_spout"))[2]
    ctx.check(
        "the whole chain hangs below the gutter outlet bracket",
        link_zs[0] < brk_z,
        details=f"top_link_z={link_zs[0]}, bracket_z={brk_z}",
    )

    # ------------------------------------------------------------------
    # Adjacent links alternate orientation (90° rotation).
    # Even links should be wider in X, odd links wider in Y.
    # ------------------------------------------------------------------
    for i, lk in enumerate(links):
        d = _ext(ctx.part_world_aabb(lk))
        if i % 2 == 0:
            ctx.check(
                f"link_{i} (even) is wider in X than Y (XZ plane)",
                d[0] > d[1] * 1.2,
                details=f"dx={d[0]:.4f}, dy={d[1]:.4f}",
            )
        else:
            ctx.check(
                f"link_{i} (odd) is wider in Y than X (YZ plane)",
                d[1] > d[0] * 1.2,
                details=f"dx={d[0]:.4f}, dy={d[1]:.4f}",
            )

    # ------------------------------------------------------------------
    # Each link hangs from the part above with real contact (no floating).
    # ------------------------------------------------------------------
    ctx.expect_contact(
        links[0], bracket,
        elem_a="oval_body", elem_b="bracket_ring",
        contact_tol=0.008,
        name="top link meets bracket ring",
    )
    for i in range(1, N_LINKS):
        ctx.expect_contact(
            links[i], links[i - 1],
            elem_a="oval_body", elem_b="oval_body",
            contact_tol=0.008,
            name=f"link_{i} meets link_{i - 1}",
        )

    # ------------------------------------------------------------------
    # Articulation: each link is a REVOLUTE pendulum about a horizontal axis.
    # ------------------------------------------------------------------
    for i, joint in enumerate(joints):
        ctx.check(
            f"swing_{i} is a revolute joint",
            str(joint.articulation_type).upper().endswith("REVOLUTE"),
            details=f"type={joint.articulation_type}",
        )
        ax = joint.axis
        is_horizontal = (
            abs(ax[2]) < 0.02
            and (abs(ax[0]) > 0.99 or abs(ax[1]) > 0.99)
        )
        ctx.check(
            f"swing_{i} pivots about a horizontal axis",
            is_horizontal,
            details=f"axis={ax}",
        )
        lo, hi = joint.motion_limits.lower, joint.motion_limits.upper
        ctx.check(
            f"swing_{i} allows a real pendulum swing both ways",
            lo < math.radians(-15.0) and hi > math.radians(15.0),
            details=f"lower={lo}, upper={hi}",
        )

    # Decisive pose check: swinging the top joint moves that link sideways.
    rest_c = _center(ctx.part_world_aabb(links[0]))
    last_rest = _center(ctx.part_world_aabb(links[-1]))
    with ctx.pose({joints[0]: math.radians(30.0)}):
        swung_c = _center(ctx.part_world_aabb(links[0]))
        last_swung = _center(ctx.part_world_aabb(links[-1]))
    ctx.check(
        "swinging the top joint moves the top link sideways",
        abs(swung_c[0] - rest_c[0]) > 0.005 or abs(swung_c[1] - rest_c[1]) > 0.005,
        details=f"rest=({rest_c[0]:.4f},{rest_c[1]:.4f}), "
                f"swung=({swung_c[0]:.4f},{swung_c[1]:.4f})",
    )
    ctx.check(
        "swinging the top joint carries the whole chain (bottom link moves too)",
        abs(last_swung[0] - last_rest[0]) > 0.01 or abs(last_swung[1] - last_rest[1]) > 0.01,
        details=f"rest=({last_rest[0]:.4f},{last_rest[1]:.4f}), "
                f"swung=({last_swung[0]:.4f},{last_swung[1]:.4f})",
    )

    # ------------------------------------------------------------------
    # Materials: copper/bronze links, steel-grey bracket.
    # ------------------------------------------------------------------
    even_rgba = links[0].get_visual("oval_body").material.rgba
    odd_rgba = links[1].get_visual("oval_body").material.rgba
    bracket_rgba = bracket.get_visual("bracket_ring").material.rgba
    ctx.check(
        "even links are warm copper/bronze",
        even_rgba[0] > even_rgba[2] and even_rgba[0] > 0.45,
        details=f"even_rgba={even_rgba}",
    )
    ctx.check(
        "bracket is neutral grey metal",
        max(bracket_rgba[:3]) - min(bracket_rgba[:3]) < 0.12
        and 0.35 < bracket_rgba[0] < 0.70,
        details=f"bracket_rgba={bracket_rgba}",
    )

    return ctx.report()


object_model = build_object_model()
