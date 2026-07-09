from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Part,
    Sphere,
    TestContext,
    TestReport,
    WirePath,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Widespread two-handle faucet with bridge-arch spout, polished chrome.
#
# Layout (meters, Z up, spout curves forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center: bridge arch tube spanning between valve columns, with a spout
#     descending from the arch center
#   - valve columns at x = +/-0.15: tapered cylindrical pedestals with
#     decorative ring ridges and narrow dark seams at the deck
#   - lever handles on each valve, tilting forward-back about the X axis
#
# Articulation: each lever handle is a revolute joint about the horizontal X
# axis, range -0.5 to +0.5 rad (~±28°). Positive q raises the lever tip.
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150  # valve column centers at +/-0.150 -> 0.30 m spread

# Valve pedestals (tapered cylinders)
V_RAD_BASE = 0.028
V_RAD_TOP = 0.018
V_HEIGHT = 0.070

# Decorative rings on pedestals
RING_TUBE_R = 0.0025  # ring cross-section radius
RING_HEIGHTS = [0.015, 0.035, 0.055]  # z positions of ring centers

# Stem on top of each valve for lever pivot
STEM_R = 0.006
STEM_H = 0.018

# Lever handle
LEVER_PIVOT_Z = V_HEIGHT + STEM_H  # pivot height above deck
LEVER_ARM_R = 0.005
LEVER_ARM_LEN = 0.055
LEVER_CAP_R = 0.007

# Bridge arch
BRIDGE_TUBE_R = 0.009
BRIDGE_PEAK_Z = 0.088
BRIDGE_END_Z = V_HEIGHT + 0.005  # just above valve tops (0.075)

# Spout
SPOUT_TUBE_R = 0.013
SPOUT_REACH_Y = 0.130  # forward reach
SPOUT_TIP_Z = 0.035  # tip height above deck

# Base seams
SEAM_H = 0.002
SEAM_EXTRA = 0.003  # extra radius beyond pedestal base

# Bridge center column
BRIDGE_COL_R = 0.016
BRIDGE_COL_H = 0.080  # reaches up to meet bridge arch

# Deck
DECK_W = 0.42
DECK_D = 0.20
DECK_H = 0.022


def _tapered_cylinder(r_base: float, r_top: float, height: float) -> cq.Workplane:
    """Tapered cylinder, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .circle(r_base)
        .workplane(offset=height)
        .circle(r_top)
        .loft()
    )


def _pedestal_radius_at(height: float) -> float:
    """Interpolate pedestal radius at a given height."""
    t = max(0.0, min(1.0, height / V_HEIGHT))
    return V_RAD_BASE + t * (V_RAD_TOP - V_RAD_BASE)


def _bridge_arch_points() -> list[tuple[float, float, float]]:
    """Points for the bridge arch path from left valve to right valve."""
    wp = (
        WirePath((-HANDLE_SPREAD_X + 0.02, 0.0, BRIDGE_END_Z))
        .bezier_to(
            (-0.06, 0.0, BRIDGE_PEAK_Z - 0.005),
            (-0.02, 0.0, BRIDGE_PEAK_Z),
            (0.0, 0.0, BRIDGE_PEAK_Z),
            samples=16,
        )
        .bezier_to(
            (0.02, 0.0, BRIDGE_PEAK_Z),
            (0.06, 0.0, BRIDGE_PEAK_Z - 0.005),
            (HANDLE_SPREAD_X - 0.02, 0.0, BRIDGE_END_Z),
            samples=16,
        )
    )
    return wp.to_points()


def _spout_path_points() -> list[tuple[float, float, float]]:
    """Points for the spout descending from the bridge center."""
    wp = (
        WirePath((0.0, 0.0, BRIDGE_PEAK_Z - 0.005))
        .bezier_to(
            (0.0, 0.04, BRIDGE_PEAK_Z - 0.01),
            (0.0, 0.08, SPOUT_TIP_Z + 0.03),
            (0.0, SPOUT_REACH_Y, SPOUT_TIP_Z),
            samples=18,
        )
    )
    return wp.to_points()


def _add_lever_handle(part: Part, chrome: str) -> None:
    """Lever handle extending along +Y, rotating about X axis.

    Local frame origin is at the pivot point (stem top).
    The lever arm extends along +Y from the pivot.
    """
    # Pivot ball (spherical joint visual)
    part.visual(
        Sphere(radius=STEM_R * 1.3),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_ball",
    )
    # Lever arm extending along +Y
    part.visual(
        Cylinder(radius=LEVER_ARM_R, length=LEVER_ARM_LEN),
        origin=Origin(
            xyz=(0.0, LEVER_ARM_LEN / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="lever_arm",
    )
    # Rounded cap at the lever tip
    part.visual(
        Sphere(radius=LEVER_CAP_R),
        origin=Origin(xyz=(0.0, LEVER_ARM_LEN, 0.0)),
        material=chrome,
        name="lever_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))
    seam_mat = model.material("seam_dark", rgba=(0.04, 0.04, 0.05, 1.0))

    # --- Dark deck plate (root) ---
    deck = model.part("deck")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_H)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_H / 2.0)),
        material=deck_mat,
        name="deck_plate",
    )

    # --- Bridge arch with spout (center piece) ---
    bridge_spout = model.part("bridge_spout")

    # Bridge center support column (rises from deck to meet bridge arch)
    bridge_spout.visual(
        mesh_from_cadquery(
            _tapered_cylinder(BRIDGE_COL_R, BRIDGE_COL_R * 0.75, BRIDGE_COL_H),
            "bridge_column",
        ),
        material=chrome.name,
        name="bridge_column",
    )

    # Bridge arch tube
    bridge_geom = tube_from_spline_points(
        _bridge_arch_points(),
        radius=BRIDGE_TUBE_R,
        samples_per_segment=14,
        radial_segments=20,
        cap_ends=True,
    )
    bridge_spout.visual(
        mesh_from_geometry(bridge_geom, "bridge_arch_mesh"),
        material=chrome.name,
        name="bridge_arch",
    )

    # Spout descending from bridge center
    spout_geom = tube_from_spline_points(
        _spout_path_points(),
        radius=SPOUT_TUBE_R,
        samples_per_segment=14,
        radial_segments=20,
        cap_ends=True,
    )
    bridge_spout.visual(
        mesh_from_geometry(spout_geom, "spout_tube_mesh"),
        material=chrome.name,
        name="spout_tube",
    )

    # Base seam at bridge center deck base
    bridge_spout.visual(
        Cylinder(radius=BRIDGE_COL_R + SEAM_EXTRA, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2.0)),
        material=seam_mat,
        name="bridge_seam",
    )
    model.articulation(
        "deck_to_bridge_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=bridge_spout,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Valve columns and lever handles (left = -X, right = +X) ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")

        # Tapered pedestal
        valve.visual(
            mesh_from_cadquery(
                _tapered_cylinder(V_RAD_BASE, V_RAD_TOP, V_HEIGHT),
                f"{side}_valve_body",
            ),
            material=chrome.name,
            name="valve_pedestal",
        )

        # Decorative ring ridges (thin raised bands around the pedestal)
        for i, h in enumerate(RING_HEIGHTS):
            r_at_h = _pedestal_radius_at(h)
            ring_outer = r_at_h + RING_TUBE_R
            valve.visual(
                Cylinder(radius=ring_outer, length=RING_TUBE_R * 2.0),
                origin=Origin(xyz=(0.0, 0.0, h)),
                material=chrome.name,
                name=f"ring_{i}",
            )

        # Base seam ring
        valve.visual(
            Cylinder(radius=V_RAD_BASE + SEAM_EXTRA, length=SEAM_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2.0)),
            material=seam_mat,
            name="base_seam",
        )

        # Stem on top for lever pivot
        valve.visual(
            Cylinder(radius=STEM_R, length=STEM_H),
            origin=Origin(xyz=(0.0, 0.0, V_HEIGHT + STEM_H / 2.0)),
            material=chrome.name,
            name="valve_stem",
        )

        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPREAD_X, 0.0, 0.0)),
        )

        # Lever handle
        handle = model.part(f"{side}_handle")
        _add_lever_handle(handle, chrome.name)

        model.articulation(
            f"{side}_handle_tilt",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, LEVER_PIVOT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-0.5, upper=0.5
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    bridge_spout = object_model.get_part("bridge_spout")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_tilt")
    j_right = object_model.get_articulation("right_handle_tilt")

    # Intentional captured fits: pivot balls over valve stems
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a="pivot_ball",
        elem_b="valve_stem",
        reason="Lever pivot ball intentionally sits on the valve stem top as a ball joint.",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a="pivot_ball",
        elem_b="valve_stem",
        reason="Lever pivot ball intentionally sits on the valve stem top as a ball joint.",
    )

    # --- Valve pedestals seated on deck ---
    for valve in (left_valve, right_valve):
        ctx.expect_gap(
            valve,
            deck,
            axis="z",
            max_gap=0.003,
            max_penetration=0.001,
            name=f"{valve.name} base seated on deck",
        )

    # --- Three-piece spread of about 0.30 m ---
    ctx.expect_origin_distance(
        left_valve,
        right_valve,
        axes="x",
        min_dist=0.29,
        max_dist=0.31,
        name="valve spread is about 0.30 m",
    )

    # --- Bridge arch spans between valves ---
    bridge_aabb = ctx.part_element_world_aabb(bridge_spout, elem="bridge_arch")
    ctx.check(
        "bridge arch spans between the valve columns",
        bridge_aabb is not None
        and bridge_aabb[0][0] < -0.10
        and bridge_aabb[1][0] > 0.10,
        details=f"bridge aabb={bridge_aabb}",
    )
    ctx.check(
        "bridge arch peak is above valve tops",
        bridge_aabb is not None and bridge_aabb[1][2] > V_HEIGHT,
        details=f"bridge peak z={bridge_aabb[1][2] if bridge_aabb else None}",
    )

    # --- Spout descends from bridge center ---
    spout_aabb = ctx.part_element_world_aabb(bridge_spout, elem="spout_tube")
    ctx.check(
        "spout reaches forward from bridge center",
        spout_aabb is not None and spout_aabb[1][1] > 0.10,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout tip descends below bridge peak",
        spout_aabb is not None and spout_aabb[0][2] < BRIDGE_PEAK_Z - 0.02,
        details=f"spout min z={spout_aabb[0][2] if spout_aabb else None}",
    )

    # --- Decorative ring ridges on pedestals ---
    for valve in (left_valve, right_valve):
        for i in range(len(RING_HEIGHTS)):
            ring_aabb = ctx.part_element_world_aabb(valve, elem=f"ring_{i}")
            ctx.check(
                f"{valve.name} has decorative ring_{i}",
                ring_aabb is not None,
                details=f"ring_{i} aabb={ring_aabb}",
            )
        # Ring ridges protrude beyond the pedestal surface
        ring0_aabb = ctx.part_element_world_aabb(valve, elem="ring_0")
        ped_aabb = ctx.part_element_world_aabb(valve, elem="valve_pedestal")
        ctx.check(
            f"{valve.name} rings protrude beyond pedestal surface",
            ring0_aabb is not None
            and ped_aabb is not None
            and (ring0_aabb[1][0] - ring0_aabb[0][0]) > (ped_aabb[1][0] - ped_aabb[0][0]) * 0.25,
            details=f"ring0 width={ring0_aabb}, ped width={ped_aabb}",
        )

    # --- Narrow seams at all three deck bases ---
    for valve in (left_valve, right_valve):
        seam_aabb = ctx.part_element_world_aabb(valve, elem="base_seam")
        ctx.check(
            f"{valve.name} has a dark base seam ring",
            seam_aabb is not None
            and (seam_aabb[1][2] - seam_aabb[0][2]) < 0.004,
            details=f"seam aabb={seam_aabb}",
        )
    bridge_seam_aabb = ctx.part_element_world_aabb(bridge_spout, elem="bridge_seam")
    ctx.check(
        "bridge_spout has a dark base seam ring at deck center",
        bridge_seam_aabb is not None
        and (bridge_seam_aabb[1][2] - bridge_seam_aabb[0][2]) < 0.004,
        details=f"bridge seam aabb={bridge_seam_aabb}",
    )

    # --- Lever handles exist and extend forward ---
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        arm_aabb = ctx.part_element_world_aabb(handle, elem="lever_arm")
        ctx.check(
            f"{handle.name} lever arm extends forward",
            arm_aabb is not None and (arm_aabb[1][1] - arm_aabb[0][1]) > 0.04,
            details=f"lever arm aabb={arm_aabb}",
        )
        # Proof: pivot ball seated on stem top
        ctx.expect_contact(
            handle,
            valve,
            elem_a="pivot_ball",
            elem_b="valve_stem",
            contact_tol=0.008,
            name=f"{handle.name} pivot ball contacts valve stem",
        )

    # --- Joint limits: lever tilts ±0.5 rad ---
    for joint in (j_left, j_right):
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} range is -0.5..+0.5 rad",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and abs(lim.lower - (-0.5)) < 0.01
            and abs(lim.upper - 0.5) < 0.01,
        )

    # --- Joint axis is horizontal (X) for forward-back tilt ---
    for joint in (j_left, j_right):
        ax = joint.axis
        ctx.check(
            f"{joint.name} axis is along X (forward-back tilt)",
            ax is not None and abs(ax[0]) > 0.9 and abs(ax[1]) < 0.1 and abs(ax[2]) < 0.1,
        )

    # --- Decisive pose: lever tip moves up/down with joint rotation ---
    def _lever_tip_y(handle: Part) -> float | None:
        aabb = ctx.part_element_world_aabb(handle, elem="lever_cap")
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    rest_left_z = _lever_tip_y(left_handle)
    with ctx.pose({j_left: 0.4}):
        posed_left_z = _lever_tip_y(left_handle)
    ctx.check(
        "left lever tip rises with positive joint angle",
        rest_left_z is not None
        and posed_left_z is not None
        and posed_left_z > rest_left_z + 0.005,
        details=f"rest_z={rest_left_z}, posed_z={posed_left_z}",
    )

    rest_right_z = _lever_tip_y(right_handle)
    with ctx.pose({j_right: -0.4}):
        posed_right_z = _lever_tip_y(right_handle)
    ctx.check(
        "right lever tip drops with negative joint angle (independent)",
        rest_right_z is not None
        and posed_right_z is not None
        and posed_right_z < rest_right_z - 0.005,
        details=f"rest_z={rest_right_z}, posed_z={posed_right_z}",
    )

    return ctx.report()


object_model = build_object_model()
