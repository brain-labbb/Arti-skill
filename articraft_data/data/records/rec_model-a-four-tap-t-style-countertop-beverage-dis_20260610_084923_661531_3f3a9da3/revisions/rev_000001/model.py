from __future__ import annotations

"""Four-tap T-style countertop beverage dispensing tower.

A brushed stainless drip tray (0.45 x 0.30 x 0.03 m) with a perforated
hole-grid top plate carries a polished steel pedestal column (d=0.06 m,
h=0.30 m) that supports a horizontal stainless header box
(0.42 x 0.10 x 0.10 m), forming a T. Four chrome faucets project from the
header front face (-Y); each carries a glossy black tapered tap handle
(~0.12 m) on a chrome lever collar. Each handle is an independent revolute
joint about the left-right X axis: 0 rad upright (closed) to ~40 deg pulled
forward over its spout.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- constants
TRAY_X, TRAY_Y, TRAY_Z = 0.450, 0.300, 0.030
POCKET_X, POCKET_Y = 0.414, 0.264
POCKET_FLOOR_Z = 0.024
GRATE_X, GRATE_Y, GRATE_T = 0.408, 0.258, 0.004
GRATE_CENTER_Z = 0.0255  # grate spans 0.0235 .. 0.0275, seated on pocket floor

COLUMN_R = 0.030
COLUMN_H = 0.300
COLUMN_BASE_Z = TRAY_Z  # column joint frame on the tray rim plane (z = 0.030)

HEADER_X, HEADER_Y, HEADER_Z = 0.420, 0.100, 0.100
HEADER_BASE_Z = COLUMN_BASE_Z + COLUMN_H  # world 0.330

FAUCET_XS = (-0.1575, -0.0525, 0.0525, 0.1575)
FAUCET_JOINT_Z = 0.045  # above header bottom -> world z = 0.375
SHANK_R = 0.011
VALVE_Y = -0.040  # valve/spout axis offset in front of the header face
PIVOT_Z = 0.020  # handle pivot height above faucet frame (top of boss)

HANDLE_MAX = math.radians(40.0)
HANDLE_GRIP_TOP = 0.135  # above pivot


# ---------------------------------------------------------------- cq shapes
def _tray_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(TRAY_X, TRAY_Y, TRAY_Z, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.024)
    )
    pocket = (
        cq.Workplane("XY", origin=(0.0, 0.0, POCKET_FLOOR_Z))
        .box(POCKET_X, POCKET_Y, 0.012, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.016)
    )
    return body.cut(pocket)


def _header_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(HEADER_X, HEADER_Y, HEADER_Z, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.005)
    )


def _faucet_body_shape() -> cq.Workplane:
    # Shank: horizontal cylinder along Y, seated 4 mm into the header face.
    shank = (
        cq.Workplane("XZ")
        .workplane(offset=-0.004)
        .circle(SHANK_R)
        .extrude(0.034)  # y from +0.004 to -0.030
    )
    # Valve body: vertical cylinder where the spout and handle pivot live.
    valve = (
        cq.Workplane("XY", origin=(0.0, VALVE_Y, -0.014))
        .circle(0.0145)
        .extrude(0.028)  # z from -0.014 to +0.014
    )
    # Downward-curved tapered spout (loft drifts slightly forward).
    spout = (
        cq.Workplane("XY", origin=(0.0, VALVE_Y, -0.010))
        .circle(0.0085)
        .workplane(offset=-0.024)
        .center(0.0, -0.0045)
        .circle(0.0068)
        .workplane(offset=-0.021)
        .center(0.0, -0.0035)
        .circle(0.0055)
        .loft(combine=True)
    )
    return shank.union(valve).union(spout)


def _pivot_boss_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, VALVE_Y, 0.012))
        .circle(0.009)
        .extrude(0.010)  # z 0.012 .. 0.022
    )


def _lever_collar_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.010))
        .circle(0.0100)
        .workplane(offset=0.020)
        .circle(0.0080)
        .loft(combine=True)  # z -0.010 .. 0.010 around the pivot
    )


def _handle_grip_shape() -> cq.Workplane:
    grip = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.006))
        .circle(0.0062)
        .workplane(offset=0.055)
        .circle(0.0078)
        .workplane(offset=0.050)
        .circle(0.0105)
        .workplane(offset=0.015)
        .circle(0.0092)
        .loft(combine=True)
    )
    dome = cq.Workplane("XY", origin=(0.0, 0.0, 0.126)).sphere(0.0092)
    return grip.union(dome)


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_tap_beverage_tower")

    steel_dark = model.material("steel_dark", rgba=(0.34, 0.36, 0.38, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.60, 0.62, 0.65, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.71, 0.73, 0.77, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.86, 1.0))
    gloss_black = model.material("gloss_black", rgba=(0.05, 0.05, 0.06, 1.0))

    # ---- drip tray (root): rounded basin with perforated hole-grid plate
    tray = model.part("drip_tray")
    tray.visual(
        mesh_from_cadquery(_tray_body_shape(), "tray_body", tolerance=0.0005),
        material=steel_dark,
        name="tray_body",
    )
    grate_geo = PerforatedPanelGeometry(
        (GRATE_X, GRATE_Y),
        GRATE_T,
        hole_diameter=0.006,
        pitch=(0.013, 0.013),
        frame=0.010,
        corner_radius=0.014,
        stagger=True,
        center=True,
    )
    tray.visual(
        mesh_from_geometry(grate_geo, "tray_grate"),
        origin=Origin(xyz=(0.0, 0.0, GRATE_CENTER_Z)),
        material=brushed_steel,
        name="tray_grate",
    )

    # ---- pedestal column with base/top flanges
    column = model.part("pedestal_column")
    column.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_H / 2.0)),
        material=polished_steel,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.046, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),  # z -0.003 .. 0.010, seats on grate
        material=polished_steel,
        name="base_flange",
    )
    column.visual(
        Cylinder(radius=0.042, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_H - 0.006)),
        material=polished_steel,
        name="top_flange",
    )
    model.articulation(
        "tray_to_column",
        ArticulationType.FIXED,
        parent=tray,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, COLUMN_BASE_Z)),
    )

    # ---- header box (the top bar of the T)
    header = model.part("header_box")
    header.visual(
        mesh_from_cadquery(_header_shape(), "header_shell", tolerance=0.0005),
        material=brushed_steel,
        name="header_shell",
    )
    model.articulation(
        "column_to_header",
        ArticulationType.FIXED,
        parent=column,
        child=header,
        origin=Origin(xyz=(0.0, 0.0, COLUMN_H)),
    )

    # ---- four faucets + tap handles (shared template meshes)
    faucet_mesh = mesh_from_cadquery(_faucet_body_shape(), "faucet_body", tolerance=0.0004)
    boss_mesh = mesh_from_cadquery(_pivot_boss_shape(), "pivot_boss", tolerance=0.0004)
    collar_mesh = mesh_from_cadquery(_lever_collar_shape(), "lever_collar", tolerance=0.0004)
    grip_mesh = mesh_from_cadquery(_handle_grip_shape(), "handle_grip", tolerance=0.0004)

    for i, fx in enumerate(FAUCET_XS):
        faucet = model.part(f"faucet_{i}")
        faucet.visual(faucet_mesh, material=chrome, name="faucet_body")
        faucet.visual(boss_mesh, material=chrome, name="pivot_boss")
        model.articulation(
            f"header_to_faucet_{i}",
            ArticulationType.FIXED,
            parent=header,
            child=faucet,
            origin=Origin(xyz=(fx, -HEADER_Y / 2.0, FAUCET_JOINT_Z)),
        )

        handle = model.part(f"tap_handle_{i}")
        handle.visual(collar_mesh, material=chrome, name="lever_collar")
        handle.visual(grip_mesh, material=gloss_black, name="handle_grip")
        model.articulation(
            f"tap_handle_{i}_pivot",
            ArticulationType.REVOLUTE,
            parent=faucet,
            child=handle,
            origin=Origin(xyz=(0.0, VALVE_Y, PIVOT_Z)),
            # Left-right horizontal axis; positive q tips the upright handle
            # forward (-Y) over the spout by the right-hand rule.
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0, velocity=2.0, lower=0.0, upper=HANDLE_MAX
            ),
        )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tray = object_model.get_part("drip_tray")
    column = object_model.get_part("pedestal_column")
    header = object_model.get_part("header_box")
    faucets = [object_model.get_part(f"faucet_{i}") for i in range(4)]
    handles = [object_model.get_part(f"tap_handle_{i}") for i in range(4)]
    pivots = [object_model.get_articulation(f"tap_handle_{i}_pivot") for i in range(4)]

    # ---- intentional seated embeddings (captured pivots / seated mounts)
    ctx.allow_overlap(
        column,
        tray,
        elem_a="base_flange",
        elem_b="tray_grate",
        reason="Column base flange is seated 0.5 mm into the perforated grate plate.",
    )
    for faucet, handle in zip(faucets, handles):
        ctx.allow_overlap(
            faucet,
            header,
            elem_a="faucet_body",
            elem_b="header_shell",
            reason="Faucet shank is intentionally seated 4 mm into the header front face.",
        )
        ctx.allow_overlap(
            handle,
            faucet,
            elem_a="lever_collar",
            elem_b="pivot_boss",
            reason="Lever collar is captured on the faucet pivot boss (revolute pivot).",
        )
        ctx.allow_overlap(
            handle,
            faucet,
            elem_a="lever_collar",
            elem_b="faucet_body",
            reason="Collar skirt locally nests onto the valve body top around the pivot.",
        )

    # ---- drip tray: footprint, grounding, perforated plate
    tray_aabb = ctx.part_world_aabb(tray)
    ctx.check(
        "drip tray is ~0.45 x 0.30 x 0.03 m and grounded at z=0",
        tray_aabb is not None
        and abs((tray_aabb[1][0] - tray_aabb[0][0]) - TRAY_X) < 0.01
        and abs((tray_aabb[1][1] - tray_aabb[0][1]) - TRAY_Y) < 0.01
        and abs(tray_aabb[0][2]) < 1e-4
        and abs(tray_aabb[1][2] - TRAY_Z) < 0.002,
        details=f"tray aabb={tray_aabb}",
    )
    grate_aabb = ctx.part_element_world_aabb(tray, elem="tray_grate")
    ctx.check(
        "perforated grate plate sits in the tray pocket below the rim",
        grate_aabb is not None
        and grate_aabb[1][2] < TRAY_Z
        and grate_aabb[0][2] > POCKET_FLOOR_Z - 0.002,
        details=f"grate aabb={grate_aabb}",
    )

    # ---- T structure: column on tray center, header on column
    ctx.expect_contact(column, tray, name="pedestal column seats on the drip tray")
    ctx.expect_contact(header, column, name="header box seats on the column top flange")
    col_pos = ctx.part_world_position(column)
    ctx.check(
        "column rises from the tray center",
        col_pos is not None and abs(col_pos[0]) < 1e-6 and abs(col_pos[1]) < 1e-6,
        details=f"column origin={col_pos}",
    )
    header_aabb = ctx.part_world_aabb(header)
    ctx.check(
        "header box is ~0.42 x 0.10 x 0.10 m atop the 0.30 m column",
        header_aabb is not None
        and abs((header_aabb[1][0] - header_aabb[0][0]) - HEADER_X) < 0.01
        and abs((header_aabb[1][1] - header_aabb[0][1]) - HEADER_Y) < 0.01
        and abs(header_aabb[0][2] - HEADER_BASE_Z) < 0.002
        and abs(header_aabb[1][2] - (HEADER_BASE_Z + HEADER_Z)) < 0.002,
        details=f"header aabb={header_aabb}",
    )

    # ---- faucets: front-mounted, evenly spaced, draining over the tray
    xs = []
    for i, faucet in enumerate(faucets):
        ctx.expect_contact(
            faucet, header, name=f"faucet_{i} shank is mounted on the header face"
        )
        ctx.expect_within(
            faucet,
            tray,
            axes="xy",
            name=f"faucet_{i} spout drains over the drip tray footprint",
        )
        pos = ctx.part_world_position(faucet)
        assert pos is not None
        xs.append(pos[0])
    gaps = [xs[i + 1] - xs[i] for i in range(3)]
    ctx.check(
        "four faucets are evenly spaced along the header front",
        len(xs) == 4 and all(abs(g - gaps[0]) < 1e-6 for g in gaps) and gaps[0] > 0.08,
        details=f"faucet x positions={xs}",
    )

    # ---- tap handles: joint plan (revolute, X axis, 0..~40 deg), upright pose
    for i, (handle, pivot) in enumerate(zip(handles, pivots)):
        ctx.check(
            f"tap_handle_{i} pivot is revolute about the left-right X axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (1.0, 0.0, 0.0),
            details=f"type={pivot.articulation_type}, axis={pivot.axis}",
        )
        limits = pivot.motion_limits
        ctx.check(
            f"tap_handle_{i} travel is 0 to ~40 deg forward",
            limits is not None
            and limits.lower == 0.0
            and abs(limits.upper - HANDLE_MAX) < 1e-6,
            details=f"limits={limits}",
        )
        grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
        ctx.check(
            f"tap_handle_{i} grip is ~0.12 m long and stands above the header top",
            grip is not None
            and grip[1][2] > HEADER_BASE_Z + HEADER_Z + 0.05
            and (grip[1][2] - grip[0][2]) > 0.115,
            details=f"grip aabb={grip}",
        )
        # Upright handle stays clear in front of the header face.
        ctx.expect_gap(
            header,
            handle,
            axis="y",
            min_gap=0.0,
            name=f"tap_handle_{i} swings clear in front of the header",
        )

    # ---- decisive pose: pulling handle 0 forward tips its grip over the spout
    handle0, pivot0 = handles[0], pivots[0]
    grip_rest = ctx.part_element_world_aabb(handle0, elem="handle_grip")
    with ctx.pose({pivot0: HANDLE_MAX}):
        grip_open = ctx.part_element_world_aabb(handle0, elem="handle_grip")
        other_rest = ctx.part_element_world_aabb(handles[1], elem="handle_grip")
    ctx.check(
        "pulled tap handle tips forward (-Y) and down over its spout",
        grip_rest is not None
        and grip_open is not None
        and grip_open[0][1] < grip_rest[0][1] - 0.04
        and grip_open[1][2] < grip_rest[1][2] - 0.015,
        details=f"rest={grip_rest}, open={grip_open}",
    )
    grip1_rest = ctx.part_element_world_aabb(handles[1], elem="handle_grip")
    ctx.check(
        "tap handles articulate independently",
        other_rest is not None
        and grip1_rest is not None
        and abs(other_rest[0][1] - grip1_rest[0][1]) < 1e-9,
        details=f"handle_1 during handle_0 pull={other_rest}, at rest={grip1_rest}",
    )

    return ctx.report()


object_model = build_object_model()
