from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Dimensions (meters). World frame: +Y is the front of the tray, +Z is up.
# ---------------------------------------------------------------------------
TRAY_X = 0.42
TRAY_Y = 0.30
TRAY_Z = 0.03
POCKET_X = 0.386
POCKET_Y = 0.266
POCKET_DEPTH = 0.008  # recess below the rim that holds the perforated plate

PLATE_X = 0.382
PLATE_Y = 0.262
PLATE_T = 0.004
POCKET_FLOOR_Z = TRAY_Z - POCKET_DEPTH  # 0.022
PLATE_CENTER_Z = POCKET_FLOOR_Z + PLATE_T / 2.0 - 0.0005  # 0.5 mm seated into floor
PLATE_TOP_Z = PLATE_CENTER_Z + PLATE_T / 2.0  # 0.0255

COL_Y = -0.065  # column axis sits near the rear of the tray
COL_R = 0.04  # 0.08 m diameter column
COL_H = 0.384  # ~0.38 m tall column
COLLAR_R0 = 0.065
COLLAR_R1 = 0.043
COLLAR_H = 0.045
CAP_R = 0.042
CAP_T = 0.012

FAUCET_WORLD_Z = 0.345  # faucet body axis height
FAUCET_LOCAL_Z = FAUCET_WORLD_Z - PLATE_TOP_Z
FAUCET_SPLAY = math.radians(35.0)  # each faucet aims 35 deg off the front axis
YAW_0 = math.radians(90.0) - FAUCET_SPLAY  # +X (right-front) faucet
YAW_1 = math.radians(90.0) + FAUCET_SPLAY  # -X (left-front) faucet

LEVER_PIVOT = (0.060, 0.0, 0.018)  # in the faucet local frame
LEVER_OPEN = math.radians(40.0)

steel = Material(name="brushed_steel", rgba=(0.62, 0.63, 0.66, 1.0))
steel_grate = Material(name="steel_grate", rgba=(0.50, 0.51, 0.54, 1.0))
chrome = Material(name="chrome", rgba=(0.82, 0.84, 0.87, 1.0))
gloss_black = Material(name="gloss_black", rgba=(0.04, 0.04, 0.05, 1.0))


def _tray_shape() -> cq.Workplane:
    """Rectangular drip tray: raised rounded rim around a recessed plate pocket."""
    body = (
        cq.Workplane("XY")
        .box(TRAY_X, TRAY_Y, TRAY_Z, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.02)
    )
    pocket = (
        cq.Workplane("XY")
        .box(POCKET_X, POCKET_Y, POCKET_DEPTH + 0.002, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, 0.0, POCKET_FLOOR_Z))
    )
    tray = body.cut(pocket)
    try:
        tray = tray.edges(">Z").fillet(0.0035)  # rounded rim top
    except Exception:
        pass  # rim round-over is cosmetic; keep the crisp rim if OCCT refuses
    return tray


def _collar_shape() -> cq.Workplane:
    """Flared circular mounting collar at the column base."""
    return (
        cq.Workplane("XY")
        .circle(COLLAR_R0)
        .workplane(offset=COLLAR_H)
        .circle(COLLAR_R1)
        .loft()
    )


def _faucet_shape() -> cq.Workplane:
    """Chrome faucet in its local frame: +X projects outward, body axis on z=0.

    Shank threads into the column, a flange seats against the column wall,
    a short horizontal body carries the lever boss, and a swept elbow tube
    drops into a tapered nozzle.
    """
    shank = cq.Workplane("YZ").workplane(offset=0.026).circle(0.011).extrude(0.029)
    flange = cq.Workplane("YZ").workplane(offset=0.040).circle(0.017).extrude(0.009)
    body = cq.Workplane("YZ").workplane(offset=0.050).circle(0.016).extrude(0.045)
    boss = cq.Workplane("XY", origin=(0.060, 0.0, 0.012)).circle(0.013).extrude(0.010)
    # Downward-curving spout: 90 deg elbow then a straight drop.
    path = (
        cq.Workplane("XZ")
        .moveTo(0.095, 0.0)
        .threePointArc((0.11056, -0.00644), (0.117, -0.022))
        .lineTo(0.117, -0.046)
    )
    profile = cq.Workplane("YZ", origin=(0.095, 0.0, 0.0)).circle(0.0105)
    spout = profile.sweep(path)
    nozzle = (
        cq.Workplane("XY", origin=(0.117, 0.0, -0.074))
        .circle(0.0062)
        .workplane(offset=0.030)
        .circle(0.0105)
        .loft()
    )
    return shank.union(flange).union(body).union(boss).union(spout).union(nozzle)


def _grip_shape() -> cq.Workplane:
    """Glossy black tap handle grip: tapered shaft widening to a domed top."""
    shaft = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.026))
        .circle(0.0085)
        .workplane(offset=0.099)
        .circle(0.0135)
        .loft()
    )
    cap = cq.Workplane("XY", origin=(0.0, 0.0, 0.122)).sphere(0.0135)
    return shaft.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_faucet_beverage_tower")

    # --- drip tray (root) ---------------------------------------------------
    tray = model.part("drip_tray")
    tray.visual(
        mesh_from_cadquery(_tray_shape(), "tray_shell"),
        material=steel,
        name="tray_shell",
    )
    grate_geom = PerforatedPanelGeometry(
        (PLATE_X, PLATE_Y),
        PLATE_T,
        hole_diameter=0.0055,
        pitch=(0.014, 0.014),
        frame=0.012,
        corner_radius=0.016,
        stagger=True,
    )
    tray.visual(
        mesh_from_geometry(grate_geom, "tray_grate"),
        origin=Origin(xyz=(0.0, 0.0, PLATE_CENTER_Z)),
        material=steel_grate,
        name="tray_grate",
    )

    # --- column -------------------------------------------------------------
    column = model.part("column")
    column.visual(
        mesh_from_cadquery(_collar_shape(), "column_collar"),
        material=steel,
        name="column_collar",
    )
    column.visual(
        Cylinder(radius=COL_R, length=COL_H),
        origin=Origin(xyz=(0.0, 0.0, COL_H / 2.0)),
        material=steel,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_T),
        origin=Origin(xyz=(0.0, 0.0, COL_H - CAP_T / 2.0 + 0.004)),
        material=steel,
        name="column_cap",
    )
    model.articulation(
        "column_mount",
        ArticulationType.FIXED,
        parent=tray,
        child=column,
        origin=Origin(xyz=(0.0, COL_Y, PLATE_TOP_Z)),
    )

    # --- faucets and tap handles ---------------------------------------------
    for idx, yaw in ((0, YAW_0), (1, YAW_1)):
        faucet = model.part(f"faucet_{idx}")
        faucet.visual(
            mesh_from_cadquery(_faucet_shape(), f"faucet_body_{idx}"),
            material=chrome,
            name="faucet_body",
        )
        model.articulation(
            f"faucet_{idx}_mount",
            ArticulationType.FIXED,
            parent=column,
            child=faucet,
            origin=Origin(xyz=(0.0, 0.0, FAUCET_LOCAL_Z), rpy=(0.0, 0.0, yaw)),
        )

        handle = model.part(f"tap_handle_{idx}")
        handle.visual(
            Cylinder(radius=0.0125, length=0.025),
            origin=Origin(xyz=(0.0, 0.0, 0.0145)),
            material=chrome,
            name="handle_collar",
        )
        handle.visual(
            mesh_from_cadquery(_grip_shape(), f"tap_grip_{idx}"),
            material=gloss_black,
            name="handle_grip",
        )
        model.articulation(
            f"tap_lever_{idx}",
            ArticulationType.REVOLUTE,
            parent=faucet,
            child=handle,
            origin=Origin(xyz=LEVER_PIVOT),
            # +Y in the faucet frame is horizontal and perpendicular to the
            # spout direction (+X); positive q pulls the handle forward over
            # the spout (closed upright -> open tilted out/down).
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=LEVER_OPEN),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tray = object_model.get_part("drip_tray")
    column = object_model.get_part("column")
    faucets = [object_model.get_part(f"faucet_{i}") for i in range(2)]
    handles = [object_model.get_part(f"tap_handle_{i}") for i in range(2)]
    levers = [object_model.get_articulation(f"tap_lever_{i}") for i in range(2)]

    # Intentional local embeddings.
    for faucet, handle in zip(faucets, handles):
        ctx.allow_overlap(
            faucet,
            column,
            elem_a="faucet_body",
            elem_b="column_shaft",
            reason="The faucet shank threads into the tower column wall (captured shaft).",
        )
        ctx.allow_overlap(
            handle,
            faucet,
            elem_a="handle_collar",
            elem_b="faucet_body",
            reason="The lever collar seats over the faucet lever boss (captured ball joint).",
        )
        ctx.allow_overlap(
            handle,
            faucet,
            elem_a="handle_grip",
            elem_b="faucet_body",
            reason="The grip root meets the lever boss at the seated collar joint.",
        )

    # --- hero features --------------------------------------------------------
    grate_aabb = ctx.part_element_world_aabb(tray, elem="tray_grate")
    ctx.check(
        "perforated drain plate spans the tray recess",
        grate_aabb is not None
        and (grate_aabb[1][0] - grate_aabb[0][0]) > 0.36
        and (grate_aabb[1][1] - grate_aabb[0][1]) > 0.24,
        details=f"grate aabb={grate_aabb}",
    )

    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "column rises ~0.38 m above the tray with its cap",
        col_aabb is not None and 0.40 < col_aabb[1][2] < 0.43,
        details=f"column aabb={col_aabb}",
    )

    # --- seating / mounting ----------------------------------------------------
    ctx.expect_contact(column, tray, contact_tol=0.001, name="column collar seated on tray plate")
    ctx.expect_within(column, tray, axes="xy", name="column collar stays on the tray footprint")
    for i, (faucet, handle) in enumerate(zip(faucets, handles)):
        ctx.expect_contact(faucet, column, name=f"faucet {i} mounted into the column")
        ctx.expect_contact(handle, faucet, name=f"tap handle {i} seated on its faucet boss")
        ctx.expect_within(
            faucet,
            tray,
            axes="xy",
            name=f"faucet {i} spout dispenses over the drip tray",
        )
        ctx.expect_gap(
            faucet,
            tray,
            axis="z",
            min_gap=0.15,
            name=f"faucet {i} spout leaves glass clearance above the tray",
        )

    # --- placement: faucets flank the column toward the front -------------------
    centers = []
    for faucet in faucets:
        aabb = ctx.part_world_aabb(faucet)
        centers.append(
            None
            if aabb is None
            else ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][1] + aabb[1][1]) / 2.0)
        )
    ctx.check(
        "faucets project front-right and front-left of the column",
        centers[0] is not None
        and centers[1] is not None
        and centers[0][0] > 0.02
        and centers[1][0] < -0.02
        and centers[0][1] > COL_Y + 0.02
        and centers[1][1] > COL_Y + 0.02,
        details=f"faucet centers (x, y)={centers}, column y={COL_Y}",
    )

    # --- lever articulation ------------------------------------------------------
    for i, (handle, lever) in enumerate(zip(handles, levers)):
        rest = ctx.part_world_aabb(handle)
        ctx.check(
            f"tap handle {i} stands upright above the column top at rest",
            rest is not None and rest[1][2] > 0.46,
            details=f"rest aabb={rest}",
        )
        with ctx.pose({lever: LEVER_OPEN}):
            opened = ctx.part_world_aabb(handle)
        if rest is None or opened is None:
            ctx.fail(f"tap handle {i} pose query", "missing aabb")
            continue
        dropped = opened[1][2] < rest[1][2] - 0.015
        forward = opened[1][1] > rest[1][1] + 0.03  # tilts toward the tray front
        if i == 0:
            outward = opened[1][0] > rest[1][0] + 0.02  # +X side handle swings +X
        else:
            outward = opened[0][0] < rest[0][0] - 0.02  # -X side handle swings -X
        ctx.check(
            f"tap handle {i} pulls forward/down over its spout when opened",
            dropped and forward and outward,
            details=f"rest={rest}, opened={opened}",
        )

    return ctx.report()


object_model = build_object_model()
