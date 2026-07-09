from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)

# Long black switched surge-protector power strip from the reference image.
# Important visual constraints:
# - five NEMA receptacles keep a flat top panel; only the two blade slots
#   and ground hole are small recessed pits, not one large recessed rectangle
# - all six red rocker switches are independent revolute joints
# - the cord has a fine black/white braided sleeve, not chunky white protrusions

LENGTH = 0.56
WIDTH = 0.118
HEIGHT = 0.036
TOP_Z = HEIGHT

OUTLET_XS = (-0.135, -0.047, 0.041, 0.129, 0.217)
MASTER_SWITCH_X = -0.213
# switch_1 is the independent master switch; switches 2..6 align one-to-one
# with outlets 1..5 below their matching receptacles.
SWITCH_XS = (MASTER_SWITCH_X, *OUTLET_XS)
OUTLET_Y = 0.027
SWITCH_Y = -0.031

OUTLET_SLOT_W = 0.006
OUTLET_SLOT_H = 0.020
OUTLET_PIT_DEPTH = 0.008
GROUND_R = 0.006
SWITCH_W = 0.048
SWITCH_H = 0.035
SWITCH_RECESS_DEPTH = 0.008


def _box(part, name: str, size, xyz, material: str, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name: str, radius: float, length: float, xyz, material: str, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Cylinder(radius, length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _housing_shape():
    shell = (
        cq.Workplane("XY")
        .box(LENGTH, WIDTH, HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
        .edges(">Z")
        .fillet(0.0025)
    )

    # Cut only the individual plug holes downward. The surrounding top surface
    # stays flat black; there is no large rectangular recessed outlet tray.
    for x in OUTLET_XS:
        for dx in (-0.012, 0.012):
            slot_cut = (
                cq.Workplane("XY")
                .box(OUTLET_SLOT_W, OUTLET_SLOT_H, OUTLET_PIT_DEPTH + 0.002, centered=(True, True, False))
                .translate((x + dx, OUTLET_Y + 0.003, TOP_Z - OUTLET_PIT_DEPTH))
            )
            shell = shell.cut(slot_cut)
        ground_cut = (
            cq.Workplane("XY")
            .circle(GROUND_R)
            .extrude(OUTLET_PIT_DEPTH + 0.002)
            .translate((x, OUTLET_Y - 0.013, TOP_Z - OUTLET_PIT_DEPTH))
        )
        shell = shell.cut(ground_cut)

    # Separate switch wells, also cut downward so the red rockers sit in sockets.
    for x in SWITCH_XS:
        cutter = (
            cq.Workplane("XY")
            .box(SWITCH_W, SWITCH_H, SWITCH_RECESS_DEPTH + 0.002, centered=(True, True, False))
            .translate((x, SWITCH_Y, TOP_Z - SWITCH_RECESS_DEPTH))
        )
        shell = shell.cut(cutter)

    return shell


def _add_recessed_outlet(body, x: float, idx: int) -> None:
    floor_z = TOP_Z - OUTLET_PIT_DEPTH + 0.0005
    for side, dx in (("left", -0.012), ("right", 0.012)):
        _box(body, f"outlet_{idx}_{side}_slot_pit_floor", (OUTLET_SLOT_W - 0.001, OUTLET_SLOT_H - 0.002, 0.001), (x + dx, OUTLET_Y + 0.003, floor_z), "socket_floor_black")
        _box(body, f"outlet_{idx}_{side}_deep_vertical_slot_wall_shadow", (OUTLET_SLOT_W - 0.0015, OUTLET_SLOT_H - 0.003, 0.001), (x + dx, OUTLET_Y + 0.003, floor_z + 0.0008), "deep_cavity")
        _box(body, f"outlet_{idx}_{side}_recessed_brass_contact", (0.0020, 0.011, 0.0008), (x + dx, OUTLET_Y + 0.003, floor_z + 0.0015), "brass")
    _cyl(body, f"outlet_{idx}_round_ground_pit_floor", GROUND_R - 0.001, 0.001, (x, OUTLET_Y - 0.013, floor_z), "socket_floor_black")
    _cyl(body, f"outlet_{idx}_round_ground_hole_shadow", GROUND_R - 0.0015, 0.001, (x, OUTLET_Y - 0.013, floor_z + 0.0008), "deep_cavity")


def _add_switch_well_details(body, x: float, idx: int) -> None:
    floor_z = TOP_Z - SWITCH_RECESS_DEPTH + 0.0005
    _box(body, f"switch_{idx}_lowered_black_well_floor", (SWITCH_W - 0.008, SWITCH_H - 0.008, 0.001), (x, SWITCH_Y, floor_z), "switch_well_black")
    _box(body, f"switch_{idx}_front_detent_stop", (0.024, 0.003, 0.003), (x, SWITCH_Y - 0.014, floor_z + 0.002), "rim_black")
    _box(body, f"switch_{idx}_rear_detent_stop", (0.024, 0.003, 0.003), (x, SWITCH_Y + 0.014, floor_z + 0.002), "rim_black")
    _box(body, f"switch_{idx}_off_label_engraved", (0.020, 0.003, 0.0008), (x + 0.003, SWITCH_Y - 0.025, TOP_Z - 0.0004), "engraved_label")
    _box(body, f"switch_{idx}_reset_label_engraved", (0.024, 0.003, 0.0008), (x - 0.002, SWITCH_Y + 0.025, TOP_Z - 0.0004), "engraved_label")


def _add_side_ribs(body) -> None:
    for i in range(13):
        z = 0.005 + i * 0.00205
        _box(body, f"left_molded_side_rib_{i:02d}", (0.490, 0.0020, 0.0008), (0.0, -WIDTH / 2 - 0.0006, z), "rib_black")
        _box(body, f"right_molded_side_rib_{i:02d}", (0.490, 0.0020, 0.0008), (0.0, WIDTH / 2 + 0.0006, z), "rib_black")


def _add_braided_cord_and_plug(body) -> None:
    # Cord core: angled like the product photo, with the weave as fine flush marks.
    cord_x0 = -0.310
    cord_len = 0.305
    _cyl(body, "round_black_rubber_cord_core", 0.0070, cord_len, (cord_x0 - cord_len / 2 + 0.004, -0.018, 0.024), "rubber_black", rpy=(0.0, math.pi / 2.0, 0.0))
    _box(body, "molded_cord_strain_relief_boot", (0.036, 0.032, 0.028), (-0.289, -0.018, 0.024), "rubber_black")
    for i in range(5):
        _box(body, f"strain_relief_fine_ring_{i}", (0.0032, 0.034, 0.030), (-0.304 + i * 0.006, -0.018, 0.024), "rib_black")

    # Fine interleaved sleeve marks: small, thin, partially embedded slashes
    # alternating on the upper-left and upper-right visible sides of the cord.
    for i in range(42):
        x = -0.590 + i * 0.0069
        if i % 2 == 0:
            y = -0.0226
            z = 0.0290
            yaw = 0.70
            name = f"braid_white_fine_left_slash_{i:02d}"
        else:
            y = -0.0134
            z = 0.0290
            yaw = -0.70
            name = f"braid_white_fine_right_slash_{i:02d}"
        _box(body, name, (0.0068, 0.00105, 0.00085), (x, y, z), "braid_white", rpy=(0.0, 0.0, yaw))
    for i in range(22):
        x = -0.586 + i * 0.0135
        _box(body, f"braid_dark_cross_shadow_{i:02d}", (0.0060, 0.0008, 0.0005), (x, -0.0178, 0.0312), "braid_dark", rpy=(0.0, 0.0, -0.55))

    _box(body, "molded_three_prong_plug_body", (0.064, 0.045, 0.033), (-0.628, -0.018, 0.024), "rubber_black")
    _box(body, "plug_taper_neck", (0.028, 0.027, 0.023), (-0.584, -0.018, 0.024), "rubber_black")
    _box(body, "plug_left_flat_metal_blade", (0.038, 0.004, 0.014), (-0.676, -0.031, 0.028), "bright_metal")
    _box(body, "plug_right_flat_metal_blade", (0.038, 0.004, 0.014), (-0.676, -0.005, 0.028), "bright_metal")
    _cyl(body, "plug_round_ground_pin", 0.0032, 0.046, (-0.676, -0.018, 0.011), "bright_metal", rpy=(0.0, math.pi / 2.0, 0.0))


def _make_rocker_part(model: ArticulatedObject, idx: int, x: float):
    part = model.part(f"red_rocker_switch_{idx}")
    _box(part, f"rocker_{idx}_red_translucent_lens", (0.030, 0.021, 0.0065), (0.0, 0.0, 0.0033), "red_lens")
    _box(part, f"rocker_{idx}_raised_on_end", (0.026, 0.004, 0.0016), (0.0, 0.0075, 0.0072), "red_highlight")
    _cyl(part, f"rocker_{idx}_brass_pivot_pin", 0.0021, 0.034, (0.0, 0.0, -0.0012), "brass", rpy=(0.0, math.pi / 2.0, 0.0))
    part.inertial = Inertial.from_geometry(Box((0.030, 0.021, 0.0065)), mass=0.014, origin=Origin(xyz=(0.0, 0.0, 0.0033)))
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surge_protector_recessed_sockets_six_rockers")

    model.material("matte_black_plastic", rgba=(0.014, 0.015, 0.016, 1.0))
    model.material("rim_black", rgba=(0.020, 0.021, 0.023, 1.0))
    model.material("rib_black", rgba=(0.006, 0.006, 0.007, 1.0))
    model.material("socket_floor_black", rgba=(0.004, 0.004, 0.005, 1.0))
    model.material("switch_well_black", rgba=(0.003, 0.003, 0.004, 1.0))
    model.material("deep_cavity", rgba=(0.0005, 0.0005, 0.0008, 1.0))
    model.material("engraved_label", rgba=(0.11, 0.11, 0.11, 1.0))
    model.material("red_lens", rgba=(0.80, 0.025, 0.022, 0.86))
    model.material("red_highlight", rgba=(1.0, 0.22, 0.18, 0.90))
    model.material("brass", rgba=(0.95, 0.65, 0.25, 1.0))
    model.material("bright_metal", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("rubber_black", rgba=(0.010, 0.010, 0.012, 1.0))
    model.material("braid_white", rgba=(0.86, 0.85, 0.78, 1.0))
    model.material("braid_dark", rgba=(0.018, 0.018, 0.020, 1.0))

    body = model.part("black_power_strip_body")
    body.visual(mesh_from_cadquery(_housing_shape(), "recessed_power_strip_housing"), material="matte_black_plastic", name="single_molded_housing_with_cut_recesses")
    _box(body, "rear_rating_label_plate", (0.090, 0.014, 0.001), (0.125, -0.003, TOP_Z - 0.0004), "engraved_label")
    _box(body, "long_panel_seam_line", (0.505, 0.0016, 0.0008), (0.0, -0.004, TOP_Z - 0.0004), "engraved_label")
    for x, y, name in (
        (-0.255, -0.050, "corner_screw_near_left"),
        (0.255, -0.050, "corner_screw_near_right"),
        (-0.255, 0.050, "corner_screw_far_left"),
        (0.255, 0.050, "corner_screw_far_right"),
    ):
        _cyl(body, name, 0.004, 0.0014, (x, y, TOP_Z - 0.0007), "deep_cavity")

    _add_side_ribs(body)
    for i, x in enumerate(OUTLET_XS, start=1):
        _add_recessed_outlet(body, x, i)
    for i, x in enumerate(SWITCH_XS, start=1):
        _add_switch_well_details(body, x, i)
    _add_braided_cord_and_plug(body)

    body.inertial = Inertial.from_geometry(Box((LENGTH + 0.38, WIDTH, HEIGHT)), mass=0.92, origin=Origin(xyz=(-0.13, 0.0, 0.019)))

    for i, x in enumerate(SWITCH_XS, start=1):
        rocker = _make_rocker_part(model, i, x)
        model.articulation(
            f"body_to_rocker_switch_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=rocker,
            origin=Origin(xyz=(x, SWITCH_Y, TOP_Z - SWITCH_RECESS_DEPTH + 0.0007)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=-0.27, upper=0.27, effort=1.0, velocity=4.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("black_power_strip_body")
    names = {visual.name for visual in body.visuals}
    rocker_parts = [part for part in object_model.parts if part.name.startswith("red_rocker_switch_")]
    rocker_joints = [joint for joint in object_model.articulations if joint.name.startswith("body_to_rocker_switch_")]

    ctx.check("has six separate rocker parts", len(rocker_parts) == 6, f"got {len(rocker_parts)}")
    ctx.check("has six revolute rocker joints", len(rocker_joints) == 6 and all(j.articulation_type == ArticulationType.REVOLUTE for j in rocker_joints))
    ctx.check("all rocker joints have +/- detent limits", all(j.motion_limits and j.motion_limits.lower <= -0.25 and j.motion_limits.upper >= 0.25 for j in rocker_joints))
    ctx.check("has ten individual recessed blade-slot pits", sum(name.endswith("_slot_pit_floor") for name in names) == 10)
    ctx.check("has five round ground pits", sum(name.endswith("_round_ground_pit_floor") for name in names) == 5)
    ctx.check("round ground pits centered between blade slots", all(abs(OUTLET_XS[i] - OUTLET_XS[i]) < 1e-9 for i in range(5)))
    ctx.check("five outlet switches align one-to-one", all(abs(SWITCH_XS[i + 1] - OUTLET_XS[i]) < 1e-9 for i in range(5)))
    ctx.check("master switch is separate at cord end", SWITCH_XS[0] < min(OUTLET_XS) - 0.04)
    ctx.check("avoids large outlet trays and raised faceplates", not any("lowered_socket_floor" in name or "flush_" in name or "raised_faceplate" in name for name in names))
    ctx.check("has no white top-panel label slab", "white_brand_rating_badge" not in names)
    ctx.check("has fine braided cord marks", sum(name.startswith("braid_white_fine_") for name in names) >= 36)
    ctx.check("has no chunky white cord blocks", not any("white_braid_dash" in name or "white_point" in name for name in names))
    ctx.check("has molded three-prong plug", {"plug_left_flat_metal_blade", "plug_right_flat_metal_blade", "plug_round_ground_pin"} <= names)
    ctx.check("has ribbed side housing", sum("molded_side_rib" in name for name in names) >= 24)

    for idx, joint in enumerate(rocker_joints, start=1):
        rocker = object_model.get_part(f"red_rocker_switch_{idx}")
        with ctx.pose({joint: 0.24}):
            expected_dist = abs(SWITCH_XS[idx - 1])
            ctx.expect_origin_distance(
                body,
                rocker,
                axes="xy",
                min_dist=max(0.0, expected_dist - 0.015),
                max_dist=expected_dist + 0.045,
                name=f"rocker {idx} remains seated in its switch well",
            )

    return ctx.report()


object_model = build_object_model()
