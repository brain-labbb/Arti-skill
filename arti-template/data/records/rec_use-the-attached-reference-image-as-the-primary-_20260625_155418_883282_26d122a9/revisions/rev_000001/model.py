from __future__ import annotations

from math import atan2, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _box_between(part, name, p0, p1, width, height, material):
    """Rectangular member whose long local-X axis runs from p0 to p1 in the X-Z plane."""
    dx = p1[0] - p0[0]
    dz = p1[2] - p0[2]
    length = sqrt(dx * dx + dz * dz)
    angle = -atan2(dz, dx)
    part.visual(
        Box((length, width, height)),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, angle, 0.0),
        ),
        material=material,
        name=name,
    )


def _cyl_between(part, name, p0, p1, radius, material, radial_segments=24):
    """Round tube/rod whose local-Z cylinder axis runs from p0 to p1 in the X-Z plane."""
    dx = p1[0] - p0[0]
    dz = p1[2] - p0[2]
    length = sqrt(dx * dx + dz * dz)
    angle = atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, angle, 0.0),
        ),
        material=material,
        name=name,
    )


def _y_cyl(part, name, xyz, radius, length, material):
    """Cylinder centered at xyz with its axis along Y (pivot pins / hub barrels)."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


# Side-view skeleton of the boom in the world X-Z plane (Y = centreline).
B = (0.00, 0.00, 2.20)  # boom root pivot on the king-post mount
A = (1.15, 0.00, 2.95)  # apex / elbow knuckle (top of the inverted V)
W = (2.65, 0.00, 1.30)  # wrist pivot where the grapple hangs


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_harvester_vehicle_arm",
        meta={
            "category": "Agricultural",
            "small_class": "Harvester vehicle (arm)",
            "description": (
                "Pedestal-mounted articulated hydraulic harvester arm: a two-section "
                "inverted-V boom on a king-post pivot driving a multi-finger mechanical "
                "grapple/claw end-effector."
            ),
        },
    )

    black = model.material("powder_coated_black_steel", rgba=(0.012, 0.013, 0.014, 1.0))
    blue_black = model.material("blue_black_arm_paint", rgba=(0.02, 0.035, 0.065, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.60, 0.62, 0.61, 1.0))
    chrome = model.material("polished_chrome_rod", rgba=(0.86, 0.86, 0.82, 1.0))
    dark_hw = model.material("dark_hardware", rgba=(0.05, 0.05, 0.055, 1.0))
    yellow = model.material("safety_yellow_hydraulics", rgba=(1.0, 0.74, 0.02, 1.0))
    grease = model.material("greased_pivot", rgba=(0.10, 0.09, 0.07, 1.0))

    # ----------------------------------------------------------------- base mount
    # King-post pedestal: the fixed bracket the whole arm pivots on (upper-left in
    # the reference) plus the down-curving parking hook.
    base = model.part("base_mount")
    base.visual(Box((0.46, 0.62, 0.78)), origin=Origin(xyz=(-0.40, 0.0, 2.30)), material=black, name="mount_backing_block")
    base.visual(Box((0.30, 0.50, 0.70)), origin=Origin(xyz=(-0.52, 0.0, 2.78)), material=blue_black, name="mount_king_post")
    base.visual(Box((0.40, 0.66, 0.14)), origin=Origin(xyz=(-0.40, 0.0, 3.18)), material=black, name="mount_top_cap")
    # clevis ears straddling the boom root barrel at B
    for y in (-0.24, 0.24):
        base.visual(Box((0.34, 0.10, 0.50)), origin=Origin(xyz=(B[0] - 0.02, y, B[2] + 0.06)), material=black, name=f"mount_clevis_{y:+.2f}")
        _y_cyl(base, f"mount_pin_cap_{y:+.2f}", (B[0], y, B[2]), 0.075, 0.04, steel)
    base.visual(Box((0.20, 0.40, 0.34)), origin=Origin(xyz=(-0.30, 0.0, 1.98)), material=black, name="mount_lower_gusset")
    # down-curving parking hook
    hook = mesh_from_geometry(
        tube_from_spline_points(
            [(-0.18, 0.0, 2.02), (-0.30, 0.0, 1.74), (-0.30, 0.0, 1.50), (-0.16, 0.0, 1.40), (-0.10, 0.0, 1.50)],
            radius=0.045,
            samples_per_segment=12,
            radial_segments=14,
        ),
        "mount_parking_hook",
    )
    base.visual(hook, material=black, name="parking_hook")

    # ----------------------------------------------------------------- boom (lift)
    # First inverted-V member: B -> A. Child frame origin is B.
    boom = model.part("boom")
    a_loc = (A[0] - B[0], 0.0, A[2] - B[2])  # apex in boom-local coords
    _len_b = sqrt(a_loc[0] ** 2 + a_loc[2] ** 2)
    _u_b = (a_loc[0] / _len_b, a_loc[2] / _len_b)  # boom axis unit (x,z)
    boom_box_end = (a_loc[0] - 0.20 * _u_b[0], 0.0, a_loc[2] - 0.20 * _u_b[1])  # stop short of apex
    _y_cyl(boom, "boom_root_barrel", (0.0, 0.0, 0.0), 0.135, 0.46, black)
    _box_between(boom, "boom_main_box", (0.0, 0.0, 0.0), boom_box_end, 0.30, 0.40, blue_black)
    _box_between(boom, "boom_lower_web", (0.04, 0.0, 0.02), (boom_box_end[0] - 0.04, 0.0, boom_box_end[2] - 0.03), 0.20, 0.22, black)
    _y_cyl(boom, "boom_elbow_barrel", a_loc, 0.125, 0.50, black)
    for y in (-0.22, 0.22):
        boom.visual(Box((0.34, 0.05, 0.40)), origin=Origin(xyz=(a_loc[0], y, a_loc[2])), material=black, name=f"boom_elbow_ear_{y:+.2f}")
        _y_cyl(boom, f"boom_root_washer_{y:+.2f}", (0.0, y, 0.0), 0.05, 0.02, steel)
    # lift hydraulic cylinder slung under the boom (cosmetic, fixed to boom)
    lc0 = (0.30, -0.16, 0.06)
    lc1 = (0.72, -0.16, 0.46)
    lc2 = (a_loc[0] - 0.18, -0.16, a_loc[2] - 0.10)
    boom.visual(Box((0.16, 0.14, 0.16)), origin=Origin(xyz=lc0, rpy=(0.0, -0.7, 0.0)), material=black, name="lift_cyl_base_lug")
    _cyl_between(boom, "lift_cyl_barrel", lc0, lc1, 0.058, steel)
    _cyl_between(boom, "lift_cyl_rod", lc1, lc2, 0.034, chrome)
    boom.visual(Box((0.14, 0.12, 0.14)), origin=Origin(xyz=lc2, rpy=(0.0, -0.7, 0.0)), material=black, name="lift_cyl_rod_lug")

    model.articulation(
        "chassis_to_boom",
        ArticulationType.REVOLUTE,
        parent=base,
        child=boom,
        origin=Origin(xyz=B),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=42000.0, velocity=0.5, lower=-0.55, upper=0.55),
    )

    # ----------------------------------------------------------------- stick (elbow)
    # Long box-section member: A -> W. Child frame origin is A.
    stick = model.part("outer_boom")
    w_loc = (W[0] - A[0], 0.0, W[2] - A[2])  # wrist in stick-local coords
    _len_s = sqrt(w_loc[0] ** 2 + w_loc[2] ** 2)
    _u_s = (w_loc[0] / _len_s, w_loc[2] / _len_s)  # stick axis unit (x,z)
    stick_box_start = (0.20 * _u_s[0], 0.0, 0.20 * _u_s[1])  # start past the apex knuckle
    _y_cyl(stick, "stick_elbow_barrel", (0.0, 0.0, 0.0), 0.125, 0.46, black)
    _box_between(stick, "stick_main_box", stick_box_start, w_loc, 0.26, 0.34, blue_black)
    _box_between(stick, "stick_top_rib", (stick_box_start[0] + 0.04, 0.0, stick_box_start[2] + 0.04), (w_loc[0] - 0.10, 0.0, w_loc[2] + 0.06), 0.18, 0.10, black)
    _y_cyl(stick, "stick_wrist_barrel", w_loc, 0.115, 0.42, black)
    for y in (-0.20, 0.20):
        stick.visual(Box((0.30, 0.05, 0.34)), origin=Origin(xyz=(w_loc[0], y, w_loc[2])), material=black, name=f"stick_wrist_ear_{y:+.2f}")
        _y_cyl(stick, f"stick_elbow_washer_{y:+.2f}", (0.0, y, 0.0), 0.05, 0.02, steel)
    # stick hydraulic cylinder running along the top of the long member
    sc0 = (0.45, 0.14, -0.18)
    sc1 = (0.95, 0.14, -0.62)
    sc2 = (w_loc[0] - 0.30, 0.14, w_loc[2] + 0.30)
    stick.visual(Box((0.15, 0.13, 0.15)), origin=Origin(xyz=sc0, rpy=(0.0, atan2(sc1[0] - sc0[0], sc1[2] - sc0[2]), 0.0)), material=black, name="stick_cyl_base_lug")
    _cyl_between(stick, "stick_cyl_barrel", sc0, sc1, 0.052, steel)
    _cyl_between(stick, "stick_cyl_rod", sc1, sc2, 0.030, chrome)
    stick.visual(Box((0.13, 0.11, 0.13)), origin=Origin(xyz=sc2, rpy=(0.0, 0.0, 0.0)), material=black, name="stick_cyl_rod_lug")
    # service hose running down the stick
    hose = mesh_from_geometry(
        tube_from_spline_points(
            [(0.05, -0.24, 0.02), (0.55, -0.26, -0.55), (1.05, -0.24, -1.05), (w_loc[0] - 0.05, -0.20, w_loc[2] + 0.10)],
            radius=0.016,
            samples_per_segment=10,
            radial_segments=12,
        ),
        "stick_hose",
    )
    stick.visual(hose, material=dark_hw, name="stick_service_hose")

    model.articulation(
        "boom_to_outer_boom",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=stick,
        origin=Origin(xyz=a_loc),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=36000.0, velocity=0.6, lower=-0.6, upper=0.6),
    )

    # ----------------------------------------------------------------- wrist yoke
    # Clevis that hangs off the wrist pin and carries the grapple rotator.
    yoke = model.part("wrist_yoke")
    _y_cyl(yoke, "yoke_top_barrel", (0.0, 0.0, 0.0), 0.10, 0.40, black)
    for y in (-0.16, 0.16):
        yoke.visual(Box((0.20, 0.06, 0.40)), origin=Origin(xyz=(0.0, y, -0.18)), material=black, name=f"yoke_side_plate_{y:+.2f}")
    yoke.visual(Box((0.30, 0.40, 0.18)), origin=Origin(xyz=(0.0, 0.0, -0.36)), material=blue_black, name="yoke_rotator_housing")
    yoke.visual(Cylinder(radius=0.12, length=0.10), origin=Origin(xyz=(0.0, 0.0, -0.46)), material=steel, name="yoke_rotator_flange")

    model.articulation(
        "outer_boom_to_head",
        ArticulationType.REVOLUTE,
        parent=stick,
        child=yoke,
        origin=Origin(xyz=w_loc),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=1.0, lower=-1.2, upper=1.2),
    )

    # ----------------------------------------------------------------- grapple hub
    # Rotator drum + palm + yellow finger-drive cylinders. Spins about the tool axis.
    hub = model.part("grapple_hub")
    hub.visual(Cylinder(radius=0.135, length=0.22), origin=Origin(xyz=(0.0, 0.0, 0.05)), material=black, name="rotator_drum")
    hub.visual(Cylinder(radius=0.115, length=0.06), origin=Origin(xyz=(0.0, 0.0, 0.18)), material=steel, name="rotator_collar")
    hub.visual(Box((0.34, 0.34, 0.18)), origin=Origin(xyz=(0.0, 0.0, -0.12)), material=blue_black, name="grapple_palm")
    hub.visual(
        mesh_from_geometry(SphereGeometry(0.085, width_segments=20, height_segments=12), "palm_dome"),
        origin=Origin(xyz=(0.0, 0.0, -0.20)),
        material=black,
        name="palm_cap",
    )

    n_fingers = 4
    finger_r = 0.155
    knuckle_z = -0.18
    # yellow hydraulic rams stand on the palm faces, BETWEEN the fingers (azimuth
    # offset 0deg vs the fingers' 45deg), clearing both the rotator flange and the
    # finger knuckles.
    for i in range(n_fingers):
        ang = 2.0 * pi * i / n_fingers
        rx, ry = 0.18 * cos(ang), 0.18 * sin(ang)
        hub.visual(
            Cylinder(radius=0.030, length=0.16),
            origin=Origin(xyz=(rx, ry, -0.12)),
            material=yellow,
            name=f"finger_drive_cyl_{i}",
        )
    # knuckle bosses the fingers pivot on (cross-part overlap with the fingers)
    for i in range(n_fingers):
        phi = 2.0 * pi * i / n_fingers + pi / 4.0
        bx, by = finger_r * cos(phi), finger_r * sin(phi)
        _y_cyl(hub, f"finger_boss_{i}", (bx, by, knuckle_z), 0.05, 0.12, grease)

    model.articulation(
        "head_rotator",
        ArticulationType.CONTINUOUS,
        parent=yoke,
        child=hub,
        origin=Origin(xyz=(0.0, 0.0, -0.52)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3000.0, velocity=2.0),
    )

    # ----------------------------------------------------------------- claw fingers
    # Each finger is a curved tine; positive joint angle closes it toward the axis.
    finger_pts = [
        (0.0, 0.0, 0.0),
        (0.06, 0.0, -0.11),
        (0.13, 0.0, -0.25),
        (0.11, 0.0, -0.40),
        (0.01, 0.0, -0.53),
    ]
    finger_tip_pts = [(0.11, 0.0, -0.40), (0.01, 0.0, -0.53), (-0.06, 0.0, -0.60)]
    for i in range(n_fingers):
        phi = 2.0 * pi * i / n_fingers + pi / 4.0
        bx, by = finger_r * cos(phi), finger_r * sin(phi)
        finger = model.part(f"claw_finger_{i}")
        # pivot knuckle (overlaps the hub boss)
        _y_cyl(finger, "finger_knuckle", (0.0, 0.0, 0.0), 0.052, 0.13, dark_hw)
        finger.visual(
            mesh_from_geometry(tube_from_spline_points(finger_pts, radius=0.045, samples_per_segment=12, radial_segments=14), f"finger_body_{i}"),
            material=black,
            name="finger_body",
        )
        finger.visual(
            mesh_from_geometry(tube_from_spline_points(finger_tip_pts, radius=0.028, samples_per_segment=12, radial_segments=12), f"finger_tip_{i}"),
            material=steel,
            name="finger_tip",
        )
        model.articulation(
            f"hub_to_claw_finger_{i}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=finger,
            origin=Origin(xyz=(bx, by, knuckle_z), rpy=(0.0, 0.0, phi)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2500.0, velocity=1.6, lower=-0.2, upper=1.15),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.check(
        "small class is harvester vehicle arm",
        object_model.meta.get("category") == "Agricultural"
        and object_model.meta.get("small_class") == "Harvester vehicle (arm)"
        and "harvester" in object_model.name,
        details=f"name={object_model.name}, meta={object_model.meta}",
    )

    part_names = {p.name for p in object_model.parts}
    required_parts = {
        "base_mount", "boom", "outer_boom", "wrist_yoke", "grapple_hub",
        "claw_finger_0", "claw_finger_1", "claw_finger_2", "claw_finger_3",
    }
    ctx.check(
        "boom + grapple subassemblies exist",
        required_parts.issubset(part_names),
        details=f"missing={sorted(required_parts - part_names)}",
    )

    joint_names = {j.name for j in object_model.articulations}
    required_joints = {"chassis_to_boom", "boom_to_outer_boom", "outer_boom_to_head", "head_rotator"}
    ctx.check(
        "boom lift/elbow/wrist/rotator joints exist",
        required_joints.issubset(joint_names),
        details=f"joints={sorted(joint_names)}",
    )

    ctx.check(
        "four articulated claw fingers exist",
        sum(1 for j in object_model.articulations if j.name.startswith("hub_to_claw_finger_")) == 4,
        details=f"joints={sorted(joint_names)}",
    )

    base = object_model.get_part("base_mount")
    boom = object_model.get_part("boom")
    stick = object_model.get_part("outer_boom")
    yoke = object_model.get_part("wrist_yoke")
    hub = object_model.get_part("grapple_hub")

    lift = object_model.get_articulation("chassis_to_boom")
    elbow = object_model.get_articulation("boom_to_outer_boom")
    wrist = object_model.get_articulation("outer_boom_to_head")

    # ----- intentional pivot captures (declare + assert connectivity)
    ctx.allow_overlap(boom, base, elem_a="boom_root_barrel", elem_b="mount_clevis_+0.24", reason="The boom root barrel is captured between the king-post clevis ears.")
    ctx.allow_overlap(boom, base, elem_a="boom_root_barrel", elem_b="mount_clevis_-0.24", reason="The boom root barrel is captured between the king-post clevis ears.")
    ctx.expect_overlap(boom, base, axes="xyz", elem_a="boom_root_barrel", elem_b="mount_clevis_+0.24", min_overlap=0.04, name="boom root captured in mount")

    ctx.allow_overlap(stick, boom, elem_a="stick_elbow_barrel", elem_b="boom_elbow_barrel", reason="Coaxial elbow bearing barrels share the captured pivot volume.")
    ctx.expect_overlap(stick, boom, axes="xyz", elem_a="stick_elbow_barrel", elem_b="boom_elbow_barrel", min_overlap=0.08, name="elbow barrels coaxial")
    ctx.allow_overlap(boom, stick, elem_a="boom_elbow_barrel", elem_b="stick_main_box", reason="The stick box section clamps over the boom elbow pivot barrel.")
    ctx.allow_overlap(stick, boom, elem_a="stick_elbow_barrel", elem_b="boom_main_box", reason="The boom box section clamps over the elbow pivot barrels at the apex knuckle.")

    ctx.allow_overlap(yoke, stick, elem_a="yoke_top_barrel", elem_b="stick_wrist_barrel", reason="The wrist yoke barrel rides on the stick wrist pin.")
    ctx.expect_overlap(yoke, stick, axes="xyz", elem_a="yoke_top_barrel", elem_b="stick_wrist_barrel", min_overlap=0.06, name="wrist yoke captured on pin")
    ctx.allow_overlap(stick, yoke, elem_a="stick_main_box", elem_b="yoke_top_barrel", reason="The stick box section wraps the wrist yoke barrel at the pivot.")
    for y in ("-0.16", "+0.16"):
        ctx.allow_overlap(stick, yoke, elem_a="stick_wrist_barrel", elem_b=f"yoke_side_plate_{y}", reason="The yoke side plates straddle and clamp the stick wrist pin barrel.")

    ctx.allow_overlap(hub, yoke, elem_a="rotator_drum", elem_b="yoke_rotator_flange", reason="The grapple rotator drum seats into the yoke rotator flange.")
    ctx.expect_overlap(hub, yoke, axes="xyz", elem_a="rotator_drum", elem_b="yoke_rotator_flange", min_overlap=0.02, name="rotator seated in yoke")
    ctx.allow_overlap(hub, yoke, elem_a="rotator_drum", elem_b="yoke_rotator_housing", reason="The rotator drum spins inside the yoke rotator housing box.")

    for i in range(4):
        finger = object_model.get_part(f"claw_finger_{i}")
        ctx.allow_overlap(finger, hub, elem_a="finger_knuckle", elem_b=f"finger_boss_{i}", reason="The claw finger knuckle pivots on its hub boss pin.")
        ctx.expect_overlap(finger, hub, axes="xyz", elem_a="finger_knuckle", elem_b=f"finger_boss_{i}", min_overlap=0.02, name=f"claw finger {i} pinned to hub")
        ctx.allow_overlap(finger, hub, elem_a="finger_knuckle", elem_b="grapple_palm", reason="The finger knuckle seats against the grapple palm rim at its pivot.")

    # ----- additional joint-cluster embeddings (barrels / ears / hoses that all
    # crowd a single pivot are intentionally interpenetrating).
    ctx.allow_overlap(boom, stick, elem_a="boom_elbow_barrel", elem_b="stick_service_hose", reason="The service hose routes past the elbow pivot barrel.")
    ctx.allow_overlap(hub, yoke, elem_a="rotator_collar", elem_b="yoke_rotator_housing", reason="The rotator collar turns inside the yoke rotator housing.")
    for y in ("-0.20", "+0.20"):
        ctx.allow_overlap(stick, yoke, elem_a=f"stick_wrist_ear_{y}", elem_b="yoke_top_barrel", reason="The stick wrist ears straddle the yoke pivot barrel.")
    for i in range(4):
        finger = object_model.get_part(f"claw_finger_{i}")
        ctx.allow_overlap(finger, hub, elem_a="finger_body", elem_b=f"finger_boss_{i}", reason="The claw finger body grows out of its pivot boss.")
        ctx.allow_overlap(finger, hub, elem_a="finger_body", elem_b="grapple_palm", reason="The claw finger root tucks under the grapple palm.")
    # root pivot: washers seated against the mount clevis ears
    for wy, cy in (("-0.22", "-0.24"), ("+0.22", "+0.24")):
        ctx.allow_overlap(boom, base, elem_a=f"boom_root_washer_{wy}", elem_b=f"mount_clevis_{cy}", reason="The boom root washers seat against the mount clevis ears.")
    # elbow: hose threads past the elbow ears
    for ey in ("-0.22", "+0.22"):
        ctx.allow_overlap(boom, stick, elem_a=f"boom_elbow_ear_{ey}", elem_b="stick_service_hose", reason="The service hose threads past the boom elbow ears.")
    # wrist/rotator: drum and stick ears crowd the yoke side plates
    for py in ("-0.16", "+0.16"):
        ctx.allow_overlap(hub, yoke, elem_a="rotator_drum", elem_b=f"yoke_side_plate_{py}", reason="The rotator drum turns between the yoke side plates.")
    for ey, py in (("-0.20", "-0.16"), ("+0.20", "+0.16")):
        ctx.allow_overlap(stick, yoke, elem_a=f"stick_wrist_ear_{ey}", elem_b=f"yoke_side_plate_{py}", reason="The stick wrist ears sit just outboard of the yoke side plates.")
    # pin caps cap the root pivot barrel; elbow ears straddle the elbow barrel
    for py in ("-0.24", "+0.24"):
        ctx.allow_overlap(base, boom, elem_a=f"mount_pin_cap_{py}", elem_b="boom_root_barrel", reason="The mount pin caps close the boom root pivot barrel.")
    for ey in ("-0.22", "+0.22"):
        ctx.allow_overlap(boom, stick, elem_a=f"boom_elbow_ear_{ey}", elem_b="stick_elbow_barrel", reason="The boom elbow ears straddle the stick elbow pivot barrel.")
    for cy, wy in (("-0.24", "-0.22"), ("+0.24", "+0.22")):
        ctx.allow_overlap(base, boom, elem_a=f"mount_pin_cap_{cy}", elem_b=f"boom_root_washer_{wy}", reason="The mount pin cap meets the boom root washer on the shared pivot pin.")
    for wy in ("-0.20", "+0.20"):
        ctx.allow_overlap(boom, stick, elem_a="boom_elbow_barrel", elem_b=f"stick_elbow_washer_{wy}", reason="The stick elbow washers seat against the boom elbow barrel on the shared pin.")
    for ey, wy in (("-0.22", "-0.20"), ("+0.22", "+0.20")):
        ctx.allow_overlap(boom, stick, elem_a=f"boom_elbow_ear_{ey}", elem_b=f"stick_elbow_washer_{wy}", reason="The boom elbow ears bracket the stick elbow washers at the shared pin.")

    # ----- scale / silhouette
    ctx.expect_origin_gap(stick, base, axis="z", min_gap=0.4, name="elbow sits above the mount")

    # ----- joint motion
    rest_hub = ctx.part_world_position(hub)
    with ctx.pose({lift: 0.45, elbow: -0.4, wrist: 0.5}):
        moved_hub = ctx.part_world_position(hub)
    ctx.check(
        "boom joints move the grapple",
        rest_hub is not None and moved_hub is not None
        and abs(moved_hub[0] - rest_hub[0]) + abs(moved_hub[2] - rest_hub[2]) > 0.3,
        details=f"rest_hub={rest_hub}, moved_hub={moved_hub}",
    )

    # claw closing brings the finger geometry toward the tool axis (use the AABB
    # centre — the finger frame origin sits on the pivot and does not move).
    def _xy_center(aabb):
        (mnx, mny, _), (mxx, mxy, _) = aabb
        return (0.5 * (mnx + mxx), 0.5 * (mny + mxy))

    f0 = object_model.get_part("claw_finger_0")
    fj0 = object_model.get_articulation("hub_to_claw_finger_0")
    hub_cx, hub_cy = _xy_center(ctx.part_world_aabb(hub))
    rest_cx, rest_cy = _xy_center(ctx.part_world_aabb(f0))
    with ctx.pose({fj0: 1.0}):
        close_cx, close_cy = _xy_center(ctx.part_world_aabb(f0))
    rest_d = sqrt((rest_cx - hub_cx) ** 2 + (rest_cy - hub_cy) ** 2)
    closed_d = sqrt((close_cx - hub_cx) ** 2 + (close_cy - hub_cy) ** 2)
    ctx.check(
        "claw fingers close toward the centre",
        closed_d < rest_d - 0.02,
        details=f"rest_d={rest_d:.3f}, closed_d={closed_d:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
