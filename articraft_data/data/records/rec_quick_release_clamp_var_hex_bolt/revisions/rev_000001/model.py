from __future__ import annotations

from math import pi, radians

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Layout constants (meters). World frame: +Z is the collar bore axis (up),
# the open Omega throat faces -X, the cross bolt runs along Y through the two
# clamping feet. The +Y side now carries a recessed socket-head bolt with a
# stowable hex key hinged from the socket mouth; the -Y side keeps the spinning
# knurled barrel adjuster nut.
# ---------------------------------------------------------------------------
BORE_R = 0.016  # inner bore radius (0.032 m diameter per prompt)
BAND_OUTER_R = 0.021  # bore + 0.005 wall
BAND_H = 0.015  # band height per prompt

# Omega throat: the ring stays OPEN on the -X side. A wedge sector is removed so
# the bore breaks out into the gap between the two feet, giving the Omega shape.
THROAT_OPEN_OUT = 0.0098  # half width of the throat at the far (outer) edge
THROAT_APEX_X = 0.0015  # wedge apex just past bore center toward +X

LUG_X_MIN, LUG_X_MAX = -0.038, -0.016  # flat boss feet flanking the throat
LUG_Y_IN, LUG_Y_OUT = 0.0050, 0.0100  # foot inner/outer faces in |y|
LUG_LEN = LUG_X_MAX - LUG_X_MIN
LUG_XC = 0.5 * (LUG_X_MIN + LUG_X_MAX)
LUG_T = LUG_Y_OUT - LUG_Y_IN
LUG_YC = 0.5 * (LUG_Y_IN + LUG_Y_OUT)

PIVOT_X = -0.0305  # cross-bolt axis
PIVOT_Z = 0.5 * BAND_H

BOLT_R = 0.0025
BOLT_Y_MIN, BOLT_Y_MAX = -0.0225, 0.0225  # spans nut, feet, and the socket head

WASHER_R = 0.0064
WASHER_LEN = 0.0016

SOCKET_HEAD_R = 0.0072
SOCKET_HEAD_LEN = 0.0085
# CadQuery's XZ workplane produces this head with a Y half-depth equal to
# SOCKET_HEAD_LEN; keep the back face seated on the thrust washer.
SOCKET_HEAD_YC = LUG_Y_OUT + 0.5 * WASHER_LEN + SOCKET_HEAD_LEN
SOCKET_FACE_Y = SOCKET_HEAD_YC + SOCKET_HEAD_LEN
SOCKET_RECESS_DEPTH = 0.0044
SOCKET_RECESS_D = 0.0065

HEX_KEY_D = 0.0030
HEX_KEY_ARM_LEN = 0.038
HEX_KEY_ARM_Y = 0.00145
HEX_KEY_HINGE_R = 0.00165
HEX_KEY_HINGE_LEN = 0.0050

NUT_D = 0.022  # knurled adjuster barrel nut per prompt
NUT_LEN = 0.020
NUT_YC = -LUG_Y_OUT - WASHER_LEN - 0.0004 - 0.5 * NUT_LEN  # outside the -Y foot face

HEX_KEY_OPEN = radians(92.0)


def _throat_notch_solid() -> cq.Workplane:
    """Wedge sector removed from the -X side to open the ring into an Omega."""
    return (
        cq.Workplane("XY")
        .polyline(
            [
                (THROAT_APEX_X, 0.0),
                (-(BAND_OUTER_R + 0.005), THROAT_OPEN_OUT),
                (-(BAND_OUTER_R + 0.005), -THROAT_OPEN_OUT),
            ]
        )
        .close()
        .extrude(BAND_H + 0.012)
        .translate((0.0, 0.0, -0.006))
    )


def _collar_band_solid() -> cq.Workplane:
    """Open split-ring band in an Omega profile: the ring wraps the bore but
    stays OPEN on the -X (foot) side, where the bore breaks out into the gap
    between the two clamping feet."""
    band = cq.Workplane("XY").circle(BAND_OUTER_R).circle(BORE_R).extrude(BAND_H)
    # Shallow counterbore step at the top opening (visible seat lip).
    counterbore = (
        cq.Workplane("XY", origin=(0.0, 0.0, BAND_H - 0.005)).circle(0.0175).extrude(0.007)
    )
    band = band.cut(counterbore)
    # Omega opening on the -X side.
    return band.cut(_throat_notch_solid())


def _annular_ring_solid(inner_r: float, outer_r: float, height: float) -> cq.Workplane:
    """Annular ring with the Omega throat removed so it never re-closes the gap."""
    ring = cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(height)
    return ring.cut(_throat_notch_solid())


def _lug_solid(yc: float) -> cq.Workplane:
    lug = (
        cq.Workplane("XY")
        .box(LUG_LEN, LUG_T, BAND_H, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.0010)
        .edges(">Z or <Z")
        .fillet(0.00045)
    )
    return lug.translate((LUG_XC, yc, 0.5 * BAND_H))


def _washer_solid() -> cq.Workplane:
    # Inner radius < bolt radius so the washer grips the bolt (stays connected).
    return _annular_washer_solid(BOLT_R * 0.8, WASHER_R, WASHER_LEN)


def _annular_washer_solid(inner_r: float, outer_r: float, height: float) -> cq.Workplane:
    """Plain (un-notched) annular ring, for washers riding on the bolt."""
    return cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(height)


def _socket_head_solid() -> cq.Workplane:
    """Cylindrical socket-head bolt head with a real recessed hex drive.

    Built around local Y so the visible socket face is the +Y end.
    """
    head = (
        cq.Workplane("XZ")
        .circle(SOCKET_HEAD_R)
        .extrude(SOCKET_HEAD_LEN, both=True)
    )
    recess = (
        cq.Workplane("XZ", origin=(0.0, 0.5 * SOCKET_HEAD_LEN + 0.0002, 0.0))
        .polygon(6, SOCKET_RECESS_D)
        .extrude(-(SOCKET_RECESS_DEPTH + 0.0004))
    )
    return head.cut(recess)


def _hex_socket_floor_solid() -> cq.Workplane:
    """Dark floor at the bottom of the recessed socket, seated in the cut."""
    y_floor = 0.5 * SOCKET_HEAD_LEN - SOCKET_RECESS_DEPTH + 0.00008
    return (
        cq.Workplane("XZ", origin=(0.0, y_floor, 0.0))
        .polygon(6, SOCKET_RECESS_D * 1.25)
        .extrude(-0.00016)
    )


def _hex_drive_bit_solid() -> cq.Workplane:
    """Short hexagonal bit that stows inside the socket recess."""
    return (
        cq.Workplane("XZ")
        .polygon(6, HEX_KEY_D)
        .extrude(SOCKET_RECESS_DEPTH - 0.00055)
    )


def _hex_key_hinge_solid() -> cq.Workplane:
    """Small round hinge knuckle visible in the socket mouth."""
    return (
        cq.Workplane("YZ")
        .circle(HEX_KEY_HINGE_R)
        .extrude(HEX_KEY_HINGE_LEN, both=True)
    )


def _folding_hex_key_arm_solid() -> cq.Workplane:
    """Long stowable Allen-key handle, a hexagonal rod folded against the head."""
    return (
        cq.Workplane("XY")
        .polygon(6, HEX_KEY_D)
        .extrude(HEX_KEY_ARM_LEN)
        .translate((0.0, HEX_KEY_ARM_Y, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quick_release_seat_clamp")

    alu_body = model.material("brushed_aluminum_body", color=(0.66, 0.67, 0.69, 1.0))
    alu_key = model.material("brushed_aluminum_hex_key", color=(0.72, 0.73, 0.75, 1.0))
    alu_nut = model.material("brushed_aluminum_nut", color=(0.69, 0.70, 0.72, 1.0))
    steel = model.material("silver_steel_bolt", color=(0.55, 0.56, 0.58, 1.0))
    brushed_dark = model.material("fine_brushed_shadow_lines", color=(0.42, 0.43, 0.45, 1.0))
    brushed_light = model.material("polished_edge_highlights", color=(0.82, 0.83, 0.84, 1.0))

    # ------------------------------------------------------------------ collar
    collar = model.part("collar")
    collar.visual(
        mesh_from_cadquery(_collar_band_solid(), "collar_band"),
        origin=Origin(),
        material=alu_body,
        name="collar_band",
    )
    collar.visual(
        mesh_from_cadquery(_lug_solid(LUG_YC), "lug_cap_side"),
        origin=Origin(),
        material=alu_body,
        name="lug_cap_side",
    )
    collar.visual(
        mesh_from_cadquery(_lug_solid(-LUG_YC), "lug_nut_side"),
        origin=Origin(),
        material=alu_body,
        name="lug_nut_side",
    )
    for z, name in ((0.0004, "lower_machined_lip"), (BAND_H - 0.0008, "upper_machined_lip")):
        collar.visual(
            mesh_from_cadquery(
                _annular_ring_solid(BORE_R + 0.0004, BAND_OUTER_R + 0.0003, 0.00045), name
            ),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brushed_light,
            name=name,
        )
    for z, name in ((0.0043, "lower_brushed_groove"), (0.0107, "upper_brushed_groove")):
        collar.visual(
            mesh_from_cadquery(
                _annular_ring_solid(BAND_OUTER_R + 0.00005, BAND_OUTER_R + 0.00025, 0.00018), name
            ),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brushed_dark,
            name=name,
        )
    collar.visual(
        Cylinder(radius=BOLT_R, length=BOLT_Y_MAX - BOLT_Y_MIN),
        origin=Origin(
            xyz=(PIVOT_X, 0.5 * (BOLT_Y_MIN + BOLT_Y_MAX), PIVOT_Z), rpy=(pi / 2.0, 0.0, 0.0)
        ),
        material=steel,
        name="cross_bolt",
    )
    collar.visual(
        mesh_from_cadquery(_washer_solid(), "socket_side_thrust_washer"),
        origin=Origin(
            xyz=(PIVOT_X, LUG_Y_OUT + 0.5 * WASHER_LEN, PIVOT_Z), rpy=(pi / 2.0, 0.0, 0.0)
        ),
        material=brushed_light,
        name="socket_side_thrust_washer",
    )
    collar.visual(
        mesh_from_cadquery(_washer_solid(), "nut_side_thrust_washer"),
        origin=Origin(
            xyz=(PIVOT_X, -LUG_Y_OUT - 0.5 * WASHER_LEN, PIVOT_Z), rpy=(pi / 2.0, 0.0, 0.0)
        ),
        material=brushed_light,
        name="nut_side_thrust_washer",
    )
    collar.visual(
        mesh_from_cadquery(_socket_head_solid(), "socket_head_bolt_y_axis"),
        origin=Origin(xyz=(PIVOT_X, SOCKET_HEAD_YC, PIVOT_Z)),
        material=steel,
        name="socket_head_bolt",
    )
    collar.visual(
        mesh_from_cadquery(_hex_socket_floor_solid(), "hex_socket_recess_y_axis"),
        origin=Origin(xyz=(PIVOT_X, SOCKET_FACE_Y - SOCKET_RECESS_DEPTH + 0.00035, PIVOT_Z)),
        material=brushed_dark,
        name="hex_socket_recess",
    )

    # ----------------------------------------------------------- folding key
    hex_key = model.part("hex_key")
    hex_key.visual(
        mesh_from_cadquery(_hex_drive_bit_solid(), "hex_drive_bit_y_axis"),
        origin=Origin(),
        material=alu_key,
        name="hex_drive_bit",
    )
    hex_key.visual(
        mesh_from_cadquery(_hex_key_hinge_solid(), "hinge_knuckle_x_axis"),
        origin=Origin(),
        material=brushed_light,
        name="hinge_knuckle",
    )
    hex_key.visual(
        mesh_from_cadquery(_folding_hex_key_arm_solid(), "folding_hex_arm"),
        origin=Origin(),
        material=alu_key,
        name="folding_hex_arm",
    )
    # ------------------------------------------------------------ adjuster nut
    adjuster_nut = model.part("adjuster_nut")
    knurled = KnobGeometry(
        NUT_D,
        NUT_LEN,
        body_style="cylindrical",
        edge_radius=0.0008,
        grip=KnobGrip(style="knurled", count=40, depth=0.0006, helix_angle_deg=0.0),
        bore=KnobBore(style="round", diameter=0.0046, through=False),
        center=True,
    )
    adjuster_nut.visual(
        mesh_from_geometry(knurled, "knurled_nut"),
        # Knob local +Z -> world -Y, so the bore opening faces the foot/bolt.
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=alu_nut,
        name="knurled_nut",
    )

    # ------------------------------------------------------------ articulations
    model.articulation(
        "hex_key_hinge",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=hex_key,
        origin=Origin(xyz=(PIVOT_X, SOCKET_FACE_Y, PIVOT_Z)),
        # The hinge pin is a real visible knuckle across X at the socket mouth;
        # positive travel folds the hex arm outward from the bolt head.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=0.0, upper=HEX_KEY_OPEN),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )
    model.articulation(
        "adjuster_nut_spin",
        ArticulationType.CONTINUOUS,
        parent=collar,
        child=adjuster_nut,
        origin=Origin(xyz=(PIVOT_X, NUT_YC, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    collar = object_model.get_part("collar")
    hex_key = object_model.get_part("hex_key")
    adjuster_nut = object_model.get_part("adjuster_nut")
    hinge = object_model.get_articulation("hex_key_hinge")
    spin = object_model.get_articulation("adjuster_nut_spin")

    # Intentional captured fits on the fixed clamp hardware.
    ctx.allow_overlap(
        collar,
        adjuster_nut,
        elem_a="cross_bolt",
        elem_b="knurled_nut",
        reason="The bolt end threads into the adjuster nut bore (thread engagement proxy).",
    )
    ctx.allow_overlap(
        collar,
        hex_key,
        elem_a="socket_head_bolt",
        elem_b="hex_drive_bit",
        reason="The stowed hex key bit is intentionally inserted into the recessed socket drive.",
    )
    ctx.allow_overlap(
        collar,
        hex_key,
        elem_a="socket_head_bolt",
        elem_b="hinge_knuckle",
        reason="The tiny folding-key hinge knuckle is seated in the socket-head mouth.",
    )

    # Joint semantics match the revised stowable hex-key mechanism.
    ctx.check(
        "hex key hinges out of the recessed socket head",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(hinge.axis) == (-1.0, 0.0, 0.0)
        and hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower - 0.0) < 1e-9
        and abs(hinge.motion_limits.upper - HEX_KEY_OPEN) < 1e-6,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={hinge.motion_limits}",
    )
    ctx.check(
        "adjuster nut spins continuously about the same bolt axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 1.0, 0.0)
        and abs(spin.origin.xyz[0] - hinge.origin.xyz[0]) < 1e-9
        and abs(spin.origin.xyz[2] - hinge.origin.xyz[2]) < 1e-9,
        details=f"type={spin.articulation_type}, axis={spin.axis}, origin={spin.origin.xyz}",
    )
    authored_part_names = tuple(part.name for part in object_model.parts)
    authored_visual_names = tuple(visual.name for part in object_model.parts for visual in part.visuals)
    ctx.check(
        "cam lever is replaced by socket-head bolt and hinged hex key",
        "cam_lever" not in authored_part_names
        and "lever_handle" not in authored_visual_names
        and "socket_head_bolt" in authored_visual_names
        and "hex_drive_bit" in authored_visual_names
        and "folding_hex_arm" in authored_visual_names,
        details=f"parts={authored_part_names}, visuals={authored_visual_names}",
    )

    # Omega throat: the ring is genuinely open on the -X side. There must be no
    # collar band material spanning the throat near the bore mouth.
    band_aabb = ctx.part_element_world_aabb(collar, elem="collar_band")
    ctx.check(
        "collar band leaves an open Omega throat on the -X side",
        band_aabb is not None and band_aabb[0][0] <= -BORE_R + 1e-4,
        details=f"collar_band aabb={band_aabb}",
    )

    # Socket-head bolt: a cylindrical head sits on the +Y lug face, with a
    # visible recessed hex socket centered on the cross-bolt axis.
    head_home = ctx.part_element_world_aabb(collar, elem="socket_head_bolt")
    ctx.check(
        "socket-head bolt protrudes from the socket-side lug face",
        head_home is not None
        and head_home[0][1] <= LUG_Y_OUT + WASHER_LEN + 0.001
        and head_home[1][1] > LUG_Y_OUT + 0.006,
        details=f"socket_head_bolt aabb={head_home}, socket-side foot outer y={LUG_Y_OUT}",
    )
    ctx.expect_gap(
        collar,
        collar,
        axis="y",
        positive_elem="socket_head_bolt",
        negative_elem="socket_side_thrust_washer",
        min_gap=-0.0002,
        max_gap=0.0006,
        name="socket head seats against the socket-side thrust washer",
    )
    ctx.expect_within(
        collar,
        collar,
        axes="xz",
        inner_elem="cross_bolt",
        outer_elem="socket_head_bolt",
        margin=0.0,
        name="cross bolt stays captured inside the socket-head cross-section",
    )
    ctx.expect_within(
        collar,
        collar,
        axes="xz",
        inner_elem="hex_socket_recess",
        outer_elem="socket_head_bolt",
        margin=0.0001,
        name="recessed hex socket is centered inside the socket head",
    )

    # Stowed pose: the bit is inserted into the socket and the folding arm is a
    # real hex rod, not the old broad cam blade.
    closed_bit = ctx.part_element_world_aabb(hex_key, elem="hex_drive_bit")
    closed_arm = ctx.part_element_world_aabb(hex_key, elem="folding_hex_arm")
    ctx.check(
        "stowed hex bit reaches into the socket head",
        closed_bit is not None
        and head_home is not None
        and closed_bit[0][1] < SOCKET_FACE_Y - 0.003
        and closed_bit[1][1] <= SOCKET_FACE_Y + 0.0002,
        details=f"closed_bit={closed_bit}, socket_face_y={SOCKET_FACE_Y}, head={head_home}",
    )
    ctx.check(
        "stowed hex key has a slender hex-rod arm",
        closed_arm is not None
        and (closed_arm[1][2] - closed_arm[0][2]) > 0.030
        and (closed_arm[1][0] - closed_arm[0][0]) < 0.0045
        and (closed_arm[1][1] - closed_arm[0][1]) < 0.0055,
        details=f"closed folding_hex_arm aabb={closed_arm}",
    )
    ctx.expect_overlap(
        hex_key,
        collar,
        axes="xz",
        elem_a="hex_drive_bit",
        elem_b="hex_socket_recess",
        min_overlap=0.0025,
        name="hex bit is coaxial with the recessed socket",
    )

    # Open pose: the key hinges out away from the socket head so it can be used
    # as a turning handle while the bolt, collar, and adjuster nut stay fixed.
    closed_head = ctx.part_element_world_aabb(collar, elem="socket_head_bolt")
    with ctx.pose({hinge: HEX_KEY_OPEN}):
        open_head = ctx.part_element_world_aabb(collar, elem="socket_head_bolt")
        arm_open = ctx.part_element_world_aabb(hex_key, elem="folding_hex_arm")
        bit_open = ctx.part_element_world_aabb(hex_key, elem="hex_drive_bit")
        ctx.check(
            "opening the hex key leaves the socket-head bolt fixed",
            closed_head is not None and open_head == closed_head,
            details=f"closed_head={closed_head}, open_head={open_head}",
        )
        ctx.check(
            "hex key arm hinges outward from the bolt face",
            arm_open is not None and closed_arm is not None and arm_open[1][1] > closed_arm[1][1] + 0.025,
            details=f"closed_arm={closed_arm}, open_arm={arm_open}",
        )
        ctx.check(
            "opened bit leaves the socket recess",
            bit_open is not None
            and bit_open[0][1] > SOCKET_FACE_Y - 0.0015
            and bit_open[1][1] > SOCKET_FACE_Y + 0.0010,
            details=f"open bit aabb={bit_open}, socket_face_y={SOCKET_FACE_Y}",
        )

    # Adjuster nut seats against the nut-side foot face, centered on the bolt.
    ctx.expect_gap(
        collar,
        adjuster_nut,
        axis="y",
        positive_elem="nut_side_thrust_washer",
        negative_elem="knurled_nut",
        min_gap=-0.001,
        max_gap=0.001,
        name="knurled nut seats against the nut-side thrust washer",
    )
    ctx.expect_overlap(
        adjuster_nut,
        collar,
        axes="xz",
        elem_a="knurled_nut",
        elem_b="cross_bolt",
        min_overlap=0.004,
        name="knurled nut is coaxial with the cross bolt",
    )
    with ctx.pose({spin: pi}):
        ctx.expect_gap(
            collar,
            adjuster_nut,
            axis="y",
            positive_elem="nut_side_thrust_washer",
            negative_elem="knurled_nut",
            min_gap=-0.001,
            max_gap=0.001,
            name="spinning the nut keeps it seated on the thrust washer",
        )

    return ctx.report()


object_model = build_object_model()
