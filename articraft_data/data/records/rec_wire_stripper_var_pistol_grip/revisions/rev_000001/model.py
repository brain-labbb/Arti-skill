"""Automatic self-adjusting wire stripper.

Red-and-black plier-style tool built from two crossed stamped-steel plates that
pivot on a large chrome center button. The root arm carries the gripping jaw,
the wire-length stop slider (prismatic) with its coil spring, the slider rail,
the brass tension screw and the pivot hardware. The lever arm carries the
stripping jaw and crosses the centerline, so squeezing the red handles
(positive pivot q) swings the stripping jaw toward the gripping jaw and closes
the wire slot. Serrated wire-cutter edges are stamped into both plate profiles
between the pivot and the grips.

Layout: tool long axis = X (head at +X, handles at -X), plate stack = Z.
Bottom plate (root arm) z in [-0.0032, -0.0002]; top plate (lever arm)
z in [0.0002, 0.0032]. Pivot axis = +Z through the origin.
"""

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- constants
PLATE_T = 0.003            # stamped plate thickness
BODY_PLATE_ZC = -0.0017    # bottom plate center (z in [-0.0032, -0.0002])
LEVER_PLATE_ZC = 0.0017    # top plate center (z in [0.0002, 0.0032])
SQUEEZE_MAX = 0.10         # rad, pivot squeeze range
SLIDE_MAX = 0.008          # m, wire-stop slider travel

HANDLE_ANGLE = 0.2405      # (unused legacy value; kept for reference)
PISTOL_GRIP_ANGLE = 1.15   # rad (~66°) steep offset for pistol-grip form

# shank edge lines (root arm; lever arm is the y-mirror)
_SHANK_X0, _SHANK_X1 = -0.0105, -0.132
_UP_Y0, _UP_Y1 = 0.0105, -0.0225      # inner (serrated) edge endpoints
_LO_Y0, _LO_Y1 = -0.0105, -0.0385     # outer edge endpoints
_UP_M = (_UP_Y1 - _UP_Y0) / (_SHANK_X1 - _SHANK_X0)
_LO_M = (_LO_Y1 - _LO_Y0) / (_SHANK_X1 - _SHANK_X0)


def _y_upper(x: float) -> float:
    return _UP_Y0 + (x - _SHANK_X0) * _UP_M


def _y_lower(x: float) -> float:
    return _LO_Y0 + (x - _SHANK_X0) * _LO_M


def _ccw(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Force counter-clockwise winding via the shoelace sign."""
    area = 0.0
    n = len(points)
    for i in range(n):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        area += x0 * y1 - x1 * y0
    return points if area > 0.0 else list(reversed(points))


def _mirror(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(x, -y) for (x, y) in points]


def _circle(radius: float, n: int = 48) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(2.0 * math.pi * i / n), radius * math.sin(2.0 * math.pi * i / n))
        for i in range(n)
    ]


def _head_half_profile() -> list[tuple[float, float]]:
    """Head half plate on +Y (root arm)."""
    return [
        (0.078, 0.001),
        (0.078, 0.0245),
        (0.048, 0.029),
        (0.016, 0.0215),
        (0.011, 0.0165),
        (0.011, 0.001),
    ]


def _shank_profile() -> list[tuple[float, float]]:
    """Handle shank plate drifting to -Y, serrated wire-cutter teeth on the
    +Y (inner) edge between x = -0.052 and x = -0.016."""
    top_edge: list[tuple[float, float]] = [(_SHANK_X0, _UP_Y0)]
    top_edge.append((-0.016, _y_upper(-0.016)))
    pitch = 0.0072
    for i in range(5):
        x_dip = -0.0196 - i * pitch
        x_ret = -0.0232 - i * pitch
        top_edge.append((x_dip, _y_upper(x_dip) - 0.0045))
        top_edge.append((x_ret, _y_upper(x_ret)))
    top_edge.append((_SHANK_X1, _UP_Y1))
    poly = [
        (_SHANK_X1, _LO_Y1),          # far outer corner
        (_SHANK_X0, _LO_Y0),          # near outer corner
    ]
    poly.extend(top_edge)             # near inner -> far inner (serrated)
    return poly


def _plate_mesh(mirror: bool):
    """One stamped arm plate: head half + pivot boss + serrated shank."""
    head = _head_half_profile()
    shank = _shank_profile()
    if mirror:
        head = _mirror(head)
        shank = _mirror(shank)
    zc = LEVER_PLATE_ZC if mirror else BODY_PLATE_ZC
    plate = ExtrudeGeometry(_ccw(head), PLATE_T, cap=True, center=True)
    # boss is 0.04 mm thicker than the plate so merged faces are not
    # exactly coplanar (avoids render z-fighting where they overlap)
    boss_t = PLATE_T + 0.00004
    if mirror:
        boss = ExtrudeWithHolesGeometry(
            _ccw(_circle(0.0165)), [_ccw(_circle(0.0105))], boss_t, cap=True, center=True
        )
    else:
        boss = CylinderGeometry(0.0165, boss_t, radial_segments=48)
    plate.merge(boss)
    plate.merge(ExtrudeGeometry(_ccw(shank), PLATE_T, cap=True, center=True))
    plate.translate(0.0, 0.0, zc)
    return plate


def _grip_slab(length: float, width: float, thickness: float,
               center: tuple[float, float, float], angle: float,
               radius: float = 0.005):
    slab = ExtrudeGeometry(
        _ccw(rounded_rect_profile(length, width, radius)), thickness, cap=True, center=True
    )
    slab.rotate_z(angle)
    slab.translate(*center)
    return slab


def _spring_mesh():
    """Coil spring anchored on the spring post; free end reaches the slider."""
    x0, x1 = 0.016, 0.0225
    turns = 4.0
    n = 40
    pts = []
    for i in range(n + 1):
        t = i / n
        ang = 2.0 * math.pi * turns * t
        pts.append(
            (
                x0 + (x1 - x0) * t,
                0.0105 + 0.0022 * math.cos(ang),
                0.0045 + 0.0022 * math.sin(ang),
            )
        )
    return tube_from_spline_points(
        pts, radius=0.0008, samples_per_segment=2, radial_segments=8, cap_ends=True
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automatic_wire_stripper")

    steel_black = model.material("steel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    jaw_black = model.material("jaw_black", rgba=(0.14, 0.14, 0.15, 1.0))
    rubber_red = model.material("rubber_red", rgba=(0.78, 0.08, 0.07, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    chrome = model.material("chrome", rgba=(0.75, 0.77, 0.80, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.58, 0.25, 1.0))
    plastic_red = model.material("plastic_red", rgba=(0.85, 0.10, 0.09, 1.0))
    steel_spring = model.material("steel_spring", rgba=(0.25, 0.26, 0.28, 1.0))

    # ------------------------------------------------------------- root arm
    body = model.part("gripping_jaw_arm")
    body.visual(
        mesh_from_geometry(_plate_mesh(mirror=False), "body_plate"),
        material=steel_black,
        name="body_plate",
    )

    # gripping jaw block + serration pad + outboard riser (straddles the
    # lever plate: block bottom 0.0040 clears the top plate at 0.0032)
    riser = BoxGeometry((0.030, 0.009, 0.0051)).translate(0.061, 0.0195, 0.00195)
    body.visual(mesh_from_geometry(riser, "gripping_jaw_riser"),
                material=jaw_black, name="gripping_jaw_riser")
    jaw = BoxGeometry((0.030, 0.0205, 0.0100)).translate(0.061, 0.013750, 0.0090)
    body.visual(mesh_from_geometry(jaw, "gripping_jaw"),
                material=jaw_black, name="gripping_jaw")
    pad = BoxGeometry((0.024, 0.0037, 0.006)).translate(0.062, 0.002650, 0.0090)
    body.visual(mesh_from_geometry(pad, "gripping_jaw_pad"),
                material=steel_spring, name="gripping_jaw_pad")

    # brass tension adjustment screw on the gripping jaw
    body.visual(Cylinder(radius=0.0055, length=0.0050),
                origin=Origin(xyz=(0.052, 0.0145, 0.0160)),
                material=brass, name="tension_screw_head")
    body.visual(Cylinder(radius=0.0038, length=0.0025),
                origin=Origin(xyz=(0.052, 0.0145, 0.01975)),
                material=brass, name="tension_screw_tip")

    # slider rail, spring anchor post and the coil spring (anchored one side)
    rail = BoxGeometry((0.024, 0.004, 0.0032)).translate(0.032, 0.0105, 0.0010)
    body.visual(mesh_from_geometry(rail, "slider_rail"),
                material=steel_spring, name="slider_rail")
    body.visual(Cylinder(radius=0.0015, length=0.0081),
                origin=Origin(xyz=(0.0155, 0.0115, 0.00345)),
                material=chrome, name="spring_post")
    body.visual(mesh_from_geometry(_spring_mesh(), "stop_spring"),
                material=steel_spring, name="stop_spring")

    # chrome pivot button: shaft through both plates + front/back caps
    body.visual(Cylinder(radius=0.0090, length=0.0120),
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=chrome, name="pivot_shaft")
    body.visual(Cylinder(radius=0.0135, length=0.0030),
                origin=Origin(xyz=(0.0, 0.0, 0.0053)),
                material=chrome, name="pivot_button_cap")
    body.visual(Cylinder(radius=0.0135, length=0.0030),
                origin=Origin(xyz=(0.0, 0.0, -0.0053)),
                material=chrome, name="pivot_rear_cap")

    # small chrome rivet on the serrated shank
    body.visual(Cylinder(radius=0.0025, length=0.0016),
                origin=Origin(xyz=(-0.058, -0.012, 0.0002)),
                material=chrome, name="shank_rivet")

    # pistol-grip backstrap: steep red overmold with black inlay and palm-swell cap
    body.visual(
        mesh_from_geometry(
            _grip_slab(0.058, 0.022, 0.016, (-0.078, -0.045, -0.0016), -PISTOL_GRIP_ANGLE, radius=0.006),
            "body_grip",
        ),
        material=rubber_red, name="body_grip",
    )
    body.visual(
        mesh_from_geometry(
            _grip_slab(0.036, 0.009, 0.0022, (-0.078, -0.045, -0.0106), -PISTOL_GRIP_ANGLE, radius=0.003),
            "body_grip_inlay",
        ),
        material=rubber_black, name="body_grip_inlay",
    )
    body.visual(
        mesh_from_geometry(
            _grip_slab(0.025, 0.028, 0.017, (-0.065, -0.072, -0.0016), -PISTOL_GRIP_ANGLE, radius=0.011),
            "body_palm_swell",
        ),
        material=rubber_black, name="body_palm_swell",
    )

    # ------------------------------------------------------------ lever arm
    lever = model.part("stripping_jaw_arm")
    lever.visual(
        mesh_from_geometry(_plate_mesh(mirror=True), "lever_plate"),
        material=steel_black,
        name="lever_plate",
    )

    # stripping jaw block + pad, authored in the closed (q = SQUEEZE_MAX)
    # configuration and pre-rotated back so rest pose reads open
    s_jaw = BoxGeometry((0.030, 0.0205, 0.0112)).translate(0.061, -0.013750, 0.0084)
    s_jaw.rotate_z(-SQUEEZE_MAX)
    lever.visual(mesh_from_geometry(s_jaw, "stripping_jaw"),
                 material=jaw_black, name="stripping_jaw")
    s_pad = BoxGeometry((0.024, 0.0037, 0.006)).translate(0.062, -0.002650, 0.0090)
    s_pad.rotate_z(-SQUEEZE_MAX)
    lever.visual(mesh_from_geometry(s_pad, "stripping_jaw_pad"),
                 material=steel_spring, name="stripping_jaw_pad")

    # chrome head rivets on the top plate (as in the reference photo)
    lever.visual(Cylinder(radius=0.0035, length=0.0022),
                 origin=Origin(xyz=(0.024, -0.019, 0.0029)),
                 material=chrome, name="head_rivet_0")
    lever.visual(Cylinder(radius=0.0035, length=0.0022),
                 origin=Origin(xyz=(0.032, -0.020, 0.0029)),
                 material=chrome, name="head_rivet_1")
    lever.visual(Cylinder(radius=0.0025, length=0.0016),
                 origin=Origin(xyz=(-0.058, 0.012, 0.0036)),
                 material=chrome, name="shank_rivet")

    # pistol-grip front handle: steep red overmold with black inlay and palm-swell cap
    lever.visual(
        mesh_from_geometry(
            _grip_slab(0.058, 0.022, 0.016, (-0.078, 0.045, 0.0016), PISTOL_GRIP_ANGLE, radius=0.006),
            "lever_grip",
        ),
        material=rubber_red, name="lever_grip",
    )
    lever.visual(
        mesh_from_geometry(
            _grip_slab(0.036, 0.009, 0.0022, (-0.078, 0.045, 0.0106), PISTOL_GRIP_ANGLE, radius=0.003),
            "lever_grip_inlay",
        ),
        material=rubber_black, name="lever_grip_inlay",
    )
    lever.visual(
        mesh_from_geometry(
            _grip_slab(0.025, 0.028, 0.017, (-0.065, 0.072, 0.0016), PISTOL_GRIP_ANGLE, radius=0.011),
            "lever_palm_swell",
        ),
        material=rubber_black, name="lever_palm_swell",
    )

    # ------------------------------------------------------ wire stop slider
    slider = model.part("wire_stop_slider")
    # block bottom at z=0.0025 embeds 0.1 mm into the rail top (seated
    # sliding fit, declared in run_tests)
    s_block = BoxGeometry((0.012, 0.012, 0.0062)).translate(0.0, 0.0, 0.0056)
    slider.visual(mesh_from_geometry(s_block, "slider_body"),
                  material=plastic_red, name="slider_body")
    s_tab = BoxGeometry((0.003, 0.012, 0.0055)).translate(0.0045, 0.0, 0.01075)
    slider.visual(mesh_from_geometry(s_tab, "slider_tab"),
                  material=plastic_red, name="slider_tab")
    for sign, tag in ((-1.0, "inner"), (1.0, "outer")):
        skirt = BoxGeometry((0.012, 0.0036, 0.0026)).translate(0.0, sign * 0.0042, 0.0018)
        slider.visual(mesh_from_geometry(skirt, f"slider_skirt_{tag}"),
                      material=plastic_red, name=f"slider_skirt_{tag}")

    # ---------------------------------------------------------------- joints
    model.articulation(
        "arm_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        # +q squeezes the handles together and closes the stripping jaw
        # against the gripping jaw (lever arm crosses the pivot).
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.0, lower=0.0, upper=SQUEEZE_MAX),
    )
    model.articulation(
        "wire_stop_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(0.030, 0.0105, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.05, lower=0.0, upper=SLIDE_MAX),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("gripping_jaw_arm")
    lever = object_model.get_part("stripping_jaw_arm")
    slider = object_model.get_part("wire_stop_slider")
    pivot = object_model.get_articulation("arm_pivot")
    slide = object_model.get_articulation("wire_stop_slide")

    # overall envelope reads as a ~215 mm hand tool
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for part in object_model.parts:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            continue
        for k in range(3):
            lo[k] = min(lo[k], aabb[0][k])
            hi[k] = max(hi[k], aabb[1][k])
    ext = [hi[k] - lo[k] for k in range(3)]
    ctx.check(
        "tool envelope is plausible for a 200 mm pistol-grip wire stripper",
        0.19 <= ext[0] <= 0.24 and 0.08 <= ext[1] <= 0.18 and 0.018 <= ext[2] <= 0.04,
        details=f"extents={ext}",
    )

    # squeeze articulation actually rotates the lever arm (3D AABB center)
    rest_aabb = ctx.part_world_aabb(lever)
    rest_c = [(rest_aabb[0][k] + rest_aabb[1][k]) * 0.5 for k in range(3)]
    rest_grip = ctx.part_element_world_aabb(lever, elem="lever_grip")
    rest_grip_cy = (rest_grip[0][1] + rest_grip[1][1]) * 0.5

    # jaws open at rest: visible wire slot between the two jaw pads
    ctx.expect_gap(
        body, lever, axis="y",
        positive_elem="gripping_jaw_pad", negative_elem="stripping_jaw_pad",
        min_gap=0.004,
        name="jaw wire slot is open at rest",
    )
    with ctx.pose({pivot: SQUEEZE_MAX}):
        sq_aabb = ctx.part_world_aabb(lever)
        sq_c = [(sq_aabb[0][k] + sq_aabb[1][k]) * 0.5 for k in range(3)]
        disp = math.dist(rest_c, sq_c)
        ctx.check(
            "squeezing rotates the lever arm about the pivot",
            disp >= 0.002,
            details=f"aabb-center displacement={disp:.5f}",
        )
        sq_grip = ctx.part_element_world_aabb(lever, elem="lever_grip")
        sq_grip_cy = (sq_grip[0][1] + sq_grip[1][1]) * 0.5
        ctx.check(
            "squeeze swings the red lever grip toward the body grip",
            rest_grip_cy - sq_grip_cy >= 0.006,
            details=f"grip center y rest={rest_grip_cy:.5f} squeezed={sq_grip_cy:.5f}",
        )
        # squeezed jaws close onto the wire slot but never touch
        ctx.expect_gap(
            body, lever, axis="y",
            positive_elem="gripping_jaw_pad", negative_elem="stripping_jaw_pad",
            min_gap=0.0005, max_gap=0.003,
            name="squeeze closes the jaw wire slot without jaw contact",
        )
        ctx.expect_gap(
            lever, body, axis="y",
            positive_elem="lever_grip", negative_elem="body_grip",
            min_gap=0.002,
            name="squeezed grips do not collide",
        )

    # brass tension screw sits proud on the gripping jaw at the head
    screw = ctx.part_element_world_aabb(body, elem="tension_screw_head")
    ctx.check(
        "brass tension screw sits on top of the head",
        screw is not None
        and screw[0][2] >= 0.0125
        and screw[1][2] >= 0.018
        and 0.04 <= (screw[0][0] + screw[1][0]) * 0.5 <= 0.08,
        details=f"screw aabb={screw}",
    )

    # both arms wear red rubber grips
    def _mat_name(part, elem: str) -> str:
        mat = part.get_visual(elem).material
        return mat if isinstance(mat, str) else getattr(mat, "name", "")

    ctx.check(
        "both handles have red rubber grips",
        _mat_name(body, "body_grip") == "rubber_red"
        and _mat_name(lever, "lever_grip") == "rubber_red",
        details="grip material check",
    )

    # the red wire-length stop slider is seated on its rail (tiny declared
    # embed so the sliding fit reads seated, not floating)
    ctx.allow_overlap(
        slider, body,
        elem_a="slider_body", elem_b="slider_rail",
        reason="Slider block intentionally seats 0.1 mm into the rail top to "
               "represent the captured sliding fit of the wire-length stop.",
    )
    ctx.expect_contact(
        slider, body, elem_a="slider_body", elem_b="slider_rail",
        name="slider block rides on the rail",
    )

    # the red wire-length stop slider travels along the rail toward the jaws
    rest_s = ctx.part_world_aabb(slider)
    rest_sx = (rest_s[0][0] + rest_s[1][0]) * 0.5
    ctx.expect_overlap(
        slider, body, axes="x", elem_b="slider_rail", min_overlap=0.006,
        name="slider engages the rail at rest",
    )
    with ctx.pose({slide: SLIDE_MAX}):
        out_s = ctx.part_world_aabb(slider)
        out_sx = (out_s[0][0] + out_s[1][0]) * 0.5
        ctx.check(
            "wire stop slider slides toward the jaws",
            out_sx - rest_sx >= 0.006,
            details=f"slider center x rest={rest_sx:.5f} extended={out_sx:.5f}",
        )
        ctx.expect_overlap(
            slider, body, axes="x", elem_b="slider_rail", min_overlap=0.006,
            name="slider stays engaged on the rail at full travel",
        )
        ctx.expect_gap(
            body, slider, axis="x", positive_elem="gripping_jaw",
            min_gap=0.0005,
            name="slider stops short of the gripping jaw",
        )

    # coil spring is anchored on the body side and reaches the slider tail
    ctx.expect_gap(
        slider, body, axis="x", negative_elem="stop_spring",
        min_gap=0.0, max_gap=0.003,
        name="coil spring free end reaches the slider tail",
    )

    # --- pistol-grip form verification ---
    # grips extend steeply from the shanks (perpendicular to tool axis),
    # with palm-swell caps at the base of each handle
    body_grip_aabb = ctx.part_element_world_aabb(body, elem="body_grip")
    lever_grip_aabb = ctx.part_element_world_aabb(lever, elem="lever_grip")
    body_palm_aabb = ctx.part_element_world_aabb(body, elem="body_palm_swell")
    lever_palm_aabb = ctx.part_element_world_aabb(lever, elem="lever_palm_swell")
    ctx.check(
        "pistol grip: body_grip extends steeply below the shank centerline",
        body_grip_aabb is not None and body_grip_aabb[0][1] < -0.048,
        details=f"body_grip min_y={body_grip_aabb[0][1]:.4f}" if body_grip_aabb else "no aabb",
    )
    ctx.check(
        "pistol grip: lever_grip extends steeply above the shank centerline",
        lever_grip_aabb is not None and lever_grip_aabb[1][1] > 0.048,
        details=f"lever_grip max_y={lever_grip_aabb[1][1]:.4f}" if lever_grip_aabb else "no aabb",
    )
    ctx.check(
        "pistol grip: palm-swell rear caps present on both handles",
        body_palm_aabb is not None and lever_palm_aabb is not None
        and body_palm_aabb[0][1] < -0.055 and lever_palm_aabb[1][1] > 0.055,
        details=f"body_palm min_y={body_palm_aabb[0][1] if body_palm_aabb else None}, "
                f"lever_palm max_y={lever_palm_aabb[1][1] if lever_palm_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
