from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    sweep_profile_along_spline,
    rounded_rect_profile,
    LatheGeometry,
    ExtrudeGeometry,
    BoxGeometry,
    CylinderGeometry,
    MeshGeometry,
)

# ---------------------------------------------------------------------------
# Coordinate convention
#   +X : wearer's right (image right)
#   +Y : forward, toward the viewer (lenses face +Y)
#   +Z : up
# The front frame sits in the XZ plane near Y=0. Temple arms run back (-Y).
# ---------------------------------------------------------------------------

# Materials -----------------------------------------------------------------
SILVER = Material(name="polished_silver", rgba=(0.82, 0.84, 0.87, 1.0))
BLACK_BEZEL = Material(name="black_bezel", rgba=(0.06, 0.06, 0.07, 1.0))
AMBER_LENS = Material(name="amber_lens", rgba=(0.42, 0.16, 0.03, 0.62))
BLACK_ACETATE = Material(name="black_acetate", rgba=(0.05, 0.05, 0.06, 1.0))

# Front-frame geometric parameters ------------------------------------------
RIM_HALF_W = 0.027         # half width of one rim opening (X)
RIM_HALF_H = 0.022         # half height of one rim opening (Z) at center
RIM_TUBE_R = 0.0019        # silver rim wire radius
RIM_CENTER_X = 0.039       # |X| of each rim center from origin
LENS_Y = 0.0               # lens plane
FRAME_Y = 0.0              # frame plane

# Cat-eye upsweep parameters
CATEYE_UPSWEEP = 0.012     # extra height at the outer-upper peak
CATEYE_PEAK_ANGLE = math.radians(55)   # angle from outer direction to peak
CATEYE_PEAK_WIDTH = math.radians(25)   # Gaussian width of the upsweep bump


def _cateye_loop(sx, half_w, half_h, n=48):
    """Cat-eye lens outline with upswept outer corner.

    sx: +1 for right lens (outer = +X), -1 for left lens (outer = -X).
    Returns a list of (x, z) points in CCW order starting near the outer edge.
    The distinctive cat-eye feature is an upward-pointed peak at the
    upper-outer corner of each lens, created by a Gaussian bump on an
    elliptical base profile.
    """
    pts = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        cos_t = math.cos(t)
        sin_t = math.sin(t)

        # Base ellipse
        x = half_w * cos_t
        z = half_h * sin_t

        # Determine angle from the outer direction (upper half only)
        # For sx=+1 (right lens), outer direction is at t=0
        # For sx=-1 (left lens), outer direction is at t=π
        if 0.0 < t < math.pi:
            # Upper half
            if sx > 0:
                angle_from_outer = t
            else:
                angle_from_outer = math.pi - t
        else:
            angle_from_outer = math.pi * 10.0  # sentinel: no upsweep

        if angle_from_outer < math.pi:
            # Gaussian upsweep bump centered at CATEYE_PEAK_ANGLE
            dist = abs(angle_from_outer - CATEYE_PEAK_ANGLE)
            bump = math.exp(-0.5 * (dist / CATEYE_PEAK_WIDTH) ** 2)
            z += CATEYE_UPSWEEP * bump
            # Slight outward extension at the peak for a pointed corner
            x += sx * 0.003 * bump

        # Subtle bottom flattening for a more organic lower profile
        if sin_t < -0.5:
            flat = (-sin_t - 0.5) / 0.5
            z *= (1.0 - 0.05 * flat)

        pts.append((x, z))

    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="silver_cateye_sunglasses")
    for m in (SILVER, BLACK_BEZEL, AMBER_LENS, BLACK_ACETATE):
        model.material(m.name, rgba=m.rgba)

    front = model.part("front_frame")

    # --- Silver rims (one continuous bent tube per rim) ---
    for side, sx in (("right", 1.0), ("left", -1.0)):
        cx = sx * RIM_CENTER_X
        loop2d = _cateye_loop(sx, RIM_HALF_W, RIM_HALF_H, n=48)
        pts3d = [(cx + x, FRAME_Y, z) for (x, z) in loop2d]
        rim = tube_from_spline_points(
            pts3d,
            radius=RIM_TUBE_R,
            closed_spline=True,
            samples_per_segment=4,
            radial_segments=12,
        )
        front.visual(mesh_from_geometry(rim, f"rim_{side}"),
                     material=SILVER, name=f"rim_{side}")

        # --- Thin black inner bezel just inside each silver rim ---
        bezel_outer = _cateye_loop(
            sx, RIM_HALF_W - RIM_TUBE_R, RIM_HALF_H - RIM_TUBE_R, n=48
        )
        bezel_inner = _cateye_loop(
            sx,
            RIM_HALF_W - RIM_TUBE_R - 0.0028,
            RIM_HALF_H - RIM_TUBE_R - 0.0028,
            n=48,
        )
        bezel_geom = _ring_extrude(bezel_outer, bezel_inner, 0.0042)
        bezel_geom.translate(cx, FRAME_Y, 0.0)
        front.visual(mesh_from_geometry(bezel_geom, f"bezel_{side}"),
                     material=BLACK_BEZEL, name=f"bezel_{side}")

        # --- Amber translucent lens (slightly domed disk filling the rim) ---
        lens = _lens_geometry(
            sx,
            RIM_HALF_W - RIM_TUBE_R - 0.001,
            RIM_HALF_H - RIM_TUBE_R - 0.001,
        )
        lens.translate(cx, FRAME_Y - 0.0009, 0.0)
        front.visual(mesh_from_geometry(lens, f"lens_{side}"),
                     material=AMBER_LENS, name=f"lens_{side}")

    # --- Double bridge: two parallel horizontal silver bars + vertical struts ---
    inner_edge = RIM_CENTER_X - RIM_HALF_W + 0.001
    bar_x0 = -inner_edge
    bar_x1 = inner_edge
    for tag, bz in (("upper", 0.0125), ("lower", -0.0035)):
        bar_pts = [
            (bar_x0 - 0.001, FRAME_Y, bz),
            (0.0, FRAME_Y, bz + (0.0006 if tag == "upper" else -0.0006)),
            (bar_x1 + 0.001, FRAME_Y, bz),
        ]
        bar = tube_from_spline_points(
            bar_pts, radius=0.0013, samples_per_segment=8, radial_segments=10,
            cap_ends=True,
        )
        front.visual(mesh_from_geometry(bar, f"bridge_bar_{tag}"),
                     material=SILVER, name=f"bridge_bar_{tag}")

    # vertical struts joining the two bars
    for i, stx in enumerate((-inner_edge + 0.004, inner_edge - 0.004)):
        strut = CylinderGeometry(radius=0.0011, height=0.0175, radial_segments=8)
        strut.translate(stx, FRAME_Y, 0.0045)
        front.visual(mesh_from_geometry(strut, f"bridge_strut_{i}"),
                     material=SILVER, name=f"bridge_strut_{i}")

    # --- Nose pads on thin curved stems descending from the rims ---
    for side, sx in (("right", 1.0), ("left", -1.0)):
        cx = sx * RIM_CENTER_X
        # Find the nose attachment point on the inner-lower cat-eye rim.
        # Evaluate the loop at ~215° (inner-lower quadrant) so the stem
        # emerges from inside the rim tube, guaranteeing connectivity.
        _nose_loop = _cateye_loop(sx, RIM_HALF_W, RIM_HALF_H, n=48)
        _nose_idx = int(round(48 * 215.0 / 360.0))
        _nx_local, _nz_local = _nose_loop[_nose_idx]
        rim_attach_x = cx + _nx_local
        rim_attach_z = _nz_local

        stem_pts = [
            (rim_attach_x, FRAME_Y, rim_attach_z),
            (rim_attach_x - sx * 0.002, FRAME_Y - 0.005, rim_attach_z - 0.010),
            (rim_attach_x - sx * 0.001, FRAME_Y - 0.009, rim_attach_z - 0.020),
        ]
        stem = tube_from_spline_points(
            stem_pts, radius=0.0009, samples_per_segment=10, radial_segments=8,
            cap_ends=True,
        )
        front.visual(mesh_from_geometry(stem, f"nose_stem_{side}"),
                     material=SILVER, name=f"nose_stem_{side}")

        pad = _nose_pad_geometry()
        pad.rotate_z(sx * 0.35)
        pad.rotate_x(math.pi)
        # Seat the pad so its top face overlaps the stem endpoint in Y
        pad.translate(rim_attach_x - sx * 0.001,
                      FRAME_Y - 0.009,
                      rim_attach_z - 0.021)
        front.visual(mesh_from_geometry(pad, f"nose_pad_{side}"),
                     material=SILVER, name=f"nose_pad_{side}")

    # --- Hinge dovetails at outer upper corners of each rim ---
    # For cat-eye, the hinge sits at the upswept outer-upper area
    hinge_data = {}
    for side, sx in (("right", 1.0), ("left", -1.0)):
        cx = sx * RIM_CENTER_X
        # Position hinge block at the cat-eye upswept outer corner
        hx = cx + sx * (RIM_HALF_W - 0.005)
        hz = RIM_HALF_H + 0.003  # in the upswept zone
        hinge = BoxGeometry((0.006, 0.009, 0.010))
        hinge.translate(hx + sx * 0.0015, FRAME_Y - 0.001, hz)
        front.visual(mesh_from_geometry(hinge, f"hinge_block_{side}"),
                     material=SILVER, name=f"hinge_block_{side}")
        hinge_data[side] = (hx + sx * 0.004, FRAME_Y - 0.001, hz)

    # --- Temple arms (separate parts, revolute fold joints) ---
    for side, sx in (("right", 1.0), ("left", -1.0)):
        hinge_x, hinge_y, hinge_z = hinge_data[side]
        temple = model.part(f"temple_{side}")

        # Arm geometry in temple local frame: joint frame at the hinge.
        # Local +X points outward along the arm; at q=0 the arm extends
        # straight back (-Y) in world space via yaw=-90°.
        L = 0.118
        arm_pts = [
            (0.0, 0.0, 0.0),
            (L * 0.45, 0.0, -0.001),
            (L * 0.80, 0.0, -0.004),
            (L * 0.93, 0.0, -0.012),
        ]
        arm = sweep_profile_along_spline(
            arm_pts,
            profile=rounded_rect_profile(0.0034, 0.0026, radius=0.0010),
            samples_per_segment=10,
            up_hint=(0.0, 0.0, 1.0),
        )
        temple.visual(mesh_from_geometry(arm, f"arm_{side}"),
                      material=SILVER, name=f"arm_{side}")

        # Black acetate ear tip, slightly down-curved, at the far end.
        tip_pts = [
            (L * 0.90, 0.0, -0.010),
            (L * 0.98, 0.0, -0.016),
            (L * 1.02, 0.0, -0.026),
            (L * 1.03, 0.0, -0.038),
        ]
        tip = tube_from_spline_points(
            tip_pts, radius=0.0026, samples_per_segment=12, radial_segments=10,
            cap_ends=True,
        )
        temple.visual(mesh_from_geometry(tip, f"ear_tip_{side}"),
                      material=BLACK_ACETATE, name=f"ear_tip_{side}")

        # Revolute fold joint at the hinge.
        yaw = -math.pi / 2
        if sx > 0:
            axis = (0.0, 0.0, -1.0)
            lower, upper = 0.0, math.radians(95.0)
        else:
            axis = (0.0, 0.0, 1.0)
            lower, upper = 0.0, math.radians(95.0)

        model.articulation(
            f"fold_{side}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=temple,
            origin=Origin(xyz=(hinge_x, hinge_y, hinge_z), rpy=(0.0, 0.0, yaw)),
            axis=axis,
            motion_limits=MotionLimits(effort=2.0, velocity=3.0,
                                       lower=lower, upper=upper),
        )

    return model


def _ring_extrude(outer_loop, inner_loop, thickness):
    """Build a flat ring/frame band between two concentric closed loops."""
    g = MeshGeometry()
    n = min(len(outer_loop), len(inner_loop))
    hz = thickness / 2.0
    idx = {}
    for i in range(n):
        ox, oz = outer_loop[i]
        ix, iz = inner_loop[i]
        idx[("of", i)] = g.add_vertex(ox, hz, oz)
        idx[("ob", i)] = g.add_vertex(ox, -hz, oz)
        idx[("if", i)] = g.add_vertex(ix, hz, iz)
        idx[("ib", i)] = g.add_vertex(ix, -hz, iz)
    for i in range(n):
        j = (i + 1) % n
        of_i, of_j = idx[("of", i)], idx[("of", j)]
        ob_i, ob_j = idx[("ob", i)], idx[("ob", j)]
        if_i, if_j = idx[("if", i)], idx[("if", j)]
        ib_i, ib_j = idx[("ib", i)], idx[("ib", j)]
        # front face ring
        g.add_face(of_i, of_j, if_j)
        g.add_face(of_i, if_j, if_i)
        # back face ring
        g.add_face(ob_i, ib_j, ob_j)
        g.add_face(ob_i, ib_i, ib_j)
        # outer side wall
        g.add_face(of_i, ob_j, of_j)
        g.add_face(of_i, ob_i, ob_j)
        # inner side wall
        g.add_face(if_i, if_j, ib_j)
        g.add_face(if_i, ib_j, ib_i)
    return g


def _lens_geometry(sx, half_w, half_h, n=48):
    """Slightly domed translucent cat-eye lens for one side."""
    loop = _cateye_loop(sx, half_w, half_h, n=n)
    g = MeshGeometry()
    front_dome = 0.0026
    back = -0.0010
    ring_f = []
    ring_b = []
    for (x, z) in loop:
        ring_f.append(g.add_vertex(x, 0.0, z))
        ring_b.append(g.add_vertex(x, back, z))
    apex_f = g.add_vertex(0.0, front_dome, 0.0)
    apex_b = g.add_vertex(0.0, back, 0.0)
    n_pts = len(loop)
    for i in range(n_pts):
        j = (i + 1) % n_pts
        # front dome
        g.add_face(ring_f[i], ring_f[j], apex_f)
        # back flat
        g.add_face(ring_b[j], ring_b[i], apex_b)
        # rim edge
        g.add_face(ring_f[i], ring_b[i], ring_b[j])
        g.add_face(ring_f[i], ring_b[j], ring_f[j])
    return g


def _nose_pad_geometry():
    """Small oval slightly-cupped metal nose pad."""
    g = MeshGeometry()
    rw, rh = 0.0048, 0.0072
    th = 0.0010
    n = 16
    ring_f = []
    ring_b = []
    for i in range(n):
        a = 2 * math.pi * i / n
        x = rw * math.cos(a)
        z = rh * math.sin(a)
        ring_f.append(g.add_vertex(x, 0.0, z))
        ring_b.append(g.add_vertex(x, -th, z))
    cf = g.add_vertex(0.0, 0.0006, 0.0)
    cb = g.add_vertex(0.0, -th, 0.0)
    for i in range(n):
        j = (i + 1) % n
        g.add_face(ring_f[i], ring_f[j], cf)
        g.add_face(ring_b[j], ring_b[i], cb)
        g.add_face(ring_f[i], ring_b[i], ring_b[j])
        g.add_face(ring_f[i], ring_b[j], ring_f[j])
    return g


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front = object_model.get_part("front_frame")
    left = object_model.get_part("temple_left")
    right = object_model.get_part("temple_right")

    # (1) exactly two temple-arm revolute fold joints
    revolute_folds = [
        j for j in object_model.articulations
        if j.articulation_type == ArticulationType.REVOLUTE
        and j.name.startswith("fold_")
    ]
    ctx.check(
        "exactly two temple revolute fold joints",
        len(revolute_folds) == 2,
        details=f"found {[j.name for j in revolute_folds]}",
    )

    # each fold joint folds through a meaningful angle
    for j in revolute_folds:
        lim = j.motion_limits
        span = (lim.upper - lim.lower) if (lim and lim.lower is not None
                                           and lim.upper is not None) else 0.0
        ctx.check(
            f"{j.name} folds a meaningful angle",
            span >= math.radians(70.0),
            details=f"span={span:.3f} rad",
        )

    # Prove both arms actually move under pose.
    for side, temple, jname in (("left", left, "fold_left"),
                                ("right", right, "fold_right")):
        joint = object_model.get_articulation(jname)
        rest = ctx.part_world_aabb(temple)
        with ctx.pose({joint: math.radians(90.0)}):
            folded = ctx.part_world_aabb(temple)
        moved = False
        if rest and folded:
            d = max(abs(rest[0][k] - folded[0][k]) +
                    abs(rest[1][k] - folded[1][k]) for k in range(3))
            moved = d > 0.02
        ctx.check(f"temple_{side} actually folds", moved,
                  details=f"rest={rest}, folded={folded}")

    # (2) the bridge is a double bar (two parallel bars present)
    bridge_bars = [v for v in front.visuals if v.name and
                   v.name.startswith("bridge_bar_")]
    ctx.check(
        "double bridge has two parallel bars",
        len(bridge_bars) == 2,
        details=f"bars={[v.name for v in bridge_bars]}",
    )

    # (3) two lenses present with translucent amber material
    lenses = [v for v in front.visuals if v.name and v.name.startswith("lens_")]
    ctx.check("two lenses present", len(lenses) == 2,
              details=f"lenses={[v.name for v in lenses]}")
    amber_ok = all(_mat_name(v.material) == "amber_lens" and
                   _mat_alpha(v.material) < 0.95 for v in lenses)
    ctx.check("lenses use translucent amber material", amber_ok and len(lenses) == 2,
              details=f"materials={[_mat_name(v.material) for v in lenses]}")

    # (4) frame uses a silver metal material
    rims = [v for v in front.visuals if v.name and v.name.startswith("rim_")]
    silver_ok = len(rims) == 2 and all(_mat_name(v.material) == "polished_silver"
                                       for v in rims)
    ctx.check("silver rims use polished silver material", silver_ok,
              details=f"materials={[_mat_name(v.material) for v in rims]}")

    # (5) nose pads sit on the rear (-Y) side of the frame
    nose_pads = [v for v in front.visuals if v.name and
                 v.name.startswith("nose_pad_")]
    nose_stems = [v for v in front.visuals if v.name and
                  v.name.startswith("nose_stem_")]
    ctx.check("two nose pads present", len(nose_pads) == 2,
              details=f"pads={[v.name for v in nose_pads]}")
    for v in nose_pads:
        aabb = ctx.part_element_world_aabb(front, elem=v.name)
        pad_back = aabb[1][1] if aabb else None
        ctx.check(
            f"{v.name} sits on rear (-Y) side of frame",
            pad_back is not None and pad_back < FRAME_Y,
            details=f"pad_max_y={pad_back}, frame_y={FRAME_Y}",
        )
    for v in nose_stems:
        aabb = ctx.part_element_world_aabb(front, elem=v.name)
        stem_back = aabb[0][1] if aabb else None
        ctx.check(
            f"{v.name} curves toward rear (-Y)",
            stem_back is not None and stem_back < FRAME_Y,
            details=f"stem_min_y={stem_back}, frame_y={FRAME_Y}",
        )

    # (6) closed pose: folded arm stays near the front frame
    rj = object_model.get_articulation("fold_right")
    with ctx.pose({rj: math.radians(92.0)}):
        ctx.expect_overlap(right, front, axes="z", min_overlap=0.005,
                           name="folded right arm stays near frame in Z")

    # (7) Cat-eye shape verification: each lens has an upswept outer corner
    # that extends significantly above the base half-height, proving the
    # distinctive cat-eye silhouette rather than a plain rounded shape.
    for side, sx in [("right", 1.0), ("left", -1.0)]:
        lens_name = f"lens_{side}"
        aabb = ctx.part_element_world_aabb(front, elem=lens_name)
        if aabb:
            max_z = aabb[1][2]
            min_z = aabb[0][2]
            center_z = (max_z + min_z) / 2.0
            # The upswept outer corner must extend well above the base half_h
            ctx.check(
                f"lens_{side} has cat-eye upswept outer corner",
                max_z > RIM_HALF_H + 0.005,
                details=f"max_z={max_z:.4f}, threshold={RIM_HALF_H + 0.005:.4f}",
            )
            # Verify the upsweep exceeds what a plain ellipse would produce.
            # The lens half_h is (RIM_HALF_H - RIM_TUBE_R - 0.001); a plain
            # ellipse could not exceed that. The cat-eye upsweep must push
            # max_z well above it.
            lens_base_h = RIM_HALF_H - RIM_TUBE_R - 0.001
            ctx.check(
                f"lens_{side} upsweep exceeds plain ellipse",
                max_z > lens_base_h + 0.005,
                details=f"max_z={max_z:.4f}, base={lens_base_h:.4f}",
            )

    # (8) Cat-eye rims also show the upswept shape
    for side, sx in [("right", 1.0), ("left", -1.0)]:
        rim_name = f"rim_{side}"
        aabb = ctx.part_element_world_aabb(front, elem=rim_name)
        if aabb:
            rim_max_z = aabb[1][2]
            ctx.check(
                f"rim_{side} follows cat-eye upswept outline",
                rim_max_z > RIM_HALF_H + 0.004,
                details=f"rim_max_z={rim_max_z:.4f}, threshold={RIM_HALF_H + 0.004:.4f}",
            )

    return ctx.report()


def _mat_name(mat):
    return getattr(mat, "name", mat) if mat is not None else None


def _mat_alpha(mat):
    rgba = getattr(mat, "rgba", None)
    if rgba and len(rgba) == 4:
        return rgba[3]
    return 1.0


object_model = build_object_model()
