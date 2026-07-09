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
RIM_HALF_W = 0.0265        # half width of one rim opening (X)
RIM_HALF_H = 0.0245        # half height of one rim opening (Z)
RIM_CORNER = 0.0085        # rim corner rounding
RIM_TUBE_R = 0.0019        # silver rim wire radius
RIM_CENTER_X = 0.039       # |X| of each rim center from origin
LENS_Y = 0.0               # lens plane
FRAME_Y = 0.0              # frame plane


def _rounded_square_loop(half_w, half_h, corner, n_corner=8):
    """Closed (x,z) loop of a rounded square, centered at origin, CCW."""
    pts = []
    # corner centers
    cx = half_w - corner
    cz = half_h - corner
    # order: start at right edge mid going CCW
    corners = [
        (cx, cz, 0.0),               # top-right
        (-cx, cz, math.pi / 2),      # top-left
        (-cx, -cz, math.pi),         # bottom-left
        (cx, -cz, 3 * math.pi / 2),  # bottom-right
    ]
    for ccx, ccz, base in corners:
        for i in range(n_corner + 1):
            a = base + (math.pi / 2) * (i / n_corner)
            pts.append((ccx + corner * math.cos(a), ccz + corner * math.sin(a)))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="silver_aviator_sunglasses")
    for m in (SILVER, BLACK_BEZEL, AMBER_LENS, BLACK_ACETATE):
        model.material(m.name, rgba=m.rgba)

    front = model.part("front_frame")

    # --- Silver rims (one continuous bent tube per rim) ---
    for side, sx in (("right", 1.0), ("left", -1.0)):
        cx = sx * RIM_CENTER_X
        loop2d = _rounded_square_loop(RIM_HALF_W, RIM_HALF_H, RIM_CORNER, n_corner=7)
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
        bezel_outer = _rounded_square_loop(
            RIM_HALF_W - RIM_TUBE_R, RIM_HALF_H - RIM_TUBE_R, RIM_CORNER, n_corner=7
        )
        bezel_inner = _rounded_square_loop(
            RIM_HALF_W - RIM_TUBE_R - 0.0028,
            RIM_HALF_H - RIM_TUBE_R - 0.0028,
            RIM_CORNER * 0.7,
            n_corner=7,
        )
        bezel_geom = ExtrudeGeometry.centered(bezel_outer, 0.0042, cap=True)
        # carve nothing; build a thin ring frame instead via two extrudes overlay
        bezel_geom = _ring_extrude(bezel_outer, bezel_inner, 0.0042)
        bezel_geom.translate(cx, FRAME_Y, 0.0)
        front.visual(mesh_from_geometry(bezel_geom, f"bezel_{side}"),
                     material=BLACK_BEZEL, name=f"bezel_{side}")

        # --- Amber translucent lens (slightly domed disk filling the rim) ---
        lens = _lens_geometry(
            RIM_HALF_W - RIM_TUBE_R - 0.001,
            RIM_HALF_H - RIM_TUBE_R - 0.001,
            RIM_CORNER * 0.7,
        )
        lens.translate(cx, FRAME_Y - 0.0009, 0.0)
        front.visual(mesh_from_geometry(lens, f"lens_{side}"),
                     material=AMBER_LENS, name=f"lens_{side}")

    # --- Double bridge: two parallel horizontal silver bars + vertical struts ---
    bridge_span_x = RIM_CENTER_X - (RIM_HALF_W - 0.001)  # from one rim inner edge
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
        strut.rotate_x(math.pi / 2)  # along Z? cylinder is along Z by default
        # cylinder default along Z; we want vertical (Z) so no rotation needed
        strut = CylinderGeometry(radius=0.0011, height=0.0175, radial_segments=8)
        strut.translate(stx, FRAME_Y, 0.0045)
        front.visual(mesh_from_geometry(strut, f"bridge_strut_{i}"),
                     material=SILVER, name=f"bridge_strut_{i}")

    # --- Nose pads on thin curved stems descending from the rims ---
    # Real glasses carry the nose pads on the BACK face of the frame (-Y, the
    # wearer's side). The slim pad-arm comes off the inner-lower rim, curves
    # down-and-back, and the small oval pad sits behind the lens plane with its
    # cupped face turned inward (toward the nose) and backward (toward -Y).
    for side, sx in (("right", 1.0), ("left", -1.0)):
        cx = sx * RIM_CENTER_X
        rim_inner_x = cx - sx * (RIM_HALF_W - 0.002)
        stem_pts = [
            (rim_inner_x, FRAME_Y, -RIM_HALF_H + 0.004),
            (cx - sx * (RIM_HALF_W + 0.0015), FRAME_Y - 0.004, -RIM_HALF_H - 0.004),
            (cx - sx * (RIM_HALF_W + 0.001), FRAME_Y - 0.008, -RIM_HALF_H - 0.012),
        ]
        stem = tube_from_spline_points(
            stem_pts, radius=0.0009, samples_per_segment=10, radial_segments=8,
            cap_ends=True,
        )
        front.visual(mesh_from_geometry(stem, f"nose_stem_{side}"),
                     material=SILVER, name=f"nose_stem_{side}")

        pad = _nose_pad_geometry()
        # Orient pad: oval flat/cupped face turned inward toward the nose
        # (+/-X) and backward toward the wearer (-Y).
        pad.rotate_z(sx * 0.35)
        pad.rotate_x(math.pi)  # flip cupped face to point toward -Y (rear)
        pad.translate(cx - sx * (RIM_HALF_W + 0.0005),
                      FRAME_Y - 0.010,
                      -RIM_HALF_H - 0.013)
        front.visual(mesh_from_geometry(pad, f"nose_pad_{side}"),
                     material=SILVER, name=f"nose_pad_{side}")

    # --- Hinge dovetails (boxy metal blocks) at outer top corners of each rim ---
    hinge_data = {}
    for side, sx in (("right", 1.0), ("left", -1.0)):
        cx = sx * RIM_CENTER_X
        hx = cx + sx * (RIM_HALF_W - 0.001)
        hz = RIM_HALF_H - 0.006
        hinge = BoxGeometry((0.006, 0.009, 0.010))
        hinge.translate(hx + sx * 0.0015, FRAME_Y - 0.001, hz)
        front.visual(mesh_from_geometry(hinge, f"hinge_block_{side}"),
                     material=SILVER, name=f"hinge_block_{side}")
        hinge_data[side] = (hx + sx * 0.004, FRAME_Y - 0.001, hz)

    # --- Temple arms (separate parts, revolute fold joints) ---
    for side, sx in (("right", 1.0), ("left", -1.0)):
        hinge_x, hinge_y, hinge_z = hinge_data[side]
        temple = model.part(f"temple_{side}")

        # Arm geometry authored in the temple's local frame, where the joint
        # frame sits at the hinge. Local +X points outward from the hinge along
        # the arm; at q=0 the arm extends straight back (-Y) in world space.
        # We author the arm along local +X so axis/origin can rotate it back.
        L = 0.118
        # straight tapered arm from hinge outward
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
        # Joint origin = hinge location in front_frame frame.
        # At q=0 we want the arm (local +X) to point straight back (-Y world).
        # rpy yaw rotates local +X. For the right side (sx=+1, arm should point
        # -Y), yaw = -90deg sends local +X -> world -Y... let's reason:
        #   yaw rotation about Z: +X -> (cos,sin). yaw=+90 -> +Y, yaw=-90 -> -Y.
        # So both sides want yaw = -90deg to point local +X to world -Y.
        # Folding: positive q about +Z swings the arm toward +X/-... we want the
        # arm to fold across the front (toward the opposite side, i.e. -X for
        # right arm). Right arm folded points roughly -X. Choose axis sign so
        # positive q folds inward.
        # At q=0 the arm (local +X) points straight back (-Y world) via yaw=-90.
        # Folding across the front means the open arm (-Y) should swing toward
        # the frame center:
        #   right arm (X>0): -Y -> -X  => positive rotation about -Z
        #   left  arm (X<0): -Y -> +X  => positive rotation about +Z
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
    """Build a flat ring/frame band between two concentric closed loops by
    connecting them with quads, plus front and back face rings."""
    g = MeshGeometry()
    n = min(len(outer_loop), len(inner_loop))
    hz = thickness / 2.0
    # vertices: for each i, outer/inner at +y(front) and -y(back)
    # plane is XZ; thickness along Y.
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
        # front face ring (between outer and inner, at +y)
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


def _lens_geometry(half_w, half_h, corner):
    """Slightly domed translucent lens filling a rounded-square rim."""
    loop = _rounded_square_loop(half_w, half_h, corner, n_corner=7)
    g = MeshGeometry()
    front_dome = 0.0026
    back = -0.0010
    # rim ring vertices (at small +y) and center apex
    ring_f = []
    ring_b = []
    for (x, z) in loop:
        ring_f.append(g.add_vertex(x, 0.0, z))
        ring_b.append(g.add_vertex(x, back, z))
    apex_f = g.add_vertex(0.0, front_dome, 0.0)
    apex_b = g.add_vertex(0.0, back, 0.0)
    n = len(loop)
    for i in range(n):
        j = (i + 1) % n
        # front dome
        g.add_face(ring_f[i], ring_f[j], apex_f)
        # back flat
        g.add_face(ring_b[j], ring_b[i], apex_b)
        # rim edge
        g.add_face(ring_f[i], ring_b[i], ring_b[j])
        g.add_face(ring_f[i], ring_b[j], ring_f[j])
    return g


def _nose_pad_geometry():
    """Small oval slightly-cupped metal nose pad (flat oval disk)."""
    g = MeshGeometry()
    rw, rh = 0.0048, 0.0072  # oval half-extents (in pad-local X,Z)
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

    # (5) nose pads and their stems sit on the -Y (rear) side of the frame
    # plane, behind the lens plane, toward the wearer's nose.
    nose_pads = [v for v in front.visuals if v.name and
                 v.name.startswith("nose_pad_")]
    nose_stems = [v for v in front.visuals if v.name and
                  v.name.startswith("nose_stem_")]
    ctx.check("two nose pads present", len(nose_pads) == 2,
              details=f"pads={[v.name for v in nose_pads]}")
    for v in nose_pads:
        aabb = ctx.part_element_world_aabb(front, elem=v.name)
        pad_back = aabb[1][1] if aabb else None  # max Y of the pad
        ctx.check(
            f"{v.name} sits on rear (-Y) side of frame",
            pad_back is not None and pad_back < FRAME_Y,
            details=f"pad_max_y={pad_back}, frame_y={FRAME_Y}",
        )
    for v in nose_stems:
        aabb = ctx.part_element_world_aabb(front, elem=v.name)
        stem_back = aabb[0][1] if aabb else None  # min Y of the stem
        ctx.check(
            f"{v.name} curves toward rear (-Y)",
            stem_back is not None and stem_back < FRAME_Y,
            details=f"stem_min_y={stem_back}, frame_y={FRAME_Y}",
        )

    # (6) closed pose: folded arm sits near the front of the frame (no fly-away)
    rj = object_model.get_articulation("fold_right")
    with ctx.pose({rj: math.radians(92.0)}):
        # arm should overlap the front frame footprint when folded across it
        ctx.expect_overlap(right, front, axes="z", min_overlap=0.005,
                           name="folded right arm stays near frame in Z")

    return ctx.report()


def _mat_name(mat):
    return getattr(mat, "name", mat) if mat is not None else None


def _mat_alpha(mat):
    rgba = getattr(mat, "rgba", None)
    if rgba and len(rgba) == 4:
        return rgba[3]
    return 1.0


object_model = build_object_model()
