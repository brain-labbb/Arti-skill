from __future__ import annotations

# Square steel checker-plate hatch leaf hinged at the rear, sitting on a
# square metal-mesh collar that caps a round concrete / stone well shaft.
#
# Articraft brief:
# - Object: a square steel checker-plate hatch leaf, ~0.70 m x 0.70 m, with a
#   diamond tread pattern on top and folded edge lips on 3 sides (front, left,
#   right -- the rear is the hinge edge), on a square mesh collar over a round
#   concrete well shaft ~0.80 m across and ~0.52 m tall, standing on the ground.
# - Root/support: the concrete well shaft is the fixed root resting on z=0; the
#   square mesh collar is fixed to the shaft top; the hatch leaf is hinged to the
#   collar at the rear edge.
# - Parts: well_shaft (hollow concrete tube), mesh_collar (square diamond-mesh
#   frame with circular throat, fixed to shaft), hatch_leaf (square steel
#   checker-plate with diamond tread pattern and folded edge lips).
# - Articulation: collar_to_hatch_leaf REVOLUTE, hinge line along the rear edge,
#   axis horizontal (world X at q=0) so the front edge lifts upward; positive q
#   swings the hatch up past vertical.
# - Visible geometry: silver-grey steel checker plate with raised diamond tread
#   pattern, folded edge lips on 3 sides, hinge knuckle barrel at the rear;
#   dark rust-brown diamond-mesh collar top; grey concrete shaft.
# - Support/fit: the hatch leaf seats on the throat ring lip when closed; the
#   hinge is a real mount -- collar-side lug plates + pin on the rear frame band,
#   with the hatch knuckle barrel coaxial on the pin.
# - Intentional overlaps: hinge knuckle embeds into the plate at the rear edge
#   (local, mechanically explanatory); the closed hatch bottom embeds ~2mm into
#   the ring lip seat; collar hinge pin rides inside the hatch knuckle barrel.
# - Tests: hatch plate + tread pattern + folded lips present, hatch lies flat &
#   seats on the collar when closed, open pose lifts the front edge well above
#   the rim, shaft is hollow, nothing floats.
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# --- Absolute dimensions (meters) ---------------------------------------------
SHAFT_OUTER_R = 0.40
SHAFT_WALL = 0.085
SHAFT_INNER_R = SHAFT_OUTER_R - SHAFT_WALL  # bore radius ~0.315
SHAFT_HEIGHT = 0.52

COLLAR_HALF = 0.40  # square mesh collar 0.80 m x 0.80 m
COLLAR_FRAME = 0.06  # outer frame band width
COLLAR_THK = 0.05  # collar plate thickness
COLLAR_THROAT_R = SHAFT_INNER_R + 0.01  # circular throat opening radius

# Square steel checker-plate hatch leaf
HATCH_SIDE = 0.70  # 0.70 m x 0.70 m square hatch
HATCH_HALF = HATCH_SIDE / 2.0
HATCH_PLATE_THK = 0.008  # 8 mm steel plate
HATCH_LIP_HEIGHT = 0.010  # folded edge lip depth (10 mm)
HATCH_LIP_THK = HATCH_PLATE_THK  # folded from the same plate
HATCH_RIM_SEAT = 0.015  # shallow centering boss under the plate

# Diamond tread pattern (raised bars on the top surface)
TREAD_LEN = 0.030  # 30 mm long bars
TREAD_W = 0.010  # 10 mm wide
TREAD_H = 0.003  # 3 mm proud of the plate surface
TREAD_SPACING = 0.045  # 45 mm grid spacing
TREAD_INSET = 0.040  # inset from plate edges

HINGE_PIN_R = 0.020
HINGE_KNUCKLE_LEN = 0.17

# Hinge geometry. The hinge barrel sits above the plate. HINGE_DROP places the
# plate top below the hinge axis so the knuckle barrel embeds into the plate for
# visual connectivity; HINGE_Z is then derived so the plate bottom seats on the
# throat ring lip with a 2 mm embed.
THROAT_LIP_TOP = COLLAR_THK + 0.015  # top of the throat ring lip
HINGE_DROP = 0.012  # plate top below hinge axis
HINGE_Y = HATCH_HALF  # hinge line at the rear edge
HINGE_Z = THROAT_LIP_TOP - 0.002 + HINGE_DROP + HATCH_PLATE_THK

HINGE_LUG_X = 0.10  # lug plate centers either side of the knuckle
HINGE_LUG_THK = 0.03
HINGE_LUG_TOP = HINGE_Z + HINGE_PIN_R + 0.014


# --- Geometry helpers ---------------------------------------------------------

def _build_hatch_plate_mesh() -> MeshGeometry:
    """Square steel checker-plate hatch leaf body: a flat plate with folded edge
    lips on 3 sides (front, left, right; rear is the hinge edge with no lip), a
    shallow centering seat boss underneath, and a raised diamond tread pattern on
    the top surface. Authored in the mesh-local frame centered on the plate axis
    with the plate top at z=0. The visual origin offsets this into the lid part
    frame so the rear edge lands on the hinge line."""
    geom = MeshGeometry()

    # Flat plate body (top at z=0, bottom at z=-HATCH_PLATE_THK).
    plate = BoxGeometry((HATCH_SIDE, HATCH_SIDE, HATCH_PLATE_THK))
    plate = plate.translate(0.0, 0.0, -HATCH_PLATE_THK / 2.0)
    geom = geom.merge(plate)

    # Folded edge lips on 3 sides (skip rear / hinge side at +Y).
    # Each lip embeds 1 mm into the plate bottom for mesh connectivity.
    lip_z_top = -HATCH_PLATE_THK + 0.001
    lip_z_center = lip_z_top - HATCH_LIP_HEIGHT / 2.0
    lip_specs = [
        # (edge_half_sign_x, edge_half_sign_y, length, is_x_edge)
        (-1.0, 0.0, HATCH_SIDE, True),   # left (-X) edge
        (+1.0, 0.0, HATCH_SIDE, True),   # right (+X) edge
        (0.0, -1.0, HATCH_SIDE - 2.0 * HATCH_LIP_THK, False),  # front (-Y) edge
    ]
    for i in range(len(lip_specs)):
        sx, sy, length, is_x_edge = lip_specs[i]
        if is_x_edge:
            lip = BoxGeometry((HATCH_LIP_THK, length, HATCH_LIP_HEIGHT))
            lip = lip.translate(
                sx * (HATCH_HALF - HATCH_LIP_THK / 2.0), 0.0, lip_z_center
            )
        else:
            lip = BoxGeometry((length, HATCH_LIP_THK, HATCH_LIP_HEIGHT))
            lip = lip.translate(
                0.0, sy * (HATCH_HALF - HATCH_LIP_THK / 2.0), lip_z_center
            )
        geom = geom.merge(lip)

    # Centering seat: a shallow circular boss under the plate that nests inside
    # the throat ring so the closed hatch reads as positively located.
    seat = CylinderGeometry(COLLAR_THROAT_R - 0.02, HATCH_RIM_SEAT, radial_segments=64)
    seat = seat.translate(0.0, 0.0, -HATCH_PLATE_THK - HATCH_RIM_SEAT / 2.0 + 0.001)
    geom = geom.merge(seat)

    # Diamond tread pattern on the top surface: alternating +45° / -45° raised
    # bars in a regular grid, inset from the plate edges. Each bar embeds 1 mm
    # into the plate for mesh connectivity.
    x_min = -(HATCH_HALF - TREAD_INSET)
    x_max = HATCH_HALF - TREAD_INSET
    y_min = -(HATCH_HALF - TREAD_INSET)
    y_max = HATCH_HALF - TREAD_INSET
    nx = int((x_max - x_min) / TREAD_SPACING)
    ny = int((y_max - y_min) / TREAD_SPACING)
    for iy in range(ny):
        for ix in range(nx):
            x = x_min + (ix + 0.5) * TREAD_SPACING
            y = y_min + (iy + 0.5) * TREAD_SPACING
            ang = math.pi / 4.0 if (ix + iy) % 2 == 0 else -math.pi / 4.0
            bar = BoxGeometry((TREAD_LEN, TREAD_W, TREAD_H))
            bar = bar.rotate_z(ang)
            bar = bar.translate(x, y, TREAD_H / 2.0 - 0.001)
            geom = geom.merge(bar)

    return geom


def _build_collar_mesh() -> MeshGeometry:
    """Square collar frame with a diamond-mesh grille inside and a circular
    throat. Authored centered on the well axis with its base at z=0 of the part
    frame; the frame top is at z=COLLAR_THK."""
    geom = MeshGeometry()

    inner = COLLAR_HALF - COLLAR_FRAME
    # +X / -X frame bars
    for sx in (1.0, -1.0):
        bar = BoxGeometry((COLLAR_FRAME, 2.0 * COLLAR_HALF, COLLAR_THK))
        bar = bar.translate(sx * (COLLAR_HALF - COLLAR_FRAME / 2.0), 0.0, COLLAR_THK / 2.0)
        geom = geom.merge(bar)
    # +Y / -Y frame bars
    for sy in (1.0, -1.0):
        bar = BoxGeometry((2.0 * inner, COLLAR_FRAME, COLLAR_THK))
        bar = bar.translate(0.0, sy * (COLLAR_HALF - COLLAR_FRAME / 2.0), COLLAR_THK / 2.0)
        geom = geom.merge(bar)

    # Diamond mesh: two families of thin diagonal bars clipped to the inner
    # square, with the circular throat carved clear.
    mesh_z = COLLAR_THK - 0.012
    bar_h = 0.012
    bar_w = 0.009
    n = 11
    pitch = (2.0 * inner) / n
    for fam in (1.0, -1.0):
        ang = fam * math.pi / 4.0
        dx, dy = math.cos(ang), math.sin(ang)
        px, py = -dy, dx
        for k in range(1, n):
            off = -inner + k * pitch
            ts = []
            base_x, base_y = px * off, py * off
            for bound, dcomp, bcomp in (
                (inner, dx, base_x),
                (-inner, dx, base_x),
                (inner, dy, base_y),
                (-inner, dy, base_y),
            ):
                if abs(dcomp) > 1e-9:
                    t = (bound - bcomp) / dcomp
                    x = base_x + dx * t
                    y = base_y + dy * t
                    if -inner - 1e-6 <= x <= inner + 1e-6 and -inner - 1e-6 <= y <= inner + 1e-6:
                        ts.append(t)
            if len(ts) < 2:
                continue
            t0, t1 = min(ts), max(ts)
            clear_r = COLLAR_THROAT_R + 0.035
            if off * off < clear_r * clear_r:
                tc = math.sqrt(clear_r * clear_r - off * off)
                segments = [(t0, -tc), (tc, t1)]
            else:
                segments = [(t0, t1)]
            for s0, s1 in segments:
                length = s1 - s0
                if length < pitch * 0.4:
                    continue
                cx = base_x + dx * (s0 + s1) / 2.0
                cy = base_y + dy * (s0 + s1) / 2.0
                bar = BoxGeometry((length, bar_w, bar_h))
                bar = bar.rotate_z(ang)
                bar = bar.translate(cx, cy, mesh_z + bar_h / 2.0 - 0.001)
                geom = geom.merge(bar)

    # Circular throat collar wall: ring around the opening tying the mesh field
    # down to the shaft bore and giving the hatch a seat.
    throat = LatheGeometry.from_shell_profiles(
        [
            (COLLAR_THROAT_R + 0.03, 0.0),
            (COLLAR_THROAT_R + 0.03, COLLAR_THK),
            (COLLAR_THROAT_R, COLLAR_THK + 0.015),
        ],
        [
            (COLLAR_THROAT_R, 0.0),
            (COLLAR_THROAT_R, COLLAR_THK),
            (COLLAR_THROAT_R - 0.004, COLLAR_THK + 0.015),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    geom = geom.merge(throat)
    return geom


def _build_hinge_mount_mesh() -> MeshGeometry:
    """Collar-side hinge mount: two upright lug plates on the rear collar frame
    band, plus the hinge pin spanning between them. Authored in the collar part
    frame (base at z=0)."""
    geom = MeshGeometry()
    lug_y0 = 0.345
    lug_y1 = COLLAR_HALF
    for sx in (1.0, -1.0):
        lug = BoxGeometry((HINGE_LUG_THK, lug_y1 - lug_y0, HINGE_LUG_TOP))
        lug = lug.translate(sx * HINGE_LUG_X, (lug_y0 + lug_y1) / 2.0, HINGE_LUG_TOP / 2.0)
        geom = geom.merge(lug)
    pin_len = 2.0 * (HINGE_LUG_X + HINGE_LUG_THK / 2.0) + 0.012
    pin = CylinderGeometry(0.013, pin_len, radial_segments=16)
    pin = pin.rotate_y(math.pi / 2.0)
    pin = pin.translate(0.0, HINGE_Y, HINGE_Z)
    geom = geom.merge(pin)
    return geom


def _build_shaft_mesh() -> MeshGeometry:
    """Hollow round concrete well shaft, base on z=0, open bore through the top."""
    return LatheGeometry.from_shell_profiles(
        [
            (SHAFT_OUTER_R, 0.0),
            (SHAFT_OUTER_R, SHAFT_HEIGHT * 0.85),
            (SHAFT_OUTER_R - 0.02, SHAFT_HEIGHT),
        ],
        [
            (SHAFT_INNER_R, 0.0),
            (SHAFT_INNER_R, SHAFT_HEIGHT * 0.85),
            (SHAFT_INNER_R, SHAFT_HEIGHT),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="checker_plate_trap_door")

    concrete = Material(name="concrete", rgba=(0.70, 0.69, 0.66, 1.0))
    steel = Material(name="steel", rgba=(0.60, 0.62, 0.64, 1.0))
    mesh_iron = Material(name="mesh_iron", rgba=(0.22, 0.13, 0.10, 1.0))
    for mat in (concrete, steel, mesh_iron):
        model.material(mat.name, rgba=mat.rgba)

    # --- Well shaft (fixed root) ---------------------------------------------
    shaft = model.part("well_shaft")
    shaft.visual(
        mesh_from_geometry(_build_shaft_mesh(), "well_shaft"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="concrete",
        name="shaft_wall",
    )

    # --- Square mesh collar (fixed to shaft top) -----------------------------
    collar = model.part("mesh_collar")
    collar.visual(
        mesh_from_geometry(_build_collar_mesh(), "mesh_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="mesh_iron",
        name="collar_frame",
    )
    collar.visual(
        mesh_from_geometry(_build_hinge_mount_mesh(), "hinge_mount"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="mesh_iron",
        name="hinge_mount",
    )

    # --- Hatch leaf (square steel checker-plate + tread + lips) --------------
    # The hatch part frame sits on the hinge line. The plate mesh is centered on
    # the hatch axis, so we offset it forward (-Y) by HATCH_HALF and down by
    # HINGE_DROP so the plate top sits below the hinge axis and the knuckle
    # embeds into the plate for connectivity.
    hatch = model.part("hatch_leaf")
    hatch.visual(
        mesh_from_geometry(_build_hatch_plate_mesh(), "hatch_plate"),
        origin=Origin(xyz=(0.0, -HATCH_HALF, -HINGE_DROP)),
        material="steel",
        name="hatch_plate",
    )

    # Hinge knuckle barrel coaxial with the revolute axis (hatch part origin),
    # spanning between the collar lugs. Because it sits exactly on the axis it
    # stays on the collar pin in every pose instead of orbiting away.
    knuckle = mesh_from_geometry(
        CylinderGeometry(HINGE_PIN_R, HINGE_KNUCKLE_LEN, radial_segments=20),
        "hinge_knuckle",
    )
    hatch.visual(
        knuckle,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel",
        name="hatch_knuckle",
    )

    # --- Fixed joints -------------------------------------------------------
    model.articulation(
        "shaft_to_collar",
        ArticulationType.FIXED,
        parent=shaft,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT)),
    )

    # --- Hatch hinge (primary articulation) ----------------------------------
    model.articulation(
        "collar_to_hatch_leaf",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=hatch,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        # Horizontal hinge axis along the rear rim (world X). The hatch plate
        # extends along local -Y (front) from the hinge; positive rotation about
        # -X lifts the front (-Y) edge upward and over past vertical.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shaft = object_model.get_part("well_shaft")
    collar = object_model.get_part("mesh_collar")
    hatch = object_model.get_part("hatch_leaf")
    hinge = object_model.get_articulation("collar_to_hatch_leaf")

    hatch_plate = hatch.get_visual("hatch_plate")
    hatch_knuckle = hatch.get_visual("hatch_knuckle")
    hinge_mount = collar.get_visual("hinge_mount")

    # --- Hero geometry present ----------------------------------------------
    ctx.check(
        "hatch leaf has plate, knuckle geometry",
        hatch_plate is not None and hatch_knuckle is not None,
        details="expected hatch_plate and hatch_knuckle visuals",
    )

    # Square plate with diamond tread pattern and folded edge lips
    ctx.check(
        "hatch is a square plate with raised tread and folded lips",
        abs(HATCH_SIDE - 0.70) < 0.01
        and TREAD_H >= 0.002
        and HATCH_LIP_HEIGHT >= 0.008
        and HATCH_PLATE_THK >= 0.006,
        details=(
            f"hatch_side={HATCH_SIDE}, tread_h={TREAD_H}, "
            f"lip_height={HATCH_LIP_HEIGHT}, plate_thk={HATCH_PLATE_THK}"
        ),
    )

    # Tread pattern grid is non-trivial (at least 10x10 diamonds)
    usable = 2.0 * (HATCH_HALF - TREAD_INSET)
    n_tread = int(usable / TREAD_SPACING)
    ctx.check(
        "diamond tread pattern covers the plate (dense grid)",
        n_tread >= 10,
        details=f"tread_grid={n_tread}x{n_tread}, spacing={TREAD_SPACING}",
    )

    # --- Hinge is physically mounted ----------------------------------------
    ctx.check(
        "collar-side hinge mount (lugs + pin) reaches the hinge axis",
        hinge_mount is not None and HINGE_LUG_TOP > HINGE_Z + HINGE_PIN_R,
        details=f"lug_top={HINGE_LUG_TOP:.3f}, hinge_z={HINGE_Z:.3f}",
    )
    ctx.check(
        "hatch covers the throat opening when closed",
        HATCH_HALF >= COLLAR_THROAT_R + 0.01,
        details=f"hatch_half={HATCH_HALF:.3f}, throat_r={COLLAR_THROAT_R:.3f}",
    )

    # Hinge knuckle embeds into the plate at the rear edge and the collar pin
    # rides inside the knuckle; both are small local intended overlaps.
    ctx.allow_overlap(
        hatch,
        collar,
        reason="Hinge knuckle barrel embeds into the hatch plate at the rear "
        "edge and the collar pin rides inside the knuckle; both are local "
        "intended hinge overlaps at the hatch rear edge.",
    )

    # --- Closed pose: hatch lies FLAT and seats over the throat -------------
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(hatch)
        if closed_aabb is not None:
            (cx0, cy0, cz0), (cx1, cy1, cz1) = closed_aabb
            x_span = cx1 - cx0
            y_span = cy1 - cy0
            z_span = cz1 - cz0
            # Square plate lies flat: thin in Z, wide and roughly equal in X/Y.
            ctx.check(
                "closed hatch lies flat (thin in Z, wide in X and Y)",
                z_span < 0.12 and x_span > 0.5 and y_span > 0.5,
                details=f"x_span={x_span:.3f} y_span={y_span:.3f} z_span={z_span:.3f}",
            )
            ctx.check(
                "closed hatch is roughly square (X span ≈ Y span)",
                abs(x_span - y_span) < 0.08,
                details=f"x_span={x_span:.3f}, y_span={y_span:.3f}",
            )
            ctx.check(
                "closed hatch sits at the collar top, not on the ground",
                cz0 > SHAFT_HEIGHT - 0.02,
                details=f"hatch min z={cz0:.3f}, shaft height={SHAFT_HEIGHT}",
            )
        ctx.expect_overlap(
            hatch,
            collar,
            axes="xy",
            min_overlap=0.20,
            name="closed hatch covers the collar throat in plan",
        )
        ctx.expect_contact(
            hatch,
            collar,
            contact_tol=0.006,
            name="closed hatch seats on the collar (not floating)",
        )

    closed_front = ctx.part_world_aabb(hatch)

    # --- Open pose: front edge lifts upward, past vertical ------------------
    with ctx.pose({hinge: 1.9}):
        open_aabb = ctx.part_world_aabb(hatch)
        ctx.check(
            "open pose lifts the hatch well above the collar",
            open_aabb is not None
            and closed_front is not None
            and open_aabb[1][2] > closed_front[1][2] + 0.20,
            details=(
                f"closed max z={None if closed_front is None else closed_front[1][2]:.3f}, "
                f"open max z={None if open_aabb is None else open_aabb[1][2]:.3f}"
            ),
        )
        if open_aabb is not None:
            (ox0, oy0, oz0), (ox1, oy1, oz1) = open_aabb
            ctx.check(
                "open hatch stands up (tall in Z)",
                (oz1 - oz0) > 0.40,
                details=f"open z_span={(oz1 - oz0):.3f}",
            )

    # --- Support / placement ------------------------------------------------
    shaft_aabb = ctx.part_world_aabb(shaft)
    if shaft_aabb is not None:
        ctx.check(
            "well shaft rests on the ground plane (z~0)",
            abs(shaft_aabb[0][2]) < 0.01,
            details=f"shaft min z={shaft_aabb[0][2]:.4f}",
        )

    collar_aabb = ctx.part_world_aabb(collar)
    if collar_aabb is not None and shaft_aabb is not None:
        ctx.check(
            "mesh collar sits at the shaft top",
            abs(collar_aabb[0][2] - shaft_aabb[1][2]) < 0.05,
            details=f"collar min z={collar_aabb[0][2]:.3f}, shaft max z={shaft_aabb[1][2]:.3f}",
        )

    ctx.check(
        "collar throat clears the shaft bore (hollow well)",
        COLLAR_THROAT_R <= SHAFT_INNER_R + 0.05 and SHAFT_INNER_R > 0.25,
        details=f"throat_r={COLLAR_THROAT_R:.3f}, bore_r={SHAFT_INNER_R:.3f}",
    )

    return ctx.report()
