from __future__ import annotations

# Wide DOUBLE-DOOR street electrical cabinet on a stepped concrete pedestal.
#
# Reference: a roughly cubic roadside cabinet, slightly wider than tall,
# weathered grey steel covered in stickers and graffiti, standing on a stepped
# poured-concrete base (a wide lower step plus a narrower upper step). The front
# face splits down the middle into two symmetric service doors that meet at a
# central vertical seam; each door is hinged on its OUTER vertical edge and
# swings outward. A louvered vent sits on each door; recessed handles flank the
# central seam.
#
# Double-door construction (per brief): ONE parametric door is authored and
# instantiated TWICE, mirrored across X. There is NO 180-degree yaw flip: the
# left door's geometry is the X-mirror of the right door, so both decorated
# faces stay forward (-Y) and each door hinges on its own OUTER edge.
#
# Coordinate convention (Z-up world):
#   +X : cabinet WIDTH  (~0.84 m)
#   +Y : cabinet DEPTH  (~0.55 m); front face at -Y (door side)
#   +Z : HEIGHT; concrete base at z=0, cabinet top ~0.97 m
#
# Parts / articulations:
#   - base (ROOT)    : stepped concrete pedestal, base at z=0
#   - body           : hollow grey-steel shell, open at the front
#   - door_0         : right-hand door, REVOLUTE about its +X outer edge
#   - door_1         : left-hand  door, REVOLUTE about its -X outer edge
#     (door_1 mimics door_0 so they open together as a paired mechanism)

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    Inertial,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)

# ---- overall dimensions (metres) ----
W = 0.84          # width (X)
D = 0.55          # depth (Y)
BODY_H = 0.72     # cabinet shell height above base
WALL = 0.012

# stepped concrete base: wide lower step + narrower upper step
BASE_LO_H = 0.10
BASE_HI_H = 0.12
BASE_H = BASE_LO_H + BASE_HI_H
BASE_LO_OVER = 0.12   # lower step overhang per side beyond body
BASE_HI_OVER = 0.05   # upper step overhang per side beyond body

DOOR_GAP = 0.006        # reveal around each door
SEAM_GAP = 0.005        # gap at the central meeting seam
DOOR_T = 0.018
# each door spans half the front width minus reveals and half the seam gap
DOOR_W = (W - 2 * DOOR_GAP - SEAM_GAP) / 2.0
DOOR_H = BODY_H - 2 * DOOR_GAP

FRONT_Y = -D / 2.0

# Outer hinge edge of the right (+X) door, in body frame X.
OUTER_X = W / 2.0 - DOOR_GAP

# ---- per-door louver-row stack ----
LOUVER_N = 8              # number of horizontal louver rows per door
LOUVER_ROW_W = 0.20       # width of each louver row (X)
LOUVER_ROW_H = 0.030      # height of each louver row (Z in door face)
LOUVER_PITCH = 0.044      # vertical center-to-center spacing between rows


def _hollow_box(outer, wall, *, open_face):
    sx, sy, sz = outer
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    pieces = []
    pieces.append((Box((sx, wall, sz)), Origin(xyz=(0.0, hy - wall / 2.0, 0.0))))  # back
    pieces.append((Box((wall, sy, sz)), Origin(xyz=(hx - wall / 2.0, 0.0, 0.0))))  # +X
    pieces.append((Box((wall, sy, sz)), Origin(xyz=(-hx + wall / 2.0, 0.0, 0.0))))  # -X
    pieces.append((Box((sx, sy, wall)), Origin(xyz=(0.0, 0.0, hz - wall / 2.0))))  # top
    pieces.append((Box((sx, sy, wall)), Origin(xyz=(0.0, 0.0, -hz + wall / 2.0))))  # bottom
    if open_face != "-y":
        pieces.append((Box((sx, sy, wall)), Origin(xyz=(0.0, -hy + wall / 2.0, 0.0))))
    return pieces


def _louver_row_geometry():
    """Shared geometry for one horizontal louver row panel.

    Returns a VentGrilleGeometry sized for a single thin horizontal louver band
    with angled flat slats and no rear sleeve (flush-mounted on the door face).
    """
    return VentGrilleGeometry(
        (LOUVER_ROW_W, LOUVER_ROW_H),
        frame=0.003,
        face_thickness=0.003,
        duct_depth=0.014,
        duct_wall=0.002,
        slat_pitch=0.012,
        slat_width=0.006,
        slat_angle_deg=35.0,
        corner_radius=0.002,
        slats=VentGrilleSlats(profile="flat", direction="down"),
        sleeve=VentGrilleSleeve(style="none"),
    )


def _build_door(model, name, *, mirror, materials):
    """Author ONE parametric door in a LOCAL frame, then return the Part.

    Local frame: door centred on local origin. Outer (hinge) edge is at local
    +X = +DOOR_W/2; inner (seam/handle) edge at local -X = -DOOR_W/2. Decorated
    face toward local -Y. For the mirrored (left) door we negate local X on every
    feature so the same authored shape becomes the left door WITHOUT a yaw flip,
    keeping the decorated face forward (-Y). The hinge edge for the mirrored door
    then sits at local -X (its outer edge), handled by the caller's articulation.
    """
    steel_door, dark_metal, steel = materials
    s = -1.0 if mirror else 1.0  # X-mirror sign
    front_face = -DOOR_T / 2.0

    # Author so the LOCAL ORIGIN is the OUTER hinge edge. For the unmirrored
    # (right) door the hinge edge is local x=0 and the skin extends toward -X
    # (skin centre at -DOOR_W/2, inner/seam handle edge near -DOOR_W). The mirror
    # sign s flips every X so the left door hinges on its own outer edge while its
    # decorated face stays forward (-Y) -- no 180-degree yaw flip.
    skin_cx = s * (-DOOR_W / 2.0)
    door = model.part(name)
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(skin_cx, 0.0, 0.0)),
        material=steel_door,
        name=f"{name}_skin",
    )
    # tall stack of N horizontal louver rows on the door face
    # Place the stack starting just below door top, descending with regular pitch.
    louver_geom = _louver_row_geometry()
    louver_stack_top = DOOR_H / 2.0 - 0.06  # top row center is 6cm below door top
    for i in range(LOUVER_N):
        row_z = louver_stack_top - i * LOUVER_PITCH
        door.visual(
            mesh_from_geometry(louver_geom, f"{name}_louver_{i}"),
            origin=Origin(xyz=(skin_cx, front_face - 0.001, row_z),
                          rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"{name}_louver_{i}",
        )
    # recessed handle on the INNER (seam) edge, near the free edge at local -DOOR_W
    handle_x = s * (-DOOR_W + 0.05)
    door.visual(
        Box((0.05, 0.006, 0.13)),
        origin=Origin(xyz=(handle_x, front_face - 0.003, 0.0)),
        material=dark_metal,
        name=f"{name}_handle_pocket",
    )
    door.visual(
        Box((0.013, 0.040, 0.06)),
        origin=Origin(xyz=(handle_x, front_face - 0.020, 0.0)),
        material=steel,
        name=f"{name}_handle_lever",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)), mass=7.0,
        origin=Origin(xyz=(skin_cx, 0.0, 0.0)),
    )
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_door_cabinet")

    steel = model.material("grey_steel", rgba=(0.64, 0.65, 0.67, 1.0))
    steel_door = model.material("door_steel", rgba=(0.71, 0.72, 0.74, 1.0))
    concrete = model.material("concrete", rgba=(0.56, 0.55, 0.52, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.35, 0.37, 1.0))

    # ---- base (ROOT) : stepped concrete pedestal, base at z=0 ----
    base = model.part("base")
    lo_w = W + 2 * BASE_LO_OVER
    lo_d = D + 2 * BASE_LO_OVER
    hi_w = W + 2 * BASE_HI_OVER
    hi_d = D + 2 * BASE_HI_OVER
    base.visual(
        Box((lo_w, lo_d, BASE_LO_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LO_H / 2.0)),
        material=concrete,
        name="base_lower_step",
    )
    base.visual(
        Box((hi_w, hi_d, BASE_HI_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LO_H + BASE_HI_H / 2.0)),
        material=concrete,
        name="base_upper_step",
    )
    base.inertial = Inertial.from_geometry(
        Box((lo_w, lo_d, BASE_H)), mass=80.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
    )

    # ---- body : hollow grey-steel shell, open at the front ----
    body = model.part("body")
    body_cz = BASE_H + BODY_H / 2.0
    for i, (geom, org) in enumerate(_hollow_box((W, D, BODY_H), WALL, open_face="-y")):
        body.visual(
            geom,
            origin=Origin(xyz=(org.xyz[0], org.xyz[1], org.xyz[2] + body_cz)),
            material=steel,
            name=f"shell_{i}",
        )
    # central mullion behind the seam so the two doors close onto a real jamb.
    # It spans slightly past full inner height so it merges with the top and
    # bottom shell walls (no floating island).
    body.visual(
        Box((0.03, WALL * 1.5, BODY_H)),
        origin=Origin(xyz=(0.0, FRONT_Y + WALL, body_cz)),
        material=steel,
        name="center_mullion",
    )
    # top drip cap
    body.visual(
        Box((W + 0.04, D + 0.04, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + BODY_H + 0.008)),
        material=steel_door,
        name="drip_cap",
    )
    # exposed barrel hinges on BOTH outer jambs (two per side)
    for side in (+1.0, -1.0):
        for j, hz in enumerate((BASE_H + 0.15, BASE_H + BODY_H - 0.15)):
            hinge = BarrelHingeGeometry(
                0.085,
                leaf_width_a=0.018,
                leaf_width_b=0.018,
                leaf_thickness=0.0024,
                pin_diameter=0.006,
                knuckle_count=5,
            )
            body.visual(
                mesh_from_geometry(hinge, f"hinge_{int(side)}_{j}"),
                origin=Origin(xyz=(side * (OUTER_X - 0.004), FRONT_Y + 0.013, hz)),
                material=dark_metal,
                name=f"hinge_{'r' if side > 0 else 'l'}_{j}",
            )
    body.inertial = Inertial.from_geometry(
        Box((W, D, BODY_H)), mass=46.0,
        origin=Origin(xyz=(0.0, 0.0, body_cz)),
    )

    # ---- two doors from the SAME parametric builder, mirrored in X ----
    materials = (steel_door, dark_metal, steel)
    door_r = _build_door(model, "door_0", mirror=False, materials=materials)
    door_l = _build_door(model, "door_1", mirror=True, materials=materials)

    door_face_y = FRONT_Y + DOOR_T / 2.0 + DOOR_GAP
    door_cz = BASE_H + BODY_H / 2.0

    # Right door (door_0): hinge at its +X outer edge. Its body extends from the
    # hinge toward -X (toward the seam). Positive q about +Z swings the free
    # (inner) edge forward (-Y) -> opens outward.
    model.articulation(
        "body_to_door_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door_r,
        origin=Origin(xyz=(OUTER_X, door_face_y, door_cz)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=math.radians(115.0), effort=22.0, velocity=2.0
        ),
    )
    # Left door (door_1): its authored geometry is X-mirrored, so its outer edge
    # is at local -X. Hinge at the -X outer edge; the door body extends toward
    # +X (toward the seam). Negative-Z axis makes positive q swing the inner edge
    # forward (-Y) symmetrically. It mimics door_0 so they open together.
    model.articulation(
        "body_to_door_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door_l,
        origin=Origin(xyz=(-OUTER_X, door_face_y, door_cz)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=math.radians(115.0), effort=22.0, velocity=2.0
        ),
        mimic=Mimic(joint="body_to_door_0", multiplier=1.0, offset=0.0),
    )

    model.articulation(
        "base_to_body",
        ArticulationType.FIXED,
        parent=base,
        child=body,
        origin=Origin(),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    door_r = object_model.get_part("door_0")
    door_l = object_model.get_part("door_1")
    j_r = object_model.get_articulation("body_to_door_0")
    j_l = object_model.get_articulation("body_to_door_1")

    # ---- concrete base at z=0 ----
    bmn, _ = ctx.part_world_aabb(base)
    ctx.check("base at z=0", abs(bmn[2]) < 1e-3, details=f"base min z={bmn[2]:.4f}")

    # ---- both door joints are REVOLUTE about a vertical (+Z) axis ----
    for joint, nm in ((j_r, "door_0"), (j_l, "door_1")):
        ax = joint.axis
        ctx.check(
            f"{nm} hinge axis vertical (Z)",
            abs(ax[2]) > 0.99 and abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6,
            details=f"axis={ax}",
        )
        ctx.check(
            f"{nm} joint is revolute",
            str(joint.articulation_type).upper().endswith("REVOLUTE"),
            details=f"type={joint.articulation_type}",
        )

    # ---- left/right symmetry: the two closed doors mirror across X ----
    r_aabb = ctx.part_world_aabb(door_r)
    l_aabb = ctx.part_world_aabb(door_l)
    r_cx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
    l_cx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
    ctx.check(
        "closed doors are mirror-symmetric about X=0",
        abs(r_cx + l_cx) < 0.02 and r_cx > 0.05,
        details=f"right_cx={r_cx:.3f}, left_cx={l_cx:.3f}",
    )
    # both doors sit on the FRONT (-Y) of the cabinet (decoration forward)
    ctx.check(
        "both doors face forward (-Y)",
        r_aabb[0][1] < FRONT_Y + 0.05 and l_aabb[0][1] < FRONT_Y + 0.05,
        details=f"right_min_y={r_aabb[0][1]:.3f}, left_min_y={l_aabb[0][1]:.3f}",
    )

    # ---- both doors swing forward (-Y) when opened (driven by door_0) ----
    r_closed_y = ctx.part_world_aabb(door_r)[0][1]
    l_closed_y = ctx.part_world_aabb(door_l)[0][1]
    with ctx.pose({j_r: math.radians(90.0)}):  # door_1 follows via mimic
        r_open_y = ctx.part_world_aabb(door_r)[0][1]
        l_open_y = ctx.part_world_aabb(door_l)[0][1]
    ctx.check(
        "right door swings forward when opened",
        r_open_y < r_closed_y - 0.10,
        details=f"closed={r_closed_y:.3f}, open={r_open_y:.3f}",
    )
    ctx.check(
        "left door follows (mimic) and swings forward",
        l_open_y < l_closed_y - 0.10,
        details=f"closed={l_closed_y:.3f}, open={l_open_y:.3f}",
    )

    # ---- each door carries N horizontal louver rows ----
    for d, dname in ((door_r, "door_0"), (door_l, "door_1")):
        for i in range(LOUVER_N):
            ctx.check(
                f"{dname} louver row {i} present",
                d.get_visual(f"{dname}_louver_{i}") is not None,
                details=f"{dname}_louver_{i} missing",
            )

    # ---- louver rows have regular vertical pitch ----
    for dname in ("door_0", "door_1"):
        zs = []
        for i in range(LOUVER_N):
            v = object_model.get_part(dname).get_visual(f"{dname}_louver_{i}")
            zs.append(v.origin.xyz[2])
        pitches = [zs[i] - zs[i + 1] for i in range(LOUVER_N - 1)]
        ok = all(abs(p - LOUVER_PITCH) < 1e-6 for p in pitches)
        ctx.check(
            f"{dname} louver rows have uniform pitch={LOUVER_PITCH:.3f}m",
            ok,
            details=f"pitches={[f'{p:.4f}' for p in pitches]}",
        )
        # rows descend from top (row 0 is highest)
        ctx.check(
            f"{dname} louver rows descend from top",
            zs[0] > zs[-1],
            details=f"row0_z={zs[0]:.4f}, row{LOUVER_N-1}_z={zs[-1]:.4f}",
        )

    # ---- closed doors nest in the opening / meet the central mullion ----
    ctx.allow_overlap(
        door_r, body,
        reason="Closed right door reveal nests in the opening; hinge knuckles meet the +X jamb.",
    )
    ctx.allow_overlap(
        door_l, body,
        reason="Closed left door reveal nests in the opening; hinge knuckles meet the -X jamb.",
    )

    return ctx.report()


object_model = build_object_model()
