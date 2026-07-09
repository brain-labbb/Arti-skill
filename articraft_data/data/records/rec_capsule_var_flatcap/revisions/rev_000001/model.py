from __future__ import annotations

# Two-piece hard gelatin capsule pill — flat-cap caplet variant.
# Oblong caplet profile: squared-off ends with light chamfer (no domes).
# Real mechanism: the slightly larger red CAP telescopes over the white BODY
# along the capsule's long axis. Pulling the cap apart along that axis splits
# the two gelatin shells -> PRISMATIC joint.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters). Roughly a pharmaceutical "size 0" capsule.
# ---------------------------------------------------------------------------
BODY_OUTER_R = 0.00370          # white body outer radius (~7.4 mm dia)
CAP_OUTER_R = 0.00400           # red cap outer radius (must clear body wall)
WALL = 0.00030                  # gelatin shell wall thickness
CAP_INNER_R = CAP_OUTER_R - WALL  # cap inner radius; > BODY_OUTER_R so it slips on

BODY_TUBE_LEN = 0.0118          # white straight tube section (the long open half)
CAP_TUBE_LEN = 0.0058           # red straight tube section (the short closed half)

# Overlap of the cap mouth over the body when fully seated (the telescoped band).
SEAT_OVERLAP = 0.0030

# Travel: how far the cap is pulled off before the shells separate.
SEPARATION_TRAVEL = 0.0090

# Rest pose: the capsule lies on the ground plane like a pill on a table.
# Its long axis runs along X; lift the whole assembly by the largest radius so
# the lowest surface point (the red cap's belly) sits exactly at z = 0.
GROUND_LIFT = CAP_OUTER_R


def _shell_half(outer_r: float, tube_len: float, *, name: str) -> object:
    """Build one hollow capsule half: an open-mouth cylindrical tube closed by
    a flat end with a light chamfer (oblong caplet profile). Axis is +X.

    Local frame: the OPEN MOUTH is at x=0, the body extends toward +x, and the
    flat chamfered end is at x = tube_len.
    """
    inner_r = outer_r - WALL
    # Small edge chamfer at the closed end — visible as the squared-off but
    # lightly beveled caplet tip. Kept smaller than the wall so it never
    # breaches the thin side wall near the end cap.
    chamfer = WALL * 0.55

    # Outer solid: flat-ended cylinder along X.
    outer = (
        cq.Workplane("YZ")
        .circle(outer_r)
        .extrude(tube_len)
    )
    # Chamfer only the outer circular edge at the closed (far) end.
    outer = outer.faces(">X").edges().chamfer(chamfer)

    # Inner cavity: open at mouth, stops WALL short of the closed end so a
    # solid end wall remains. Extend past the mouth (toward -X) for a clean
    # open rim at x=0.
    inner = (
        cq.Workplane("YZ")
        .circle(inner_r)
        .extrude(tube_len)
        .translate((-WALL, 0.0, 0.0))
    )

    shell = outer.cut(inner)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="capsule_pill")

    gel_white = model.material("gelatin_white", rgba=(0.95, 0.95, 0.94, 1.0))
    gel_red = model.material("gelatin_red", rgba=(0.86, 0.10, 0.10, 1.0))
    imprint = model.material("imprint_ink", rgba=(0.78, 0.05, 0.05, 1.0))

    # -- WHITE BODY (root) ---------------------------------------------------
    # Frame: mouth at x=0 opening toward -X, tube + flat end extend to +X.
    body = model.part("capsule_body")
    body_shell = _shell_half(BODY_OUTER_R, BODY_TUBE_LEN, name="body")
    body.visual(
        mesh_from_cadquery(body_shell, "capsule_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, GROUND_LIFT)),
        material=gel_white,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_TUBE_LEN, 2 * BODY_OUTER_R, 2 * BODY_OUTER_R)),
        mass=0.0004,
        origin=Origin(xyz=(BODY_TUBE_LEN / 2.0, 0.0, GROUND_LIFT)),
    )

    # -- RED CAP -------------------------------------------------------------
    # Authored in its own +X frame (mouth at x=0, flat end at +X), then mirrored
    # so its mouth faces +X (toward the body) and the flat chamfered end points -X.
    cap = model.part("capsule_cap")
    cap_shell = _shell_half(CAP_OUTER_R, CAP_TUBE_LEN, name="cap")
    # Mirror about YZ so the mouth opens toward +X and the flat end points -X.
    cap_shell = cap_shell.mirror("YZ")
    cap.visual(
        mesh_from_cadquery(cap_shell, "capsule_cap_shell"),
        material=gel_red,
        name="cap_shell",
    )

    # Embossed "82" imprint on the red cap's side, slightly proud of the surface.
    imprint_mesh = _imprint_82(CAP_OUTER_R)
    cap.visual(
        mesh_from_cadquery(imprint_mesh, "capsule_cap_imprint"),
        material=imprint,
        name="cap_imprint",
    )
    cap.inertial = Inertial.from_geometry(
        Box((CAP_TUBE_LEN, 2 * CAP_OUTER_R, 2 * CAP_OUTER_R)),
        mass=0.0003,
        origin=Origin(xyz=(-CAP_TUBE_LEN / 2.0, 0.0, 0.0)),
    )

    # -- TELESCOPING PRISMATIC JOINT ----------------------------------------
    # At q=0 the cap is fully seated: its mouth (x=0 in cap frame) overlaps the
    # body's open mouth by SEAT_OVERLAP. The cap frame sits SEAT_OVERLAP into
    # the body (+X side of the body mouth at x=0).
    # The joint origin carries the same GROUND_LIFT as the body geometry so the
    # cap (authored about its own local axis) rides on the lifted capsule axis.
    # Positive q pulls the cap apart toward -X.
    model.articulation(
        "body_to_cap",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(SEAT_OVERLAP, 0.0, GROUND_LIFT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=SEPARATION_TRAVEL,
        ),
    )

    return model


def _imprint_82(cap_outer_r: float) -> object:
    """Build a small embossed "82" sitting proud on the red cap side.

    Authored directly in the cap's FINAL (post-mirror) part frame: the red cap
    tube occupies negative X (mouth at x=0, flat chamfered end toward -X), so the
    imprint is centered over the tube at a negative X and placed on the +Y
    surface, matching the reference photo's marking. Returned as raised bars
    that read as the characters '8' and '2'.
    """
    # Center the imprint over the middle of the red tube, on the +Y surface.
    x_center = -CAP_TUBE_LEN * 0.5
    surf_y = cap_outer_r
    proud = 0.00012   # how far the ink stands above the shell
    t = 0.00018       # stroke thickness (along X)
    # Reach the glyph bars deep enough to bite well inside the curved wall so
    # they solidly fuse with the shell across the whole imprint footprint.
    depth = WALL + proud

    glyphs = cq.Workplane("XY")

    def bar(cx: float, cz: float, lx: float, lz: float) -> cq.Workplane:
        # A thin slab on the cap surface: extends in X (lx) and Z (lz),
        # thin in Y, pushed outward through the surface so it reads embossed.
        return (
            cq.Workplane("XZ")
            .workplane(offset=surf_y - WALL * 0.5)
            .center(cx, cz)
            .rect(lx, lz)
            .extrude(depth)
        )

    # "8": two stacked rings approximated by a rounded outline of bars.
    eight_cx = x_center - 0.0013
    ring_w = 0.0013
    # outer frame of the 8 (top loop + bottom loop share the middle bar)
    parts = [
        bar(eight_cx, 0.0010, ring_w, t),       # top
        bar(eight_cx, 0.0, ring_w, t),           # middle
        bar(eight_cx, -0.0010, ring_w, t),       # bottom
        bar(eight_cx - ring_w / 2 + t / 2, 0.0005, t, 0.0012),   # upper-left
        bar(eight_cx + ring_w / 2 - t / 2, 0.0005, t, 0.0012),   # upper-right
        bar(eight_cx - ring_w / 2 + t / 2, -0.0005, t, 0.0012),  # lower-left
        bar(eight_cx + ring_w / 2 - t / 2, -0.0005, t, 0.0012),  # lower-right
    ]

    # "2"
    two_cx = x_center + 0.0013
    two_w = 0.0013
    parts += [
        bar(two_cx, 0.0010, two_w, t),                              # top
        bar(two_cx + two_w / 2 - t / 2, 0.0005, t, 0.0012),         # upper-right
        bar(two_cx, 0.0, two_w, t),                                  # middle
        bar(two_cx - two_w / 2 + t / 2, -0.0005, t, 0.0012),       # lower-left
        bar(two_cx, -0.0010, two_w, t),                             # bottom
    ]

    # Thin backing band connecting both glyphs into a single solid island, so
    # the imprint reads as one continuous printed marking rather than loose
    # bars. It sits just below the glyph faces (mostly inside the wall) and
    # spans the full "82" footprint.
    backing = (
        cq.Workplane("XZ")
        .workplane(offset=surf_y - WALL * 0.65)
        .center(x_center, 0.0)
        .rect(0.0042, 0.0026)
        .extrude(WALL * 0.7)
    )

    glyphs = backing
    for p in parts:
        glyphs = glyphs.union(p)
    # The XZ-workplane extrude grows toward -Y; mirror to the +Y surface so the
    # marking faces the same visible side as the reference photo.
    glyphs = glyphs.mirror("XZ")
    return glyphs


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("capsule_body")
    cap = object_model.get_part("capsule_cap")
    joint = object_model.get_articulation("body_to_cap")

    # --- Mechanism: joint type, axis, and prismatic limits -----------------
    ctx.check(
        "body_to_cap is prismatic",
        joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={joint.articulation_type}",
    )
    ax = tuple(round(v, 6) for v in joint.axis)
    ctx.check(
        "cap slides along capsule long axis (X)",
        abs(ax[0]) == 1.0 and ax[1] == 0.0 and ax[2] == 0.0,
        details=f"axis={ax}",
    )
    lim = joint.motion_limits
    ctx.check(
        "separation travel limits present",
        lim is not None and lim.lower == 0.0 and lim.upper is not None and lim.upper > 0.0,
        details=f"limits=({lim.lower if lim else None},{lim.upper if lim else None})",
    )

    # --- Visual hero parts present -----------------------------------------
    ctx.check(
        "white body shell visual present",
        any(v.name == "body_shell" for v in body.visuals),
        details=str([v.name for v in body.visuals]),
    )
    ctx.check(
        "red cap shell visual present",
        any(v.name == "cap_shell" for v in cap.visuals),
        details=str([v.name for v in cap.visuals]),
    )
    ctx.check(
        "embossed 82 imprint present on cap",
        any(v.name == "cap_imprint" for v in cap.visuals),
        details=str([v.name for v in cap.visuals]),
    )

    # --- Flat-end caplet profile (no hemispherical domes) ------------------
    # The body shell should extend only ~BODY_TUBE_LEN along X, not the longer
    # BODY_TUBE_LEN + BODY_OUTER_R that a dome would produce.
    body_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    if body_aabb is not None:
        body_dx = body_aabb[1][0] - body_aabb[0][0]
        dome_threshold = BODY_TUBE_LEN + BODY_OUTER_R * 0.3
        ctx.check(
            "body shell has flat end (no dome extension)",
            body_dx < dome_threshold,
            details=f"body_dx={body_dx:.6f}, expected<{dome_threshold:.6f}",
        )
    cap_aabb = ctx.part_element_world_aabb(cap, elem="cap_shell")
    if cap_aabb is not None:
        cap_dx = cap_aabb[1][0] - cap_aabb[0][0]
        dome_threshold_cap = CAP_TUBE_LEN + CAP_OUTER_R * 0.3
        ctx.check(
            "cap shell has flat end (no dome extension)",
            cap_dx < dome_threshold_cap,
            details=f"cap_dx={cap_dx:.6f}, expected<{dome_threshold_cap:.6f}",
        )

    # --- Telescoping fit: cap nests over body at the seam (intentional) ----
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="body_shell",
        reason=(
            "The red cap intentionally telescopes over the white body's open "
            "mouth; the two gelatin shells nest at the seated seam."
        ),
    )
    # Imprint is pushed slightly through the cap wall so it reads embossed.
    ctx.allow_overlap(
        cap,
        cap,
        elem_a="cap_imprint",
        elem_b="cap_shell",
        reason="The 82 imprint is intentionally embedded into the cap surface so it reads embossed.",
    )

    # Seated pose: cap and body overlap along the slide axis (retained insertion)
    # and the cap stays concentric (centered) on the body in the YZ plane.
    with ctx.pose({joint: 0.0}):
        ctx.expect_overlap(
            cap,
            body,
            axes="x",
            elem_a="cap_shell",
            elem_b="body_shell",
            min_overlap=0.0015,
            name="seated cap overlaps body along slide axis",
        )
        ctx.expect_within(
            body,
            cap,
            axes="yz",
            inner_elem="body_shell",
            outer_elem="cap_shell",
            margin=0.0002,
            name="body stays concentric inside the cap mouth when seated",
        )
        seated_pos = ctx.part_world_position(cap)

    # Pulled-apart pose: cap moves toward -X and the shells separate.
    with ctx.pose({joint: joint.motion_limits.upper}):
        pulled_pos = ctx.part_world_position(cap)
        ctx.expect_gap(
            body,
            cap,
            axis="x",
            min_gap=0.0,
            positive_elem="body_shell",
            negative_elem="cap_shell",
            name="cap fully separates from body at max travel",
        )

    ctx.check(
        "pulling the joint slides the cap off toward -X",
        seated_pos is not None
        and pulled_pos is not None
        and pulled_pos[0] < seated_pos[0] - 0.005,
        details=f"seated={seated_pos}, pulled={pulled_pos}",
    )

    return ctx.report()


object_model = build_object_model()
