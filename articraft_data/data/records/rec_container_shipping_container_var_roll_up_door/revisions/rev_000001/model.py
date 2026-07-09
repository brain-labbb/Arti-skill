from __future__ import annotations

# White 20-foot intermodal shipping container with roll-up overhead door.
# Frame: container long axis along +X (cargo-door end at +X), width along Y,
# height along Z. Container interior floor at z=0; body spans z in [0, H].
# Corrugated steel walls (vertical corrugations on the two long side walls and
# on the front end wall), 8 corner castings, a steel floor, and at the +X end
# a single roll-up overhead door: a segmented rolling curtain (horizontal slats
# linked top to bottom) that rises vertically along doorway tracks.
#
# Articulations:
#   - body_to_curtain: PRISMATIC along +Z, 0..2.30 m

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- container outer dimensions (meters, realistic ISO 20ft) ----
L = 6.06   # length along X
W = 2.44   # width along Y
H = 2.59   # height along Z

WALL_T = 0.035   # nominal wall/skin thickness
CORNER = 0.16    # corner casting cube size

# Body occupies x in [BODY_X0, BODY_X1]; the doorway is at +X.
DOOR_T = 0.06    # nominal door-plane depth (used for body layout)
BODY_X1 = L / 2.0 - DOOR_T   # +X face of the body opening
BODY_X0 = -L / 2.0

# Doorway frame X-center (posts, sill, header all centered here)
FRAME_X = BODY_X1 - 0.05   # = 2.92

CORR_PITCH = 0.30   # corrugation pitch
CORR_DEPTH = 0.04   # corrugation rib depth
CORR_WIDTH = 0.12   # corrugation rib width

# ---- roll-up curtain dimensions ----
SLAT_H = 0.10       # slat height (Z)
SLAT_T = 0.025      # slat thickness (X)
SLAT_SPACING = 0.099  # center-to-center (slight overlap for mesh connectivity)
N_SLATS = 22        # number of horizontal slats (fills opening minus tuck zone)
CURTAIN_W = 2.08    # curtain width (Y) — fits between track guides with clearance
BOTTOM_BAR_H = 0.06 # bottom pull-bar height
BOTTOM_BAR_T = 0.05 # bottom pull-bar thickness

# Doorway frame reference heights
SILL_H = 0.14       # sill height
HEADER_H = 0.14     # header height
SILL_TOP = SILL_H   # sill top surface z
HEADER_BOT = H - HEADER_H  # header bottom surface z
OPENING_H = HEADER_BOT - SILL_TOP  # ~2.31 m

# Track guide dimensions
TRACK_XW = 0.04     # track width along X
TRACK_YW = 0.03     # track rail depth along Y
TRACK_H = OPENING_H # track spans the full opening height

# Post geometry
POST_DEPTH = 0.14   # post depth along Y
POST_INNER_Y = W / 2.0 - POST_DEPTH   # = 1.08

# Drum dimensions
DRUM_R = 0.08
DRUM_LEN = CURTAIN_W - 0.06  # shorter than curtain so it clears the posts

# Prismatic joint travel
CURTAIN_TRAVEL = 2.30  # metres of upward travel to open

# Joint origin: curtain part origin at the bottom of the bottom bar.
# Bottom bar local z extent: [0, BOTTOM_BAR_H] = [0, 0.06].
# World bottom bar bottom = JOINT_Z. Set to sill top for contact at rest.
JOINT_Z = SILL_TOP   # = 0.14


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _corrugated_wall(length_x: float, height_z: float, base_y: float, depth_sign: float) -> MeshGeometry:
    """Vertical-corrugation side wall in the XZ plane at y=base_y."""
    geo = BoxGeometry((length_x, WALL_T, height_z))
    geo.translate(0.0, base_y, 0.0)
    n = max(1, int(length_x / CORR_PITCH))
    x0 = -length_x / 2.0 + CORR_PITCH / 2.0
    for i in range(n):
        x = x0 + i * CORR_PITCH
        rib = BoxGeometry((CORR_WIDTH, CORR_DEPTH, height_z * 0.96))
        rib.translate(x, base_y + depth_sign * (WALL_T / 2.0 + CORR_DEPTH / 2.0), 0.0)
        geo.merge(rib)
    return geo


def _corrugated_end_wall(width_y: float, height_z: float, base_x: float, depth_sign: float) -> MeshGeometry:
    """Front (-X) end wall in the YZ plane at x=base_x with vertical ribs along Y."""
    geo = BoxGeometry((WALL_T, width_y, height_z))
    geo.translate(base_x, 0.0, 0.0)
    n = max(1, int(width_y / CORR_PITCH))
    y0 = -width_y / 2.0 + CORR_PITCH / 2.0
    for i in range(n):
        y = y0 + i * CORR_PITCH
        rib = BoxGeometry((CORR_DEPTH, CORR_WIDTH, height_z * 0.96))
        rib.translate(base_x + depth_sign * (WALL_T / 2.0 + CORR_DEPTH / 2.0), y, 0.0)
        geo.merge(rib)
    return geo


def _curtain_slat(width_y: float) -> MeshGeometry:
    """One horizontal rolling-shutter slat centered at local origin."""
    return BoxGeometry((SLAT_T, width_y, SLAT_H))


def _corner_casting() -> MeshGeometry:
    return BoxGeometry((CORNER, CORNER, CORNER))


def _drum_assembly() -> MeshGeometry:
    """Drum roller with end caps as one connected mesh.

    The drum is positioned inside the container, behind the curtain plane,
    near the top of the doorway.
    """
    # Drum center: behind curtain plane (-X), just below header
    drum_cx = FRAME_X - 0.14
    drum_cz = HEADER_BOT - DRUM_R - 0.01

    # Drum cylinder (axis along Y) - the main roller
    drum = CylinderGeometry(DRUM_R, DRUM_LEN, radial_segments=24)
    drum.rotate_x(math.pi / 2.0)
    drum.translate(drum_cx, 0.0, drum_cz)

    # End caps: larger cylinders that clearly overlap the drum ends
    # These represent the bearing housings
    for sy in (-1.0, 1.0):
        cap_r = DRUM_R + 0.02  # slightly larger than drum
        cap_len = 0.06
        cap = CylinderGeometry(cap_r, cap_len, radial_segments=16)
        cap.rotate_x(math.pi / 2.0)
        # Position cap so it overlaps the drum end
        cap.translate(drum_cx, sy * (DRUM_LEN / 2.0 - cap_len / 2.0), drum_cz)
        drum.merge(cap)

    # Mounting brackets: vertical plates that overlap the end caps and extend up
    for sy in (-1.0, 1.0):
        bracket_w = 0.06
        bracket_d = 0.06
        bracket_h = 0.20
        bracket = BoxGeometry((bracket_d, bracket_w, bracket_h))
        # Position bracket to overlap the cap (at same Y) and extend up to header
        by = sy * (DRUM_LEN / 2.0 - bracket_w / 2.0)
        bz = drum_cz + 0.08  # centered above drum, ensuring overlap
        bracket.translate(drum_cx, by, bz)
        drum.merge(bracket)

    return drum


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shipping_container")

    white = model.material("white_steel", rgba=(0.90, 0.91, 0.92, 1.0))
    grey = model.material("grey_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    dark = model.material("dark_hardware", rgba=(0.18, 0.19, 0.21, 1.0))
    curtain_mat = model.material("curtain_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    bar_mat = model.material("bar_steel", rgba=(0.45, 0.47, 0.50, 1.0))

    # =================================================================
    # BODY (root): corrugated box open at the +X end, with floor, roof,
    # front end wall, two side walls, 8 corner castings, and the
    # doorway frame (header, sill, posts, track rails, drum assembly).
    # =================================================================
    body = model.part("body")

    body_len = BODY_X1 - BODY_X0
    body_cx = (BODY_X0 + BODY_X1) / 2.0

    # --- floor (steel plate) ---
    floor = BoxGeometry((body_len, W, 0.10))
    floor.translate(body_cx, 0.0, 0.05)
    body.visual(mesh_from_geometry(floor, "floor"), material=grey, name="floor")

    # --- roof ---
    roof = BoxGeometry((body_len, W, 0.05))
    roof.translate(body_cx, 0.0, H - 0.025)
    body.visual(mesh_from_geometry(roof, "roof"), material=white, name="roof")

    # --- two long corrugated side walls ---
    wall_z_center = H / 2.0
    side_h = H - 0.10
    for sy, nm in ((1.0, "side_wall_p"), (-1.0, "side_wall_n")):
        wall = _corrugated_wall(body_len, side_h, sy * (W / 2.0 - WALL_T / 2.0), sy)
        wall.translate(body_cx, 0.0, wall_z_center)
        body.visual(mesh_from_geometry(wall, nm), material=white, name=nm)

    # --- front end wall (at -X) ---
    end = _corrugated_end_wall(W - 0.10, side_h, BODY_X0 + WALL_T / 2.0, -1.0)
    end.translate(0.0, 0.0, wall_z_center)
    body.visual(mesh_from_geometry(end, "front_wall"), material=white, name="front_wall")

    # --- door header & sill rails at the +X opening ---
    header = BoxGeometry((0.10, W, HEADER_H))
    header.translate(FRAME_X, 0.0, H - HEADER_H / 2.0)
    body.visual(mesh_from_geometry(header, "door_header"), material=grey, name="door_header")
    sill = BoxGeometry((0.10, W, SILL_H))
    sill.translate(FRAME_X, 0.0, SILL_H / 2.0)
    body.visual(mesh_from_geometry(sill, "door_sill"), material=grey, name="door_sill")

    # --- vertical corner posts at the doorway ---
    for sy, nm in ((1.0, "post_p"), (-1.0, "post_n")):
        post = BoxGeometry((0.12, POST_DEPTH, H))
        post.translate(FRAME_X, sy * (W / 2.0 - POST_DEPTH / 2.0), H / 2.0)
        body.visual(mesh_from_geometry(post, nm), material=grey, name=nm)

    # --- 8 corner castings ---
    ci = 0
    for sx in (BODY_X0 + CORNER / 2.0, L / 2.0 - CORNER / 2.0):
        for sy in (-1.0, 1.0):
            for sz in (CORNER / 2.0, H - CORNER / 2.0):
                cc = _corner_casting()
                cc.translate(sx, sy * (W / 2.0 - CORNER / 2.0), sz)
                body.visual(mesh_from_geometry(cc, f"corner_{ci}"), material=dark, name=f"corner_{ci}")
                ci += 1

    # --- track guide rails (vertical C-rails on each side of doorway) ---
    for sy, nm in ((1.0, "track_p"), (-1.0, "track_n")):
        track_y = sy * (POST_INNER_Y - TRACK_YW / 2.0)
        track = BoxGeometry((TRACK_XW, TRACK_YW, TRACK_H))
        track.translate(FRAME_X, track_y, SILL_TOP + TRACK_H / 2.0)
        body.visual(mesh_from_geometry(track, nm), material=dark, name=nm)

    # --- drum assembly (roller + brackets, one connected mesh) ---
    drum_mesh = _drum_assembly()
    body.visual(mesh_from_geometry(drum_mesh, "drum"), material=dark, name="drum")

    body.inertial = Inertial.from_geometry(
        Box((L, W, H)), mass=2200.0, origin=Origin(xyz=(0.0, 0.0, H / 2.0))
    )

    # =================================================================
    # CURTAIN: roll-up overhead door — horizontal slats linked top to
    # bottom, plus a bottom pull-bar. One part, one PRISMATIC joint.
    # =================================================================
    curtain = model.part("curtain")

    # Bottom pull-bar
    bar_geo = BoxGeometry((BOTTOM_BAR_T, CURTAIN_W, BOTTOM_BAR_H))
    bar_geo.translate(0.0, 0.0, BOTTOM_BAR_H / 2.0)
    curtain.visual(mesh_from_geometry(bar_geo, "bottom_bar"), material=bar_mat, name="bottom_bar")

    # Horizontal slats generated via loop with name_i naming
    for i in range(N_SLATS):
        slat = _curtain_slat(CURTAIN_W)
        z_center = BOTTOM_BAR_H + SLAT_H / 2.0 + i * SLAT_SPACING
        slat.translate(0.0, 0.0, z_center)
        curtain.visual(mesh_from_geometry(slat, f"slat_{i}"), material=curtain_mat, name=f"slat_{i}")

    curtain_h = BOTTOM_BAR_H + (N_SLATS - 1) * SLAT_SPACING + SLAT_H
    curtain.inertial = Inertial.from_geometry(
        Box((SLAT_T, CURTAIN_W, curtain_h)),
        mass=150.0,
        origin=Origin(xyz=(0.0, 0.0, curtain_h / 2.0)),
    )

    # Prismatic joint: curtain slides up in +Z to open the doorway.
    model.articulation(
        "body_to_curtain",
        ArticulationType.PRISMATIC,
        parent=body,
        child=curtain,
        origin=Origin(xyz=(FRAME_X, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=500.0, velocity=0.5, lower=0.0, upper=CURTAIN_TRAVEL
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    curtain = object_model.get_part("curtain")
    curtain_joint = object_model.get_articulation("body_to_curtain")

    # ---- container is a long box (X >> Y, X >> Z) ----
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "container is a long box (X longest)",
        body_ext[0] > body_ext[1] + 1.0 and body_ext[0] > body_ext[2] + 1.0,
        details=f"body extents={body_ext}",
    )

    # ---- curtain is at the +X cargo end ----
    curtain_pos = ctx.part_world_position(curtain)
    ctx.check(
        "curtain is at the +X cargo end",
        curtain_pos is not None and curtain_pos[0] > L / 2.0 - 0.5,
        details=f"curtain origin={curtain_pos}",
    )

    # ---- curtain bottom bar overlaps the sill footprint in XY ----
    ctx.expect_overlap(
        curtain, body, axes="xy", min_overlap=0.04,
        elem_a="bottom_bar", elem_b="door_sill",
        name="curtain bottom bar overlaps sill footprint in XY",
    )

    # ---- curtain bottom bar contacts sill at rest ----
    ctx.expect_contact(
        curtain, body,
        elem_a="bottom_bar", elem_b="door_sill",
        contact_tol=0.002,
        name="curtain bottom bar seated on sill",
    )

    # ---- curtain fits within the posts in Y (small clearance) ----
    ctx.expect_gap(
        body, curtain, axis="y",
        positive_elem="post_p", negative_elem="slat_11",
        min_gap=0.005,
        name="curtain clears +Y post inner face",
    )
    ctx.expect_gap(
        curtain, body, axis="y",
        positive_elem="slat_11", negative_elem="post_n",
        min_gap=0.005,
        name="curtain clears -Y post inner face",
    )

    # ---- curtain fits within the track guides in Y ----
    ctx.expect_gap(
        body, curtain, axis="y",
        positive_elem="track_p", negative_elem="slat_11",
        min_gap=0.002,
        name="curtain clears +Y track guide",
    )
    ctx.expect_gap(
        curtain, body, axis="y",
        positive_elem="slat_11", negative_elem="track_n",
        min_gap=0.002,
        name="curtain clears -Y track guide",
    )

    # ---- allow overlap: top slat tucks behind header when closed ----
    ctx.allow_overlap(
        curtain, body,
        elem_a="slat_21", elem_b="door_header",
        reason="Top curtain slat tucks behind the door header when closed, as in a real roll-up door where the upper slats nest inside the header recess.",
    )
    # Proof: top slat stays within the header footprint in XY
    ctx.expect_overlap(
        curtain, body, axes="xy", min_overlap=0.01,
        elem_a="slat_21", elem_b="door_header",
        name="top slat stays within header footprint",
    )

    # ---- curtain rises vertically when opened (prismatic +Z) ----
    rest_z = ctx.part_world_position(curtain)
    with ctx.pose({curtain_joint: 1.5}):
        mid_z = ctx.part_world_position(curtain)
    ctx.check(
        "curtain rises vertically when partially opened",
        mid_z is not None and rest_z is not None and mid_z[2] > rest_z[2] + 1.0,
        details=f"rest={rest_z}, mid={mid_z}",
    )

    # ---- at full open the bottom bar clears the opening ----
    with ctx.pose({curtain_joint: CURTAIN_TRAVEL}):
        open_pos = ctx.part_world_position(curtain)
    ctx.check(
        "curtain bottom bar clears the doorway when fully open",
        open_pos is not None and open_pos[2] > SILL_TOP + OPENING_H - 0.2,
        details=f"open_pos={open_pos}",
    )

    # ---- curtain has horizontal slats (verify count) ----
    slat_names = [v.name for v in curtain.visuals if v.name and v.name.startswith("slat_")]
    ctx.check(
        f"curtain has {N_SLATS} horizontal slats",
        len(slat_names) == N_SLATS,
        details=f"found {len(slat_names)} slat visuals",
    )

    # ---- prismatic joint axis is +Z ----
    art = object_model.get_articulation("body_to_curtain")
    ctx.check(
        "curtain joint axis is +Z (vertical rise)",
        art.axis is not None and abs(art.axis[2] - 1.0) < 0.01,
        details=f"axis={art.axis}",
    )

    # ---- curtain stays within the doorway opening XY at full travel ----
    with ctx.pose({curtain_joint: CURTAIN_TRAVEL}):
        ctx.expect_overlap(
            curtain, body, axes="xy", min_overlap=0.01,
            elem_a="bottom_bar", elem_b="door_header",
            name="curtain bottom bar still within doorway XY at full open",
        )

    return ctx.report()


object_model = build_object_model()
