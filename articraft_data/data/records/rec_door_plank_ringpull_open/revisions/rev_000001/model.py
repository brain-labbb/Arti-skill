from __future__ import annotations

# Rustic vertical-plank wooden door with a ring pull, set in a rough stone
# arched surround.
#
# Real object: a heavy ledged-and-braced cottage/castle door (~0.85 m wide,
# ~2.0 m tall leaf) built from vertical timber planks, hung on black wrought-iron
# strap hinges on the RIGHT jamb, with a large round iron ring-pull handle on the
# free (LEFT) edge. The leaf is set inside a rough stone surround whose head is a
# rounded arch built from chunky voussoir blocks on two stacked-stone jambs. The
# stone surround is the fixed root and stands on the ground (z=0 upward).
#
# Coordinate convention:
#   +Z is up (world). The opening is centered on world X=0 and the door spans X.
#   The leaf stands vertical in the X/Z plane and swings about a VERTICAL (Z)
#   hinge axis at the RIGHT jamb. The wall plane is at Y~0; positive q swings the
#   leaf outward toward the viewer (+Y). At q=0 the door is closed and vertical.
#
# Articulation:
#   stone_surround (fixed root) -> door (REVOLUTE about world +Z at the right
#   jamb). The leaf is authored in a local hinge frame (hinge edge at local x=0,
#   leaf body extending along local +X) and mounted rotated 180 deg about Z so it
#   reaches back across the opening from the right jamb. The axis is chosen so
#   positive q opens the leaf outward (a mirror of a left-hinged door).

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

# Clear opening (the gap the leaf fills)
OPENING_W = 0.90              # clear opening width along X
OPENING_H = 2.00              # clear opening straight-side height (springline of arch)

# Door leaf
LEAF_CLEARANCE = 0.035        # running clearance between leaf edge and each jamb
LEAF_W = OPENING_W - 2 * LEAF_CLEARANCE   # leaf width along X (0.83)
LEAF_H = 1.97                 # leaf height along Z
LEAF_T = 0.045                # leaf thickness along Y (heavy plank door)
LEAF_BOTTOM_Z = 0.018         # small ground gap so the leaf can swing
PLANK_COUNT = 6               # number of vertical planks across the leaf

# Stone surround (massive rough-hewn megalithic blocks)
JAMB_W = 0.34                 # stone jamb thickness along X (chunky rough blocks)
JAMB_DEPTH = 0.40             # stone depth along Y (heavy wall thickness)
ARCH_RISE = 0.45              # how far the arch head rises above the springline
WALL_TOP_Z = OPENING_H + ARCH_RISE + 0.12   # top of the stone block mass

# Reveal: the opening is set back into the wall thickness so the leaf seats in a
# rebate rather than sitting proud of the stones.
REVEAL_Y = 0.0                # leaf wall plane (front faces of leaf near y=0)

# Strap hinges (black wrought iron) on the RIGHT jamb edge
STRAP_LEN = 0.66              # how far the strap reaches across the leaf
STRAP_W = 0.090               # strap width (Z) - bold wrought-iron strap
STRAP_T = 0.013              # strap thickness (proud of the leaf -Y face)
STRAP_STUD_R = 0.013         # decorative bolt-head studs along the strap
STRAP_STUD_PROUD = 0.006     # how far the stud caps stand proud of the strap
HINGE_BARREL_R = 0.020       # pintle knuckle (barrel) outer radius at the pivot
HINGE_BARREL_LEN = 0.075     # knuckle height (Z)
# The knuckle projects past the leaf hinge edge into the leaf->jamb clearance gap
# (local -X, which maps to world +X toward the jamb) and stands proud on the
# local -Y face so it wraps a free-standing vertical pintle pin clear of the wood.
BARREL_GAP_OFFSET = 0.020    # how far the knuckle sits past the hinge edge (local -X)
BARREL_PROUD_Y = LEAF_T / 2.0 + HINGE_BARREL_R + 0.004   # knuckle center stands proud

# Ring pull (free / LEFT edge) - large dark iron ring on a four-pointed star plate
RING_PLATE_R = 0.082         # star backplate point radius (tip-to-center)
RING_PLATE_WAIST = 0.030     # star "waist" half-width between adjacent points
RING_PLATE_T = 0.013         # backplate thickness (proud, leaf -Y face)
RING_OUTER_R = 0.066         # ring outer radius (centerline of tube to outer edge)
RING_TUBE_R = 0.012          # ring bar (torus tube) radius
RING_BOSS_R = 0.016          # boss/stud the ring hangs from
RING_BOSS_PROUD = 0.030      # how far the boss stands proud of the leaf face (-Y)
# Ring authored in its OWN part frame: pivot (boss center) at local origin, ring
# hangs below along -Z, standing proud of the door face toward local -Y. The hang
# is chosen so the ring's TOP tube straddles the boss center and stays captured.
RING_HANG = RING_OUTER_R     # boss-center to ring-center drop (Z); top tube on boss

# Sliding door bolt (门销): a horizontal iron rod below the ring that slides
# through two iron keepers (staples) fixed to the leaf.
BOLT_ROD_R = 0.012           # bolt rod radius
BOLT_ROD_LEN = 0.40          # bolt rod length (long enough to stay in both keepers)
BOLT_ROD_CX = -0.05          # rod center along X (shifted -X so it stays engaged at throw)
BOLT_TRAVEL = 0.10           # usable prismatic throw
BOLT_PROUD_Y = LEAF_T / 2.0 + BOLT_ROD_R + 0.004   # rod center proud of leaf -Y face
BOLT_KNOB_R = 0.016          # grip knob radius at the throw end
KEEPER_SPACING = 0.16        # distance between the two keepers along the rod
KEEPER_R_OUTER = BOLT_ROD_R + 0.009   # keeper loop outer radius
KEEPER_BASE_W = 0.030        # keeper base plate footprint
# Leaf-local placement of the bolt assembly (below the ring on the free side).
# Origin sits at the inner (knob-side) keeper; the rod throws toward the free edge.
BOLT_LOCAL_X = LEAF_W - 0.24     # along leaf width (free side, below the ring)
BOLT_LOCAL_Z = LEAF_H * 0.40     # below the ring

# ---------------------------------------------------------------------------
# Materials (restrained, real)
# ---------------------------------------------------------------------------

WOOD_RGBA = (0.80, 0.77, 0.70, 1.0)      # light weathered grey-tan plank wood
WOOD_DARK_RGBA = (0.55, 0.50, 0.43, 1.0)  # plank groove shadow lines
IRON_RGBA = (0.08, 0.08, 0.09, 1.0)      # black wrought iron
STONE_RGBA = (0.62, 0.62, 0.59, 1.0)     # weathered rough grey stone
MOSS_RGBA = (0.40, 0.44, 0.37, 1.0)      # subtle greyed-green moss patches


# ---------------------------------------------------------------------------
# Stone surround geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_stone_surround_shape() -> cq.Workplane:
    """Rough stone arched surround: two stacked-stone jambs carrying a rounded
    arch head built from a thick ring (voussoir band). The opening (and the arch
    soffit) is hollow so the door reads as set into a real wall.

    World frame: opening centered at X=0, wall plane centered at Y=0,
    Z from 0 (ground) upward.
    """
    half_open = OPENING_W / 2.0
    outer_half = half_open + JAMB_W           # outer X extent of the stone mass
    spring_z = OPENING_H                      # springline (top of straight jambs)

    # ---- Solid outer block mass (rectangular wall slab up to springline +
    # extra, then the arch head will be carved out of it). ----
    # Lower rectangular wall block from ground to a bit above the springline.
    block_h = spring_z + ARCH_RISE + 0.12
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, block_h / 2.0))
        .box(2.0 * outer_half, JAMB_DEPTH, block_h)
    )

    # ---- Carve the doorway: a rectangular opening up to the springline,
    # topped by a semicircular (rounded-arch) head of radius half_open. ----
    cut_depth = JAMB_DEPTH + 0.02

    # Rectangular part of the opening (ground to springline).
    rect_cut = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, spring_z / 2.0))
        .box(OPENING_W, cut_depth, spring_z)
    )

    # Semicircular arch head: a half-cylinder (axis along Y) sitting on the
    # springline. CadQuery cylinder default axis is Z, so build on a YZ-style
    # workplane: easiest is a cylinder along Y via a transformed workplane.
    arch_cut = (
        cq.Workplane("XZ")              # plane whose normal is Y
        .transformed(offset=(0.0, spring_z, 0.0))
        .circle(half_open)
        .extrude(cut_depth / 2.0, both=True)
    )

    doorway = rect_cut.union(arch_cut)
    surround = outer.cut(doorway)

    # ---- Trim the outer silhouette of the stone so the head follows the arch
    # instead of staying a full rectangle: cut away the two upper outer corners
    # above the springline beyond the arch ring thickness. ----
    ring_thick = JAMB_W                      # voussoir band thickness
    arch_outer_r = half_open + ring_thick
    # Remove everything above springline that is OUTSIDE the outer arch radius by
    # subtracting two tall corner blocks, then re-adding the arch ring is implicit
    # because the arch ring is the material between inner and outer radius.
    # Corner blocks: from arch_outer top down to springline, at far left/right.
    corner_h = block_h - spring_z
    for sx in (-1.0, 1.0):
        corner = (
            cq.Workplane("XY")
            .transformed(offset=(
                sx * (outer_half + arch_outer_r) / 2.0,   # outer strip x-center
                0.0,
                spring_z + corner_h / 2.0,
            ))
            .box(outer_half - arch_outer_r + 0.001, JAMB_DEPTH + 0.02, corner_h)
        )
        surround = surround.cut(corner)
    # Cut the flat top above the arch crown down to the arch outer radius so the
    # crown is rounded, not a square cap.
    crown_z = spring_z + arch_outer_r
    if block_h > crown_z:
        top_trim = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, 0.0, (block_h + crown_z) / 2.0))
            .box(2.0 * outer_half + 0.01, JAMB_DEPTH + 0.02, block_h - crown_z + 0.002)
        )
        # Only trim the region OUTSIDE the arch crown footprint; but simplest is
        # to round the top by cutting the two shoulders. Cut shoulder wedges that
        # sit above the outer arch radius circle.
        # Approximate by cutting the top rectangle then re-rounding via the outer
        # arch ring: subtract material above the outer arch semicircle.
        outer_arch_solid = (
            cq.Workplane("XZ")
            .transformed(offset=(0.0, spring_z, 0.0))
            .circle(arch_outer_r)
            .extrude(cut_depth / 2.0, both=True)
        )
        # Region above springline and above the arch = block minus (arch solid +
        # below-springline). Subtract (top_trim minus outer_arch_solid).
        shoulders = top_trim.cut(outer_arch_solid)
        surround = surround.cut(shoulders)

    # ---- Rough-hewn megalithic detailing ----
    # The surround must read as a few MASSIVE rough-cut stones, not a smooth slab:
    # deep irregular bed joints between heavy courses, jagged vertical perpends so
    # block ends step in/out, deep voussoir joints in the arch, weathered rounded
    # outer edges, and shallow facets gouged into each block face.
    face_y = JAMB_DEPTH / 2.0      # front face plane

    # Deep horizontal bed joints across the two jambs. Heavy irregular courses:
    # the gaps between course lines vary so the stones are different thicknesses.
    # Course fractions of the springline height (rough megalithic stacking).
    bed_fracs = (0.20, 0.46, 0.74)
    seam_d = 0.030                 # deep joints so blocks separate visibly
    for ci, frac in enumerate(bed_fracs):
        cz = spring_z * frac
        # Slight per-course wobble so beds are not perfectly level.
        wob = 0.012 * (1.0 if ci % 2 == 0 else -1.0)
        seam_w = 0.022 + 0.006 * (ci % 2)   # joints vary in width
        for fy, sign in ((face_y, 1.0), (-face_y, -1.0)):
            seam = (
                cq.Workplane("XY")
                .transformed(offset=(0.0, fy - sign * (seam_d / 2.0), cz + wob),
                             rotate=(0.0, sign * 2.0, 0.0))
                .box(2.0 * outer_half + 0.02, seam_d + 0.002, seam_w)
            )
            surround = surround.cut(seam)

    # Jagged vertical perpend joints: short vertical gouges on the jamb faces so
    # adjacent stones in a course read as separate rough blocks with stepped ends.
    perp_d = 0.026
    perp_w = 0.020
    jamb_cx = half_open + JAMB_W / 2.0       # center of each jamb in X
    perp_specs = (
        (0.10, 0.32),   # (z fraction of springline, x jitter within jamb)
        (0.58, -0.28),
        (0.86, 0.20),
    )
    for zf, xj in perp_specs:
        cz = spring_z * zf
        for sx in (-1.0, 1.0):
            px = sx * (jamb_cx + xj * JAMB_W * 0.5)
            for fy, sign in ((face_y, 1.0), (-face_y, -1.0)):
                perp = (
                    cq.Workplane("XY")
                    .transformed(offset=(px, fy - sign * (perp_d / 2.0), cz))
                    .box(perp_w, perp_d + 0.002, spring_z * 0.22)
                )
                surround = surround.cut(perp)

    # Deep radial voussoir joints in the arch ring (heavy wedge stones).
    n_vous = 7
    vseam_w = 0.024
    vseam_d = 0.030
    for k in range(1, n_vous):
        ang = math.pi * k / n_vous          # 0..pi sweeping the semicircular head
        cx = math.cos(ang)
        cz = math.sin(ang)
        mid_r = half_open + JAMB_W / 2.0    # midpoint of the ring band radially
        px = mid_r * cx
        pz = spring_z + mid_r * cz
        for fy, sign in ((face_y, 1.0), (-face_y, -1.0)):
            voslot = (
                cq.Workplane("XY")
                .transformed(offset=(px, fy - sign * (vseam_d / 2.0), pz),
                             rotate=(0.0, 0.0, math.degrees(ang) - 90.0))
                .box(JAMB_W + 0.04, vseam_d + 0.002, vseam_w)
            )
            surround = surround.cut(voslot)

    # Weathered rounded outer corners: chamfer the long front/back vertical outer
    # edges of the two jambs so the stone reads as worn, not sharp-machined.
    try:
        surround = (
            surround.edges("|Z")
            .edges(cq.selectors.BoxSelector(
                (-outer_half - 0.01, -face_y - 0.01, -0.01),
                (-outer_half + 0.02, face_y + 0.01, spring_z),
            ))
            .chamfer(0.022)
        )
        surround = (
            surround.edges("|Z")
            .edges(cq.selectors.BoxSelector(
                (outer_half - 0.02, -face_y - 0.01, -0.01),
                (outer_half + 0.01, face_y + 0.01, spring_z),
            ))
            .chamfer(0.022)
        )
    except Exception:
        # Chamfer can fail on degenerate selections; the model is still valid.
        pass

    # Shallow rough facets gouged into the block faces so each stone reads pitted
    # and hand-cut rather than flat. Small shallow scoops on the jamb fronts.
    facet_specs = (
        (0.36, 0.32, 1.0), (0.68, 0.30, -1.0),
        (0.18, 0.28, -1.0), (0.50, 0.26, 1.0),
    )
    for zf, xj, sx in facet_specs:
        cz = spring_z * zf
        px = sx * (jamb_cx + xj * JAMB_W * 0.4 - JAMB_W * 0.1)
        for fy, sign in ((face_y, 1.0), (-face_y, -1.0)):
            scoop = (
                cq.Workplane("XZ")
                .transformed(offset=(px, fy, cz))
                .circle(0.055)
                .extrude(sign * 0.010)
            )
            surround = surround.cut(scoop)

    # Proud rough block caps: a few heavy individual stones stand slightly proud of
    # the jamb face so the surround reads as stacked rough-cut megaliths, not a flat
    # slab. Each cap is an irregular box bulging out of the front/back face.
    proud_specs = (
        # (z fraction, x jitter frac, half-height frac of a course, proud amount)
        (0.085, 0.10, 0.085, 0.026),
        (0.40, -0.12, 0.10, 0.030),
        (0.66, 0.08, 0.075, 0.024),
        (0.90, -0.06, 0.07, 0.028),
    )
    for zf, xjf, hf, proud in proud_specs:
        cz = spring_z * zf
        bh = spring_z * hf
        for sx in (-1.0, 1.0):
            cx = sx * (jamb_cx + xjf * JAMB_W * 0.5)
            bw = JAMB_W * 0.78
            for fy, sign in ((face_y, 1.0), (-face_y, -1.0)):
                cap = (
                    cq.Workplane("XY")
                    .transformed(offset=(cx, fy + sign * (proud / 2.0 - 0.001), cz),
                                 rotate=(0.0, sign * 1.5, 0.0))
                    .box(bw, proud, bh)
                )
                surround = surround.union(cap)

    return surround


def _build_moss_patches_shape() -> cq.Workplane:
    """A few thin greyed-green moss skins clinging to the weathered jamb stones
    and the lower arch shoulders, for subtle organic weathering (front face)."""
    half_open = OPENING_W / 2.0
    spring_z = OPENING_H
    jamb_cx = half_open + JAMB_W / 2.0
    face_y = JAMB_DEPTH / 2.0
    patches = None
    specs = (
        # (x-center, z-center, w, h, thickness-proud)
        (-jamb_cx - JAMB_W * 0.18, spring_z * 0.16, 0.10, 0.14, 0.004),
        (jamb_cx + JAMB_W * 0.10, spring_z * 0.30, 0.08, 0.10, 0.004),
        (-jamb_cx + JAMB_W * 0.05, spring_z * 0.62, 0.07, 0.09, 0.0035),
        (jamb_cx - JAMB_W * 0.10, spring_z * 0.04, 0.12, 0.08, 0.004),
    )
    for cx, cz, w, h, t in specs:
        # A thin slab clinging proud of the front face (+Y).
        patch = (
            cq.Workplane("XY")
            .transformed(offset=(cx, face_y + t / 2.0, cz))
            .box(w, t, h)
        )
        patches = patch if patches is None else patches.union(patch)
    return patches


# Pintle gudgeons driven into the right jamb. Each is an L-shaped iron fitting:
# a horizontal arm embedded in the jamb masonry plus an upright pintle stub that
# the door's strap-hinge knuckle wraps. This carries the moving leaf on the jamb.
PINTLE_R = 0.008                          # upright pintle stub radius
PINTLE_ARM_R = 0.010                      # embedded horizontal arm radius
JAMB_INNER_X = OPENING_W / 2.0            # right jamb inner face (world +X)
HINGE_EDGE_X = OPENING_W / 2.0 - LEAF_CLEARANCE   # leaf hinge edge (world +X)
# Knuckle world position after the leaf is mounted (local -X,-Y mapped via yaw=pi).
KNUCKLE_WORLD_X = HINGE_EDGE_X + BARREL_GAP_OFFSET
KNUCKLE_WORLD_Y = BARREL_PROUD_Y
# World hinge heights = leaf bottom + leaf-local strap z heights.
STRAP_LOCAL_ZS = (LEAF_H * 0.78, LEAF_H * 0.22)
HINGE_WORLD_ZS = tuple(LEAF_BOTTOM_Z + z for z in STRAP_LOCAL_ZS)


def _build_pintle_pins_shape() -> cq.Workplane:
    """Two L-shaped iron gudgeon pintles, placed directly in the world frame.

    Each pintle has a horizontal arm running in X from inside the jamb masonry
    out to the knuckle, then an upright pintle stub at the knuckle position that
    the door knuckle wraps. The arm provides the support path to the stone; the
    stub provides the rotation pivot.
    """
    pins = None
    for wz in HINGE_WORLD_ZS:
        # Horizontal arm: from the knuckle (in the gap) across the gap and ~40 mm
        # into the jamb masonry, at the knuckle Y and height.
        arm_x0 = KNUCKLE_WORLD_X           # at the knuckle (in the gap)
        arm_x1 = JAMB_INNER_X + 0.040      # embedded into the jamb masonry
        arm_len = arm_x1 - arm_x0
        arm = (
            cq.Workplane("XY")
            .circle(PINTLE_ARM_R)
            .extrude(arm_len)                  # +Z
            .rotate((0, 0, 0), (0, 1, 0), 90)  # +Z -> +X
            .translate((arm_x0, KNUCKLE_WORLD_Y, wz))
        )
        # Upright pintle stub: vertical cylinder at the knuckle, spanning the
        # knuckle height so the door knuckle fully wraps it.
        stub = (
            cq.Workplane("XY")
            .transformed(offset=(KNUCKLE_WORLD_X, KNUCKLE_WORLD_Y, wz))
            .circle(PINTLE_R)
            .extrude(HINGE_BARREL_LEN / 2.0 + 0.006, both=True)
        )
        fitting = arm.union(stub)
        pins = fitting if pins is None else pins.union(fitting)
    return pins


# ---------------------------------------------------------------------------
# Door leaf geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_leaf_shape() -> cq.Workplane:
    """Vertical-plank door leaf with shallow V-grooves between planks.

    Authored in the leaf-local hinge frame:
      - local X runs 0 .. LEAF_W (hinge edge at x=0, free edge at x=LEAF_W)
      - local Z runs 0 .. LEAF_H
      - local Y is the leaf thickness, centered at y=0
    """
    w = LEAF_W
    h = LEAF_H
    t = LEAF_T

    leaf = (
        cq.Workplane("XY")
        .transformed(offset=(w / 2.0, 0.0, h / 2.0))
        .box(w, t, h)
    )

    # Plank seams: shallow rectangular grooves cut into BOTH faces between planks.
    groove_w = 0.012
    groove_d = 0.010
    plank_w = w / PLANK_COUNT
    for i in range(1, PLANK_COUNT):
        gx = i * plank_w
        for face_y in (t / 2.0, -t / 2.0):
            sign = 1.0 if face_y > 0 else -1.0
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(gx, face_y - sign * groove_d / 2.0, h / 2.0))
                .box(groove_w, groove_d + 0.001, h + 0.01)
            )
            leaf = leaf.cut(groove)
    return leaf


def _build_strap_hinge_shape(reach: float) -> cq.Workplane:
    """One black iron strap hinge in the leaf-local frame.

    Three pieces:
      - a long tapered strap lying flat on the leaf's local -Y face, running
        along local +X from the hinge edge (x=0) across the leaf,
      - a vertical pintle knuckle (barrel) that projects past the hinge edge into
        the leaf->jamb clearance gap (local -X) and stands proud on local -Y so it
        wraps the free-standing jamb pintle pin clear of the wood,
      - a short neck linking the knuckle back to the strap root.

    After the leaf is mounted rotated 180 deg about Z, local -Y maps to world +Y
    (toward the viewer) and local -X maps to world +X (toward the jamb), so the
    knuckle ends up in the gap facing the viewer.
    """
    bw = STRAP_W
    bt = STRAP_T
    face_y = -LEAF_T / 2.0    # local -Y face of the leaf
    strap_y = face_y - bt / 2.0

    # Tapered strap: full-width root tapering to a narrower spade tip.
    root_w = bw
    tip_w = bw * 0.55
    root_len = reach * 0.42
    tip_len = reach - root_len

    root = (
        cq.Workplane("XY")
        .transformed(offset=(root_len / 2.0, strap_y, 0.0))
        .box(root_len, bt, root_w)
    )
    tip = (
        cq.Workplane("XY")
        .transformed(offset=(root_len + tip_len / 2.0, strap_y, 0.0))
        .box(tip_len, bt, tip_w)
    )
    strap = root.union(tip)

    # Diamond/spade end finial at the strap tip (classic wrought-iron hinge end).
    finial_x = root_len + tip_len
    finial = (
        cq.Workplane("XY")
        .transformed(offset=(finial_x, strap_y, 0.0), rotate=(0.0, 0.0, 45.0))
        .box(tip_w * 1.15, bt, tip_w * 1.15)
    )
    strap = strap.union(finial)

    # Decorative proud bolt-head studs riveting the strap to the planks.
    stud_xs = (root_len * 0.30, root_len + tip_len * 0.45, finial_x)
    for sx in stud_xs:
        # Short cylinder whose axis is Y, sitting just proud of the strap face.
        stud = (
            cq.Workplane("XZ")
            .transformed(offset=(sx, 0.0, 0.0))
            .circle(STRAP_STUD_R)
            .extrude(-(bt + STRAP_STUD_PROUD))
            .translate((0.0, face_y, 0.0))
        )
        strap = strap.union(stud)

    # Vertical pintle knuckle in the gap, standing proud on local -Y.
    barrel_x = -BARREL_GAP_OFFSET
    barrel_y = -BARREL_PROUD_Y
    barrel = (
        cq.Workplane("XY")
        .transformed(offset=(barrel_x, barrel_y, 0.0))
        .circle(HINGE_BARREL_R)
        .extrude(HINGE_BARREL_LEN / 2.0, both=True)
    )

    # Neck: a short flat bar linking the knuckle (in the gap, proud) back to the
    # strap root on the leaf face, so the knuckle is structurally carried.
    neck = (
        cq.Workplane("XY")
        .transformed(offset=(barrel_x / 2.0, barrel_y / 2.0, 0.0))
        .box(abs(barrel_x) + 2 * HINGE_BARREL_R, abs(barrel_y) + bt, bw * 0.7)
    )
    return strap.union(neck).union(barrel)


def _build_ring_plate_shape() -> cq.Workplane:
    """Four-pointed-star (quatrefoil-diamond) forged-iron escutcheon for the ring
    pull, seated flush on the leaf -Y face (which maps to world +Y, toward the
    viewer, after the leaf is mounted). The plate carries a central BOSS/PROTRUSION
    stud the ring hangs from, and the boss center is the ring's pivot.

    Built (in the X/Z plane, normal along Y) as the union of two crossed slender
    diamonds: one vertical (long axis Z), one horizontal (long axis X). Their
    overlap forms a star with four sharp points and a pinched waist between them.
    """
    face_y = -LEAF_T / 2.0
    tip = RING_PLATE_R          # point tip distance from center
    waist = RING_PLATE_WAIST    # half-width at the waist between points

    def _diamond(half_long: float, half_short: float, vertical: bool) -> cq.Workplane:
        # Rhombus in the X/Z plane, points on the long axis.
        if vertical:
            pts = [(0.0, half_long), (half_short, 0.0), (0.0, -half_long), (-half_short, 0.0)]
        else:
            pts = [(half_long, 0.0), (0.0, half_short), (-half_long, 0.0), (0.0, -half_short)]
        return (
            cq.Workplane("XZ")
            .polyline(pts)
            .close()
            # On an XZ workplane a POSITIVE extrude grows toward local -Y, which is
            # the proud direction (it maps to world +Y, toward the viewer).
            .extrude(RING_PLATE_T)
        )

    star = _diamond(tip, waist, vertical=True).union(
        _diamond(tip, waist, vertical=False)
    )
    star = star.translate((0.0, face_y, 0.0))

    # Central boss/stud the ring hangs from (stands further proud than the plate).
    boss = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, 0.0, 0.0))
        .circle(RING_BOSS_R)
        .extrude(RING_BOSS_PROUD)
    )
    boss = boss.translate((0.0, face_y, 0.0))
    return star.union(boss)


def _build_ring_shape() -> cq.Workplane:
    """The large round iron ring (torus) that hangs from the boss.

    Authored in the RING PART's OWN local frame: the pivot (the boss center) is at
    the local origin, the ring hangs straight down along local -Z, and the ring
    body stands proud of the door face toward local -Y. After the ring part is
    mounted on the door at the boss, local -Y maps to world +Y (toward viewer).
    The top of the ring intentionally overlaps the boss so it stays captured.
    """
    # Torus standing in the X/Z plane (symmetry axis Y, thin in Y, faces viewer):
    # revolve a tube circle about the global Y axis from a profile in the X/Y plane.
    ring = (
        cq.Workplane("XY")
        .moveTo(RING_OUTER_R, 0.0)
        .circle(RING_TUBE_R)
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    )
    # Hang the ring RING_HANG below the boss/pivot (local origin) along -Z, and
    # stand it proud of the door face (toward -Y) so it clears the planks while its
    # TOP tube straddles the boss and stays captured (intentional small overlap).
    ring = ring.translate(
        (0.0, -(LEAF_T / 2.0 + RING_TUBE_R - 0.002), -RING_HANG)
    )
    return ring


# ---------------------------------------------------------------------------
# Sliding door bolt (门销): rod + grip knob, sliding through two fixed keepers
# ---------------------------------------------------------------------------

def _build_bolt_keepers_shape() -> cq.Workplane:
    """Two iron keepers (staples) fixed to the door leaf that the bolt rod passes
    through. Authored in the bolt-assembly local frame: the rod axis runs along
    local X, the rod center sits at local (x=0, y=-BOLT_PROUD_Y, z=0), and the two
    keepers straddle the origin at +/- KEEPER_SPACING/2. The origin keeper (at
    local x=0) is the prismatic joint origin reference.
    """
    keepers = None
    half_t = LEAF_T / 2.0
    for kx in (-KEEPER_SPACING / 2.0, KEEPER_SPACING / 2.0):
        # A small square base plate seated on the leaf face (-Y), carrying a loop
        # (open staple) the rod runs through. Modeled as a short tube around the
        # rod path plus a base tab so it reads as a bracket bolted to the leaf.
        base = (
            cq.Workplane("XY")
            .transformed(offset=(kx, -half_t - 0.004, 0.0))
            .box(0.022, 0.010, KEEPER_BASE_W)
        )
        # Loop: a short ring (tube) coaxial with the rod (axis along X), wrapping
        # the rod so the rod is captured but free to slide along X.
        loop = (
            cq.Workplane("YZ")             # plane normal along X
            .transformed(offset=(0.0, 0.0, kx))
            .circle(KEEPER_R_OUTER)
            .circle(BOLT_ROD_R + 0.002)    # clearance bore so the rod slides
            .extrude(0.018, both=True)
            .translate((0.0, -BOLT_PROUD_Y, 0.0))
        )
        keeper = base.union(loop)
        keepers = keeper if keepers is None else keepers.union(keeper)
    return keepers


def _build_bolt_rod_shape() -> cq.Workplane:
    """The sliding iron bolt rod with a grip knob, authored in the bolt-assembly
    local frame. The rod axis runs along local X, centered so that at q=0 it is
    engaged in BOTH keepers with retained insertion on each side; positive
    prismatic q slides it along +X to throw the bolt. The knob is on the -X
    (retract/grip) end so it never collides with the throw keeper."""
    # Rod centered slightly toward -X so that at full throw (+X) it still passes
    # through the +X keeper, and at q=0 it passes through both keepers.
    rod_cx = BOLT_ROD_CX
    rod = (
        cq.Workplane("YZ")                 # circle in Y/Z, extrude along X
        .transformed(offset=(0.0, 0.0, 0.0))
        .circle(BOLT_ROD_R)
        .extrude(BOLT_ROD_LEN / 2.0, both=True)
        .translate((rod_cx, -BOLT_PROUD_Y, 0.0))
    )
    # Grip knob: a small ball/cap on the -X end, standing proud of the rod (-Y).
    knob = (
        cq.Workplane("XY")
        .transformed(offset=(rod_cx - BOLT_ROD_LEN / 2.0 - 0.004,
                             -BOLT_PROUD_Y - BOLT_ROD_R - 0.004, 0.0))
        .sphere(BOLT_KNOB_R)
    )
    # Short neck linking knob to the rod end.
    neck = (
        cq.Workplane("YZ")
        .transformed(offset=(0.0, 0.0, 0.0))
        .circle(BOLT_ROD_R * 0.7)
        .extrude(0.010, both=True)
        .translate((rod_cx - BOLT_ROD_LEN / 2.0, -BOLT_PROUD_Y - BOLT_ROD_R - 0.004, 0.0))
    )
    return rod.union(neck).union(knob)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plank_door_ringpull")

    model.material("wood", rgba=WOOD_RGBA)
    model.material("iron", rgba=IRON_RGBA)
    model.material("stone", rgba=STONE_RGBA)
    model.material("moss", rgba=MOSS_RGBA)

    # --- Stone surround (fixed root) ---
    surround = model.part("stone_surround")
    surround.visual(
        mesh_from_cadquery(_build_stone_surround_shape(), "stone_surround"),
        material="stone",
        name="surround_shell",
    )
    # Subtle moss skins clinging to the weathered jamb stones.
    surround.visual(
        mesh_from_cadquery(_build_moss_patches_shape(), "moss_patches"),
        material="moss",
        name="moss_patches",
    )
    # Iron gudgeon pintle pins driven into the right jamb; the door's strap-hinge
    # barrels wrap these, so the leaf is physically carried by the stone jamb.
    surround.visual(
        mesh_from_cadquery(_build_pintle_pins_shape(), "pintle_pins"),
        material="iron",
        name="pintle_pins",
    )

    # --- Door leaf (authored in local hinge frame) ---
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_build_leaf_shape(), "door_leaf"),
        material="wood",
        name="leaf_planks",
    )

    # Two strap hinges across the leaf, near top and bottom thirds. They reach
    # from the hinge edge (local x=0) along +X.
    for idx, sz in enumerate(STRAP_LOCAL_ZS):   # leaf-local z heights
        door.visual(
            mesh_from_cadquery(_build_strap_hinge_shape(STRAP_LEN), f"strap_hinge_{idx}"),
            material="iron",
            origin=Origin(xyz=(0.0, 0.0, sz)),
            name=f"strap_hinge_{idx}",
        )

    # Ring-pull STAR backplate (escutcheon) on the FREE edge (local x near LEAF_W),
    # at a comfortable hand height. The plate + central boss are fixed to the leaf;
    # the ring itself is a separate rotatable part hung on the boss (below).
    ring_x = LEAF_W - 0.10
    ring_z = LEAF_H * 0.50
    door.visual(
        mesh_from_cadquery(_build_ring_plate_shape(), "ring_plate"),
        material="iron",
        origin=Origin(xyz=(ring_x, 0.0, ring_z)),
        name="ring_plate",
    )

    # Two iron keepers (staples) for the slide bolt, fixed to the leaf. The bolt
    # rod (separate sliding part, below) passes through both keeper loops.
    door.visual(
        mesh_from_cadquery(_build_bolt_keepers_shape(), "bolt_keepers"),
        material="iron",
        origin=Origin(xyz=(BOLT_LOCAL_X, 0.0, BOLT_LOCAL_Z)),
        name="bolt_keepers",
    )

    # --- Ring pull: its OWN rotatable part, pivoting on the escutcheon boss ---
    ring = model.part("ring_pull")
    ring.visual(
        mesh_from_cadquery(_build_ring_shape(), "ring_body"),
        material="iron",
        name="ring_body",
    )

    # --- Door bolt: its OWN sliding part (rod + grip knob) ---
    bolt = model.part("door_bolt")
    bolt.visual(
        mesh_from_cadquery(_build_bolt_rod_shape(), "bolt_rod"),
        material="iron",
        name="bolt_rod",
    )

    # --- Articulation: hinge on the RIGHT jamb (world +X side) ---
    # The leaf is authored with its hinge edge at local x=0, body extending +X.
    # Mount it at the right jamb inner face and rotate 180 deg about Z so the leaf
    # body reaches back toward the center (-X world). With rpy yaw=pi the local +X
    # maps to world -X; choosing axis=(0,0,-1) makes positive q swing the free
    # (left) edge outward toward the viewer (+Y), i.e. the door opens.
    # Place the hinge edge a running-clearance inboard of the stone jamb inner
    # face so the leaf does not bind/coincide with the stone.
    right_jamb_x = OPENING_W / 2.0 - LEAF_CLEARANCE   # hinge edge x (world +X side)
    hinge_origin_z = LEAF_BOTTOM_Z           # leaf local z=0 maps to world z here
    model.articulation(
        "surround_to_door",
        ArticulationType.REVOLUTE,
        parent=surround,
        child=door,
        origin=Origin(xyz=(right_jamb_x, REVEAL_Y, hinge_origin_z), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    # --- Articulation: ring pivots on the escutcheon boss ---
    # The ring hangs from the boss; the joint frame sits at the boss center (the
    # ring's local origin). The ring part is authored in the leaf-local orientation
    # (its proud direction is leaf-local -Y), so the joint axis is the leaf-local X
    # (horizontal, parallel to the door face). Right-hand rotation about +X drives
    # the ring's bottom toward leaf-local +Y (into the door); negating to -X makes
    # positive q tilt the ring's bottom OUT off the face (leaf-local -Y = world +Y),
    # i.e. the ring is lifted/pulled away from the wood.
    model.articulation(
        "door_to_ring",
        ArticulationType.REVOLUTE,
        parent=door,
        child=ring,
        origin=Origin(xyz=(ring_x, 0.0, ring_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.2),
    )

    # --- Articulation: slide bolt throws horizontally through the two keepers ---
    # The bolt assembly is authored with its rod axis along leaf-local X. The joint
    # origin sits at the inner (knob-side) keeper; positive prismatic q slides the
    # rod along leaf-local +X (toward the free edge) to throw the bolt. The rod is
    # long enough to remain engaged in BOTH keepers across the full travel.
    model.articulation(
        "door_to_bolt",
        ArticulationType.PRISMATIC,
        parent=door,
        child=bolt,
        origin=Origin(xyz=(BOLT_LOCAL_X, 0.0, BOLT_LOCAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.2, lower=0.0, upper=BOLT_TRAVEL),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    surround = object_model.get_part("stone_surround")
    door = object_model.get_part("door")
    ring = object_model.get_part("ring_pull")
    bolt = object_model.get_part("door_bolt")
    hinge = object_model.get_articulation("surround_to_door")
    ring_joint = object_model.get_articulation("door_to_ring")
    bolt_joint = object_model.get_articulation("door_to_bolt")

    # --- Intentional overlaps (local, mechanically explanatory) ---
    # Strap hinges lie proud on the leaf front face and their barrels wrap the
    # iron pintle pins; the strap-to-leaf contact is intended capture.
    for idx in (0, 1):
        ctx.allow_overlap(
            door, door,
            elem_a=f"strap_hinge_{idx}",
            elem_b="leaf_planks",
            reason="Strap hinge is bolted flat onto the leaf face; the barrel wraps the pivot.",
        )
        # Each strap-hinge barrel wraps an iron pintle pin set into the jamb. This
        # captured pin-in-barrel fit is the real hinge mount and support path.
        ctx.allow_overlap(
            surround, door,
            elem_a="pintle_pins",
            elem_b=f"strap_hinge_{idx}",
            reason="Strap-hinge barrel wraps the jamb pintle pin (captured pin-in-barrel hinge).",
        )
        # The knuckle sits hard against the jamb pivot, so it locally nudges into
        # the stone jamb face by a few millimeters at the pivot line.
        ctx.allow_overlap(
            surround, door,
            elem_a="surround_shell",
            elem_b=f"strap_hinge_{idx}",
            reason="The hinge knuckle seats against the jamb at the pivot line.",
        )
    # The pintle pins are driven into the stone masonry of the jamb.
    ctx.allow_overlap(
        surround, surround,
        elem_a="pintle_pins",
        elem_b="surround_shell",
        reason="Iron gudgeon pins are driven into the stone jamb masonry.",
    )
    # Ring-pull star escutcheon + boss are seated onto the leaf face.
    ctx.allow_overlap(
        door, door,
        elem_a="ring_plate",
        elem_b="leaf_planks",
        reason="Iron star backplate is mounted flush onto the leaf face.",
    )
    # The ring (separate rotatable part) hangs from and is captured by the boss;
    # its top intentionally overlaps the boss so it stays captured (no floating).
    ctx.allow_overlap(
        ring, door,
        elem_a="ring_body",
        elem_b="ring_plate",
        reason="The ring is hung on the escutcheon boss; its top wraps the boss (captured ring-on-boss).",
    )
    # The bolt rod slides through the two fixed keeper loops; the rod intentionally
    # overlaps the keeper bores (proxy sleeve fit), staying engaged across travel.
    ctx.allow_overlap(
        bolt, door,
        elem_a="bolt_rod",
        elem_b="bolt_keepers",
        reason="The slide-bolt rod passes through the keeper loops (captured rod-in-keeper slide fit).",
    )
    # The keepers are bolted flat onto the leaf face.
    ctx.allow_overlap(
        door, door,
        elem_a="bolt_keepers",
        elem_b="leaf_planks",
        reason="Iron keepers/staples are bolted flat onto the leaf face.",
    )

    # --- Support proof: the door's strap-hinge knuckles wrap the jamb pintle
    # pins, which is how the moving leaf is carried by the stone surround. ---
    for idx in (0, 1):
        ctx.expect_contact(
            door, surround,
            elem_a=f"strap_hinge_{idx}",
            elem_b="pintle_pins",
            name=f"strap hinge {idx} knuckle engages the jamb pintle pin",
        )

    # --- Closed pose (q=0): leaf is vertical and seats in the opening ---
    with ctx.pose({hinge: 0.0}):
        leaf_aabb = ctx.part_world_aabb(door)
        lo, hi = leaf_aabb
        leaf_h = hi[2] - lo[2]
        leaf_w = hi[0] - lo[0]
        leaf_thk = hi[1] - lo[1]
        # The door stands UPRIGHT: tall in Z, wide in X, thin in Y.
        ctx.check(
            "door stands upright (tall, not lying flat)",
            leaf_h > 1.5 and leaf_h > leaf_w and leaf_thk < 0.30,
            details=f"leaf w={leaf_w:.3f} h={leaf_h:.3f} thk={leaf_thk:.3f}",
        )
        # The leaf sits just above the ground, not sunk below or floating.
        ctx.check(
            "leaf rests on the ground plane (not sunk/floating)",
            -0.001 < lo[2] < 0.05,
            details=f"leaf min z={lo[2]:.3f}",
        )
        # The closed leaf fills the opening: its X span covers most of the opening.
        ctx.check(
            "closed leaf fills the opening width",
            leaf_w > OPENING_W - 0.10,
            details=f"leaf width={leaf_w:.3f}, opening={OPENING_W:.3f}",
        )

    # --- Surround stands on the ground and rises into an arch above the leaf ---
    surround_aabb = ctx.part_world_aabb(surround)
    s_lo, s_hi = surround_aabb
    ctx.check(
        "stone surround stands on the ground",
        -0.001 < s_lo[2] < 0.02,
        details=f"surround min z={s_lo[2]:.3f}",
    )
    ctx.check(
        "arched surround rises well above the door leaf",
        s_hi[2] > OPENING_H + 0.20,
        details=f"surround top z={s_hi[2]:.3f}, opening h={OPENING_H:.3f}",
    )
    # Surround is wider than the clear opening (real jambs on each side).
    s_width = s_hi[0] - s_lo[0]
    ctx.check(
        "surround is wider than the opening (real jambs)",
        s_width > OPENING_W + 2 * JAMB_W - 0.05,
        details=f"surround width={s_width:.3f}, opening={OPENING_W:.3f}",
    )

    # --- Ring pull (separate part) is on the FREE (left) edge, proud of leaf ---
    with ctx.pose({hinge: 0.0, ring_joint: 0.0}):
        r_aabb = ctx.part_world_aabb(ring)
        r_lo, r_hi = r_aabb
        r_cx = (r_lo[0] + r_hi[0]) / 2.0
        # Free edge is the LEFT (world -X) edge of the closed leaf.
        leaf_lo, leaf_hi = ctx.part_world_aabb(door)
        free_edge_x = leaf_lo[0]   # leftmost X = free edge when closed
        ctx.check(
            "ring pull sits on the free (left) edge of the leaf",
            r_cx < (leaf_lo[0] + leaf_hi[0]) / 2.0,
            details=f"ring x={r_cx:.3f}, free edge x={free_edge_x:.3f}",
        )
        # Ring stands proud of the leaf front face (+Y).
        ctx.check(
            "ring pull stands proud of the leaf face",
            r_hi[1] > LEAF_T / 2.0,
            details=f"ring max y={r_hi[1]:.3f}, leaf face y={LEAF_T/2.0:.3f}",
        )

    # The ring hangs below its pivot (the boss is near leaf z mid-height).
    with ctx.pose({hinge: 0.0, ring_joint: 0.0}):
        rr_lo, rr_hi = ctx.part_world_aabb(ring)
        ring_top_z = rr_hi[2]
        boss_world_z = LEAF_BOTTOM_Z + LEAF_H * 0.50   # ring_z in world
        ctx.check(
            "ring hangs below the escutcheon boss",
            ring_top_z <= boss_world_z + 0.02,
            details=f"ring top z={ring_top_z:.3f}, boss z~{boss_world_z:.3f}",
        )

    # --- HERO: the ring rotates on its boss, tilting OUT off the door face ---
    with ctx.pose({hinge: 0.0, ring_joint: 0.0}):
        ring_rest = ctx.part_world_aabb(ring)
        rest_max_y = ring_rest[1][1]
        rest_bottom_z = ring_rest[0][2]
    with ctx.pose({hinge: 0.0, ring_joint: 1.0}):
        ring_lift = ctx.part_world_aabb(ring)
        lift_max_y = ring_lift[1][1]
        lift_bottom_z = ring_lift[0][2]
        # Lifting the ring swings its bottom OUT (toward +Y, off the door face)
        # and raises the bottom of the ring (it pivots up about the boss).
        ctx.check(
            "ring pivots out off the door face when lifted",
            lift_max_y > rest_max_y + 0.02,
            details=f"rest max y={rest_max_y:.3f}, lifted max y={lift_max_y:.3f}",
        )
        ctx.check(
            "ring bottom swings up about the boss when lifted",
            lift_bottom_z > rest_bottom_z + 0.02,
            details=f"rest bottom z={rest_bottom_z:.3f}, lifted bottom z={lift_bottom_z:.3f}",
        )

    # --- Slide bolt sits in its two keepers and slides between them ---
    # Both keepers are fixed to the leaf below the ring; the rod passes through.
    with ctx.pose({hinge: 0.0, bolt_joint: 0.0}):
        keep_aabb = ctx.part_element_world_aabb(door, elem="bolt_keepers")
        if keep_aabb is not None:
            k_lo, k_hi = keep_aabb
            # Keepers stand proud of the leaf front face (+Y) like the rod.
            ctx.check(
                "bolt keepers stand proud of the leaf face",
                k_hi[1] > LEAF_T / 2.0,
                details=f"keepers max y={k_hi[1]:.3f}",
            )
        bolt_rest = ctx.part_world_aabb(bolt)
        b_lo, b_hi = bolt_rest
        # The bolt rod is a horizontal bar (wide in X, short in Z).
        bolt_w = b_hi[0] - b_lo[0]
        bolt_h = b_hi[2] - b_lo[2]
        ctx.check(
            "bolt rod lies horizontal across the door",
            bolt_w > 0.20 and bolt_w > bolt_h,
            details=f"bolt w={bolt_w:.3f} h={bolt_h:.3f}",
        )
        rest_bolt_cx = (b_lo[0] + b_hi[0]) / 2.0

    # The rod stays engaged in BOTH keepers across the full travel (retained
    # insertion): at rest and at full throw the rod overlaps the keepers in X.
    with ctx.pose({hinge: 0.0, bolt_joint: 0.0}):
        ctx.expect_overlap(
            bolt, door,
            axes="x",
            elem_a="bolt_rod",
            elem_b="bolt_keepers",
            name="bolt rod stays engaged in keepers at rest",
        )

    # --- HERO: positive prismatic q throws the bolt along the leaf face ---
    with ctx.pose({hinge: 0.0, bolt_joint: BOLT_TRAVEL}):
        bolt_thrown = ctx.part_world_aabb(bolt)
        tb_lo, tb_hi = bolt_thrown
        thrown_bolt_cx = (tb_lo[0] + tb_hi[0]) / 2.0
        # The closed-door free edge is world -X, and leaf-local +X maps to world -X,
        # so throwing the bolt (local +X) moves it toward -X in world.
        ctx.check(
            "bolt slides when thrown (rod translates through the keepers)",
            abs(thrown_bolt_cx - rest_bolt_cx) > 0.05,
            details=f"rest cx={rest_bolt_cx:.3f}, thrown cx={thrown_bolt_cx:.3f}",
        )
    # Retained insertion still holds at full throw.
    with ctx.pose({hinge: 0.0, bolt_joint: BOLT_TRAVEL}):
        ctx.expect_overlap(
            bolt, door,
            axes="x",
            elem_a="bolt_rod",
            elem_b="bolt_keepers",
            name="bolt rod stays engaged in keepers at full throw",
        )

    # --- Strap hinges read on the leaf and sit toward the hinge (right) edge ---
    for idx in (0, 1):
        strap_aabb = ctx.part_element_world_aabb(door, elem=f"strap_hinge_{idx}")
        if strap_aabb is not None:
            st_lo, st_hi = strap_aabb
            # Straps stand proud of the front face like the ring.
            ctx.check(
                f"strap hinge {idx} stands proud of the leaf face",
                st_hi[1] > LEAF_T / 2.0,
                details=f"strap{idx} max y={st_hi[1]:.3f}",
            )

    # --- HERO: positive q opens the leaf outward (door swings open) ---
    with ctx.pose({hinge: 0.0}):
        closed = ctx.part_world_aabb(door)
        closed_cy = (closed[0][1] + closed[1][1]) / 2.0
        closed_free_x = closed[0][0]    # free edge x when closed
    open_q = 1.4
    with ctx.pose({hinge: open_q}):
        opened = ctx.part_world_aabb(door)
        opened_cy = (opened[0][1] + opened[1][1]) / 2.0
        opened_max_y = opened[1][1]
        # The leaf swings outward: its body moves to +Y (toward the viewer).
        ctx.check(
            "positive q swings the leaf outward (opens)",
            opened_cy > closed_cy + 0.20 and opened_max_y > closed[1][1] + 0.20,
            details=f"closed cy={closed_cy:.3f}, opened cy={opened_cy:.3f}",
        )
        # The leaf remains hinged at the right jamb: the hinge edge barely moves
        # in X while the free edge (ring side) swings away from the opening. The
        # ring is a child of the door, so it follows the leaf when it opens.
        opened_free = ctx.part_world_aabb(ring)
        if opened_free is not None:
            of_x = (opened_free[0][0] + opened_free[1][0]) / 2.0
            ctx.check(
                "free edge (ring side) swings away from the opening when open",
                of_x > closed_free_x + 0.15,
                details=f"closed free x={closed_free_x:.3f}, opened ring x={of_x:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
