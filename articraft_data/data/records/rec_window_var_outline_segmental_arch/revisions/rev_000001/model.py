from __future__ import annotations

# Side-hung wooden casement window with SEGMENTAL-ARCH head, swung open, with
# visible brass butt hinges and a brass casement stay arm.
#
# Variant of the plain rectangular casement: the frame head is a smooth
# segmental arc spanning the full width while the jambs and sill remain
# straight.  The sash top rail follows the same concentric curve so the
# operable leaf still seats cleanly in the curved frame rebate.
#
# Coordinate convention (Z up):
#   - Height runs along +Z (sill at z~0, head near the top).
#   - Width runs along X.
#   - Frame depth / glazing thickness / swing-normal runs along Y.
#   - The sash is side-hung on the LEFT jamb (hinge line is the left vertical
#     edge). At q=0 the sash is shut, coplanar with the frame. Positive q
#     swings the free (right) edge outward toward +Y.
#
# Structure:
#   - frame (static root): segmental-arch hollow wood profile built with
#     CadQuery threePointArc for the curved head, inner opening follows a
#     concentric arc.
#   - sash (moving): arch-shaped wood frame ring + thin tinted glass pane,
#     authored in a hinge-local frame.
#   - two brass butt hinges (BarrelHingeGeometry) on the hinge jamb.
#   - a slim brass stay bar linking the sash to the frame head.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    Cylinder,
    HingeHolePattern,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

WIN_W = 0.90          # overall window width (X)
WIN_H = 1.20          # overall window height (Z, sill to arch crown)
FRAME_FACE = 0.060    # outer frame member face width (in-plane, X/Z)
FRAME_DEPTH = 0.070   # outer frame depth (Y)

ARCH_RISE = 0.12      # segmental arch rise above the spring line

SILL_Z = 0.0          # bottom of outer frame at z=0
SPRING_Z = WIN_H - ARCH_RISE   # height where the arch curve begins (1.08)

# Shared arc geometry — all arch profiles use a concentric arc center so the
# frame head, opening head, sash head, and glass head follow the same curve.
_hw_outer = WIN_W / 2.0
_d_outer = (_hw_outer ** 2 - ARCH_RISE ** 2) / (2.0 * ARCH_RISE)
_R_outer = _d_outer + ARCH_RISE
_arc_center_z = SPRING_Z - _d_outer   # world Z of the arc center


def _rise_for(half_w: float) -> float:
    """Arch rise above the spring line for a given half-span, using the shared
    concentric arc center."""
    R = math.sqrt(half_w ** 2 + _d_outer ** 2)
    return (_arc_center_z + R) - SPRING_Z


# Opening (clear span inside the outer frame, at the spring line).
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = SILL_Z + FRAME_FACE
OPENING_W = OPEN_X1 - OPEN_X0
_open_hw = OPENING_W / 2.0
_inner_rise = _rise_for(_open_hw)

# Sash: sits in the opening, slightly inset.
SASH_GAP = 0.006
SASH_FACE = 0.055
SASH_DEPTH = 0.050
SASH_W = OPENING_W - 2 * SASH_GAP
_sash_hw = SASH_W / 2.0
SASH_BOTTOM_Z = OPEN_Z0 + SASH_GAP
_sash_local_spring = SPRING_Z - SASH_BOTTOM_Z  # spring line in sash local Z
_sash_rise = _rise_for(_sash_hw)
SASH_H = _sash_local_spring + _sash_rise        # sash total height at crown (local)

# Hinge edge world X.
HINGE_X = OPEN_X0 + SASH_GAP

# Glass.
GLASS_T = 0.008
GLASS_REBATE = 0.006

# Brass butt hinge geometry.
HINGE_LEN = 0.110
HINGE_LEAF = 0.022
HINGE_LEAF_T = 0.0028
HINGE_PIN_D = 0.006
HINGE_KNUCKLE_D = 0.0125

# Stay bar.
STAY_W = 0.018
STAY_T = 0.006
STAY_LEN = 0.230

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

WOOD_RGBA = (0.255, 0.145, 0.075, 1.0)
GLASS_RGBA = (0.16, 0.17, 0.22, 0.38)
BRASS_RGBA = (0.80, 0.62, 0.22, 1.0)


# ---------------------------------------------------------------------------
# Segmental-arch solid builder
# ---------------------------------------------------------------------------

def _arch_solid(
    half_w: float,
    bottom_z: float,
    spring_z: float,
    rise: float,
    depth: float,
    cx: float = 0.0,
) -> cq.Workplane:
    """Build a segmental-arch profile solid, centered on Y=0.

    The 2-D profile is drawn on the XY workplane (X horizontal, Y = height),
    extruded along +Z by *depth*, then rotated 90° around the X axis so that
    the profile Y becomes world Z and the extrusion Z becomes world -Y.
    A final Y-translation centers the solid on Y=0.

    Parameters
    ----------
    half_w : half the chord span.
    bottom_z : Z of the bottom edge (sill / lower rail).
    spring_z : Z where the arch curve starts (top of straight jambs).
    rise : height of the arc above the spring line.
    depth : extrusion depth along Y.
    cx : center-X offset for profiles that are not centered on X=0
         (e.g. the sash in its hinge-local frame).
    """
    crown_z = spring_z + rise
    return (
        cq.Workplane("XY")
        .moveTo(cx - half_w, bottom_z)
        .lineTo(cx + half_w, bottom_z)           # sill / bottom rail
        .lineTo(cx + half_w, spring_z)            # right jamb / stile
        .threePointArc((cx, crown_z), (cx - half_w, spring_z))  # arch head
        .close()                                  # left jamb / stile
        .extrude(depth)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0, depth / 2.0, 0))
    )


# ---------------------------------------------------------------------------
# Frame geometry
# ---------------------------------------------------------------------------

def _build_outer_frame() -> cq.Workplane:
    """Static wood outer frame: segmental-arch shell with concentric opening."""
    outer = _arch_solid(_hw_outer, 0.0, SPRING_Z, ARCH_RISE, FRAME_DEPTH)
    inner = _arch_solid(
        _open_hw, OPEN_Z0, SPRING_Z, _inner_rise, FRAME_DEPTH + 0.02
    )
    return outer.cut(inner)


# ---------------------------------------------------------------------------
# Sash geometry (hinge-local frame)
#   local x: 0 at hinge edge .. SASH_W at free edge
#   local z: 0 at sash bottom .. SASH_H at crown
#   local y: centered on 0
# ---------------------------------------------------------------------------

_sash_cx = _sash_hw  # sash profile center-X in local frame


def _build_sash_frame() -> cq.Workplane:
    """Sash wood frame ring with segmental-arch head, glass opening hollow."""
    outer = _arch_solid(
        _sash_hw, 0.0, _sash_local_spring, _sash_rise, SASH_DEPTH, _sash_cx
    )
    ghw = _sash_hw - SASH_FACE
    grise = _rise_for(ghw)
    inner = _arch_solid(
        ghw, SASH_FACE, _sash_local_spring, grise, SASH_DEPTH + 0.02, _sash_cx
    )
    return outer.cut(inner)


def _build_sash_glass() -> cq.Workplane:
    """Thin tinted glass pane following the arch head, rebated under sash lip."""
    ghw = _sash_hw - SASH_FACE + GLASS_REBATE
    gb = SASH_FACE - GLASS_REBATE
    grise = _rise_for(ghw)
    return _arch_solid(
        ghw, gb, _sash_local_spring, grise, GLASS_T, _sash_cx
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="segmental_arch_casement")

    model.material("wood", rgba=WOOD_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("brass", rgba=BRASS_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_outer_frame(), "frame"),
        material="wood",
        name="frame_shell",
    )

    # --- Operable sash (arch frame ring + tinted glass) ---
    sash = model.part("sash")
    sash.visual(
        mesh_from_cadquery(_build_sash_frame(), "sash_frame"),
        material="wood",
        name="sash_frame",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass(), "sash_glass"),
        material="glass",
        name="sash_glass",
    )

    # --- Brass butt hinges on the hinge (left) edge, two down the jamb ---
    hinge_geom = BarrelHingeGeometry(
        HINGE_LEN,
        leaf_width_a=HINGE_LEAF,
        leaf_width_b=HINGE_LEAF,
        leaf_thickness=HINGE_LEAF_T,
        pin_diameter=HINGE_PIN_D,
        knuckle_outer_diameter=HINGE_KNUCKLE_D,
        knuckle_count=5,
        open_angle_deg=90.0,
        holes_a=HingeHolePattern(
            style="round", count=3, diameter=0.003, edge_margin=0.006
        ),
        holes_b=HingeHolePattern(
            style="round", count=3, diameter=0.003, edge_margin=0.006
        ),
    )
    hinge_y = -(SASH_DEPTH / 2.0)
    # Place hinges on the straight jamb (below the spring line).
    jamb_h = _sash_local_spring - SASH_FACE
    for i, frac in enumerate((0.80, 0.20)):
        z = SASH_FACE / 2.0 + frac * jamb_h
        sash.visual(
            mesh_from_geometry(hinge_geom, f"hinge_{i}"),
            origin=Origin(xyz=(0.0, hinge_y, z)),
            material="brass",
            name=f"hinge_{i}",
        )

    # --- Brass casement stay bar ---
    stay = Box((STAY_LEN, STAY_T, STAY_W))
    stay_anchor_x = SASH_W * 0.30
    stay_z = SASH_H - SASH_FACE / 2.0
    stay_y = -(SASH_DEPTH / 2.0)
    sash.visual(
        stay,
        origin=Origin(xyz=(stay_anchor_x, stay_y, stay_z), rpy=(0.0, 0.0, 0.0)),
        material="brass",
        name="stay_bar",
    )
    sash.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(
            xyz=(stay_anchor_x - STAY_LEN / 2.0, stay_y, stay_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="brass",
        name="stay_pivot",
    )

    # ----- Articulation -----
    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="sash",
        origin=Origin(xyz=(HINGE_X, 0.0, SASH_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.5),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

OPEN_POSE = 0.78  # radians, the photo's swung-open pose


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    hinge = object_model.get_articulation("frame_to_sash")

    # --- Intentional overlaps ---
    ctx.allow_overlap(
        sash, sash,
        elem_a="sash_glass",
        elem_b="sash_frame",
        reason="Tinted glass pane is rebated under the sash wood frame lip.",
    )
    ctx.allow_overlap(
        frame, sash,
        reason="Side-hung sash hinges at the left jamb; brass butt hinges and sash jamb meet the frame jamb at the hinge line.",
    )
    for i in range(2):
        ctx.allow_overlap(
            sash, sash,
            elem_a=f"hinge_{i}",
            elem_b="sash_frame",
            reason="Butt hinge leaf is screwed onto the sash jamb.",
        )
    ctx.allow_overlap(
        sash, sash,
        elem_a="stay_bar",
        elem_b="sash_frame",
        reason="Brass stay bar is mounted onto the sash top rail.",
    )
    ctx.allow_overlap(
        sash, sash,
        elem_a="stay_pivot",
        elem_b="sash_frame",
        reason="Stay pivot stud seats into the sash top rail.",
    )

    # --- Frame proportions and arch head ---
    frame_aabb = ctx.part_world_aabb(frame)
    with ctx.pose({hinge: 0.0}):
        sash_aabb = ctx.part_world_aabb(sash)
        fw = frame_aabb[1][0] - frame_aabb[0][0]
        fh = frame_aabb[1][2] - frame_aabb[0][2]
        sw = sash_aabb[1][0] - sash_aabb[0][0]
        sh = sash_aabb[1][2] - sash_aabb[0][2]

        ctx.check(
            "frame is wider than the sash",
            fw > sw + 0.04,
            details=f"frame_w={fw:.3f}, sash_w={sw:.3f}",
        )
        ctx.check(
            "frame is taller than the sash",
            fh > sh + 0.04,
            details=f"frame_h={fh:.3f}, sash_h={sh:.3f}",
        )
        ctx.check(
            "frame sits on the floor (z>=0)",
            frame_aabb[0][2] >= -1e-4 and frame_aabb[0][2] < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "window is taller than wide (stands upright)",
            fh > fw,
            details=f"frame_h={fh:.3f}, frame_w={fw:.3f}",
        )

        # --- Segmental-arch specific checks ---
        # Frame crown is above the spring line by more than one frame face
        # (proves the head is a real arch, not a flat lintel).
        ctx.check(
            "frame head has segmental arch crown above spring line",
            frame_aabb[1][2] > SPRING_Z + FRAME_FACE + 0.01,
            details=(
                f"frame zmax={frame_aabb[1][2]:.3f}, "
                f"spring_z+face={SPRING_Z + FRAME_FACE:.3f}"
            ),
        )
        # The arch crown reaches the nominal window height.
        ctx.check(
            "frame arch crown reaches window height",
            abs(frame_aabb[1][2] - WIN_H) < 0.01,
            details=f"frame zmax={frame_aabb[1][2]:.3f}, WIN_H={WIN_H:.3f}",
        )

        # Closed sash is shallow in Y (seated in frame plane).
        sash_dy = sash_aabb[1][1] - sash_aabb[0][1]
        ctx.check(
            "closed sash is shallow in Y (seated in frame plane)",
            sash_dy < 0.12,
            details=f"sash Y depth={sash_dy:.3f}",
        )

        # Closed sash stays within frame width.
        ctx.expect_within(
            sash, frame,
            axes="x",
            inner_elem="sash_frame",
            margin=0.02,
            name="closed sash stays within the frame width",
        )

        # Sash arch: sash frame extends above the spring line (follows the arch).
        ctx.check(
            "sash frame follows arch above spring line",
            sash_aabb[1][2] > SPRING_Z,
            details=f"sash zmax={sash_aabb[1][2]:.3f}, spring_z={SPRING_Z:.3f}",
        )

        # Sash crown stays below the frame arch crown (seated in the rebate).
        ctx.check(
            "sash arch crown seats below frame arch crown",
            sash_aabb[1][2] < frame_aabb[1][2],
            details=f"sash zmax={sash_aabb[1][2]:.3f}, frame zmax={frame_aabb[1][2]:.3f}",
        )

        closed_free_y = sash_aabb[1][1]
        closed_free_x = sash_aabb[1][0]

    # --- Open-pose swing checks ---
    with ctx.pose({hinge: OPEN_POSE}):
        open_aabb = ctx.part_world_aabb(sash)
        open_free_y = open_aabb[1][1]
        open_hinge_x_min = open_aabb[0][0]
        ctx.check(
            "open sash free edge swings out toward +Y",
            open_free_y > closed_free_y + 0.20,
            details=f"closed max_y={closed_free_y:.3f}, open max_y={open_free_y:.3f}",
        )
        ctx.check(
            "hinge edge stays anchored at the left jamb",
            abs(open_hinge_x_min - HINGE_X) < 0.06,
            details=f"open hinge xmin={open_hinge_x_min:.3f}, hinge_x={HINGE_X:.3f}",
        )
        ctx.check(
            "open sash free edge rotates inward in X",
            open_aabb[1][0] < closed_free_x - 0.05,
            details=f"closed free x={closed_free_x:.3f}, open free x={open_aabb[1][0]:.3f}",
        )

    # --- Brass hinges on the hinge (left) edge ---
    h0 = ctx.part_element_world_aabb(sash, elem="hinge_0")
    if h0 is not None:
        with ctx.pose({hinge: 0.0}):
            h0c = ctx.part_element_world_aabb(sash, elem="hinge_0")
            h0_x = (h0c[0][0] + h0c[1][0]) / 2.0
            ctx.check(
                "brass hinge sits on the left (hinge) edge",
                abs(h0_x - HINGE_X) < 0.05,
                details=f"hinge_0 x center={h0_x:.3f}, hinge_x={HINGE_X:.3f}",
            )

    # --- Stay bar near top, off the glass plane ---
    stay_aabb = ctx.part_element_world_aabb(sash, elem="stay_bar")
    if stay_aabb is not None:
        with ctx.pose({hinge: 0.0}):
            sa = ctx.part_element_world_aabb(sash, elem="stay_bar")
            stay_y = (sa[0][1] + sa[1][1]) / 2.0
            stay_z = (sa[0][2] + sa[1][2]) / 2.0
            ctx.check(
                "stay bar extends proud of the sash outer face (-Y)",
                sa[0][1] < -SASH_DEPTH / 2.0,
                details=f"stay ymin={sa[0][1]:.3f}, sash face=-{SASH_DEPTH / 2.0:.3f}",
            )
            ctx.check(
                "stay bar is near the top of the sash",
                stay_z > SASH_BOTTOM_Z + 0.6 * SASH_H,
                details=f"stay Z center={stay_z:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
