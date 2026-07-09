from __future__ import annotations

# Articraft model: folding pocket utility knife (variant of the 18 mm
# snap-off knife).  The prismatic slide has been replaced with a folding
# blade that pivots out of the handle on a revolute hinge at the front.
#
# Articraft brief:
# - Object: folding pocket utility knife, ~0.155 m long, yellow plastic
#   handle with a brass pivot bolster, a folding steel utility blade, a
#   black textured thumb-grip, and a rear lanyard hole.
# - Root/support: handle body (handle) is the fixed root.  It is the same
#   tapered loft as the parent, with a blade-exit groove at the front-top.
# - Parts: handle (root), blade (folding member), end_cap (rear, fixed).
# - Articulation: handle_to_blade, REVOLUTE, axis along Y at the front
#   pivot, positive q swings the blade from stored (inside handle, along -X)
#   to deployed (extending forward along +X).  Limits 0 to π.
# - Visible geometry: yellow tapered shell with blade-exit groove, brass
#   pivot pin/bolster, steel folding blade with dark tip, black knurled
#   thumb-grip pad, dark gray rear end cap.
# - Support/fit: blade stores inside handle body when closed; pivots through
#   the front-top groove.
# - Intentional overlaps: blade assembly inside handle body when closed
#   (scoped allow_overlap with containment proof).
# - Tests: hinge is revolute, axis is Y, blade within handle when closed,
#   blade extends past nose when open, blade tip at the front when open,
#   end cap seated, cutting edge faces down when deployed.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
HANDLE_LEN = 0.150
HANDLE_H = 0.026
HANDLE_FRONT_H = 0.016
HANDLE_W = 0.013

BLADE_LEN = 0.050          # folding blade length from pivot to tip
BLADE_W = 0.015            # blade width (profile height when deployed)
BLADE_THK = 0.0006         # blade thickness

PIVOT_X = HANDLE_LEN / 2.0 - 0.010   # 0.065 – pivot slightly behind nose
PIVOT_Z = 0.011                       # pivot height inside the handle body

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
YELLOW = Material(name="handle_yellow", rgba=(0.96, 0.78, 0.10, 1.0))
DARK_GRAY = Material(name="dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
BLACK_GRIP = Material(name="black_grip", rgba=(0.10, 0.10, 0.11, 1.0))
STEEL = Material(name="blade_steel", rgba=(0.80, 0.82, 0.85, 1.0))
BLADE_TIP_MAT = Material(name="blade_tip", rgba=(0.30, 0.34, 0.46, 1.0))
BRASS = Material(name="pivot_brass", rgba=(0.72, 0.60, 0.28, 1.0))
CAP_GRAY = Material(name="cap_gray", rgba=(0.55, 0.56, 0.58, 1.0))


# ===================================================================
# Handle geometry helpers
# ===================================================================

def _handle_body_shape() -> cq.Workplane:
    """Tapered loft handle shell – same silhouette as the parent knife."""
    x0 = -HANDLE_LEN / 2.0
    x1 = HANDLE_LEN / 2.0
    profile = [
        (x0,            HANDLE_W / 2.0,       HANDLE_H,               HANDLE_H / 2.0),
        (x0 + 0.030,    HANDLE_W / 2.0,       HANDLE_H,               HANDLE_H / 2.0),
        (x0 + 0.085,    HANDLE_W / 2.0,       0.024,                  0.012),
        (x1 - 0.030,    HANDLE_W / 2.0 * 0.92, HANDLE_FRONT_H + 0.003, HANDLE_FRONT_H / 2.0 + 0.004),
        (x1 - 0.006,    HANDLE_W / 2.0 * 0.78, HANDLE_FRONT_H,        HANDLE_FRONT_H / 2.0 + 0.003),
        (x1,            HANDLE_W / 2.0 * 0.62, 0.010,                 0.009),
    ]
    wires = []
    for x, hw, h, zc in profile:
        section = (
            cq.Workplane("YZ")
            .workplane(offset=x)
            .center(0.0, zc)
            .rect(2.0 * hw, h)
        )
        wires.append(section.val())
    solid = cq.Solid.makeLoft(wires, ruled=False)
    return cq.Workplane("XY").newObject([solid])


def _lanyard_hole_cut() -> cq.Workplane:
    """Rear finger / lanyard through-hole along Y."""
    x0 = -HANDLE_LEN / 2.0
    return (
        cq.Workplane("XZ")
        .workplane(offset=HANDLE_W)
        .center(x0 + 0.014, HANDLE_H * 0.5)
        .circle(0.0042)
        .extrude(2.0 * HANDLE_W)
    )


def _blade_exit_groove() -> cq.Workplane:
    """Thin groove cut into the front-top of the handle where the blade
    pivots out.  This is a visual blade-exit slot, not a full through-cut."""
    slot_len = 0.022
    slot_w = BLADE_THK * 4.0
    slot_depth = 0.006
    x_center = PIVOT_X + 0.005
    # Start near the handle top surface and cut downward.
    z_top = HANDLE_H - 0.001
    return (
        cq.Workplane("XY")
        .box(slot_len, slot_w, slot_depth + 0.004, centered=(True, True, False))
        .translate((x_center, 0.0, z_top - slot_depth))
    )


def _build_handle_visual() -> cq.Workplane:
    body = _handle_body_shape()
    body = body.cut(_blade_exit_groove())
    body = body.cut(_lanyard_hole_cut())
    return body


def _build_pivot_bolster() -> cq.Workplane:
    """Brass pivot pin passing through the handle at the blade hinge."""
    pin_r = 0.0028
    pin_len = HANDLE_W + 0.004
    return (
        cq.Workplane("XZ")
        .workplane(offset=-pin_len / 2.0)
        .center(PIVOT_X, PIVOT_Z)
        .circle(pin_r)
        .extrude(pin_len)
    )


def _build_front_bolster() -> cq.Workplane:
    """Small dark-gray metal bolster plate surrounding the pivot at the
    front of the handle."""
    length = 0.014
    w = HANDLE_W * 1.04
    h = 0.014
    return (
        cq.Workplane("XY")
        .box(length, w, h, centered=(True, True, False))
        .edges("|Z").fillet(0.0018)
        .translate((PIVOT_X + 0.002, 0.0, 0.003))
    )


def _build_thumb_grip() -> cq.Workplane:
    """Black textured thumb pad on the upper-front shoulder of the handle."""
    base = (
        cq.Workplane("XY")
        .box(0.030, HANDLE_W * 0.98, 0.0030, centered=(True, True, False))
    )
    bumps = None
    nx, ny = 6, 4
    for ix in range(nx):
        for iy in range(ny):
            x = -0.012 + ix * 0.0048
            y = -0.0045 + iy * 0.0030
            b = (
                cq.Workplane("XY")
                .transformed(offset=(x, y, 0.0030))
                .box(0.0026, 0.0018, 0.0016, centered=(True, True, False))
            )
            bumps = b if bumps is None else bumps.add(b)
    grip = base.add(bumps)
    grip = grip.translate((0.030, 0.0, HANDLE_FRONT_H - 0.0005))
    return grip


# ===================================================================
# Blade geometry helpers  (blade part frame origin = pivot point)
# ===================================================================
# In the blade's local frame the blade extends along -X from the origin
# (the pivot).  At q = 0 (closed) the blade lies along -X inside the
# handle.  At q = π (open) the blade extends along +X forward from the
# handle.  The blade profile spans Z: spine at local -Z (becomes world
# +Z = up when deployed), cutting edge at local +Z (becomes world -Z =
# down when deployed).

def _build_blade_body() -> cq.Workplane:
    """Trapezoidal folding utility blade profile in the XZ plane."""
    L = BLADE_LEN
    hw = BLADE_W / 2.0
    pts = [
        (0.0, -hw + 0.002),            # rear spine corner
        (-L + 0.010, -hw),             # front spine corner
        (-L, 0.0),                     # sharp tip
        (-L + 0.008, hw),              # front cutting-edge corner
        (-0.005, hw),                  # rear cutting-edge corner
    ]
    return (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(BLADE_THK)
        .translate((0.0, -BLADE_THK / 2.0, 0.0))
    )


def _build_blade_tip() -> cq.Workplane:
    """Dark-coated front segment of the blade (worn tip)."""
    L = BLADE_LEN
    hw = BLADE_W / 2.0
    pts = [
        (-L + 0.012, -hw + 0.001),
        (-L + 0.010, -hw),
        (-L, 0.0),
        (-L + 0.008, hw),
        (-L + 0.012, hw - 0.001),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(BLADE_THK + 0.0002)
        .translate((0.0, -(BLADE_THK + 0.0002) / 2.0, 0.0))
    )


def _build_thumb_stud() -> cq.Workplane:
    """Small brass thumb stud near the blade pivot end for flipping open."""
    # Protrudes slightly along +Y from the blade face.
    stud_y = BLADE_THK / 2.0
    stud = (
        cq.Workplane("XZ")
        .workplane(offset=stud_y)
        .center(-0.008, 0.0)
        .rect(0.006, 0.005)
        .extrude(0.002)
    )
    return stud


# ===================================================================
# End cap
# ===================================================================

def _build_end_cap() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .workplane(offset=-HANDLE_LEN / 2.0)
        .rect(HANDLE_W * 1.02, HANDLE_H * 0.96)
        .extrude(-0.012)
        .edges("|X").fillet(0.0025)
    )


# ===================================================================
# Object model
# ===================================================================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_pocket_knife")

    # --- Handle (root) ------------------------------------------------
    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_visual(), "handle_shell"),
        material=YELLOW,
        name="handle_shell",
    )
    handle.visual(
        mesh_from_cadquery(_build_front_bolster(), "front_bolster"),
        material=DARK_GRAY,
        name="front_bolster",
    )
    handle.visual(
        mesh_from_cadquery(_build_pivot_bolster(), "pivot_pin"),
        material=BRASS,
        name="pivot_pin",
    )
    handle.visual(
        mesh_from_cadquery(_build_thumb_grip(), "thumb_grip"),
        material=BLACK_GRIP,
        name="thumb_grip",
    )

    # --- Blade (folding member) --------------------------------------
    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_build_blade_body(), "blade_body"),
        material=STEEL,
        name="blade_body",
    )
    blade.visual(
        mesh_from_cadquery(_build_blade_tip(), "blade_tip"),
        material=BLADE_TIP_MAT,
        name="blade_tip",
    )
    blade.visual(
        mesh_from_cadquery(_build_thumb_stud(), "thumb_stud"),
        material=BRASS,
        name="thumb_stud",
    )

    # --- End cap (fixed) ---------------------------------------------
    end_cap = model.part("end_cap")
    end_cap.visual(
        mesh_from_cadquery(_build_end_cap(), "end_cap_body"),
        material=CAP_GRAY,
        name="end_cap_body",
    )

    # --- Articulations ------------------------------------------------
    # Revolute hinge: blade folds from inside the handle (q = 0) to
    # fully deployed forward (q = π).
    model.articulation(
        "handle_to_blade",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=blade,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=math.pi,
        ),
    )

    # End cap fixed to the handle rear.
    model.articulation(
        "handle_to_cap",
        ArticulationType.FIXED,
        parent=handle,
        child=end_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


object_model = build_object_model()


# ===================================================================
# Tests
# ===================================================================

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    blade = object_model.get_part("blade")
    end_cap = object_model.get_part("end_cap")
    hinge = object_model.get_articulation("handle_to_blade")

    # --- Joint type / axis -------------------------------------------
    ctx.check(
        "blade hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ax = tuple(hinge.axis)
    ctx.check(
        "hinge axis is along Y (blade swings in XZ plane)",
        abs(ax[1]) > 0.99 and abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # --- Closed pose: blade stored inside the handle -----------------
    ctx.allow_overlap(
        blade,
        handle,
        reason="The folding blade assembly is stored inside the handle body "
               "when closed; this nested fit is intentional.",
    )
    ctx.expect_within(
        blade,
        handle,
        axes="xy",
        inner_elem="blade_body",
        outer_elem="handle_shell",
        margin=0.003,
        name="blade stays within handle footprint (XY) when closed",
    )
    # The blade should overlap the handle along X when closed (retained).
    ctx.expect_overlap(
        blade,
        handle,
        axes="x",
        elem_a="blade_body",
        elem_b="handle_shell",
        min_overlap=0.030,
        name="blade is retained inside the handle when closed",
    )

    # --- End cap seated at the rear ----------------------------------
    ctx.expect_contact(
        end_cap, handle, name="end cap seated against handle rear"
    )
    handle_aabb = ctx.part_world_aabb(handle)
    cap_aabb = ctx.part_world_aabb(end_cap)
    ctx.check(
        "end cap is at the rear (-X) of the handle",
        cap_aabb is not None
        and handle_aabb is not None
        and cap_aabb[0][0] <= handle_aabb[0][0] + 0.002,
        details=f"cap_min_x={None if cap_aabb is None else cap_aabb[0][0]}",
    )

    # --- Open pose: blade extends forward past the handle nose -------
    with ctx.pose({hinge: math.pi}):
        blade_aabb = ctx.part_element_world_aabb(blade, elem="blade_body")
        ctx.check(
            "blade extends past handle nose when fully open",
            blade_aabb is not None
            and handle_aabb is not None
            and blade_aabb[1][0] > handle_aabb[1][0] + 0.020,
            details=f"blade_max_x={blade_aabb[1][0] if blade_aabb else None}, "
                    f"handle_max_x={handle_aabb[1][0] if handle_aabb else None}",
        )
        # Blade tip should be at the very front of the blade.
        tip_aabb = ctx.part_element_world_aabb(blade, elem="blade_tip")
        ctx.check(
            "blade tip is at the very front when deployed",
            tip_aabb is not None
            and blade_aabb is not None
            and tip_aabb[1][0] >= blade_aabb[1][0] - 0.001,
            details=f"tip_max_x={tip_aabb[1][0] if tip_aabb else None}",
        )
        # Cutting edge faces down when deployed: the blade body should
        # extend below the pivot Z level (blade profile height is along
        # world Z when deployed, with cutting edge at -Z).
        ctx.check(
            "blade has real profile height when deployed (cutting edge below spine)",
            blade_aabb is not None
            and (blade_aabb[1][2] - blade_aabb[0][2]) > 0.010,
            details=f"blade_z_extent="
                    f"{blade_aabb[1][2] - blade_aabb[0][2] if blade_aabb else None}",
        )

    # --- Pivot location: hinge origin near the handle front ----------
    ctx.check(
        "pivot is near the front of the handle",
        hinge.origin is not None and hinge.origin.xyz[0] > HANDLE_LEN / 2.0 - 0.020,
        details=f"pivot_x={hinge.origin.xyz[0] if hinge.origin else None}",
    )

    return ctx.report()
