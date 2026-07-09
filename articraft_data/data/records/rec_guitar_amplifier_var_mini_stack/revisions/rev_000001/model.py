from __future__ import annotations

# Marshall-style mini guitar amplifier stack (half-stack variant).
# Two separate black vinyl enclosures stacked vertically:
#   - Lower (root): speaker cabinet with perforated grille, white piping, logo,
#     corner caps.
#   - Upper (child): amplifier head with recessed gold control panel, four
#     knurled rotary knobs (CONTINUOUS joints), and a carry handle.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobTopFeature,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- speaker cabinet dimensions (meters) ----
SPEAKER_W = 0.180
SPEAKER_D = 0.100
SPEAKER_H = 0.130

# ---- head box dimensions ----
HEAD_W = 0.180
HEAD_D = 0.100
HEAD_H = 0.070

# ---- gold control panel (on head top face) ----
PANEL_RECESS = 0.012
PANEL_W = 0.150
PANEL_D = 0.060
PANEL_X = 0.012
PANEL_GOLD_Z = HEAD_H - PANEL_RECESS

# ---- knobs ----
KNOB_COUNT = 4
KNOB_DIAM = 0.018
KNOB_H = 0.014
KNOB_YS = tuple(-0.054 + i * 0.036 for i in range(KNOB_COUNT))
KNOB_X = PANEL_X + 0.004

# ---- front face reference ----
SPEAKER_FRONT_X = SPEAKER_D / 2.0


def _rounded_box_mesh(w_y: float, d_x: float, h_z: float, fillet: float, z_offset: float = 0.0):
    """CadQuery rounded box (vertical edges filleted), optionally offset in Z."""
    wp = cq.Workplane("XY").box(d_x, w_y, h_z)
    try:
        wp = wp.edges("|Z").fillet(min(fillet, min(d_x, w_y) / 3.0))
    except Exception:
        pass
    if z_offset != 0.0:
        wp = wp.translate((0.0, 0.0, z_offset))
    return mesh_from_cadquery(wp, f"rounded_box_{id(wp)}")


def _make_knob_geometry() -> KnobGeometry:
    """Shared knurled rotary knob with indicator line and raised pointer tab."""
    knob_geo = KnobGeometry(
        KNOB_DIAM,
        KNOB_H,
        body_style="cylindrical",
        edge_radius=0.0008,
        grip=KnobGrip(style="knurled", count=28, depth=0.0008),
        indicator=KnobIndicator(style="line", mode="raised",
                                length=0.009, width=0.0014, depth=0.0010),
        top_feature=KnobTopFeature(style="recessed_disk", diameter=0.010, depth=0.0008),
        center=False,
    )
    pointer = BoxGeometry((0.0030, 0.0060, 0.0024))
    pointer.translate(0.0, KNOB_DIAM / 2.0 - 0.0010, KNOB_H + 0.0008)
    knob_geo.merge(pointer)
    return knob_geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_guitar_amp_stack")

    # ---- materials ----
    black_vinyl = model.material("black_vinyl", rgba=(0.07, 0.07, 0.08, 1.0))
    gold_panel = model.material("gold_panel", rgba=(0.86, 0.62, 0.18, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    white_piping = model.material("white_piping", rgba=(0.92, 0.92, 0.90, 1.0))
    logo_cream = model.material("logo_cream", rgba=(0.95, 0.93, 0.86, 1.0))
    metal_knob = model.material("metal_knob", rgba=(0.78, 0.78, 0.80, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.85, 0.12, 0.10, 1.0))
    trim_black = model.material("trim_black", rgba=(0.04, 0.04, 0.05, 1.0))

    # ========================================= speaker cabinet (root)
    speaker = model.part("speaker_cabinet")

    # Cabinet shell (rounded vinyl box)
    speaker.visual(
        _rounded_box_mesh(SPEAKER_W, SPEAKER_D, SPEAKER_H, fillet=0.008),
        material=black_vinyl, name="cabinet_shell",
    )

    # Front perforated speaker grille
    # After rotate_y(pi/2): local X -> world Z, local Y -> world Y.
    # So panel_size[0] becomes height (Z) and panel_size[1] becomes width (Y).
    grille = PerforatedPanelGeometry(
        (SPEAKER_H - 0.030, SPEAKER_W - 0.030),
        0.006,
        hole_diameter=0.0035,
        pitch=0.007,
        frame=0.006,
        stagger=True,
    )
    grille.rotate_y(math.pi / 2.0)
    grille.translate(SPEAKER_FRONT_X - 0.004, 0.0, 0.0)
    speaker.visual(
        mesh_from_geometry(grille, "speaker_grille"),
        material=grille_dark, name="speaker_grille",
    )

    # White piping frame around grille
    pip_t = 0.005
    pip_w = 0.006
    fx = SPEAKER_FRONT_X - 0.001
    fw = SPEAKER_W - 0.020
    fh = SPEAKER_H - 0.020
    for i, zc in enumerate((fh / 2.0, -fh / 2.0)):
        bar = BoxGeometry((pip_t, fw, pip_w))
        bar.translate(fx, 0.0, zc)
        speaker.visual(
            mesh_from_geometry(bar, f"piping_h_{i}"),
            material=white_piping, name=f"piping_h_{i}",
        )
    for i, yc in enumerate((fw / 2.0, -fw / 2.0)):
        bar = BoxGeometry((pip_t, pip_w, fh))
        bar.translate(fx, yc, 0.0)
        speaker.visual(
            mesh_from_geometry(bar, f"piping_v_{i}"),
            material=white_piping, name=f"piping_v_{i}",
        )

    # Cream logo plate
    logo = BoxGeometry((0.004, 0.080, 0.022))
    logo.translate(SPEAKER_FRONT_X + 0.001, -0.006, -0.040)
    speaker.visual(
        mesh_from_geometry(logo, "marshall_logo"),
        material=logo_cream, name="marshall_logo",
    )

    # Corner caps
    cap_y = (SPEAKER_W / 2.0) - 0.006
    cap_z = (SPEAKER_H / 2.0) - 0.006
    for iy, yc in enumerate((-cap_y, cap_y)):
        for iz, zc in enumerate((-cap_z, cap_z)):
            cap = BoxGeometry((0.018, 0.016, 0.016))
            cap.translate(SPEAKER_FRONT_X - 0.006, yc, zc)
            speaker.visual(
                mesh_from_geometry(cap, f"corner_cap_{iy}_{iz}"),
                material=trim_black, name=f"corner_cap_{iy}_{iz}",
            )

    speaker.inertial = Inertial.from_geometry(
        Box((SPEAKER_D, SPEAKER_W, SPEAKER_H)), mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ========================================= head box
    head = model.part("head_box")

    # Head shell (rounded vinyl box, origin at bottom face)
    head.visual(
        _rounded_box_mesh(HEAD_W, HEAD_D, HEAD_H, fillet=0.006, z_offset=HEAD_H / 2.0),
        material=black_vinyl, name="head_shell",
    )

    # Gold control panel (sits at head top surface)
    panel = BoxGeometry((PANEL_D, PANEL_W, 0.004))
    panel.translate(PANEL_X, 0.0, HEAD_H + 0.002)
    head.visual(
        mesh_from_geometry(panel, "gold_panel"),
        material=gold_panel, name="gold_panel",
    )

    # Power LED on gold panel
    led = CylinderGeometry(0.0035, 0.004)
    led.translate(PANEL_X - 0.004, 0.072, HEAD_H + 0.004)
    head.visual(
        mesh_from_geometry(led, "power_led"),
        material=indicator_red, name="power_led",
    )

    # Carry handle (box-based arch in front of gold panel)
    handle_x = PANEL_X - PANEL_D / 2.0 - 0.016
    handle_z_base = HEAD_H  # handle mounts on head top surface
    # Handle strap (horizontal bar above the head)
    handle_strap = BoxGeometry((0.016, 0.104, 0.006))
    handle_strap.translate(handle_x, 0.0, handle_z_base + 0.018)
    head.visual(
        mesh_from_geometry(handle_strap, "handle_strap"),
        material=trim_black, name="handle_strap",
    )
    # Handle side legs connecting strap to head top
    for i, y in enumerate((-0.052, 0.052)):
        leg = BoxGeometry((0.016, 0.010, 0.018))
        leg.translate(handle_x, y, handle_z_base + 0.009)
        head.visual(
            mesh_from_geometry(leg, f"handle_leg_{i}"),
            material=trim_black, name=f"handle_leg_{i}",
        )

    head.inertial = Inertial.from_geometry(
        Box((HEAD_D, HEAD_W, HEAD_H)), mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, HEAD_H / 2.0)),
    )

    # FIXED joint: head sits on top face of speaker cabinet
    model.articulation(
        "cabinet_to_head",
        ArticulationType.FIXED,
        parent=speaker,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, SPEAKER_H / 2.0)),
    )

    # ========================================= knobs on head gold panel
    # Gold panel top surface is at HEAD_H + 0.004; knob seat slightly into it
    seat_z = HEAD_H + 0.003
    for i in range(KNOB_COUNT):
        ky = KNOB_YS[i]
        knob_part = model.part(f"knob_{i}")
        knob_part.visual(
            mesh_from_geometry(_make_knob_geometry(), f"knob_{i}"),
            material=metal_knob, name=f"knob_{i}",
        )
        knob_part.inertial = Inertial.from_geometry(
            Cylinder(KNOB_DIAM / 2.0, KNOB_H), mass=0.01,
        )
        model.articulation(
            f"head_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=head,
            child=knob_part,
            origin=Origin(xyz=(KNOB_X, ky, seat_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.3, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    speaker = object_model.get_part("speaker_cabinet")
    head = object_model.get_part("head_box")
    knobs = [object_model.get_part(f"knob_{i}") for i in range(KNOB_COUNT)]
    knob_joints = [object_model.get_articulation(f"head_to_knob_{i}") for i in range(KNOB_COUNT)]

    # ---- two distinct stacked enclosures ----
    ctx.check("speaker cabinet exists", speaker is not None, details="root part")
    ctx.check("head box exists", head is not None, details="upper enclosure")

    # Head sits on top of speaker cabinet
    head_pos = ctx.part_world_position(head)
    spk_top_z = SPEAKER_H / 2.0
    ctx.check(
        "head sits on speaker cabinet top face",
        head_pos is not None and abs(head_pos[2] - spk_top_z) < 0.003,
        details=f"head z={head_pos[2] if head_pos else None}, speaker top={spk_top_z:.3f}",
    )

    # Head and speaker are distinct
    spk_mn, spk_mx = ctx.part_world_aabb(speaker)
    head_mn, head_mx = ctx.part_world_aabb(head)
    ctx.check(
        "head bottom near speaker top (stacked)",
        head_mn[2] >= spk_mx[2] - 0.003,
        details=f"speaker top z={spk_mx[2]:.3f}, head bottom z={head_mn[2]:.3f}",
    )

    # Gold panel is on the head
    panel_aabb = ctx.part_element_world_aabb(head, elem="gold_panel")
    ctx.check(
        "gold panel is on the head box",
        panel_aabb is not None and panel_aabb[0][2] > spk_top_z,
        details=f"panel min z={panel_aabb[0][2]:.3f}, speaker top={spk_top_z:.3f}",
    )

    # Speaker grille is on the speaker cabinet
    g_mn, g_mx = ctx.part_element_world_aabb(speaker, elem="speaker_grille")
    ctx.check(
        "speaker grille is on the speaker cabinet",
        g_mx[2] < spk_top_z + 0.005 and g_mn[2] > -spk_top_z - 0.005,
        details=f"grille z=[{g_mn[2]:.3f}, {g_mx[2]:.3f}], speaker z=[{-spk_top_z:.3f}, {spk_top_z:.3f}]",
    )

    # Grille faces front
    gx_center = (g_mn[0] + g_mx[0]) / 2.0
    gx_ext = g_mx[0] - g_mn[0]
    gy_ext = g_mx[1] - g_mn[1]
    gz_ext = g_mx[2] - g_mn[2]
    ctx.check(
        "speaker grille is a thin front-facing panel",
        gx_center > 0.03 and gx_ext < gy_ext and gx_ext < gz_ext,
        details=f"center x={gx_center:.3f}, extents=({gx_ext:.3f},{gy_ext:.3f},{gz_ext:.3f})",
    )

    # ---- four knobs ----
    ctx.check(
        "four control knobs present",
        len(knobs) == KNOB_COUNT and all(k is not None for k in knobs),
        details=f"knob parts={[k.name for k in knobs]}",
    )

    for i, (k, j) in enumerate(zip(knobs, knob_joints)):
        ctx.allow_overlap(
            k, head,
            elem_a=f"knob_{i}", elem_b="gold_panel",
            reason="Knob base is intentionally press-fit a hair into the gold panel surface.",
        )
        ctx.expect_contact(k, head, name=f"knob_{i} rests on head panel")

    # Knobs are above speaker cabinet
    for i, k in enumerate(knobs):
        mn, mx = ctx.part_world_aabb(k)
        ctx.check(
            f"knob_{i} is above speaker cabinet (on head)",
            mn[2] > spk_top_z,
            details=f"knob_{i} min z={mn[2]:.3f}, speaker top={spk_top_z:.3f}",
        )

    # ---- knob rotation ----
    for i, (k, j) in enumerate(zip(knobs, knob_joints)):
        ax = ctx.part_world_position(k)
        mn0, mx0 = ctx.part_world_aabb(k)
        z_top0 = mx0[2]
        cen0 = ((mn0[0] + mx0[0]) / 2.0 - ax[0], (mn0[1] + mx0[1]) / 2.0 - ax[1])
        with ctx.pose({j: math.pi / 2.0}):
            mn1, mx1 = ctx.part_world_aabb(k)
            z_top1 = mx1[2]
            cen1 = ((mn1[0] + mx1[0]) / 2.0 - ax[0], (mn1[1] + mx1[1]) / 2.0 - ax[1])
        rotated = abs(cen0[1]) > abs(cen0[0]) and abs(cen1[0]) > abs(cen1[1])
        moved = math.hypot(cen1[0] - cen0[0], cen1[1] - cen0[1])
        ctx.check(
            f"knob_{i} spins about the vertical axis",
            rotated and moved > 0.0008 and abs(z_top1 - z_top0) < 0.0015,
            details=f"rest_off={cen0}, turn_off={cen1}, moved={moved:.4f}, dz={abs(z_top1-z_top0):.5f}",
        )

    return ctx.report()


object_model = build_object_model()
