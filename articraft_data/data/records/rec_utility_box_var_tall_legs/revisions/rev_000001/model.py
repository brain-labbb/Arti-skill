from __future__ import annotations

# Street electrical utility cabinet raised on N tall slender steel support legs.
#
# Reference: a roadside steel enclosure cabinet elevated above the pavement on
# tall corner legs, with a single front service door hinged on the left vertical
# edge (REVOLUTE about Z), louvered venting on the side walls, a flat roof cap
# with slight overhang, conduit entry on one side, and painted weathered grey
# steel finish.
#
# Coordinate convention (Z-up world):
#   +X : cabinet WIDTH  (~0.60 m)
#   +Y : cabinet DEPTH  (~0.35 m); FRONT (door) at -Y, REAR at +Y
#   +Z : HEIGHT; legs stand on ground at z=0, cabinet top ~1.40 m
#
# Parts / articulations:
#   - cabinet (ROOT) : hollow steel enclosure shell, roof cap, vents, conduit,
#                      door frame, hinge barrels
#   - door           : front service door panel, REVOLUTE about vertical Z axis
#                      at the left (-X) hinge edge, opens outward (-Y)
#   - leg_0..leg_3   : tall slender steel posts at cabinet corners, FIXED
#
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    VentGrilleSlats,
    mesh_from_geometry,
)

# ---- overall dimensions (metres) ----
N_LEGS = 4
W = 0.60          # cabinet width (X)
D = 0.35          # cabinet depth (Y)
BOX_H = 0.80      # enclosure shell height
LEG_H = 0.60      # tall leg height (cabinet stands this far off ground)
WALL = 0.012

FRONT_Y = -D / 2.0
REAR_Y = +D / 2.0
BOX_BOTTOM = LEG_H
BOX_TOP = LEG_H + BOX_H
BOX_CZ = LEG_H + BOX_H / 2.0

# Door
DOOR_T = 0.015
DOOR_MARGIN = 0.018
DOOR_W = W - 2.0 * DOOR_MARGIN
DOOR_H = BOX_H - 2.0 * DOOR_MARGIN

# Roof
ROOF_OVERHANG = 0.022
ROOF_T = 0.018

# Legs
LEG_W = 0.040       # square steel post section
FOOT_W = 0.060      # foot pad width
FOOT_H = 0.008      # foot pad thickness
LEG_INSET = 0.035   # inset from cabinet outer walls to leg center

# Vent grille
VENT_W = 0.22       # vent width along the wall (Y direction)
VENT_H = 0.20       # vent height (Z direction)


def _leg_visuals():
    """Shared geometry helper: returns visuals for one tall slender leg.

    The leg part frame sits at the top of the leg (where it meets the cabinet
    bottom). The post extends downward (-Z) and a foot pad sits at the ground.
    """
    steel = "steel"
    post_len = LEG_H - FOOT_H
    return [
        # tall slender post
        (
            Box((LEG_W, LEG_W, post_len)),
            Origin(xyz=(0.0, 0.0, -post_len / 2.0)),
            steel,
            "post",
        ),
        # foot plate at ground
        (
            Box((FOOT_W, FOOT_W, FOOT_H)),
            Origin(xyz=(0.0, 0.0, -LEG_H + FOOT_H / 2.0)),
            steel,
            "foot_pad",
        ),
        # gusset plate connecting leg top to cabinet underside
        (
            Box((LEG_W + 0.016, LEG_W + 0.016, 0.006)),
            Origin(xyz=(0.0, 0.0, -0.003)),
            steel,
            "top_gusset",
        ),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_cabinet")

    # ---- materials ----
    grey_paint = model.material("grey_paint", rgba=(0.62, 0.64, 0.66, 1.0))
    door_paint = model.material("door_paint", rgba=(0.58, 0.62, 0.60, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.30, 0.32, 1.0))
    steel = model.material("steel", rgba=(0.52, 0.54, 0.56, 1.0))
    shadow = model.material("shadow_inside", rgba=(0.35, 0.36, 0.38, 1.0))
    roof_paint = model.material("roof_paint", rgba=(0.55, 0.57, 0.59, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.18, 0.19, 0.21, 1.0))

    # ================================================================
    # CABINET (ROOT) : hollow enclosure shell on leg mounts
    # ================================================================
    cab = model.part("cabinet")

    # --- shell walls (5 walls; front face is open for the door) ---
    # Back wall (+Y)
    cab.visual(
        Box((W, WALL, BOX_H)),
        origin=Origin(xyz=(0.0, REAR_Y - WALL / 2.0, BOX_CZ)),
        material=grey_paint,
        name="back_wall",
    )
    # Left wall (-X)
    cab.visual(
        Box((WALL, D, BOX_H)),
        origin=Origin(xyz=(-W / 2.0 + WALL / 2.0, 0.0, BOX_CZ)),
        material=grey_paint,
        name="left_wall",
    )
    # Right wall (+X)
    cab.visual(
        Box((WALL, D, BOX_H)),
        origin=Origin(xyz=(W / 2.0 - WALL / 2.0, 0.0, BOX_CZ)),
        material=grey_paint,
        name="right_wall",
    )
    # Bottom plate
    cab.visual(
        Box((W, D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, BOX_BOTTOM + WALL / 2.0)),
        material=grey_paint,
        name="bottom_plate",
    )

    # --- front door frame (rails and stiles around the opening) ---
    frame_w = 0.025
    # Top rail
    cab.visual(
        Box((W, WALL + 0.004, frame_w)),
        origin=Origin(xyz=(0.0, FRONT_Y + WALL / 2.0, BOX_TOP - frame_w / 2.0)),
        material=grey_paint,
        name="front_top_rail",
    )
    # Bottom rail
    cab.visual(
        Box((W, WALL + 0.004, frame_w)),
        origin=Origin(xyz=(0.0, FRONT_Y + WALL / 2.0, BOX_BOTTOM + frame_w / 2.0)),
        material=grey_paint,
        name="front_bottom_rail",
    )
    # Left stile (hinge side)
    cab.visual(
        Box((frame_w, WALL + 0.004, BOX_H)),
        origin=Origin(xyz=(-W / 2.0 + frame_w / 2.0, FRONT_Y + WALL / 2.0, BOX_CZ)),
        material=grey_paint,
        name="front_left_stile",
    )
    # Right stile (latch side)
    cab.visual(
        Box((frame_w, WALL + 0.004, BOX_H)),
        origin=Origin(xyz=(W / 2.0 - frame_w / 2.0, FRONT_Y + WALL / 2.0, BOX_CZ)),
        material=grey_paint,
        name="front_right_stile",
    )

    # --- roof cap with slight overhang ---
    cab.visual(
        Box((W + 2.0 * ROOF_OVERHANG, D + 2.0 * ROOF_OVERHANG, ROOF_T)),
        origin=Origin(xyz=(0.0, 0.0, BOX_TOP + ROOF_T / 2.0)),
        material=roof_paint,
        name="roof_cap",
    )
    # roof drip edge lip (thin raised rim on three sides, open at front)
    lip_h = 0.010
    lip_w = 0.006
    roof_top_z = BOX_TOP + ROOF_T
    cab.visual(
        Box((W + 2.0 * ROOF_OVERHANG, lip_w, lip_h)),
        origin=Origin(xyz=(0.0, REAR_Y + ROOF_OVERHANG - lip_w / 2.0, roof_top_z + lip_h / 2.0)),
        material=roof_paint,
        name="roof_rear_lip",
    )
    for sx, name in [(-1, "left"), (1, "right")]:
        cab.visual(
            Box((lip_w, D + 2.0 * ROOF_OVERHANG, lip_h)),
            origin=Origin(
                xyz=(sx * (W / 2.0 + ROOF_OVERHANG - lip_w / 2.0), 0.0, roof_top_z + lip_h / 2.0)
            ),
            material=roof_paint,
            name=f"roof_{name}_lip",
        )

    # --- louvered vent grilles on side walls ---
    vent_grille = VentGrilleGeometry(
        (VENT_H, VENT_W),
        frame=0.010,
        face_thickness=0.004,
        duct_depth=0.024,
        duct_wall=0.003,
        slat_pitch=0.016,
        slat_width=0.008,
        slat_angle_deg=35.0,
        corner_radius=0.004,
        slats=VentGrilleSlats(profile="flat", direction="down"),
    )
    vent_mesh = mesh_from_geometry(vent_grille, "side_vent")

    # +X side wall vent: face normal (local +Z) must point toward world +X
    # Ry(-pi/2) maps local +Z → world +X, local X → world -Z, local Y → world Y
    cab.visual(
        vent_mesh,
        origin=Origin(
            xyz=(W / 2.0 + 0.002, 0.0, BOX_CZ + 0.06),
            rpy=(0.0, -math.pi / 2.0, 0.0),
        ),
        material=dark_metal,
        name="right_vent",
    )
    # -X side wall vent: face normal toward world -X
    cab.visual(
        vent_mesh,
        origin=Origin(
            xyz=(-W / 2.0 - 0.002, 0.0, BOX_CZ + 0.06),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_metal,
        name="left_vent",
    )

    # --- exterior stiffening ribs on side walls ---
    for sx, name in [(-1, "left"), (1, "right")]:
        cab.visual(
            Box((0.008, D - 0.060, 0.016)),
            origin=Origin(xyz=(sx * (W / 2.0 + 0.002), 0.0, BOX_CZ - 0.12)),
            material=steel,
            name=f"{name}_lower_rib",
        )
        cab.visual(
            Box((0.008, D - 0.060, 0.016)),
            origin=Origin(xyz=(sx * (W / 2.0 + 0.002), 0.0, BOX_CZ + 0.25)),
            material=steel,
            name=f"{name}_upper_rib",
        )

    # --- conduit cable entry on the rear wall ---
    cab.visual(
        Cylinder(radius=0.020, length=0.10),
        origin=Origin(
            xyz=(0.12, REAR_Y + 0.03, BOX_BOTTOM + 0.10),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="conduit_stub",
    )
    cab.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(
            xyz=(0.12, REAR_Y + 0.004, BOX_BOTTOM + 0.10),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="conduit_collar",
    )

    # --- inner shadowed floor recess (sits on the bottom plate) ---
    cab.visual(
        Box((W - 0.050, D - 0.050, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, BOX_BOTTOM + WALL + 0.002)),
        material=shadow,
        name="inner_floor_recess",
    )

    # --- barrel hinges on the left vertical edge of the front face ---
    # Two hinges: upper and lower, pin axis along Z (vertical)
    hinge_length = 0.12
    hinge_geom = BarrelHingeGeometry(
        hinge_length,
        leaf_width_a=0.016,
        leaf_width_b=0.016,
        leaf_thickness=0.0024,
        pin_diameter=0.005,
        knuckle_count=5,
    )
    hinge_mesh = mesh_from_geometry(hinge_geom, "door_hinge")
    # BarrelHinge pin axis is local Z; no rotation needed for vertical pin.
    for z_off, hname in [(BOX_BOTTOM + 0.10, "lower_hinge"), (BOX_TOP - 0.10, "upper_hinge")]:
        cab.visual(
            hinge_mesh,
            origin=Origin(xyz=(-W / 2.0 + 0.002, FRONT_Y - 0.004, z_off)),
            material=dark_metal,
            name=hname,
        )

    # --- hasp/latch keeper on the right stile (door latch receiver) ---
    cab.visual(
        Box((0.030, 0.012, 0.060)),
        origin=Origin(xyz=(W / 2.0 - 0.015, FRONT_Y - 0.006, BOX_CZ)),
        material=steel,
        name="latch_keeper",
    )

    cab.inertial = Inertial.from_geometry(
        Box((W, D, BOX_H)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, BOX_CZ)),
    )

    # ================================================================
    # DOOR : front service door panel, REVOLUTE about vertical Z axis
    # ================================================================
    # Part frame at the hinge line (left vertical edge of the front face).
    # Door panel extends along +X from the hinge, and along +Y for thickness
    # (inward toward the cabinet interior).
    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(DOOR_W / 2.0, DOOR_T / 2.0, 0.0)),
        material=door_paint,
        name="door_panel",
    )
    # raised centre pressing on the outer face
    door.visual(
        Box((DOOR_W - 0.080, 0.005, DOOR_H - 0.100)),
        origin=Origin(xyz=(DOOR_W / 2.0, -0.0025, 0.0)),
        material=door_paint,
        name="door_pressing",
    )
    # door handle / swing latch on the free (right) edge
    door.visual(
        Box((0.025, 0.030, 0.120)),
        origin=Origin(xyz=(DOOR_W - 0.030, -0.015, 0.0)),
        material=handle_metal,
        name="door_handle",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(
            xyz=(DOOR_W - 0.030, -0.030, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_metal,
        name="handle_grip_bar",
    )
    # rivets along the hinge edge
    for dz in (-0.28, -0.14, 0.14, 0.28):
        door.visual(
            Cylinder(radius=0.003, length=0.002),
            origin=Origin(xyz=(0.010, -0.001, dz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"door_hinge_rivet_{dz:+.2f}",
        )
    # latch hook on the free edge
    door.visual(
        Box((0.020, 0.016, 0.040)),
        origin=Origin(xyz=(DOOR_W - 0.005, DOOR_T / 2.0 + 0.008, 0.0)),
        material=steel,
        name="door_latch_hook",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        mass=5.5,
        origin=Origin(xyz=(DOOR_W / 2.0, DOOR_T / 2.0, 0.0)),
    )

    # Door articulation: REVOLUTE about vertical Z at the left hinge edge.
    # The joint origin is at the hinge line on the cabinet frame.
    # Door part frame is coincident at q=0. Door extends along +X.
    # axis=(0,0,-1): right-hand rule about -Z rotates +X toward -Y (outward).
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cab,
        child=door,
        origin=Origin(xyz=(-W / 2.0 + DOOR_MARGIN, FRONT_Y, BOX_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.radians(135.0),
            effort=12.0,
            velocity=1.5,
        ),
    )

    # ================================================================
    # LEGS : N tall slender steel posts at cabinet corners (FIXED)
    # ================================================================
    corner_signs = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    for i in range(N_LEGS):
        sx, sy = corner_signs[i]
        leg_x = sx * (W / 2.0 - LEG_INSET)
        leg_y = sy * (D / 2.0 - LEG_INSET)

        leg = model.part(f"leg_{i}")
        for geom, org, mat_name, vname in _leg_visuals():
            mat = {"steel": steel}[mat_name]
            leg.visual(geom, origin=org, material=mat, name=vname)

        leg.inertial = Inertial.from_geometry(
            Box((LEG_W, LEG_W, LEG_H)),
            mass=3.0,
            origin=Origin(xyz=(0.0, 0.0, -LEG_H / 2.0)),
        )

        # FIXED joint: leg top meets the cabinet underside at this corner
        model.articulation(
            f"cabinet_to_leg_{i}",
            ArticulationType.FIXED,
            parent=cab,
            child=leg,
            origin=Origin(xyz=(leg_x, leg_y, BOX_BOTTOM)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cab = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("cabinet_to_door")

    # ---- all N=4 legs exist and are separate parts ----
    for i in range(N_LEGS):
        leg = object_model.get_part(f"leg_{i}")
        ctx.check(
            f"leg_{i} exists",
            leg is not None,
            details=f"leg_{i} not found in model",
        )

    # ---- legs stand on the ground at z=0 ----
    for i in range(N_LEGS):
        leg = object_model.get_part(f"leg_{i}")
        mn, mx = ctx.part_world_aabb(leg)
        ctx.check(
            f"leg_{i} foot at z=0",
            abs(mn[2]) < 0.005,
            details=f"leg_{i} min z={mn[2]:.4f}",
        )

    # ---- cabinet is elevated well above the ground ----
    cab_mn, cab_mx = ctx.part_world_aabb(cab)
    ctx.check(
        "cabinet elevated above ground",
        cab_mn[2] > 0.40,
        details=f"cabinet min z={cab_mn[2]:.3f}",
    )

    # ---- cabinet box is taller than wide (upright enclosure) ----
    cab_ext = (cab_mx[0] - cab_mn[0], cab_mx[1] - cab_mn[1], cab_mx[2] - cab_mn[2])
    ctx.check(
        "cabinet is taller than wide (upright enclosure)",
        cab_ext[2] > cab_ext[0],
        details=f"extents (x,y,z)={tuple(round(e, 3) for e in cab_ext)}",
    )

    # ---- door joint is REVOLUTE about vertical (Z) axis ----
    ax = door_joint.axis
    ctx.check(
        "door hinge axis is vertical (Z)",
        abs(ax[2]) > 0.99 and abs(ax[0]) < 1e-3 and abs(ax[1]) < 1e-3,
        details=f"axis={ax}",
    )
    ctx.check(
        "door joint is revolute",
        str(door_joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={door_joint.articulation_type}",
    )

    # ---- door swings outward (toward -Y) when opened ----
    closed_aabb = ctx.part_world_aabb(door)
    closed_front_y = closed_aabb[0][1]  # most -Y extent
    closed_right_x = closed_aabb[1][0]  # most +X extent (free edge)
    with ctx.pose({door_joint: math.radians(90.0)}):
        open_aabb = ctx.part_world_aabb(door)
        open_front_y = open_aabb[0][1]
        open_right_x = open_aabb[1][0]
    ctx.check(
        "door free edge swings outward when opened",
        open_front_y < closed_front_y - 0.10,
        details=f"closed_front_y={closed_front_y:.3f}, open_front_y={open_front_y:.3f}",
    )
    ctx.check(
        "door opens (free edge X moves toward hinge side)",
        open_right_x < closed_right_x - 0.10,
        details=f"closed_right_x={closed_right_x:.3f}, open_right_x={open_right_x:.3f}",
    )

    # ---- closed door seats on the front face ----
    ctx.allow_overlap(
        door,
        cab,
        reason="Closed door sits in the front frame opening; hinge barrels embed in the stile.",
    )

    # ---- roof cap is above the cabinet shell ----
    roof_vis = cab.get_visual("roof_cap")
    ctx.check(
        "roof cap exists",
        roof_vis is not None,
        details="roof_cap visual not found on cabinet",
    )

    # ---- vent grilles exist on both side walls ----
    for vname in ("left_vent", "right_vent"):
        v = cab.get_visual(vname)
        ctx.check(
            f"{vname} exists",
            v is not None,
            details=f"{vname} visual not found on cabinet",
        )

    return ctx.report()


object_model = build_object_model()
