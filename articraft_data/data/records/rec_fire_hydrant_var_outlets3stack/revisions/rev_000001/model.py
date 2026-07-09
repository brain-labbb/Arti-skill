from __future__ import annotations

# Cast-iron pillar fire hydrant.
# Bright-red body: flange base bolted at ground, ribbed barrel, an upper
# valve chamber carrying three equal hose outlets spaced at 120° around the
# chamber, a bolted bonnet flange, a domed bonnet, and a square top operating
# nut. Each outlet wears a brass lift-off cap tethered by a short articulated
# round-link chain. All outlet caps slide straight along their outlet axes
# without flipping; the top operating nut turns about the vertical axis.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

CHAIN_LINK_R = 0.007
CHAIN_LINK_WIRE_R = 0.0009
CHAIN_LINK_STEP = 2.0 * CHAIN_LINK_R - 2.0 * CHAIN_LINK_WIRE_R
CHAIN_ROOT_CLEARANCE = CHAIN_LINK_R * 0.85
CHAIN_SWING = math.radians(35.0)
CHAIN_LINK_COUNTS: dict[str, int] = {}


def _rpy_aim_negz(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    """Aim a child so its local -Z axis points along direction in parent frame."""
    dx, dy, dz = direction
    n = math.sqrt(dx * dx + dy * dy + dz * dz)
    if n < 1e-9:
        return (0.0, 0.0, 0.0)
    vx, vy, vz = -dx / n, -dy / n, -dz / n
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(math.hypot(vx, vy), vz)
    return (0.0, pitch, yaw)


def _round_chain_link_mesh(in_yz_plane: bool, name: str):
    """Circular chain link with origin at the top contact point."""
    pts = []
    for j in range(40):
        t = 2.0 * math.pi * j / 40
        side = CHAIN_LINK_R * math.cos(t)
        drop = -CHAIN_LINK_R + CHAIN_LINK_R * math.sin(t)
        if in_yz_plane:
            pts.append((0.0, side, drop))
        else:
            pts.append((side, 0.0, drop))
    geom = tube_from_spline_points(
        pts,
        radius=CHAIN_LINK_WIRE_R,
        samples_per_segment=8,
        closed_spline=True,
        radial_segments=12,
        cap_ends=False,
    )
    return mesh_from_geometry(geom, name)


def _world_vec_to_outlet_local(yaw: float, vec: tuple[float, float, float]) -> tuple[float, float, float]:
    """Inverse of the outlet cap frame: local +Z is radial, local +X is down."""
    dx, dy, dz = vec
    nx, ny = math.cos(yaw), math.sin(yaw)
    tx, ty = -ny, nx
    return (-dz, dx * tx + dy * ty, dx * nx + dy * ny)


def _add_round_link_chain(model, prefix, parent_part, root_origin, n_links, material) -> list:
    """Add a continuous serial chain of round links starting from parent_part."""
    parent = parent_part
    origin = root_origin
    links = []
    for i in range(n_links):
        link = model.part(f"{prefix}_{i}")
        in_yz = i % 2 == 1
        link.visual(
            _round_chain_link_mesh(in_yz, f"{prefix}_{i}_round"),
            material=material,
            name="round_body",
        )
        model.articulation(
            f"{prefix}_swing_{i}",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=link,
            origin=origin,
            axis=(1.0, 0.0, 0.0) if in_yz else (0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.8, velocity=2.0, lower=-CHAIN_SWING, upper=CHAIN_SWING
            ),
        )
        parent = link
        origin = Origin(xyz=(0.0, 0.0, -CHAIN_LINK_STEP))
        links.append(link)
    return links


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pillar_fire_hydrant")

    hydrant_red = model.material("hydrant_red", rgba=(0.74, 0.10, 0.09, 1.0))
    brass = model.material("brass", rgba=(0.82, 0.62, 0.16, 1.0))
    dark_chain = model.material("dark_chain", rgba=(0.30, 0.27, 0.13, 1.0))

    # ---------- Overall scale ----------
    # Real pillar hydrant ~1.0 m tall above grade. Body axis is +Z.
    base_flange_r = 0.155
    base_flange_h = 0.040
    lower_band_z = 0.040  # top of base flange

    barrel_r = 0.090
    barrel_top_z = 0.560  # top of the plain ribbed barrel

    chamber_r = 0.125  # widened upper valve chamber that carries the outlets
    chamber_bottom_z = 0.560
    chamber_top_z = 0.760

    bonnet_flange_z = 0.760
    bonnet_flange_h = 0.045
    bonnet_base_z = bonnet_flange_z + bonnet_flange_h  # 0.805

    # ---------- Root: hydrant body ----------
    body = model.part("body")

    # Base flange (bolted, sits on ground at z=0).
    base_flange = LatheGeometry(
        [
            (0.0, 0.0),
            (base_flange_r, 0.0),
            (base_flange_r, base_flange_h * 0.6),
            (base_flange_r - 0.018, base_flange_h),
            (barrel_r + 0.012, base_flange_h),
            (barrel_r + 0.012, base_flange_h + 0.02),
            (0.0, base_flange_h + 0.02),
        ],
        segments=48,
    )
    body.visual(
        mesh_from_geometry(base_flange, "base_flange"),
        material=hydrant_red,
        name="base_flange",
    )

    # Base flange bolts (ring of hex heads around the flange).
    n_base_bolts = 8
    for i in range(n_base_bolts):
        ang = 2.0 * math.pi * i / n_base_bolts
        bx = (base_flange_r - 0.020) * math.cos(ang)
        by = (base_flange_r - 0.020) * math.sin(ang)
        bolt = CylinderGeometry(0.011, 0.016, radial_segments=6)
        body.visual(
            mesh_from_geometry(bolt, f"base_bolt_{i}"),
            origin=Origin(xyz=(bx, by, base_flange_h - 0.002)),
            material=brass,
            name=f"base_bolt_{i}",
        )

    # Ribbed barrel: a revolved shell with three raised bands.
    barrel_profile = [(0.0, lower_band_z)]
    z = lower_band_z + 0.01
    # smooth rising barrel with a slight taper, plus raised ribs at three heights
    rib_zs = [0.170, 0.330, 0.480]
    pts = [(barrel_r, lower_band_z)]
    for rz in rib_zs:
        pts.append((barrel_r, rz - 0.022))
        pts.append((barrel_r + 0.016, rz - 0.012))
        pts.append((barrel_r + 0.016, rz + 0.012))
        pts.append((barrel_r, rz + 0.022))
    pts.append((barrel_r, barrel_top_z))
    barrel_profile = [(0.0, lower_band_z)] + pts + [(0.0, barrel_top_z)]
    barrel = LatheGeometry(barrel_profile, segments=48)
    body.visual(
        mesh_from_geometry(barrel, "barrel"),
        material=hydrant_red,
        name="barrel",
    )

    # Upper valve chamber (widened, carries outlets), with a step shoulder.
    chamber_profile = [
        (0.0, chamber_bottom_z),
        (barrel_r, chamber_bottom_z),
        (chamber_r, chamber_bottom_z + 0.022),
        (chamber_r, chamber_top_z - 0.022),
        (chamber_r - 0.022, chamber_top_z),
        (0.0, chamber_top_z),
    ]
    chamber = LatheGeometry(chamber_profile, segments=48)
    body.visual(
        mesh_from_geometry(chamber, "valve_chamber"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hydrant_red,
        name="valve_chamber",
    )

    # Bonnet flange (bolted ring between chamber and bonnet).
    bonnet_flange = LatheGeometry(
        [
            (0.0, bonnet_flange_z),
            (chamber_r + 0.015, bonnet_flange_z),
            (chamber_r + 0.015, bonnet_flange_z + bonnet_flange_h * 0.6),
            (chamber_r - 0.010, bonnet_flange_z + bonnet_flange_h),
            (0.0, bonnet_flange_z + bonnet_flange_h),
        ],
        segments=48,
    )
    body.visual(
        mesh_from_geometry(bonnet_flange, "bonnet_flange"),
        material=hydrant_red,
        name="bonnet_flange",
    )

    n_bonnet_bolts = 8
    for i in range(n_bonnet_bolts):
        ang = 2.0 * math.pi * i / n_bonnet_bolts + math.pi / n_bonnet_bolts
        bx = (chamber_r + 0.002) * math.cos(ang)
        by = (chamber_r + 0.002) * math.sin(ang)
        bolt = CylinderGeometry(0.010, 0.018, radial_segments=6)
        body.visual(
            mesh_from_geometry(bolt, f"bonnet_bolt_{i}"),
            origin=Origin(xyz=(bx, by, bonnet_flange_z + bonnet_flange_h * 0.5)),
            material=brass,
            name=f"bonnet_bolt_{i}",
        )

    # Domed bonnet (rounded cast top).
    dome = DomeGeometry(0.105, radial_segments=48, height_segments=18)
    dome.scale(1.0, 1.0, 1.05)
    body.visual(
        mesh_from_geometry(dome, "bonnet_dome"),
        origin=Origin(xyz=(0.0, 0.0, bonnet_base_z)),
        material=hydrant_red,
        name="bonnet_dome",
    )

    # Small collar (boss) on the bonnet apex that seats the operating nut.
    nut_boss_z = bonnet_base_z + 0.105 * 1.05
    boss = CylinderGeometry(0.030, 0.018, radial_segments=24)
    body.visual(
        mesh_from_geometry(boss, "nut_boss"),
        origin=Origin(xyz=(0.0, 0.0, nut_boss_z + 0.009)),
        material=hydrant_red,
        name="nut_boss",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=barrel_r, length=barrel_top_z), mass=60.0
    )

    # ---------- Top operating nut (revolute about vertical Z) ----------
    op_nut = model.part("operating_nut")
    nut_geom = CylinderGeometry(0.024, 0.030, radial_segments=4)  # square pentagon-ish nut
    op_nut.visual(
        mesh_from_geometry(nut_geom, "operating_nut"),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=brass,
        name="operating_nut",
    )
    op_nut.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.030), mass=0.4
    )
    model.articulation(
        "body_to_operating_nut",
        ArticulationType.REVOLUTE,
        parent=body,
        child=op_nut,
        origin=Origin(xyz=(0.0, 0.0, nut_boss_z + 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=-12.0, upper=12.0),
    )

    # ---------- Outlets + lift-off caps ----------
    # Each outlet: a short stub on the chamber, then a brass cap visibly
    # tethered by a short chain from a body eye to a small ring on the cap.
    def add_outlet(
        tag: str,
        yaw: float,
        center_z: float,
        outlet_r: float,
        stub_len: float,
        cap_r: float,
        cap_len: float,
        eye_yaw_deg: float = 0.0,
    ) -> None:
        nx, ny = math.cos(yaw), math.sin(yaw)
        # outlet stub root on the chamber surface
        root_r = chamber_r - 0.010
        sx, sy = root_r * nx, root_r * ny
        # stub flange ring + tube on the body
        stub = CylinderGeometry(outlet_r, stub_len, radial_segments=28)
        # cylinder axis local Z -> rotate so axis points radially (yaw about Z, tilt to horizontal)
        body.visual(
            mesh_from_geometry(stub, f"{tag}_stub"),
            origin=Origin(
                xyz=(sx + nx * stub_len * 0.5, sy + ny * stub_len * 0.5, center_z),
                rpy=(0.0, math.pi / 2.0, yaw),
            ),
            material=hydrant_red,
            name=f"{tag}_stub",
        )
        # threaded collar / outer flange ring at the stub mouth
        collar = TorusGeometry(outlet_r + 0.006, 0.010, radial_segments=20, tubular_segments=28)
        mouth_x = sx + nx * stub_len
        mouth_y = sy + ny * stub_len
        body.visual(
            mesh_from_geometry(collar, f"{tag}_collar"),
            origin=Origin(xyz=(mouth_x, mouth_y, center_z), rpy=(0.0, math.pi / 2.0, yaw)),
            material=brass,
            name=f"{tag}_collar",
        )
        # ----- tether sub-assembly: body eye -----
        tether = model.part(f"{tag}_tether")
        eye = TorusGeometry(0.011, 0.0045, radial_segments=10, tubular_segments=16)
        # Keep the eye on the widened chamber wall so it stays seated on the body
        # (clamp above the chamber's lower shoulder).
        eye_z = max(center_z - outlet_r - 0.030, chamber_bottom_z + 0.022)
        eye_r = chamber_r  # sit on the chamber surface
        # The eye/chain may be anchored at a yaw offset so they clear a large
        # outlet stub while staying on the curved chamber surface.
        eye_yaw = yaw + math.radians(eye_yaw_deg)
        enx, eny = math.cos(eye_yaw), math.sin(eye_yaw)
        eye_x = eye_r * enx
        eye_y = eye_r * eny
        # torus default lies in its local XY plane (axis = local Z). Rotate so the
        # ring axis points radially outward at the eye's own yaw.
        tether.visual(
            mesh_from_geometry(eye, f"{tag}_chain_eye"),
            origin=Origin(xyz=(eye_x, eye_y, eye_z), rpy=(0.0, math.pi / 2.0, eye_yaw)),
            material=brass,
            name=f"{tag}_chain_eye",
        )

        # ----- cap part (prismatic slide, no hinge flip) -----
        cap = model.part(f"{tag}_cap")
        default_lift = 0.0
        # cap body: a knurled brass cap. Author it in a local frame where the
        # cap axis is +Z. With the joint frame rotated onto the outlet, local +Z
        # points straight outward along that outlet axis.
        cap_geom = LatheGeometry(
            [
                (0.0, 0.0),
                (cap_r, 0.0),
                (cap_r, cap_len * 0.45),
                (cap_r - 0.004, cap_len * 0.78),
                (cap_r - 0.016, cap_len),
                (0.0, cap_len),
            ],
            segments=36,
        )
        # The cap seats over the outlet mouth: place it so its z=0 face sits at
        # the mouth and it extends outward along +Z (joint frame).
        cap.visual(
            mesh_from_geometry(cap_geom, f"{tag}_cap_body"),
            origin=Origin(xyz=(-default_lift, 0.0, -0.004)),
            material=brass,
            name=f"{tag}_cap_body",
        )
        # ribbed grip lugs around the cap, kept outboard of the collar plane
        # (z>=~0.30*cap_len) so they do not collide with the body collar at z~0.
        lug_h = cap_len * 0.5
        lug_cz = cap_len * 0.55
        for j in range(6):
            la = 2.0 * math.pi * j / 6
            lug = Box((0.010, 0.006, lug_h))
            cap.visual(
                lug,
                origin=Origin(
                    xyz=(
                        -default_lift + (cap_r - 0.002) * math.cos(la),
                        (cap_r - 0.002) * math.sin(la),
                        lug_cz,
                    ),
                    rpy=(0.0, 0.0, la),
                ),
                material=brass,
                name=f"{tag}_cap_lug_{j}",
            )
        # Small tether ring on the underside of the cap. With the cap frame
        # rotated onto the outlet, local +X points downward in world space.
        cap_ring_z = cap_len * 0.82
        cap_ring_drop = cap_r + 0.014
        cap_ring = TorusGeometry(0.014, 0.0042, radial_segments=12, tubular_segments=18)
        cap.visual(
            mesh_from_geometry(cap_ring, f"{tag}_cap_chain_ring"),
            origin=Origin(
                xyz=(cap_ring_drop - default_lift, 0.0, cap_ring_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name=f"{tag}_cap_chain_ring",
        )
        cap.inertial = Inertial.from_geometry(
            Cylinder(radius=cap_r, length=cap_len), mass=0.3
        )

        slide_axis = (0.0, 0.0, 1.0)
        lower = 0.0
        upper = 0.120
        # Joint frame at the outlet mouth; the cap pulls straight away from
        # that outlet.
        model.articulation(
            f"{tag}_to_cap",
            ArticulationType.PRISMATIC,
            parent=body,
            child=cap,
            origin=Origin(xyz=(mouth_x, mouth_y, center_z), rpy=(0.0, math.pi / 2.0, yaw)),
            axis=slide_axis,
            motion_limits=MotionLimits(effort=20.0, velocity=0.25, lower=lower, upper=upper),
        )

        model.articulation(
            f"body_to_{tag}_tether",
            ArticulationType.FIXED,
            parent=body,
            child=tether,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )

        # ----- articulated round-link chain -----
        # Like the pipe reference, the chain is one serial chain whose root link
        # is jointed to the cap ring, so it follows the cap instead of becoming
        # a separate static strand. This variant uses circular links.
        cap_attach = (
            mouth_x + nx * cap_ring_z,
            mouth_y + ny * cap_ring_z,
            center_z - cap_ring_drop + default_lift - CHAIN_ROOT_CLEARANCE,
        )
        eye_attach = (eye_x, eye_y, eye_z - 0.011)
        chain_vec_world = (
            eye_attach[0] - cap_attach[0],
            eye_attach[1] - cap_attach[1],
            eye_attach[2] - cap_attach[2],
        )
        chain_span = math.sqrt(sum(c * c for c in chain_vec_world))
        n_chain_links = max(5, round(chain_span / CHAIN_LINK_STEP))
        CHAIN_LINK_COUNTS[tag] = n_chain_links
        cap_attach_local = (cap_ring_drop - default_lift + CHAIN_ROOT_CLEARANCE, 0.0, cap_ring_z)
        _add_round_link_chain(
            model,
            f"{tag}_chain",
            cap,
            Origin(
                xyz=cap_attach_local,
                rpy=_rpy_aim_negz(_world_vec_to_outlet_local(yaw, chain_vec_world)),
            ),
            n_chain_links,
            dark_chain,
        )
        tether.inertial = Inertial.from_geometry(
            Cylinder(radius=0.012, length=0.10), mass=0.05
        )

    # Three equal hose outlets spaced at 120° around the upper valve chamber.
    n_outlets = 3
    outlet_z = chamber_bottom_z + (chamber_top_z - chamber_bottom_z) * 0.5
    for i in range(n_outlets):
        yaw = 2.0 * math.pi * i / n_outlets
        add_outlet(
            f"outlet_{i}",
            yaw,
            outlet_z,
            0.044,
            0.048,
            0.056,
            0.048,
            eye_yaw_deg=55.0,
        )

    return model


def run_tests() -> TestReport:
    from sdk import TestContext

    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    op_nut = object_model.get_part("operating_nut")

    nut_joint = object_model.get_articulation("body_to_operating_nut")

    outlet_tags = [f"outlet_{i}" for i in range(3)]
    caps = [object_model.get_part(f"{tag}_cap") for tag in outlet_tags]
    cap_joints = [object_model.get_articulation(f"{tag}_to_cap") for tag in outlet_tags]

    # --- Base sits at ground z=0 and footprint reads as a flange ---
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    ctx.check(
        "base_at_ground",
        abs(bb[0][2]) < 0.006,
        details=f"body min z = {bb[0][2]:.4f} (expected ~0)",
    )
    ctx.check(
        "hydrant_tall_enough",
        bb[1][2] > 0.85,
        details=f"body max z = {bb[1][2]:.4f} (expected > 0.85 m)",
    )

    # --- Operating nut sits at the very top, on the vertical axis ---
    nut_bb = ctx.part_world_aabb(op_nut)
    assert nut_bb is not None
    ctx.check(
        "operating_nut_on_top",
        nut_bb[0][2] > bb[1][2] - 0.06,
        details=f"nut min z={nut_bb[0][2]:.3f}, body top={bb[1][2]:.3f}",
    )
    ctx.expect_origin_distance(op_nut, body, axes="xy", max_dist=0.02, name="nut_centered_xy")

    # --- Operating nut turns about vertical axis ---
    assert abs(nut_joint.axis[2]) > 0.99, "operating nut axis must be vertical (Z)"
    rest = ctx.part_world_aabb(op_nut)
    with ctx.pose({nut_joint: 0.6}):
        turned = ctx.part_world_aabb(op_nut)
    assert rest is not None and turned is not None
    moved = any(abs(rest[k][i] - turned[k][i]) > 1e-4 for k in (0, 1) for i in (0, 1))
    ctx.check("operating_nut_rotates", moved, details="nut AABB should change when turned about Z")

    # --- All three caps are seated by default ---
    for tag, cap in zip(outlet_tags, caps):
        ctx.expect_contact(cap, body, contact_tol=0.006, name=f"{tag}_cap_seated")

    # --- Three outlets at equal height, arranged radially at 120° ---
    cap_positions = [ctx.part_world_position(cap) for cap in caps]
    assert all(p is not None for p in cap_positions)
    # All three at roughly the same Z height (on the chamber midline).
    z_vals = [p[2] for p in cap_positions]
    ctx.check(
        "outlets_at_same_height",
        max(z_vals) - min(z_vals) < 0.01,
        details=f"cap z values={z_vals}",
    )
    # Radial distances from body axis should be similar (equal outlets).
    radii = [math.sqrt(p[0] ** 2 + p[1] ** 2) for p in cap_positions]
    ctx.check(
        "outlets_equal_radial_distance",
        max(radii) - min(radii) < 0.015,
        details=f"radii={radii}",
    )
    # Angular spacing ~120° between consecutive outlets.
    angles = [math.atan2(p[1], p[0]) for p in cap_positions]
    for i in range(3):
        j = (i + 1) % 3
        delta_ang = (angles[j] - angles[i]) % (2.0 * math.pi)
        ctx.check(
            f"outlet_{i}_to_{j}_120deg_spacing",
            abs(delta_ang - 2.0 * math.pi / 3.0) < 0.15,
            details=f"angle_i={math.degrees(angles[i]):.1f}°, angle_j={math.degrees(angles[j]):.1f}°, delta={math.degrees(delta_ang):.1f}°",
        )

    # --- Each cap slides straight outward along its own outlet axis ---
    for i, (tag, joint, cap) in enumerate(zip(outlet_tags, cap_joints, caps)):
        yaw = 2.0 * math.pi * i / 3
        world_axis = (math.cos(yaw), math.sin(yaw), 0.0)
        with ctx.pose({joint: 0.0}):
            before = ctx.part_world_position(cap)
        with ctx.pose({joint: 0.090}):
            after = ctx.part_world_position(cap)
        assert before is not None and after is not None
        delta = tuple(after[k] - before[k] for k in range(3))
        axial = sum(delta[k] * world_axis[k] for k in range(3))
        lateral = math.sqrt(max(0.0, sum(d * d for d in delta) - axial * axial))
        ctx.check(
            f"{tag}_cap_slides_straight_off",
            joint.articulation_type == ArticulationType.PRISMATIC and axial > 0.075 and lateral < 0.006,
            details=f"{tag} delta={delta}, axial={axial:.3f}, lateral={lateral:.3f}",
        )

    # --- Cap seat overlap allowances (cap embeds slightly into its collar and stub) ---
    for tag, cap in zip(outlet_tags, caps):
        ctx.allow_overlap(
            cap, body,
            elem_a=f"{tag}_cap_body",
            elem_b=f"{tag}_collar",
            reason="Lift-off cap is seated over its outlet collar; small seating embed is intended.",
        )
        ctx.allow_overlap(
            cap, body,
            elem_a=f"{tag}_cap_body",
            elem_b=f"{tag}_stub",
            reason="Lift-off cap wraps over the outlet stub when seated; this nesting is intended.",
        )

    # --- Tethers: body eye plus articulated round-link chain from cap to eye ---
    for tag in outlet_tags:
        tether = object_model.get_part(f"{tag}_tether")
        cap = object_model.get_part(f"{tag}_cap")
        chain_links = [
            object_model.get_part(f"{tag}_chain_{i}") for i in range(CHAIN_LINK_COUNTS[tag])
        ]
        # The eye is set into the chamber wall to anchor it.
        ctx.allow_overlap(
            tether, body, elem_a=f"{tag}_chain_eye", elem_b="valve_chamber",
            reason="Chain anchor eye is cast into the body wall (mounted, not a collision).",
        )
        ctx.expect_contact(tether, body, contact_tol=0.006, name=f"{tag}_tether_anchored")

        root_joint = object_model.get_articulation(f"{tag}_chain_swing_0")
        root_parent = getattr(root_joint.parent, "name", root_joint.parent)
        root_child = getattr(root_joint.child, "name", root_joint.child)
        ctx.check(
            f"{tag}_chain_rooted_on_cap",
            root_parent == cap.name and root_child == chain_links[0].name,
            details=f"root parent={root_parent}, child={root_child}",
        )
        ctx.check(
            f"{tag}_chain_has_enough_round_links",
            len(chain_links) >= 5,
            details=f"link count={len(chain_links)}",
        )

        for ci in range(CHAIN_LINK_COUNTS[tag]):
            cj = object_model.get_articulation(f"{tag}_chain_swing_{ci}")
            ctx.check(
                f"{tag}_chain_joint_{ci}_is_revolute",
                cj.articulation_type == ArticulationType.REVOLUTE
                and abs(cj.motion_limits.lower + CHAIN_SWING) < 1e-6
                and abs(cj.motion_limits.upper - CHAIN_SWING) < 1e-6,
                details=f"type={cj.articulation_type}, limits={cj.motion_limits}",
            )

        ctx.expect_contact(
            chain_links[0], cap, contact_tol=0.014, name=f"{tag}_chain_root_touches_cap_ring"
        )
        ctx.expect_contact(
            chain_links[-1], tether, contact_tol=0.030, name=f"{tag}_chain_tail_reaches_body_eye"
        )
        for a, b in zip(chain_links, chain_links[1:]):
            ctx.allow_overlap(
                a, b, reason="Consecutive round chain links interlink intentionally."
            )
        ctx.allow_overlap(
            chain_links[0], cap,
            reason="The chain root link passes through the cap tether ring.",
        )
        ctx.allow_overlap(
            chain_links[-1], tether,
            reason="The final chain link wraps the body eye.",
        )
        ctx.allow_overlap(
            chain_links[-1], body,
            elem_a="round_body",
            elem_b="valve_chamber",
            reason="The chain tail link rests against the chamber wall near the tether eye anchor.",
        )

    return ctx.report()


object_model = build_object_model()
