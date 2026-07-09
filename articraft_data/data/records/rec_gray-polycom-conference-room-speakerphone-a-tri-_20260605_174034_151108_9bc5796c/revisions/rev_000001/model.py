from __future__ import annotations

# Gray Polycom conference-room speakerphone.
# Frame: flat tri-star (Y) console lying in the XY plane, low profile along +Z.
#   - z=0 is the desk/underside; the body is a thin slightly-domed slab.
#   - Three arms radiate from a central hub at 120 deg. One arm (toward +Y) is the
#     CONTROL arm carrying the LCD, soft keys, a 4x3 numeric keypad, call/end keys,
#     and volume keys. The other two arms (lower-left, lower-right) carry small
#     secondary mic/speaker grilles.
#   - The central hub has a large round perforated SPEAKER grille.
# Articulations:
#   - ~18 keys (12 numeric, 4 soft, green call, red end, vol up/down) each a
#     PRISMATIC press straight down (-Z, ~1mm travel). Built in a loop.
#   - Body + speaker grille + LCD + secondary grilles are STATIC (one root part).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- Global geometry constants ----------------------------------------------
BODY_Z = 0.050           # overall body height (low profile)
SLAB_BOT = 0.006         # underside ring height
SLAB_TOP = 0.030         # nominal top deck height of the arms
HUB_TOP = 0.038          # central dome top (shallow dome)
ARM_LEN = 0.130          # arm reach from center to tip
ARM_HALF_W = 0.052       # half-width of an arm near the hub
ARM_TIP_HALF_W = 0.028   # half-width of an arm near the tip
HUB_R = 0.058            # central hub radius

# Control arm points toward +Y. Secondary arms at +120 and -120 from it.
CONTROL_ANGLE = math.pi / 2.0
ARM_ANGLES = (CONTROL_ANGLE, CONTROL_ANGLE + 2.0 * math.pi / 3.0, CONTROL_ANGLE - 2.0 * math.pi / 3.0)

SPEAKER_R = 0.044        # round central speaker grille radius
# The speaker/dome sit slightly back (-Y) from the geometric center so the +Y
# control arm has a clear flat deck for the keypad; still reads as central.
HUB_OFFSET_Y = -0.016
DECK_Z = SLAB_TOP        # working top surface of control deck


def _arm_wire(angle: float) -> cq.Workplane:
    # A rounded tapered arm footprint as a closed wire in the XY plane,
    # pointing along `angle` from origin. Built then rounded with fillets.
    ca, sa = math.cos(angle), math.sin(angle)
    # local frame: u = outward, v = sideways
    def world(u: float, v: float) -> tuple[float, float]:
        x = u * ca - v * sa
        y = u * sa + v * ca
        return (x, y)

    pts = [
        world(0.020, ARM_HALF_W),
        world(ARM_LEN * 0.55, ARM_HALF_W * 0.92),
        world(ARM_LEN, ARM_TIP_HALF_W),
        world(ARM_LEN + 0.012, 0.0),
        world(ARM_LEN, -ARM_TIP_HALF_W),
        world(ARM_LEN * 0.55, -ARM_HALF_W * 0.92),
        world(0.020, -ARM_HALF_W),
    ]
    wp = cq.Workplane("XY").polyline(pts).close()
    return wp


def _body_solid() -> cq.Workplane:
    # Tri-star slab: a central disc unioned with three tapered arms, extruded thin
    # and given a gentle top dome by lofting the hub. Then the top edge is filleted
    # so it reads as a smooth molded console.
    base = cq.Workplane("XY").circle(HUB_R).extrude(SLAB_TOP)
    for ang in ARM_ANGLES:
        arm = _arm_wire(ang).extrude(SLAB_TOP)
        base = base.union(arm)
    # Gentle central dome on top of the hub (slightly back toward -Y).
    dome = (
        cq.Workplane("XY")
        .center(0.0, HUB_OFFSET_Y)
        .workplane(offset=SLAB_TOP)
        .circle(HUB_R * 0.90)
        .workplane(offset=(HUB_TOP - SLAB_TOP))
        .circle(HUB_R * 0.60)
        .loft(ruled=False)
    )
    base = base.union(dome)
    # Round the bottom outer edge so the console reads as molded and sits flat.
    try:
        base = base.edges("|Z").fillet(0.010)
    except Exception:
        pass
    try:
        base = base.faces(">Z").edges().fillet(0.004)
    except Exception:
        pass
    return base


def _speaker_grille_mesh():
    # Round perforated speaker face recessed into the central dome top.
    panel = PerforatedPanelGeometry(
        (2 * SPEAKER_R + 0.006, 2 * SPEAKER_R + 0.006),
        0.004,
        hole_diameter=0.0042,
        pitch=(0.0085, 0.0085),
        frame=0.006,
        corner_radius=SPEAKER_R,  # rounds the square panel into a disc
        stagger=True,
        center=True,
    )
    return mesh_from_geometry(panel, "speaker_grille")


def _secondary_grille_mesh(name: str):
    panel = PerforatedPanelGeometry(
        (0.046, 0.046),
        0.003,
        hole_diameter=0.0034,
        pitch=(0.007, 0.007),
        frame=0.005,
        corner_radius=0.022,
        stagger=True,
        center=True,
    )
    return mesh_from_geometry(panel, name)


def _keycap_mesh(name: str, w: float, d: float, h: float):
    # Small rounded keycap: a short loft from a wide base to a slightly smaller,
    # rounded top so the cap reads as proud and pressable.
    cap = (
        cq.Workplane("XY")
        .rect(w, d)
        .workplane(offset=h * 0.6)
        .rect(w * 0.96, d * 0.96)
        .workplane(offset=h * 0.4)
        .rect(w * 0.82, d * 0.82)
        .loft(ruled=False)
    )
    try:
        cap = cap.edges(">Z").fillet(min(w, d) * 0.12)
    except Exception:
        pass
    return mesh_from_cadquery(cap, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conference_speakerphone")

    body_gray = model.material("body_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.13, 0.13, 0.14, 1.0))
    lcd_dark = model.material("lcd_dark", rgba=(0.10, 0.16, 0.14, 1.0))
    key_gray = model.material("key_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    call_green = model.material("call_green", rgba=(0.15, 0.62, 0.28, 1.0))
    end_red = model.material("end_red", rgba=(0.72, 0.16, 0.16, 1.0))
    bezel = model.material("bezel", rgba=(0.18, 0.19, 0.21, 1.0))

    # ---- BODY (root) --------------------------------------------------------
    body = model.part("body")
    body_solid = _body_solid()
    body.visual(mesh_from_cadquery(body_solid, "body_shell"), material=body_gray, name="body_shell")

    # Central round speaker grille, recessed slightly below the dome top.
    grille_z = HUB_TOP - 0.001
    body.visual(
        _speaker_grille_mesh(),
        origin=Origin(xyz=(0.0, HUB_OFFSET_Y, grille_z)),
        material=grille_dark,
        name="speaker_grille",
    )
    # A thin dark rim ring around the speaker for definition.
    rim = CylinderGeometry(SPEAKER_R + 0.004, 0.004, radial_segments=64).translate(
        0.0, HUB_OFFSET_Y, HUB_TOP - 0.003
    )
    body.visual(mesh_from_geometry(rim, "speaker_rim"), material=grille_dark, name="speaker_rim")

    # Secondary mic/speaker grilles on the two non-control arms.
    for i, ang in enumerate(ARM_ANGLES[1:]):
        gx = math.cos(ang) * (ARM_LEN * 0.66)
        gy = math.sin(ang) * (ARM_LEN * 0.66)
        body.visual(
            _secondary_grille_mesh(f"secondary_grille_{i}"),
            origin=Origin(xyz=(gx, gy, SLAB_TOP - 0.001)),
            material=grille_dark,
            name=f"secondary_grille_{i}",
        )

    # ----- Control deck on the +Y control arm (recessed LCD + key wells) -----
    # The control arm runs along +Y, usable deck roughly y in [0.050, 0.128].
    # Speaker grille is centered at the hub; keys live above it on the +Y arm.
    # LCD recess (dark) near the hub end of the control arm (highest +Y).
    lcd_y = 0.118
    body.visual(
        Box((0.066, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, lcd_y, DECK_Z - 0.002)),
        material=lcd_dark,
        name="lcd_display",
    )
    # LCD bezel surround.
    body.visual(
        Box((0.078, 0.036, 0.006)),
        origin=Origin(xyz=(0.0, lcd_y, DECK_Z - 0.004)),
        material=bezel,
        name="lcd_bezel",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.26, 0.26, BODY_Z)), mass=1.1, origin=Origin(xyz=(0.0, 0.02, SLAB_TOP * 0.5))
    )

    # ---- KEYS (prismatic press straight down) -------------------------------
    # Each cap is a separate part embedded slightly into the deck (seated well)
    # and pressing straight down (-Z) ~1mm. The shallow embed is intentional and
    # allowed below as a seated-in-well overlap.
    deck_top = DECK_Z
    cap_embed = 0.0020  # how far the cap base sinks into the deck (seated well)
    press_limit = MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.001)

    key_specs: list[tuple[str, float, float, float, float, float, object]] = []
    # (name, x, y, w, d, cap_h, material)

    # Soft keys: a row of 4 just below the LCD.
    soft_y = lcd_y - 0.018
    soft_w, soft_d, soft_h = 0.013, 0.008, 0.005
    for c in range(4):
        sx = -0.024 + c * 0.016
        key_specs.append((f"soft_key_{c}", sx, soft_y, soft_w, soft_d, soft_h, soft_gray))

    # Numeric keypad 4 rows x 3 cols: 1 2 3 / 4 5 6 / 7 8 9 / * 0 #
    keypad_labels = [
        ["1", "2", "3"],
        ["4", "5", "6"],
        ["7", "8", "9"],
        ["star", "0", "pound"],
    ]
    kp_w, kp_d, kp_h = 0.014, 0.011, 0.006
    kp_x0 = -0.018          # leftmost column center x
    kp_dx = 0.018           # column pitch
    kp_y0 = soft_y - 0.016  # first (top) keypad row y
    kp_dy = 0.0145          # row pitch (rows go toward -Y / down the arm)
    for r, row in enumerate(keypad_labels):
        for c, lab in enumerate(row):
            kx = kp_x0 + c * kp_dx
            ky = kp_y0 - r * kp_dy
            key_specs.append((f"keypad_{r}_{c}", kx, ky, kp_w, kp_d, kp_h, key_gray))

    kp_bottom_y = kp_y0 - 3 * kp_dy  # y of the bottom keypad row (* 0 #)

    # Call control keys flank the keypad sides: green call (left), red end (right).
    cc_y = kp_y0 - 1.5 * kp_dy  # vertically centered on the keypad
    key_specs.append(("call_key", -0.040, cc_y, 0.015, 0.012, 0.006, call_green))
    key_specs.append(("end_key", 0.040, cc_y, 0.015, 0.012, 0.006, end_red))

    # Volume up / down keys in a short row just below the keypad.
    vol_y = kp_bottom_y - 0.015
    key_specs.append(("volume_up_key", -0.012, vol_y, 0.014, 0.010, 0.005, soft_gray))
    key_specs.append(("volume_down_key", 0.012, vol_y, 0.014, 0.010, 0.005, soft_gray))

    # Build each key part with a prismatic joint pressing down (-Z).
    for name, kx, ky, w, d, cap_h, mat in key_specs:
        part = model.part(name)
        part.visual(
            _keycap_mesh(f"{name}_cap", w, d, cap_h),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=mat,
            name=f"{name}_cap",
        )
        part.inertial = Inertial.from_geometry(
            Box((w, d, cap_h)), mass=0.004, origin=Origin(xyz=(0.0, 0.0, cap_h * 0.5))
        )
        # Cap base sinks `cap_embed` into the deck so it reads as seated in a well.
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=part,
            origin=Origin(xyz=(kx, ky, deck_top - cap_embed)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=press_limit,
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


KEY_NAMES: list[str] = (
    [f"soft_key_{c}" for c in range(4)]
    + [f"keypad_{r}_{c}" for r in range(4) for c in range(3)]
    + ["call_key", "end_key", "volume_up_key", "volume_down_key"]
)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")

    # Every keycap is seated into a shallow deck well: its base intentionally
    # embeds a little into the body shell so it reads as a real pressable key in
    # a recessed well. Scope each allowance to the cap/body-shell element pair.
    for kname in KEY_NAMES:
        kpart = object_model.get_part(kname)
        ctx.allow_overlap(
            kpart,
            body,
            elem_a=f"{kname}_cap",
            elem_b="body_shell",
            reason="Keycap base is intentionally seated into its shallow deck well.",
        )

    # ---- Body is a tri-star: three arms reach out at ~120 deg ----
    # Confirm by sampling the body shell footprint extent along the three arm
    # directions. Use overall span: a tri-star is broad in X and Y.
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body has a broad flat tri-star footprint",
        body_ext[0] > 0.18 and body_ext[1] > 0.18 and body_ext[2] < 0.07,
        details=f"body extents={body_ext}",
    )

    # ---- Central round speaker grille is central ----
    grille_aabb = ctx.part_element_world_aabb(body, elem="speaker_grille")
    gmn, gmx = grille_aabb
    gcx = (gmn[0] + gmx[0]) * 0.5
    gcy = (gmn[1] + gmx[1]) * 0.5
    gw = gmx[0] - gmn[0]
    gh = gmx[1] - gmn[1]
    ctx.check(
        "speaker grille is centered on the hub",
        abs(gcx) < 0.02 and abs(gcy) < 0.02,
        details=f"grille center=({gcx:.4f},{gcy:.4f})",
    )
    ctx.check(
        "speaker grille is round (roughly equal width/height)",
        abs(gw - gh) < 0.012 and gw > 0.07,
        details=f"grille w={gw:.4f}, h={gh:.4f}",
    )

    # ---- Keypad lives on one (the +Y control) arm ----
    # Representative keys: a number key, *, #, a soft key, the call key.
    reps = [
        ("keypad_0_0", "number key 1"),
        ("keypad_3_0", "star key"),
        ("keypad_3_2", "pound key"),
        ("soft_key_0", "soft key"),
        ("call_key", "green call key"),
        ("end_key", "red end key"),
    ]

    for pname, label in reps:
        part = object_model.get_part(pname)
        pos = ctx.part_world_position(part)
        # On the control arm => positive Y region.
        ctx.check(
            f"{label} sits on the control arm (+Y)",
            pos is not None and pos[1] > 0.0,
            details=f"{pname} pos={pos}",
        )

    # Keypad spread spans a contiguous block on the control arm (a real keypad).
    k_top = ctx.part_world_position(object_model.get_part("keypad_0_1"))
    k_bot = ctx.part_world_position(object_model.get_part("keypad_3_1"))
    ctx.check(
        "keypad forms a column block down the control arm",
        k_top is not None and k_bot is not None and k_top[1] > k_bot[1] + 0.03,
        details=f"top={k_top}, bottom={k_bot}",
    )

    # ---- Each sampled key presses straight down and stays seated ----
    for pname, label in reps:
        part = object_model.get_part(pname)
        joint = object_model.get_articulation(f"body_to_{pname}")
        rest = ctx.part_world_position(part)
        with ctx.pose({joint: 0.001}):
            pressed = ctx.part_world_position(part)
        ctx.check(
            f"{label} presses straight down",
            rest is not None
            and pressed is not None
            and pressed[2] < rest[2] - 0.0006
            and abs(pressed[0] - rest[0]) < 1e-4
            and abs(pressed[1] - rest[1]) < 1e-4,
            details=f"{pname} rest={rest}, pressed={pressed}",
        )
        # Cap stays seated in its deck well: its base sits at the deck surface
        # (small embed), not floating above and not sunk to the desk.
        cap_aabb = ctx.part_element_world_aabb(part, elem=f"{pname}_cap")
        cap_base_z = cap_aabb[0][2]
        ctx.check(
            f"{label} cap is seated at the deck surface",
            abs(cap_base_z - (SLAB_TOP - 0.002)) < 0.0025,
            details=f"{pname} cap_base_z={cap_base_z:.4f} (deck~{SLAB_TOP:.3f})",
        )
        # Cap footprint actually sits over the body deck (no floating off the arm).
        ctx.expect_overlap(
            part,
            body,
            axes="xy",
            elem_a=f"{pname}_cap",
            elem_b="body_shell",
            min_overlap=0.006,
            name=f"{label} cap sits over the deck",
        )

    return ctx.report()


object_model = build_object_model()
