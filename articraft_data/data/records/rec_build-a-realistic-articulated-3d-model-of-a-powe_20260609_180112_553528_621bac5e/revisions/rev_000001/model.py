from __future__ import annotations

# Industrial pendant control station (hanging control box), modeled from
# picture/Equipment/Power switch/002.png.
#
# Identity (per the reference + brief):
#   Main body: a rounded-rectangular housing with chamfered octagonal corners,
#   matte gray engineering plastic. Two metallic conduit TUBES enter the top
#   through a WHITE sealing gland/bushing collar (the tubes are FIXED). The front
#   face carries a recessed dark-gray field. At its centre sits a raised
#   rectangular selector faceplate with FOUR corner screws and a tall central
#   SLOT; a grab-handle SLIDER rides in that slot and is pulled UP / pushed DOWN
#   (a linear disconnect/isolator lever). The faceplate is flanked by a tidy
#   cage of EIGHT small dark bars (3 horizontal rungs per side + 2 top struts)
#   running between two vertical side rails. A bottom row of three mushroom-cap
#   push-buttons can be pressed.
#
# Coordinate convention (meters):
#   X = width (horizontal), Y = height (+Y up),
#   Z = depth out of the panel toward the viewer (+Z faces the room).
#
# Articulation (4 joints):
#   * housing_to_slider     -- PRISMATIC along +Y, the central grab-handle that
#                              slides UP and DOWN inside the faceplate slot.
#   * housing_to_button_{0,1,2} -- PRISMATIC, each mushroom button presses in (-Z).
#   The two top conduit tubes are part of the fixed housing (intentionally NOT
#   jointed).

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

# ----------------------------------------------------------------------------
# Real-world dimensions (meters).
# ----------------------------------------------------------------------------
PLATE_W = 0.090          # housing width (X)
PLATE_H = 0.110          # housing height (Y)
PLATE_T = 0.014          # housing thickness (Z, toward viewer +Z)
PLATE_CHAMFER = 0.020    # octagonal corner cut

RECESS_INSET = 0.009     # how far the recessed field sits in from the edge
RECESS_DEPTH = 0.005     # recessed front pocket depth (inset below the shell)
FIELD_FRONT = PLATE_T - RECESS_DEPTH   # z of the recessed field front face

# ---- Dark cage in the recessed field: 8 small bars + 2 side rails ----------
BAR_PROUD = 0.0022       # bars stand just above the recessed field floor
BAR_THK = 0.0050         # horizontal-rung / strut bar thickness (the short side)
RAIL_THK = 0.0042        # vertical side-rail thickness
CAGE_CY = 0.006          # vertical centre of the cage / faceplate group
RAIL_X = 0.0265          # |X| of the two vertical side rails
RAIL_HALF_H = 0.0210     # half height of the side rails
RUNG_LEVELS = (0.0130, 0.0, -0.0130)   # 3 rung y-offsets from CAGE_CY (per side)
STRUT_X = 0.0070         # |X| of the two top vertical struts
STRUT_TOP = 0.0300       # strut upper y offset from CAGE_CY (toward the collar)

# ---- Central rectangular selector faceplate (raised) -----------------------
FACE_W = 0.0300
FACE_H = 0.0380
FACE_PROUD = 0.0024      # faceplate sits proud of the cage bars
FACE_CHAMFER = 0.0045    # octagonal faceplate corners
SCREW_INSET = 0.0042     # corner screws inset from the faceplate edges

# ---- Central SLOT + grab-handle SLIDER (prismatic, vertical) ---------------
SLOT_W = 0.0095          # slot width (X)
SLOT_H = 0.0270          # slot height (Y) -- the travel channel
SLIDER_W = 0.0072        # slider block width (rides inside the slot)
SLIDER_H = 0.0098        # slider block height
SLIDER_CLEAR = 0.0010    # clearance at each slot end
SLIDER_TRAVEL = SLOT_H - SLIDER_H - 2.0 * SLIDER_CLEAR   # up/down stroke
# Rest = slider parked at the BOTTOM of the slot (lever in the "down" position).
SLIDER_REST_CY = CAGE_CY - SLOT_H / 2.0 + SLIDER_CLEAR + SLIDER_H / 2.0

# Top collar (white gland) and the two FIXED metallic conduit tubes.
COLLAR_CY = PLATE_H / 2.0 - 0.006
COLLAR_W = 0.026
COLLAR_H = 0.013
COLLAR_PROUD = 0.005
TUBE_R = 0.0036
TUBE_X = 0.008
TUBE_Z = PLATE_T + 0.001
TUBE_BOTTOM = COLLAR_CY - 0.004
TUBE_TOP = PLATE_H / 2.0 + 0.090

# Bottom row of mushroom-cap push-buttons (pressable).
BTN_CAP_R = 0.0058       # cap radius (rests on the shell surface -> real contact)
BTN_SHAFT_R = 0.0040     # narrower shaft seated in the bore
BTN_BORE_R = 0.0046      # mounting bore radius
BTN_PROUD = 0.0026       # low domed cap proud of the shell at rest
BTN_GAP = 0.016
BTN_Y = -PLATE_H / 2.0 + 0.013
BTN_PRESS = 0.0020       # press-in travel (-Z)


def _octagon(w: float, h: float, chamfer: float) -> list[tuple[float, float]]:
    hx, hy, c = w / 2.0, h / 2.0, chamfer
    return [
        (-hx + c, -hy), (hx - c, -hy), (hx, -hy + c), (hx, hy - c),
        (hx - c, hy), (-hx + c, hy), (-hx, hy - c), (-hx, -hy + c),
    ]


def _build_shell_mesh():
    """Octagonal ABS housing shell with a recessed front field inset below the rim."""
    outer = _octagon(PLATE_W, PLATE_H, PLATE_CHAMFER)
    plate = cq.Workplane("XY").polyline(outer).close().extrude(PLATE_T)

    inner = _octagon(
        PLATE_W - 2.0 * RECESS_INSET,
        PLATE_H - 2.0 * RECESS_INSET,
        PLATE_CHAMFER - 0.005,
    )
    pocket = (
        cq.Workplane("XY")
        .workplane(offset=FIELD_FRONT)
        .polyline(inner)
        .close()
        .extrude(RECESS_DEPTH + 0.001)
    )
    plate = plate.cut(pocket)

    # Four small corner mounting-screw dimples sunk into the border.
    for sx in (-1, 1):
        for sy in (-1, 1):
            bx = sx * (PLATE_W / 2.0 - 0.008)
            by = sy * (PLATE_H / 2.0 - 0.008)
            screw = (
                cq.Workplane("XY")
                .workplane(offset=PLATE_T)
                .center(bx, by)
                .circle(0.0021)
                .extrude(-0.004)
            )
            plate = plate.cut(screw)

    # Fillet the clean outer/pocket edges BEFORE cutting the deep button bores
    # (filleting after the bores leaves edges OCC cannot resolve).
    plate = plate.edges("|Z").fillet(0.0015)

    # Deep blind mounting bores the three buttons plug into (so each button shaft
    # is seated INSIDE the housing rather than floating above the surface).
    for sx in (-1, 0, 1):
        bore = (
            cq.Workplane("XY")
            .center(sx * BTN_GAP, BTN_Y)
            .workplane(offset=PLATE_T + 0.001)
            .circle(BTN_BORE_R)
            .extrude(-0.009)
        )
        plate = plate.cut(bore)

    return plate


def _build_bezels_mesh():
    """Raised bezel rings framing each button hole (part of the housing) so the
    buttons read as flush-mounted hardware, not floating discs."""
    asm = None
    for sx in (-1, 0, 1):
        ring_o = (
            cq.Workplane("XY")
            .center(sx * BTN_GAP, BTN_Y)
            .workplane(offset=PLATE_T - 0.001)
            .circle(BTN_CAP_R + 0.0022)
            .extrude(0.0022)
        )
        ring_i = (
            cq.Workplane("XY")
            .center(sx * BTN_GAP, BTN_Y)
            .workplane(offset=PLATE_T - 0.002)
            .circle(BTN_CAP_R + 0.0004)
            .extrude(0.005)
        )
        ring = ring_o.cut(ring_i)
        asm = ring if asm is None else asm.union(ring)
    return asm


# Eight small cage bars (the "周围八个小条"): 3 horizontal rungs on each side of
# the faceplate + 2 vertical top struts. Documented here so the count is testable.
def _cage_bar_specs() -> list[tuple[float, float, float, float]]:
    """Return (cx, cy, w, h) for each of the 8 small bars, in field coordinates."""
    specs: list[tuple[float, float, float, float]] = []
    rung_len = RAIL_X - FACE_W / 2.0 + 0.0015     # rail -> faceplate span (overlaps both)
    rung_cx = (RAIL_X + FACE_W / 2.0) / 2.0
    for dy in RUNG_LEVELS:                          # 3 left + 3 right horizontal rungs
        specs.append((-rung_cx, CAGE_CY + dy, rung_len, BAR_THK))
        specs.append((+rung_cx, CAGE_CY + dy, rung_len, BAR_THK))
    strut_bot = FACE_H / 2.0 - 0.0015               # start at the faceplate top edge
    strut_cy = CAGE_CY + (STRUT_TOP + strut_bot) / 2.0
    strut_h = STRUT_TOP - strut_bot
    for sx in (-1, 1):                              # 2 vertical top struts
        specs.append((sx * STRUT_X, strut_cy, BAR_THK, strut_h))
    return specs


CAGE_BAR_SPECS = _cage_bar_specs()


FIELD_BD_W = 0.060       # dark field backdrop behind the cage (gives the recess depth)
FIELD_BD_H = 0.064
FIELD_BD_CY = CAGE_CY
FIELD_BD_CHAMFER = 0.012


def _build_field_backdrop_mesh():
    """Thin dark field plate filling the recess behind the cage so the bars and
    faceplate read against a dark sunken background (as in the reference)."""
    return (
        cq.Workplane("XY")
        .center(0.0, FIELD_BD_CY)
        .workplane(offset=FIELD_FRONT + 0.0002)
        .polyline(_octagon(FIELD_BD_W, FIELD_BD_H, FIELD_BD_CHAMFER))
        .close()
        .extrude(0.0008)
        .edges("|Z")
        .fillet(0.0010)
    )


def _build_cage_mesh():
    """Dark cage in the recessed field: two vertical side rails plus the EIGHT
    small bars (3 horizontal rungs per side + 2 top struts) that frame the
    central faceplate."""

    def slab(w, h, cx=0.0, cy=0.0, proud=BAR_PROUD):
        return (
            cq.Workplane("XY")
            .center(cx, cy)
            .workplane(offset=FIELD_FRONT)
            .rect(w, h)
            .extrude(proud)
        )

    # Two vertical side rails (a touch lower so the bars read as the focal frame).
    asm = slab(RAIL_THK, RAIL_HALF_H * 2.0, cx=-RAIL_X, cy=CAGE_CY, proud=BAR_PROUD - 0.0004)
    asm = asm.union(slab(RAIL_THK, RAIL_HALF_H * 2.0, cx=RAIL_X, cy=CAGE_CY, proud=BAR_PROUD - 0.0004))

    # The eight small bars.
    for (cx, cy, w, h) in CAGE_BAR_SPECS:
        asm = asm.union(slab(w, h, cx=cx, cy=cy))

    # Soften the bar edges so the cage reads crisp/refined, not slab-like.
    asm = asm.edges("|Z").fillet(0.0008)
    return asm


def _build_faceplate_mesh():
    """Raised octagonal selector faceplate carrying four corner screw heads and a
    tall central SLOT. The grab-handle slider (a separate moving part) rides in
    the slot."""
    # Faceplate sits ON the recessed field floor and stands proud of the cage bars.
    face_back = FIELD_FRONT
    face_h = BAR_PROUD + FACE_PROUD
    plate = (
        cq.Workplane("XY")
        .center(0.0, CAGE_CY)
        .workplane(offset=face_back)
        .polyline(_octagon(FACE_W, FACE_H, FACE_CHAMFER))
        .close()
        .extrude(face_h)
        .edges("|Z")
        .fillet(0.0012)
    )
    face_front = face_back + face_h

    # Tall central slot (a recessed channel the slider travels in).
    slot = (
        cq.Workplane("XY")
        .center(0.0, CAGE_CY)
        .workplane(offset=face_front + 0.001)
        .rect(SLOT_W, SLOT_H)
        .extrude(-(FACE_PROUD + 0.0016))
        .edges("|Z")
        .fillet(0.0018)
    )
    plate = plate.cut(slot)

    # Four corner screw heads (zinc) with a single drive slot each.
    for sx in (-1, 1):
        for sy in (-1, 1):
            dx = sx * (FACE_W / 2.0 - SCREW_INSET)
            dy = CAGE_CY + sy * (FACE_H / 2.0 - SCREW_INSET)
            head = (
                cq.Workplane("XY")
                .center(dx, dy)
                .workplane(offset=face_front - 0.0004)
                .circle(0.0023)
                .extrude(0.0011)
                .edges(">Z")
                .fillet(0.0004)
            )
            slit = (
                cq.Workplane("XY")
                .center(dx, dy)
                .workplane(offset=face_front + 0.0006)
                .box(0.0042, 0.0008, 0.0010, centered=(True, True, False))
            )
            plate = plate.union(head).cut(slit)
    return plate


def _build_slider_mesh():
    """Central grab-handle slider authored in its joint frame (origin at the
    slider rest centre). A block rides in the faceplate slot; a forward-curving
    grab loop is the part you pull UP / push DOWN. Travels along +Y."""
    face_front = FIELD_FRONT + BAR_PROUD + FACE_PROUD
    block_front = face_front + 0.0006
    # Block back bears on the recessed field floor behind the slot (a tiny
    # intentional overlap) so the slider is supported by the housing.
    block_back = FIELD_FRONT - 0.0003

    # Slider block seated in the slot (its front sits just proud of the faceplate).
    block = (
        cq.Workplane("XY")
        .workplane(offset=block_back)
        .rect(SLIDER_W, SLIDER_H)
        .extrude(block_front - block_back)
        .edges("|Z")
        .fillet(0.0010)
    )

    # Neck joining the block to the forward grab loop (loop hangs low on the block).
    # It must reach down past the loop's inner radius so the loop stays connected.
    neck = (
        cq.Workplane("XY")
        .center(0.0, -0.0022)
        .workplane(offset=block_front - 0.0008)
        .rect(0.0036, 0.0084)
        .extrude(0.0030)
    )

    # Forward-protruding grab loop (the finger pull) -- a flat ring lying proud of
    # the panel, the dark handle seen in the reference.
    loop_cy = -0.0032
    loop_z = block_front + 0.0008
    ring_outer = (
        cq.Workplane("XY")
        .center(0.0, loop_cy)
        .workplane(offset=loop_z)
        .circle(0.0072)
        .extrude(0.0030)
    )
    ring_inner = (
        cq.Workplane("XY")
        .center(0.0, loop_cy)
        .workplane(offset=loop_z - 0.0004)
        .circle(0.0040)
        .extrude(0.0038)
    )
    loop = ring_outer.cut(ring_inner)
    return block.union(neck).union(loop)


def _build_button_mesh():
    """One push-button in its own joint frame (origin at the button centre on the
    shell front). A narrow shaft seats inside the housing bore; a wider domed cap
    RESTS ON the shell surface (the cap underside bears on the shell around the
    bore), so the button is physically supported -- mounted, not floating."""
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_T - 0.0050)
        .circle(BTN_SHAFT_R)
        .extrude(0.0050 + 0.001)
    )
    cap = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_T - 0.0006)
        .circle(BTN_CAP_R)
        .extrude(0.0006 + BTN_PROUD)
        .edges(">Z")
        .fillet(0.0014)
    )
    return shaft.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pendant_control_station")

    abs_gray = Material(name="abs_gray", rgba=(0.55, 0.56, 0.58, 1.0))
    dark_gray = Material(name="dark_gray", rgba=(0.27, 0.28, 0.30, 1.0))
    field_dark = Material(name="field_dark", rgba=(0.34, 0.35, 0.37, 1.0))
    white_gland = Material(name="white_gland", rgba=(0.88, 0.89, 0.90, 1.0))
    conduit_metal = Material(name="conduit_metal", rgba=(0.52, 0.54, 0.57, 1.0))
    # Medium-gray selector faceplate so it reads like the reference's integrated
    # recessed panel (lighter than the dark cage, darker than the shell).
    faceplate_metal = Material(name="faceplate_metal", rgba=(0.44, 0.45, 0.47, 1.0))
    screw_zinc = Material(name="screw_zinc", rgba=(0.76, 0.78, 0.80, 1.0))
    anodized = Material(name="anodized_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    gunmetal = Material(name="gunmetal", rgba=(0.26, 0.27, 0.29, 1.0))

    # ---- Housing (root) -----------------------------------------------------
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_shell_mesh(), "housing_shell"),
        material=abs_gray,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_build_field_backdrop_mesh(), "field_backdrop"),
        material=field_dark,
        name="field_backdrop",
    )
    housing.visual(
        mesh_from_cadquery(_build_cage_mesh(), "cage_bars"),
        material=dark_gray,
        name="cage_bars",
    )
    housing.visual(
        mesh_from_cadquery(_build_bezels_mesh(), "button_bezels"),
        material=abs_gray,
        name="button_bezels",
    )
    housing.visual(
        mesh_from_cadquery(_build_faceplate_mesh(), "selector_faceplate"),
        material=faceplate_metal,
        name="selector_faceplate",
    )

    # White sealing gland collar (static, on the housing).
    collar = (
        cq.Workplane("XY")
        .center(0.0, COLLAR_CY)
        .workplane(offset=PLATE_T - 0.002)
        .rect(COLLAR_W, COLLAR_H)
        .extrude(COLLAR_PROUD + 0.002)
        .edges("|Z")
        .fillet(0.0025)
    )
    housing.visual(
        mesh_from_cadquery(collar, "gland_collar"),
        material=white_gland,
        name="gland_collar",
    )

    # Two FIXED metallic conduit tubes (no joint).
    tubes = None
    tube_len = TUBE_TOP - TUBE_BOTTOM
    for sx in (-1, 1):
        tube = (
            cq.Workplane("XZ")
            .workplane(offset=-TUBE_TOP)
            .center(sx * TUBE_X, TUBE_Z)
            .circle(TUBE_R)
            .extrude(tube_len)
        )
        tubes = tube if tubes is None else tubes.union(tube)
    housing.visual(
        mesh_from_cadquery(tubes, "conduit_tubes"),
        material=conduit_metal,
        name="conduit_tubes",
    )

    # ---- Central grab-handle slider (prismatic, vertical +Y) ---------------
    slider = model.part("slider")
    slider.visual(
        mesh_from_cadquery(_build_slider_mesh(), "slide_handle"),
        material=anodized,
        name="slide_handle",
    )
    model.articulation(
        "housing_to_slider",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=slider,
        origin=Origin(xyz=(0.0, SLIDER_REST_CY, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=0.1, lower=0.0, upper=SLIDER_TRAVEL
        ),
    )

    # ---- Three pressable mushroom-cap buttons (prismatic, -Z) --------------
    for i, sx in enumerate((-1, 0, 1)):
        bx = sx * BTN_GAP
        btn = model.part(f"button_{i}")
        btn.visual(
            mesh_from_cadquery(_build_button_mesh(), f"button_{i}_cap"),
            material=gunmetal,
            name=f"button_{i}_cap",
        )
        model.articulation(
            f"housing_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=btn,
            origin=Origin(xyz=(bx, BTN_Y, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=0.1, lower=0.0, upper=BTN_PRESS
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    slider = object_model.get_part("slider")
    slide_joint = object_model.get_articulation("housing_to_slider")

    # The slider block seats into the faceplate slot and bears on the recessed
    # field floor behind it (intended assembly overlaps / support contact).
    ctx.allow_overlap(
        housing,
        slider,
        elem_a="selector_faceplate",
        elem_b="slide_handle",
        reason="The grab-handle slider rides inside the faceplate slot.",
    )
    ctx.allow_overlap(
        housing,
        slider,
        elem_a="housing_shell",
        elem_b="slide_handle",
        reason="The slider block bears on the recessed field floor behind the slot.",
    )

    # --- Tubes are FIXED housing geometry (explicitly NOT a joint) -----------
    joint_names = {a.name for a in object_model.articulations}
    ctx.check(
        "no joint drives the conduit tubes",
        not any("tube" in n.lower() or "conduit" in n.lower() for n in joint_names),
        details=f"joints={sorted(joint_names)}",
    )
    ctx.check(
        "conduit tubes belong to the fixed housing part",
        any(v.name == "conduit_tubes" for v in housing.visuals),
        details=str(sorted(v.name for v in housing.visuals)),
    )
    ctx.check(
        "housing has a white gland collar, cage bars and a faceplate",
        {"gland_collar", "cage_bars", "selector_faceplate"} <= {v.name for v in housing.visuals},
        details=str(sorted(v.name for v in housing.visuals)),
    )

    # --- The cage is built from exactly EIGHT small bars --------------------
    ctx.check(
        "faceplate is framed by exactly eight small cage bars",
        len(CAGE_BAR_SPECS) == 8,
        details=f"bar_count={len(CAGE_BAR_SPECS)}",
    )

    # --- Central slider joint contract (vertical up/down slide) -------------
    ctx.check(
        "central switch is a prismatic slider (not a rotary)",
        str(slide_joint.articulation_type).lower().endswith("prismatic"),
        details=f"type={slide_joint.articulation_type}",
    )
    sax = tuple(float(a) for a in slide_joint.axis)
    ctx.check(
        "slider travels vertically (along Y)",
        abs(sax[1]) > 0.99 and abs(sax[0]) < 1e-6 and abs(sax[2]) < 1e-6,
        details=f"axis={sax}",
    )
    ctx.check(
        "slider has a usable up/down stroke",
        SLIDER_TRAVEL > 0.008,
        details=f"travel={SLIDER_TRAVEL:.4f}",
    )

    # Pulling the handle up actually raises the slider (track its world AABB).
    def _slider_y():
        box = ctx.part_world_aabb(slider)
        return None if box is None else (float(box[0][1]), float(box[1][1]))

    with ctx.pose({slide_joint: 0.0}):
        down = _slider_y()
    with ctx.pose({slide_joint: SLIDER_TRAVEL}):
        up = _slider_y()
    ctx.check("slider span present", down is not None and up is not None, details=str(down))
    if down is not None and up is not None:
        raised = (up[0] - down[0]) > 0.008 and (up[1] - down[1]) > 0.008
        ctx.check(
            "sliding the handle up moves it up the slot",
            raised,
            details=f"down_y={down}, up_y={up}",
        )

    # The slider stays within the faceplate slot footprint (no sideways escape).
    slot_aabb = ctx.part_element_world_aabb(housing, elem="selector_faceplate")
    sl_aabb = ctx.part_world_aabb(slider)
    if slot_aabb is not None and sl_aabb is not None:
        ctx.check(
            "slider sits inside the faceplate width",
            sl_aabb[0][0] > slot_aabb[0][0] - 1e-4 and sl_aabb[1][0] < slot_aabb[1][0] + 1e-4,
            details=f"slider_x=({sl_aabb[0][0]:.4f},{sl_aabb[1][0]:.4f}), "
                    f"face_x=({slot_aabb[0][0]:.4f},{slot_aabb[1][0]:.4f})",
        )

    # --- Three pressable mushroom buttons -----------------------------------
    for i in range(3):
        btn = object_model.get_part(f"button_{i}")
        bj = object_model.get_articulation(f"housing_to_button_{i}")
        # The cap underside bears on the shell around the bore (a seated mounting
        # fit, not a floating disc): allow that small contact overlap.
        ctx.allow_overlap(
            housing,
            btn,
            elem_a="housing_shell",
            elem_b=f"button_{i}_cap",
            reason="Button cap is seated on the shell around its mounting bore.",
        )
        ctx.check(
            f"button {i} is a prismatic press",
            str(bj.articulation_type).lower().endswith("prismatic"),
            details=f"type={bj.articulation_type}",
        )
        bax = tuple(float(a) for a in bj.axis)
        ctx.check(
            f"button {i} presses along Z",
            abs(bax[2]) > 0.99 and abs(bax[0]) < 1e-6 and abs(bax[1]) < 1e-6,
            details=f"axis={bax}",
        )

        def _front(part=btn):
            box = ctx.part_world_aabb(part)
            return None if box is None else float(box[1][2])
        rest_z = _front()
        with ctx.pose({bj: BTN_PRESS}):
            pressed_z = _front()
        if rest_z is not None and pressed_z is not None:
            ctx.check(
                f"button {i} presses inward",
                pressed_z < rest_z - 0.0010,
                details=f"rest_z={rest_z:.4f}, pressed_z={pressed_z:.4f}",
            )

    # --- Octagonal housing proportions --------------------------------------
    shell_aabb = ctx.part_element_world_aabb(housing, elem="housing_shell")
    ctx.check("shell aabb present", shell_aabb is not None, details=str(shell_aabb))
    if shell_aabb is not None:
        pmin, pmax = shell_aabb
        pw = float(pmax[0] - pmin[0])
        ph = float(pmax[1] - pmin[1])
        pt = float(pmax[2] - pmin[2])
        ctx.check("housing width matches", abs(pw - PLATE_W) < 0.003, details=f"w={pw}")
        ctx.check("housing height matches", abs(ph - PLATE_H) < 0.003, details=f"h={ph}")
        ctx.check("housing is a slab", pt < 0.016, details=f"t={pt}")

    # --- Tubes rise above the housing ---------------------------------------
    tube_aabb = ctx.part_element_world_aabb(housing, elem="conduit_tubes")
    if tube_aabb is not None:
        ctx.check(
            "conduit tubes rise well above the housing top",
            float(tube_aabb[1][1]) > PLATE_H / 2.0 + 0.05,
            details=f"tube_top={float(tube_aabb[1][1])}",
        )

    return ctx.report()


object_model = build_object_model()
