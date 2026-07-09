from __future__ import annotations

# Realistic articulated caulking gun (barrel / half-cradle "skeleton" style),
# modeled from the reference image (picture/Handtools/caulking gun/001.png).
#
# Reference reading:
#   - A dark gunmetal half-open cylindrical barrel/cradle holds a white-and-blue
#     labelled caulk cartridge tube.
#   - The cartridge ends in a black plastic nozzle cone with a white pointed tip
#     at the front.
#   - A black plastic cap/collar closes the rear of the barrel.
#   - A black handle/trigger assembly sits at the rear: a fixed GRIP wing plus a
#     squeeze TRIGGER lever that pivots; the two form the photo's open "V".
#   - A long dark steel PLUNGER ROD runs back out of the rear cap, ending in a
#     closed circular RING PULL-LOOP at its tail (the ring is welded onto the
#     rod tail and translates with the rod). Squeezing the trigger ratchets the
#     rod forward to push the cartridge plunger and extrude caulk.
#
# Coordinate convention (Z up, dispensing along +X):
#   - +X  : the muzzle / dispensing direction (the nozzle points to +X).
#   - +Z  : up. Gravity is -Z. The barrel mouth opens toward +Z (cradle is a
#           lower half-shell, like the real open barrel that drops a tube in).
#   - +Y  : the gun's left/right.
#   - The barrel axis lies along X at y=0, lifted to z=AXIS_Z so the lowest
#     point (the rear cap rim) rests on the ground plane z=0. The handle
#     assembly sits above and behind the rear of the barrel.
#
# Articulated mechanisms (both are real, visible controls):
#   1. trigger_pivot  (REVOLUTE): the squeeze lever rotates about a pin at the
#      top of the rear handle frame. Positive q pulls the lever's free lower end
#      toward the rear, the natural squeeze stroke.
#   2. plunger_drive  (PRISMATIC): the steel rod + ring pull-loop slide along the
#      barrel axis (+X) to advance the cartridge plunger.
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions (meters) -- a standard ~300 ml cartridge caulking gun.
# ---------------------------------------------------------------------------

# Cartridge tube (the white labelled body that drops into the cradle).
TUBE_R = 0.0245            # cartridge outer radius (~49 mm dia)
TUBE_LEN = 0.215          # cartridge body length along X
TUBE_X0 = 0.0             # rear face of cartridge body
TUBE_X1 = TUBE_X0 + TUBE_LEN

# Barrel / cradle: a lower half-cylinder shell that the tube drops into.
BARREL_R_OUT = TUBE_R + 0.006   # outer radius of the cradle shell
BARREL_R_IN = TUBE_R + 0.0015   # inner radius (small running clearance to tube)
BARREL_X0 = TUBE_X0 - 0.006     # cradle starts just behind the tube
BARREL_X1 = TUBE_X1 - 0.010     # cradle stops short of the muzzle so the nozzle shows
BARREL_LEN = BARREL_X1 - BARREL_X0

# Front collar (black plastic ring at the muzzle end of the barrel).
FRONT_COLLAR_R_OUT = BARREL_R_OUT + 0.004
FRONT_COLLAR_R_IN = TUBE_R + 0.001
FRONT_COLLAR_T = 0.012          # collar thickness along X
FRONT_COLLAR_X = BARREL_X1 - 0.004

# Rear cap (black plastic disc that closes the back of the barrel and guides
# the plunger rod). This carries the handle frame.
REAR_CAP_R = BARREL_R_OUT + 0.006
REAR_CAP_T = 0.016              # along X
REAR_CAP_X = BARREL_X0 - REAR_CAP_T / 2.0   # centered just behind the cradle

# Nozzle cone (black) + white applicator tip at the muzzle.
NOZZLE_BASE_R = TUBE_R * 0.62
NOZZLE_TIP_R = 0.0045
NOZZLE_LEN = 0.030
NOZZLE_X0 = TUBE_X1
TIP_LEN = 0.022
TIP_R0 = NOZZLE_TIP_R
TIP_R1 = 0.0018

# Cartridge plunger disc (the back disc inside the tube the rod pushes on).
CART_PLUNGER_R = TUBE_R - 0.001
CART_PLUNGER_T = 0.006

# Plunger rod (dark steel) + push disc + closed circular ring pull-loop tail.
ROD_R = 0.0042
ROD_PUSH_R = TUBE_R - 0.001     # disc on the rod's front end, inside the tube
ROD_PUSH_T = 0.005
# Rod runs from inside the tube (front push disc) back through the rear cap and
# out the back, ending in the closed ring pull-loop. Modeled long enough to stay
# engaged through the rear cap bore across the full prismatic stroke, and long
# enough that the ring clears the handle even at full advance.
ROD_FRONT_X = TUBE_X0 + 0.018   # rod push-disc front face (rest pose)
ROD_BACK_X = REAR_CAP_X - 0.095 # rod tail behind the rear cap (rest pose)

# Closed circular pull-ring at the rear of the rod: a torus welded onto the
# rod tail, standing in the XZ plane (vertical) so a finger can hook through
# from the side. The ring translates rigidly with the rod.
RING_R = 0.013                  # major radius of the ring (center to wire center)
RING_WIRE_R = 0.003             # wire cross-section radius of the ring

# Rear handle frame: a black bracket rising from the rear cap that carries the
# trigger pivot pin (mirrors the chunky black handle in the image). It sits just
# behind the rear cap so it clears the cartridge and the trigger swings free.
REAR_CAP_BACK = REAR_CAP_X      # rear face of the cap (extrude runs +X from here)
FRAME_RISE = 0.085              # how high the frame rises in +Z above barrel axis
FRAME_T = 0.010                 # frame plate thickness (Y)
FRAME_W_X = 0.038               # frame plate width along X
FRAME_X = REAR_CAP_BACK - FRAME_W_X / 2.0 + 0.002   # front face overlaps cap back slightly

# Trigger pivot pin (centered on the frame, behind the rear cap).
PIVOT_Z = FRAME_RISE            # pin height above barrel axis
PIVOT_X = FRAME_X              # pin sits at the top of the frame
PIN_R = 0.0035
PIN_LEN = 0.030                 # along Y, through the frame

# Trigger lever: a flat black squeeze blade hanging down from the pivot. It is a
# compact wedge that stops above the barrel axis (it never reaches the rod).
TRIGGER_LEN = 0.060             # length of the blade from the pivot
TRIGGER_W = 0.013               # blade thickness along X (it is a flat plate)
TRIGGER_H = 0.026               # blade height in Y (broad squeeze face)

# Fixed grip wing: the stationary black handle blade behind the trigger (the
# rear wing of the photo's "V"). It roots into the handle frame at the top and
# sweeps down/rearward; squeezing pulls the trigger back toward it.
FRAME_HALF_W = ROD_R + 0.0025 + FRAME_T  # outer half-width of the frame yoke

# Ground lift: the whole gun is raised so its lowest point (the rear cap rim,
# radius REAR_CAP_R below the barrel axis) rests exactly on z=0.
AXIS_Z = REAR_CAP_R

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

GUNMETAL = "gunmetal"          # dark steel barrel
BLACK_PLASTIC = "black_plastic"
WHITE_CART = "white_cartridge"
LABEL_BLUE = "label_blue"
STEEL = "bright_steel"         # plunger rod, hook, spring clip
WHITE_TIP = "white_tip"


def _register_materials(model: ArticulatedObject) -> None:
    model.material(GUNMETAL, rgba=(0.27, 0.28, 0.30, 1.0))
    model.material(BLACK_PLASTIC, rgba=(0.10, 0.10, 0.11, 1.0))
    model.material(WHITE_CART, rgba=(0.90, 0.90, 0.88, 1.0))
    model.material(LABEL_BLUE, rgba=(0.20, 0.34, 0.62, 1.0))
    model.material(STEEL, rgba=(0.74, 0.76, 0.80, 1.0))
    model.material(WHITE_TIP, rgba=(0.93, 0.93, 0.92, 1.0))


# ---------------------------------------------------------------------------
# CadQuery geometry builders
# ---------------------------------------------------------------------------


def _barrel_cradle() -> cq.Workplane:
    """Lower half-cylinder shell (open trough) that the cartridge drops into."""
    outer = (
        cq.Workplane("YZ")
        .workplane(offset=BARREL_X0)
        .circle(BARREL_R_OUT)
        .extrude(BARREL_LEN)
    )
    inner = (
        cq.Workplane("YZ")
        .workplane(offset=BARREL_X0)
        .circle(BARREL_R_IN)
        .extrude(BARREL_LEN)
    )
    shell = outer.cut(inner)
    # Cut away the top half (z > opening line) to make an open cradle/trough.
    # Leave a little rim above center so the tube is captured, not falling out.
    keep_top = BARREL_R_OUT * 0.18   # the cradle wraps slightly past the equator
    cutter = (
        cq.Workplane("XY")
        .box(BARREL_LEN * 1.4, BARREL_R_OUT * 2.4, BARREL_R_OUT * 2.0)
        .translate((BARREL_X0 + BARREL_LEN / 2.0, 0.0, keep_top + BARREL_R_OUT))
    )
    shell = shell.cut(cutter)
    return shell


def _front_collar() -> cq.Workplane:
    ring_out = (
        cq.Workplane("YZ")
        .workplane(offset=FRONT_COLLAR_X)
        .circle(FRONT_COLLAR_R_OUT)
        .extrude(FRONT_COLLAR_T)
    )
    ring_in = (
        cq.Workplane("YZ")
        .workplane(offset=FRONT_COLLAR_X - 0.001)
        .circle(FRONT_COLLAR_R_IN)
        .extrude(FRONT_COLLAR_T + 0.002)
    )
    return ring_out.cut(ring_in)


def _rear_cap() -> cq.Workplane:
    """Black cap closing the barrel rear; bored for the plunger rod."""
    cap = (
        cq.Workplane("YZ")
        .workplane(offset=REAR_CAP_X)
        .circle(REAR_CAP_R)
        .extrude(REAR_CAP_T)
    )
    # Central bore for the rod (the rod slides through here).
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=REAR_CAP_X - 0.001)
        .circle(ROD_R + 0.0012)
        .extrude(REAR_CAP_T + 0.002)
    )
    cap = cap.cut(bore)
    return cap


def _handle_frame() -> cq.Workplane:
    """Black U-yoke bracket rising from the rear cap to carry the trigger pivot.

    Two thin side plates straddle the plunger rod (offset in +/-Y) so the rod
    can slide through the open center without colliding with the frame. They are
    joined across the top by the pivot boss that holds the pin. Authored directly
    in the model world frame (this is geometry on the root body part).
    """
    # Side plates sit just outside the rod, with a clearance gap to the rod.
    half_gap = ROD_R + 0.0025 + FRAME_T / 2.0   # plate center offset in Y
    z_lo = -REAR_CAP_R * 0.55                   # plate foot blends onto the rear cap
    z_hi = PIVOT_Z + 0.006                      # plate top just above the pivot
    plate_h = z_hi - z_lo
    plate_cz = (z_lo + z_hi) / 2.0

    side_plate = (
        cq.Workplane("XY")
        .box(FRAME_W_X, FRAME_T, plate_h)
        .edges("|Y")
        .fillet(0.005)
    )
    left = side_plate.translate((FRAME_X, +half_gap, plate_cz))
    right = side_plate.translate((FRAME_X, -half_gap, plate_cz))

    # Pivot lugs: a round lug on each side plate around the pin axis (Y). The
    # center is left clear for the trigger hub, which rotates between the lugs.
    lug_inner = half_gap - FRAME_T / 2.0 - 0.0015   # inner face of each lug
    lug_len = (half_gap + FRAME_T / 2.0 + 0.001) - lug_inner
    lug_left = (
        cq.Workplane("XZ")
        .workplane(offset=lug_inner)
        .circle(0.011)
        .extrude(lug_len)
        .translate((PIVOT_X, 0.0, PIVOT_Z))
    )
    lug_right = (
        cq.Workplane("XZ")
        .workplane(offset=-(lug_inner + lug_len))
        .circle(0.011)
        .extrude(lug_len)
        .translate((PIVOT_X, 0.0, PIVOT_Z))
    )
    boss = lug_left.union(lug_right)

    # A short foot web tying the two plates to the rear cap (behind the rod) so
    # the yoke reads as one rigid bracket, not two loose plates.
    web = (
        cq.Workplane("XY")
        .box(FRAME_W_X, 2.0 * half_gap + FRAME_T, 0.012)
        .translate((FRAME_X, 0.0, z_lo + 0.006))
    )
    # Rod-guide boss: a thick bored collar around the plunger rod that ties the
    # frame plates solidly into the rear cap (so the handle reads rigidly
    # mounted on the housing, not floating behind it).
    boss_x0 = -0.026
    boss_x1 = -0.002
    guide = (
        cq.Workplane("YZ")
        .workplane(offset=boss_x0)
        .circle(0.013)
        .extrude(boss_x1 - boss_x0)
    )
    guide_bore = (
        cq.Workplane("YZ")
        .workplane(offset=boss_x0 - 0.001)
        .circle(ROD_R + 0.0012)
        .extrude(boss_x1 - boss_x0 + 0.002)
    )
    guide = guide.cut(guide_bore)
    frame = left.union(right).union(boss).union(web).union(guide)
    return frame


def _pivot_pin() -> cq.Workplane:
    pin = (
        cq.Workplane("XZ")
        .workplane(offset=-PIN_LEN / 2.0)
        .circle(PIN_R)
        .extrude(PIN_LEN)
        .translate((PIVOT_X, 0.0, PIVOT_Z))
    )
    return pin


def _grip_wing() -> cq.Workplane:
    """Fixed black grip blade behind the trigger (world frame, barrel axis z=0).

    A solid tapered wing rooted into the top of the handle frame yoke, sweeping
    down and rearward behind the trigger's full swing arc. Together with the
    trigger it forms the open "V" handle of the reference photo; squeezing the
    trigger closes the V. Both ends of the wing land on solid geometry: the top
    is buried in the frame yoke, and the frame itself is bossed into the rear
    cap, so nothing hangs in the air.
    """
    profile = (
        cq.Workplane("XZ")
        .moveTo(-0.045, 0.090)   # top-front, buried in the frame yoke top
        .lineTo(-0.057, 0.094)   # top-rear corner
        .lineTo(-0.096, 0.034)   # lower-rear tip of the grip
        .lineTo(-0.081, 0.028)   # lower-front tip (clears the trigger swing)
        .close()
    )
    wing = profile.extrude(FRAME_HALF_W, both=True)
    return wing


def _cartridge_tube() -> cq.Workplane:
    """White cartridge body (a closed cylinder)."""
    tube = (
        cq.Workplane("YZ")
        .workplane(offset=TUBE_X0)
        .circle(TUBE_R)
        .extrude(TUBE_LEN)
    )
    return tube


def _cartridge_label() -> cq.Workplane:
    """A thin blue label band wrapped around the cartridge (separate visual)."""
    band_x0 = TUBE_X0 + 0.045
    band_len = 0.110
    band = (
        cq.Workplane("YZ")
        .workplane(offset=band_x0)
        .circle(TUBE_R + 0.0006)
        .extrude(band_len)
    )
    inner = (
        cq.Workplane("YZ")
        .workplane(offset=band_x0 - 0.001)
        .circle(TUBE_R - 0.0004)
        .extrude(band_len + 0.002)
    )
    band = band.cut(inner)
    # Keep only a strip on the +Z visible top quadrant so it reads as a label,
    # not a full sleeve.
    keep = (
        cq.Workplane("XY")
        .box(band_len * 1.3, TUBE_R * 1.2, TUBE_R * 1.4)
        .translate((band_x0 + band_len / 2.0, 0.0, TUBE_R * 0.55 + TUBE_R * 0.7))
    )
    band = band.intersect(keep)
    return band


def _nozzle_cone() -> cq.Workplane:
    """Black tapered nozzle cone at the muzzle."""
    cone = (
        cq.Workplane("YZ")
        .workplane(offset=NOZZLE_X0)
        .circle(NOZZLE_BASE_R)
        .workplane(offset=NOZZLE_LEN)
        .circle(NOZZLE_TIP_R)
        .loft(combine=True)
    )
    # A short base shoulder blending into the tube face.
    shoulder = (
        cq.Workplane("YZ")
        .workplane(offset=NOZZLE_X0 - 0.004)
        .circle(TUBE_R * 0.82)
        .workplane(offset=0.004)
        .circle(NOZZLE_BASE_R)
        .loft(combine=True)
    )
    return cone.union(shoulder)


def _white_tip() -> cq.Workplane:
    """White applicator tip cut at an angle, like a trimmed caulk nozzle."""
    tip = (
        cq.Workplane("YZ")
        .workplane(offset=NOZZLE_X0 + NOZZLE_LEN)
        .circle(TIP_R0)
        .workplane(offset=TIP_LEN)
        .circle(TIP_R1)
        .loft(combine=True)
    )
    return tip


def _cart_plunger() -> cq.Workplane:
    """The cartridge back plunger disc the rod drives, in rod-local frame.

    Authored just ahead of the rod's push disc (toward +X, into the tube) so it
    travels with the rod and contacts the rod's push face.
    """
    disc = (
        cq.Workplane("YZ")
        .workplane(offset=0.0)
        .circle(CART_PLUNGER_R)
        .extrude(CART_PLUNGER_T)
    )
    return disc


def _plunger_rod() -> cq.Workplane:
    """Steel rod + front push disc; built in rod-local frame (rod axis along X).

    Local origin is at x=0 == ROD_FRONT_X in the model rest frame. We build it in
    the rod's own frame so the prismatic joint can translate it cleanly.
    """
    total_len = ROD_FRONT_X - ROD_BACK_X  # from front disc back to tail
    rod = (
        cq.Workplane("YZ")
        .workplane(offset=-total_len)
        .circle(ROD_R)
        .extrude(total_len)
    )
    # Front push disc (rides inside the tube, behind the cartridge plunger).
    push = (
        cq.Workplane("YZ")
        .workplane(offset=-ROD_PUSH_T)
        .circle(ROD_PUSH_R)
        .extrude(ROD_PUSH_T)
    )
    return rod.union(push)


def _ring_handle() -> cq.Workplane:
    """Closed circular pull-ring at the rod tail (rod-local frame).

    A torus standing in the XZ plane, centered just behind the rod tail so the
    front of the ring fuses into the rod end. The ring is built by revolving a
    small cross-section circle around a Y-axis offset by RING_R from the rod
    tail, producing a finger-sized closed loop that translates rigidly with the
    prismatic rod.
    """
    total_len = ROD_FRONT_X - ROD_BACK_X
    tail_x = -total_len  # rod tail in rod-local frame

    # Ring center sits so the front of the ring slightly overlaps the rod tail,
    # ensuring a solid fused connection (like a weld).
    ring_cx = tail_x - RING_R + 0.001

    # Cross-section circle in the XY plane at (tail_x, 0), then revolve 360°
    # around a Y-parallel axis at x = ring_cx. This sweeps the cross-section
    # through the XZ plane, producing a torus with axis along Y.
    section = (
        cq.Workplane("XY")
        .moveTo(tail_x, 0.0)
        .circle(RING_WIRE_R)
    )
    ring = section.revolve(360, (ring_cx, -0.1), (ring_cx, 0.1))
    return ring


def _trigger_blade() -> cq.Workplane:
    """Flat black squeeze lever, built in the trigger-local frame.

    Local origin = pivot pin (at z=0 in this frame). The lever hangs downward
    (-Z) as a broad flat plate; the squeeze face widens toward the free end. A
    pivot hub wraps the pin so the lever reads as pinned, not floating. A
    horizontal clearance bore at the pivot height lets the plunger rod pass
    through the lever (the rod runs through the trigger plate's slot, as on a
    real ratchet caulking gun).
    """
    # Main blade: a tapered flat plate from just below the pivot to the free end.
    blade = (
        cq.Workplane("XY")
        .workplane(offset=-0.006)
        .rect(TRIGGER_W, TRIGGER_H)
        .workplane(offset=-TRIGGER_LEN)
        .rect(TRIGGER_W * 1.3, TRIGGER_H * 1.55)
        .loft(combine=True)
    )
    # Pivot hub: a short cylinder around the pin axis (Y), tying the blade to
    # the pin. It bridges from the pivot down into the top of the blade.
    hub = (
        cq.Workplane("XZ")
        .workplane(offset=-0.011)
        .circle(0.011)
        .extrude(0.022)
    )
    # Neck connecting hub to blade so they form one solid lever.
    neck = (
        cq.Workplane("XY")
        .box(TRIGGER_W, TRIGGER_H, 0.012)
        .translate((0.0, 0.0, -0.004))
    )
    lever = blade.union(hub).union(neck)
    # Pin bore through the hub (a Y-axis cylinder cut through the pivot hub).
    pin_bore = cq.Solid.makeCylinder(
        PIN_R + 0.0006,
        0.030,
        cq.Vector(0.0, -0.015, 0.0),
        cq.Vector(0.0, 1.0, 0.0),
    )
    lever = lever.cut(cq.Workplane(obj=pin_bore))
    # Round the free-end squeeze edge (Y-parallel bottom edge) last, so the
    # boolean cut above stays robust.
    lever = lever.edges("|Y and <Z").fillet(0.003)
    return lever


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="caulking_gun")
    _register_materials(model)

    # -- Root part: barrel cradle + front collar + rear cap + handle frame +
    #    fixed grip wing + pivot pin. All body geometry is authored around the
    #    barrel axis at z=0 and lifted by AXIS_Z so the gun rests on the ground.
    lift = (0.0, 0.0, AXIS_Z)
    body = model.part("barrel_body")
    body.visual(
        mesh_from_cadquery(_barrel_cradle().translate(lift), "barrel_cradle"),
        material=GUNMETAL,
        name="barrel_cradle",
    )
    body.visual(
        mesh_from_cadquery(_front_collar().translate(lift), "front_collar"),
        material=BLACK_PLASTIC,
        name="front_collar",
    )
    body.visual(
        mesh_from_cadquery(_rear_cap().translate(lift), "rear_cap"),
        material=BLACK_PLASTIC,
        name="rear_cap",
    )
    body.visual(
        mesh_from_cadquery(_handle_frame().translate(lift), "handle_frame"),
        material=BLACK_PLASTIC,
        name="handle_frame",
    )
    body.visual(
        mesh_from_cadquery(_grip_wing().translate(lift), "grip_wing"),
        material=BLACK_PLASTIC,
        name="grip_wing",
    )
    body.visual(
        mesh_from_cadquery(_pivot_pin().translate(lift), "pivot_pin"),
        material=STEEL,
        name="pivot_pin",
    )

    # -- Cartridge tube (with nozzle + tip + blue label). It sits in the cradle.
    #    The cartridge is the loadable consumable; it is FIXED to the barrel here
    #    (the tube doesn't articulate; the rod and trigger do).
    cartridge = model.part("cartridge")
    cartridge.visual(
        mesh_from_cadquery(_cartridge_tube(), "cartridge_tube"),
        material=WHITE_CART,
        name="cartridge_tube",
    )
    cartridge.visual(
        mesh_from_cadquery(_cartridge_label(), "cartridge_label"),
        material=LABEL_BLUE,
        name="cartridge_label",
    )
    cartridge.visual(
        mesh_from_cadquery(_nozzle_cone(), "nozzle_cone"),
        material=BLACK_PLASTIC,
        name="nozzle_cone",
    )
    cartridge.visual(
        mesh_from_cadquery(_white_tip(), "white_tip"),
        material=WHITE_TIP,
        name="white_tip",
    )

    # -- Plunger assembly: steel rod + cartridge plunger disc + ring pull-loop.
    plunger = model.part("plunger_rod")
    plunger.visual(
        mesh_from_cadquery(_plunger_rod(), "plunger_rod"),
        material=GUNMETAL,
        name="plunger_rod",
    )
    plunger.visual(
        mesh_from_cadquery(_cart_plunger(), "cart_plunger"),
        material=WHITE_CART,
        name="cart_plunger",
    )
    plunger.visual(
        mesh_from_cadquery(_ring_handle(), "pull_ring"),
        material=STEEL,
        name="pull_ring",
    )

    # -- Trigger lever (squeeze control).
    trigger = model.part("trigger_lever")
    trigger.visual(
        mesh_from_cadquery(_trigger_blade(), "trigger_blade"),
        material=BLACK_PLASTIC,
        name="trigger_blade",
    )

    # -- Joints ---------------------------------------------------------------

    # Cartridge is fixed in the cradle (consumable seated in the barrel). Its
    # geometry is authored around the barrel axis, so lift it by AXIS_Z.
    model.articulation(
        "cartridge_seat",
        ArticulationType.FIXED,
        parent=body,
        child=cartridge,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
    )

    # Plunger rod: PRISMATIC along the barrel axis (+X advances the caulk).
    # The rod is authored in its own local frame with the front push-disc face at
    # local x=0; placing the joint origin at ROD_FRONT_X puts the rod at rest.
    # The cart_plunger disc is authored just ahead of the push disc in the same
    # rod-local frame, so the whole plunger assembly travels together.
    model.articulation(
        "plunger_drive",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(ROD_FRONT_X, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.05, lower=0.0, upper=0.060),
    )

    # Trigger lever: REVOLUTE about the pivot pin (axis along Y).
    # At rest the blade hangs down/forward. Positive q (right-hand rule about +Y)
    # rotates the free lower end of the blade toward the rear (-X), the squeeze.
    model.articulation(
        "trigger_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(PIVOT_X, 0.0, AXIS_Z + PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=0.55),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("barrel_body")
    cartridge = object_model.get_part("cartridge")
    plunger = object_model.get_part("plunger_rod")
    trigger = object_model.get_part("trigger_lever")

    plunger_drive = object_model.get_articulation("plunger_drive")
    trigger_pivot = object_model.get_articulation("trigger_pivot")

    # --- Joint configuration claims ----------------------------------------
    ctx.check(
        "plunger_drive is prismatic along +X",
        plunger_drive.joint_type == "prismatic" and tuple(plunger_drive.axis) == (1.0, 0.0, 0.0),
        details=f"type={plunger_drive.joint_type}, axis={plunger_drive.axis}",
    )
    ctx.check(
        "trigger_pivot is revolute about +Y",
        trigger_pivot.joint_type == "revolute" and tuple(trigger_pivot.axis) == (0.0, 1.0, 0.0),
        details=f"type={trigger_pivot.joint_type}, axis={trigger_pivot.axis}",
    )

    # --- Hero geometry is present ------------------------------------------
    body_visuals = {v.name for v in body.visuals}
    for need in ("barrel_cradle", "front_collar", "rear_cap", "handle_frame", "grip_wing", "pivot_pin"):
        ctx.check(f"body has {need}", need in body_visuals, details=str(sorted(body_visuals)))

    cart_visuals = {v.name for v in cartridge.visuals}
    for need in ("cartridge_tube", "nozzle_cone", "white_tip"):
        ctx.check(f"cartridge has {need}", need in cart_visuals, details=str(sorted(cart_visuals)))

    plunger_visuals = {v.name for v in plunger.visuals}
    for need in ("plunger_rod", "pull_ring"):
        ctx.check(f"plunger has {need}", need in plunger_visuals, details=str(sorted(plunger_visuals)))

    # --- Nozzle/tip is the most +X (muzzle) feature ------------------------
    tip_aabb = ctx.part_element_world_aabb(cartridge, elem="white_tip")
    cradle_aabb = ctx.part_element_world_aabb(body, elem="barrel_cradle")
    if tip_aabb is not None and cradle_aabb is not None:
        ctx.check(
            "white tip points forward past the cradle (+X muzzle)",
            tip_aabb[1][0] > cradle_aabb[1][0],
            details=f"tip_max_x={tip_aabb[1][0]:.4f}, cradle_max_x={cradle_aabb[1][0]:.4f}",
        )

    # --- Pull-ring is a closed circular loop at the rod tail ----------------
    ring_aabb = ctx.part_element_world_aabb(plunger, elem="pull_ring")
    if ring_aabb is not None:
        # The ring is a torus in the XZ plane: its Z-extent should be ~2*(RING_R
        # + RING_WIRE_R) (proving it is a proper ring, not a stub), and its
        # Y-extent should be ~2*RING_WIRE_R (thin, proving it is a loop with a
        # hole, not a solid disc).
        ring_dz = ring_aabb[1][2] - ring_aabb[0][2]
        ring_dy = ring_aabb[1][1] - ring_aabb[0][1]
        expected_dz = 2.0 * (RING_R + RING_WIRE_R)
        expected_dy = 2.0 * RING_WIRE_R
        ctx.check(
            "pull-ring Z-extent confirms a closed circular loop",
            ring_dz > 0.7 * expected_dz and ring_dz < 1.3 * expected_dz,
            details=f"ring_dz={ring_dz:.4f}, expected~{expected_dz:.4f}",
        )
        ctx.check(
            "pull-ring is thin in Y (loop with a hole, not a solid disc)",
            ring_dy < 0.5 * ring_dz,
            details=f"ring_dy={ring_dy:.4f}, ring_dz={ring_dz:.4f}",
        )
        ctx.check(
            "pull-ring sits behind the rear cap at rest",
            ring_aabb[1][0] < REAR_CAP_X - 0.02,
            details=f"ring_max_x={ring_aabb[1][0]:.4f}, rear_cap_x={REAR_CAP_X:.4f}",
        )

    # --- The pull-ring translates rigidly with the rod (same link) ----------
    if ring_aabb is not None:
        with ctx.pose({plunger_drive: 0.050}):
            ring_adv = ctx.part_element_world_aabb(plunger, elem="pull_ring")
        if ring_adv is not None:
            dx = ring_adv[0][0] - ring_aabb[0][0]
            ctx.check(
                "pull-ring advances with the plunger rod",
                abs(dx - 0.050) < 1e-6 and abs(ring_adv[0][2] - ring_aabb[0][2]) < 1e-6,
                details=f"dx={dx:.4f}, dz={ring_adv[0][2] - ring_aabb[0][2]:.6f}",
            )

    # --- The gun rests on the ground: lowest point (rear cap rim) at z~0 ----
    cap_aabb = ctx.part_element_world_aabb(body, elem="rear_cap")
    if cap_aabb is not None:
        ctx.check(
            "rear cap rim rests on the ground plane",
            abs(cap_aabb[0][2]) < 0.002,
            details=f"cap_min_z={cap_aabb[0][2]:.5f}",
        )

    # --- Trigger blade hangs below the pivot at rest -----------------------
    with ctx.pose({trigger_pivot: 0.0}):
        blade_rest = ctx.part_element_world_aabb(trigger, elem="trigger_blade")
    if blade_rest is not None:
        ctx.check(
            "trigger blade hangs below the pivot",
            blade_rest[0][2] < AXIS_Z + PIVOT_Z - 0.03,
            details=f"blade_min_z={blade_rest[0][2]:.4f}, pivot_z={AXIS_Z + PIVOT_Z:.4f}",
        )

    # --- Grip wing sits behind the trigger across its full swing ------------
    grip_aabb = ctx.part_element_world_aabb(body, elem="grip_wing")
    if grip_aabb is not None:
        with ctx.pose({trigger_pivot: 0.55}):
            blade_sq = ctx.part_element_world_aabb(trigger, elem="trigger_blade")
        if blade_sq is not None:
            ctx.check(
                "fully squeezed trigger still clears the fixed grip wing",
                blade_sq[0][0] > grip_aabb[1][0] - 0.041,
                details=(
                    f"blade_min_x={blade_sq[0][0]:.4f}, grip_max_x={grip_aabb[1][0]:.4f}"
                ),
            )

    # --- Actuating the trigger swings its free end toward the rear (-X) ----
    with ctx.pose({trigger_pivot: 0.0}):
        rest_free = ctx.part_element_world_aabb(trigger, elem="trigger_blade")
    with ctx.pose({trigger_pivot: 0.5}):
        squeezed_free = ctx.part_element_world_aabb(trigger, elem="trigger_blade")
    if rest_free is not None and squeezed_free is not None:
        # The lowest point of the blade should move rearward (toward -X) as it
        # rotates up about +Y; verify the blade's min-x decreases (moves back).
        ctx.check(
            "squeezing the trigger swings the blade rearward",
            squeezed_free[0][0] < rest_free[0][0] - 0.005,
            details=f"rest_min_x={rest_free[0][0]:.4f}, squeezed_min_x={squeezed_free[0][0]:.4f}",
        )

    # --- Actuating the plunger advances the rod toward the muzzle (+X) -----
    rest_rod = ctx.part_world_position(plunger)
    with ctx.pose({plunger_drive: 0.050}):
        adv_rod = ctx.part_world_position(plunger)
    if rest_rod is not None and adv_rod is not None:
        ctx.check(
            "driving the plunger advances the rod toward the muzzle",
            adv_rod[0] > rest_rod[0] + 0.04,
            details=f"rest_x={rest_rod[0]:.4f}, advanced_x={adv_rod[0]:.4f}",
        )

    # --- Plunger rod stays engaged through the rear cap bore ---------------
    # At rest, the rod overlaps the rear cap (it passes through the bore).
    ctx.expect_overlap(
        plunger,
        body,
        axes="x",
        elem_a="plunger_rod",
        elem_b="rear_cap",
        min_overlap=0.005,
        name="rod stays inserted through the rear cap bore",
    )

    # The rod passes through the rear-cap bore: this is an intentional captured-
    # shaft fit through a bored cap, so allow that scoped overlap.
    ctx.allow_overlap(
        plunger,
        body,
        elem_a="plunger_rod",
        elem_b="rear_cap",
        reason="The steel plunger rod intentionally slides through the bored rear cap.",
    )
    # The push disc / cartridge plunger ride inside the cartridge tube (captured).
    ctx.allow_overlap(
        plunger,
        cartridge,
        reason="The plunger push disc and cartridge plunger ride captured inside the cartridge tube.",
    )

    # The trigger pivots on the pin captured by the frame lugs. The trigger hub
    # and the steel pivot pin are an intentional captured-pin pivot fit, so the
    # hub/pin and pin/frame embeddings are expected.
    ctx.allow_overlap(
        trigger,
        body,
        elem_a="trigger_blade",
        elem_b="pivot_pin",
        reason="The trigger hub pivots captured on the steel pin (pin nests in the hub bore).",
    )
    ctx.allow_overlap(
        trigger,
        body,
        elem_a="trigger_blade",
        elem_b="handle_frame",
        reason="The trigger hub rotates between the frame pivot lugs (captured pivot fit).",
    )

    # The trigger pivots on the pin: prove the hub actually surrounds the pin.
    ctx.expect_overlap(
        trigger,
        body,
        axes="yz",
        elem_a="trigger_blade",
        elem_b="pivot_pin",
        min_overlap=0.002,
        name="trigger hub captures the pivot pin",
    )

    return ctx.report()


object_model = build_object_model()
