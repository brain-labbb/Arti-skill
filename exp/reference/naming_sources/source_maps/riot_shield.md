# riot_shield — SourceMap

source_map_schema: 1
export_category: riot_shield
picture_category: Law Enforcement_Protective Gear
picture_subcategory: Riot shield
category_scope: One police riot shield whose root is a protective panel carried by a rear hand/forearm grip. The active pool contains two source-valid standing bi-fold topologies and two single-body handheld topologies; those whole-host topologies are structural families, not freely interchangeable bodies, leaves and hinges. Articulation may occur at the family-owned rear support, handle, forearm strap, or vision shutter. Out of scope: body armour, helmets, batons, vehicle glazing, fixed fencing, signs with shield-shaped profiles, and any object without a protective shield panel and carried grip. The original referenced PNG binaries are absent from the repository, but the user supplied the tactical standing-shield reference image during this repair; review combines that image with every active revision's complete model.py.

sync_records:
  - rec_black-tactical-bi-fold-riot-shield-standing-as-a_20260708_144725_562096_4985a0ef
  - rec_folding-two-panel-soft-armor-riot-shield-g-fold-_20260708_144344_902932_d961ac50
  - rec_riot_shield_var_form_curved
  - rec_riot_shield_var_form_round
  - rec_riot_shield_var_grip_forearm
  - rec_riot_shield_var_mechanism_gunport
  - rec_riot_shield_var_n3
  - rec_riot_shield_var_n4

## Source review

| Record/Revision | Review status | Decision | Note |
|---|---|---|---|
| rec_black-tactical-bi-fold-riot-shield-standing-as-a_20260708_144725_562096_4985a0ef/rev_000001 | reviewed | used | Complete model review: three-facet bent front plate with three molded ribs and POLICE relief, two cheek-and-strap steel clamp brackets with through-bolts and pins, and a folding rigid rear leaf with real viewport cut, perforated screen, hinge notches, tongues and pin bores. Its explicit picture metadata points to missing `001.png`; no image binary is available locally. |
| rec_folding-two-panel-soft-armor-riot-shield-g-fold-_20260708_144344_902932_d961ac50/rev_000001 | reviewed | used | Complete model review: thick rectangular ballistic-fabric slabs, side/free-edge binding, logo patch and lettering bar, lower pull webbing and three grommets, full-width sleeve hinge, plus a filleted cast-aluminium open grip loop fixed by four domed carriage bolts. Its explicit picture metadata points to missing `002.png`; no image binary is available locally. |
| rec_riot_shield_var_form_curved/rev_000001 | reviewed | used | Complete model review: closed two-surface cylindrical-shell mesh with edge caps, smoke-tinted vision band, rubber perimeter channels, three molded face ribs, bolted open grip and a separate strap/buckle part pivoting about the grip's lower rail. Picture metadata inherits missing `002.png`. |
| rec_riot_shield_var_form_round/rev_000001 | reviewed | used | Complete model review: lathed shallow spherical-cap disc shell, toroidal rubber rim and painted unit ring, rear filleted loop handle with four through-bolts, and a central handle mount revolving about the shield normal. Picture metadata inherits missing `002.png`. |
| rec_riot_shield_var_grip_forearm/rev_000001 | reviewed | used | Complete model review: replaces the open loop with a thin-wall half-cylinder forearm cradle and a cylindrical grip bar carried on two overlapping standoffs, while retaining the soft folding host and four-bolt plate. Picture metadata inherits missing `002.png`. |
| rec_riot_shield_var_mechanism_gunport/rev_000001 | reviewed | used | Complete model review: adds a real three-sided frame around the tactical rear-leaf window and a separate latch-bearing shutter plate on a top-edge revolute hinge with 0..1.4 rad travel. Picture metadata inherits missing `001.png`. |
| rec_riot_shield_var_n3/rev_000001 | reviewed | reference_only | Complete model review: the three-panel source deliberately changes the final leaf's local direction so it can become a tall deployed barrier. That topology produced an implausible stacked rest silhouette when generalized across unrelated hosts, so it is retained only as reference and contributes no independent slot or multiplicity. |
| rec_riot_shield_var_n4/rev_000001 | reviewed | reference_only | Complete model review: the four-panel source is a dedicated alternating accordion barrier with three family-owned hinges. It is not evidence that arbitrary handheld or standing hosts accept N leaves; it remains reference-only until represented as its own visually certified structural-family source. |

## Candidate map

| Slot | Candidate | Component type | Record/Revision | Exact model.py:Lx-Ly | Distinction | Evidence |
|---|---|---|---|---|---|---|
| shield_system | soft_bifold | bound soft-armour A-frame shield system | rec_folding-two-panel-soft-armor-riot-shield-g-fold-_20260708_144344_902932_d961ac50/rev_000001 | model.py:L90-L106, model.py:L122-L237 | structure+motion | A bound ballistic root slab with patch, pull strap and grommets owns one full-width sleeve hinge and one equally bound child slab extending locally downward; the zero pose tilts that child rearward into an A-frame. Body, support direction and sleeve therefore form one host topology. |
| shield_system | tactical_bifold | faceted rigid A-frame shield system | rec_black-tactical-bi-fold-riot-shield-standing-as-a_20260708_144725_562096_4985a0ef/rev_000001 | model.py:L52-L148, model.py:L160-L261 | structure+motion | The three-facet front plate, rearward top deflector, forward kick flare, stiffening ribs, paired clamp-clevis stations, bored tongues and rigid rear support are one free-standing mechanism. At q=0 the child hangs along local -Z with a 0.42 rad rear tilt, reaches the ground and never grows above the main shield. |
| shield_system | curved_handheld | bowed transparent single-body shield system | rec_riot_shield_var_form_curved/rev_000001 | model.py:L87-L216, model.py:L233-L372 | structure+motion | A closed cylindrical shell, rubber perimeter channels and surface-following ribs form a handheld shield with no rear fold panel; its optional motion belongs to the grip strap rather than a host-level fold chain. |
| shield_system | round_handheld | dished circular single-body shield system | rec_riot_shield_var_form_round/rev_000001 | model.py:L59-L89, model.py:L132-L220 | structure+motion | A shallow spherical-cap disc, toroidal rim, painted unit ring and central rotating grip form a single-body handheld topology with no top rail or rear support leaf. |
| viewport | tinted_lens | smoke-tinted vision band | rec_riot_shield_var_form_curved/rev_000001 | model.py:L243-L252 | structure | A broad translucent band follows the face at a source-derived shell surface location, visually distinct from a punched grille and from an opaque moving shutter. |
| viewport | perforated_screen | cut aperture with framed punched screen | rec_black-tactical-bi-fold-riot-shield-standing-as-a_20260708_144725_562096_4985a0ef/rev_000001 | model.py:L102-L119, model.py:L234-L248 | structure | The host plate is cut through by a real window and filled with a framed staggered `PerforatedPanelGeometry` steel screen with 9 mm holes on 16 mm pitch. |
| viewport | gunport_shutter | framed opening with hinged latch shutter | rec_riot_shield_var_mechanism_gunport/rev_000001 | model.py:L110-L202, model.py:L287-L306 | structure+motion | The window has bottom/side raised frame rails and a separate opaque plate with a lower latch nub; it pivots from the window top edge through 1.4 rad, changing both part tree and joint topology. |
| grip | open_loop | filleted cast open grip frame | rec_folding-two-panel-soft-armor-riot-shield-g-fold-_20260708_144344_902932_d961ac50/rev_000001 | model.py:L65-L87, model.py:L174-L185 | structure | A rounded mounting plate carries a deep rectangular annular extrusion with a true hand opening; it is neither a solid block nor a cylindrical bar. |
| grip | cradle_bar | half-cylinder forearm cuff plus bar | rec_riot_shield_var_grip_forearm/rev_000001 | model.py:L75-L148, model.py:L236-L257 | structure | A full tube is hollowed and cut to an open C-channel, then paired with a separate rubberised cylinder on two load-carrying standoffs below it. |
| grip | loop_with_strap | open loop plus articulated retention strap | rec_riot_shield_var_form_curved/rev_000001 | model.py:L194-L216, model.py:L287-L372 | structure+motion | The open loop gains a separate webbing body, metal buckle and transverse captured pivot pin; the strap revolves ±0.55 rad from the lower grip rail. |
| grip_mount | four_bolt_fixed | rigid four-bolt mounting plate | rec_folding-two-panel-soft-armor-riot-shield-g-fold-_20260708_144344_902932_d961ac50/rev_000001 | model.py:L174-L210 | structure | Four cylinder shanks and four domed heads occupy the plate corners and the handle is fixed to the root panel; the visible fastener logic survives independently of grip shape. |
| grip_mount | rotating_boss | central bearing boss about shield normal | rec_riot_shield_var_form_round/rev_000001 | model.py:L169-L220 | structure+motion | The handle/plate assembly is supported at a central rear boss and revolves ±0.7 rad about X, changing joint type, axis and swept footprint relative to the fixed mount. |
