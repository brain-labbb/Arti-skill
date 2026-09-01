# pictureX_0611_kitchen_cabinet — SourceMap

source_map_schema: 1
export_category: pictureX_0611_kitchen_cabinet
picture_category: 0611
picture_subcategory: kitchen_cabinet
category_scope: Compact floor-standing kitchen base cabinets with a panel carcass, visible worktop, floor supports, storage cavity, and front door, drawer, or lift access; wall-hung and tall hosts are not candidates.

sync_records:
  - rec_kitchen_cabinet_var_double_door_sink_base
  - rec_kitchen_cabinet_var_drawer_stack
  - rec_kitchen_cabinet_var_lift_up_wall_cabinet_refill
  - rec_picturex_0611__kitchen_cabinet__001__png_5f29ccaadea944c09b6f9832561aa405
  - rec_picturex_0611__kitchen_cabinet__002__png_241b2c4fbfaa407c837bd61a7fb1b21f
  - rec_picturex_0611__kitchen_cabinet__003__png_bd4b17b1bb1d45059fc34b510868a618
  - rec_picturex_0611__kitchen_cabinet__004__png_5a44a3fb595d482486a4d5792aac684a
  - rec_picturex_0611__kitchen_cabinet__005__png_32d18ad6c4bc42d1850e18a6ce8fa5cb

## Source review

| Record/Revision | Review status | Decision | Note |
|---|---|---|---|
| rec_kitchen_cabinet_var_double_door_sink_base/rev_000001 | reviewed | used | Mirrored recessed-panel leaves provide the clean two-door hinge policy and centre reveal. |
| rec_kitchen_cabinet_var_drawer_stack/rev_000001 | reviewed | used | Three repeated hollow drawers provide bilateral rails, regular pitch, and outward prismatic travel. |
| rec_kitchen_cabinet_var_lift_up_wall_cabinet_refill/rev_000001 | reviewed | used | Only the top-hinged flap and paired stay mechanism are reused; its wall-cabinet host is explicitly excluded. |
| rec_picturex_0611__kitchen_cabinet__001__png_5f29ccaadea944c09b6f9832561aa405/rev_000001 | reviewed | used | Narrow base carcass, white laminate worktop, four adjustable legs, and one side-hinged Shaker door establish the compact host. |
| rec_picturex_0611__kitchen_cabinet__002__png_241b2c4fbfaa407c837bd61a7fb1b21f/rev_000001 | reviewed | used | Re-reviewed: its fronts duplicate 001, but its carcass does not — a raised `bottom_deck` over a `recessed_toe_rail` replaces 001's open floor panel, which is the plinth-deck body candidate. |
| rec_picturex_0611__kitchen_cabinet__003__png_bd4b17b1bb1d45059fc34b510868a618/rev_000001 | reviewed | used | Equal paired Shaker fronts provide the clearest reusable stile, rail, recessed field, pull, and concealed-hinge construction. |
| rec_picturex_0611__kitchen_cabinet__004__png_5a44a3fb595d482486a4d5792aac684a/rev_000001 | reviewed | used | Re-reviewed: its front is a full `door_slab` with proud rails/stiles and four thin `panel_seam` strips around a **flush** field, not the recessed field of the Shaker candidate; it also carries the raised deck and toe rail. |
| rec_picturex_0611__kitchen_cabinet__005__png_32d18ad6c4bc42d1850e18a6ce8fa5cb/rev_000001 | reviewed | reference_only | Confirms wider two-door proportions, the `toe_kick` plinth read taken from 002/004, and four adjustable legs; it adds no further front mechanism of its own. |

## Candidate map

| Slot | Candidate | Component type | Record/Revision | Exact model.py:Lx-Ly | Distinction | Evidence |
|---|---|---|---|---|---|---|
| cabinet_body | plinth_deck_cabinet | plinth-deck panel carcass | rec_picturex_0611__kitchen_cabinet__002__png_241b2c4fbfaa407c837bd61a7fb1b21f/rev_000001 | model.py:L60-L120 | structure | `bottom_deck` sits a plinth height above the floor line and a `recessed_toe_rail` is set back from the front, so the base reads as a plinth instead of an open leg bay. |
| door_face | seamed_slab | flush-field slab front | rec_picturex_0611__kitchen_cabinet__004__png_5a44a3fb595d482486a4d5792aac684a/rev_000001 | model.py:L150-L210 | structure | `door_slab` plus proud `top_rail`/`bottom_rail`/`side_stile_i` and four thin `panel_seam_*` strips outline a flush centre field, unlike the Shaker recess. |
| cabinet_body | base_cabinet | open-front panel carcass | rec_picturex_0611__kitchen_cabinet__001__png_5f29ccaadea944c09b6f9832561aa405/rev_000001 | model.py:L50-L89 | structure | White laminate side, back, floor, and upper rail members form a real compact storage cavity. |
| access_module | hinged_doors | hinged door leaves | rec_kitchen_cabinet_var_double_door_sink_base/rev_000001 | model.py:L165-L312 | structure+motion | Mirrored recessed-panel leaves rotate on outer vertical hinge axes and preserve a narrow centre reveal. |
| access_module | drawer_stack | hollow drawer trays | rec_kitchen_cabinet_var_drawer_stack/rev_000001 | model.py:L97-L223 | structure+motion | Repeated open-top boxes run on paired side rails with uniform outward prismatic motion. |
| access_module | lift_up_flap | top-hinged flap | rec_kitchen_cabinet_var_lift_up_wall_cabinet_refill/rev_000001 | model.py:L172-L328 | structure+motion | A full front flap rotates about its top horizontal axis and is supported by two linked side stays. |
| door_face | shaker_panel | framed recessed front | rec_picturex_0611__kitchen_cabinet__003__png_bd4b17b1bb1d45059fc34b510868a618/rev_000001 | model.py:L55-L152 | structure | Stiles and rails stand proud of a connected recessed centre field with a mounted bar pull and concealed hinge details. |
| door_face | flat_panel | plain laminate front | rec_kitchen_cabinet_var_drawer_stack/rev_000001 | model.py:L140-L165 | structure | A connected plain red front with a restrained edge seam provides the source-backed flat treatment used by the drawer family. |
| top_treatment | laminate_worktop | thin laminate worktop | rec_picturex_0611__kitchen_cabinet__001__png_5f29ccaadea944c09b6f9832561aa405/rev_000001 | model.py:L90-L95 | structure | A thin white laminate slab closes the cabinet and slightly overhangs the panel carcass. |
| support_base | adjustable_legs | four adjustable legs | rec_picturex_0611__kitchen_cabinet__001__png_5f29ccaadea944c09b6f9832561aa405/rev_000001 | model.py:L97-L124 | structure | Four recessed metal stems connect through mounting plates and terminate in broad levelling feet. |
