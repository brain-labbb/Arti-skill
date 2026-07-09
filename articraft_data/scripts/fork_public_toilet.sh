#!/bin/bash
# Expand the porta-potty parent into structurally distinct portable-toilet-cabin
# variants for the downstream `public_toilet` procedural template. Each fork is a
# single-axis change so the template gets clean module candidates per slot
# (roof / walls / door / vent-utility) plus the single->multi-cabin multiplicity.
set -u
cd /mnt/zsn/lyb/arti-skill/articraft_data
source .venv/bin/activate

PARENT="data/records/rec_portable-toilet-cabin-porta-potty-a-magenta-red-_20260612_113231_642886_5cec201b"

COMMON="It must still clearly read as a portable toilet cabin (porta-potty) of the same family: a roughly square molded-plastic restroom unit standing on the ground at z=0. Keep the single full-height front DOOR as a REVOLUTE joint hinged on a VERTICAL (+Z) axis at a front corner (door is the one moving part). Keep the cabin (floor pan + walls + roof + a vent) as the grounded root. Emit every repeated sub-part (wall ribs, panels, any repeated fixtures) with a 'for i in range(count)' loop driven by a single clear count variable plus a shared geometry helper, so the copy logic is mechanically readable. Change ONLY the one feature described above; keep every other functional layer identical to the parent. Keep it workbench-only."

run () {  # $1=label  $2=prompt
  echo "[launch] $1"
  articraft fork --provider dashscope --model qwen3.7-max --thinking-level high \
    --max-turns 80 --tag public_toilet_variant --label "$1" \
    "$PARENT" "$2 $COMMON" > "/tmp/fork_pt_$1.log" 2>&1 &
}

run roof_gable      "Change ONLY the roof to a peaked / gabled pitched roof (a clear ridge line running front-to-back), replacing the gently sloped flat roof. Keep the translucent skylight feel."
run roof_domed      "Change ONLY the roof to a smoothly curved / domed barrel-vault roof, replacing the sloped flat roof."
run roof_flat       "Change ONLY the roof to a totally FLAT low-profile roof with a small raised edge lip, replacing the sloped roof; lower overall height."
run door_window     "Change ONLY the door: hinge it on the OPPOSITE front corner and give it a large clear vision window panel in the upper half plus a kick panel, instead of the louvered top. Keep it a vertical REVOLUTE door."
run walls_corrugated "Change ONLY the wall surface treatment to bold horizontal CORRUGATED ribbing (deep wavy horizontal flutes), replacing the vertical ribs."
run walls_smooth    "Change ONLY the wall surface treatment to smooth flat recessed rectangular PANELS (framed flush panels, no ribs)."
run vent_twin       "Change ONLY the vent: replace the single corner vent stack with TWO tall vent stack pipes (one at each rear corner), each with a cap."
run utility_handwash "Change ONLY by adding an exterior hand-wash / sink module: a molded basin unit bolted to one side wall with a paddle tap, plus a roof skylight panel. Do not add new moving joints."
run interior_urinal "Change ONLY the interior fixture (visible through the open door) to a wall-mounted URINAL with a privacy screen instead of the seated toilet pan."
run base_skid       "Change ONLY the base to a heavy steel forklift SKID base / raised platform with fork pockets, lifting the cabin slightly off the ground (the cabin still reads as resting on its skid)."
run accessible_wide "Change ONLY the footprint to a WIDER wheelchair-accessible cabin (about 1.5x wider plan) with an interior grab bar on the side wall and a wider door leaf."
run tall_narrow     "Change ONLY the proportions to a TALLER, NARROWER slim cabin silhouette, keeping the same layout."

wait
echo "ALL PUBLIC_TOILET FORKS DONE"
