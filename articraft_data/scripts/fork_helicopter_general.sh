#!/bin/bash
# Parallel Qwen forks of the CH-47 tandem-rotor parent into single-axis variants
# for the Military/helicopter subcategory sample pool.
set -u
cd /mnt/zsn/lyb/arti-skill/articraft_data
source .venv/bin/activate

PARENT="data/records/rec_model-a-ch-47-chinook-style-tandem-rotor-militar_20260610_080515_971792_e6faf6f7"

COMMON="It must still clearly read as a tandem-rotor heavy-lift helicopter of the CH-47 Chinook family (two main rotors fore and aft, no tail rotor). Keep the front and rear main rotors as CONTINUOUS spin joints, the cargo ramp as a REVOLUTE joint, and all four landing-gear wheels as CONTINUOUS spin joints. Keep the hollow cargo cabin (open interior with troop seats) and the rear exhaust-nozzle cover plate. Emit every repeated sub-part (rotor blades, troop seats, exhaust nozzles) with a 'for i in range(count)' loop driven by a single clear count variable plus a shared geometry helper, so the copy logic is mechanically readable. Change ONLY the one feature described above; keep every other functional layer identical to the parent. Keep it workbench-only."

run () {  # $1=label  $2=prompt
  echo "[launch] $1"
  articraft fork --provider dashscope --model qwen3.7-max --thinking-level high \
    --max-turns 80 --tag helicopter_general_variant --label "$1" \
    "$PARENT" "$2 $COMMON" > "/tmp/fork_$1.log" 2>&1 &
}

run front_blades_4 "Change ONLY the FRONT main rotor to have 4 blades (the rear rotor stays 3 blades). Paint it olive-drab green."
run front_blades_6 "Change ONLY the FRONT main rotor to have 6 blades (the rear rotor stays 3 blades). Use a naval medium-gray paint scheme."
run rear_blades_4  "Change ONLY the REAR main rotor to have 4 blades (the front rotor stays 3 blades). Use a two-tone desert sand-and-brown scheme."
run rear_blades_6  "Change ONLY the REAR main rotor to have 6 blades (the front rotor stays 3 blades). Use a dark gunship gray-black scheme."
run seats_4        "Change ONLY the cabin troop seats to 4 per side (fewer, more widely spaced). Use a forest-green camouflage scheme."
run seats_14       "Change ONLY the cabin troop seats to 14 per side (more, densely packed). Use a khaki/tan scheme."
run exhaust_1      "Change ONLY the rear exhaust to a SINGLE central nozzle (set the nozzle loop count to 1). Use a dark-green scheme."
run exhaust_4      "Change ONLY the rear exhaust to FOUR nozzles in a row across the cover plate (nozzle loop count = 4). Use a light-gray scheme."
run hull_boxy      "Change ONLY the fuselage cross-section profile family to a BOXIER, more slab-sided hull (squarer cross-sections); keep the overall length, the rotors, cabin and gear unchanged. Use a blue-gray scheme."
run hull_round     "Change ONLY the fuselage cross-section profile to a ROUNDER, fatter / more bulbous hull (rounder cross-sections); keep everything else unchanged. Use a medium-green scheme."

wait
echo "ALL FORKS DONE"
