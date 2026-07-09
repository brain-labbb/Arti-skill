#!/bin/bash
# Two additional HIGH-divergence hull-shape forks of the CH-47 tandem-rotor parent
# for the Military/helicopter subcategory pool. These vary the LONGITUDINAL profile /
# overall massing (orthogonal to the earlier hull_boxy / hull_round cross-section
# variants), so the downstream template's shape slot gets distinct silhouettes.
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

run hull_stretch "Change ONLY the fuselage longitudinal proportions to a STRETCHED long-haul body: elongate the cargo cabin section by roughly 40-50% (add more side window stations along the lengthened cabin) for a long-bodied heavy-lift silhouette. Keep the tandem layout correct: the front main rotor stays over the forward fuselage, and the rear main rotor and its aft pylon move rearward so the rear rotor still sits over the lengthened tail end. Keep the cabin interior, landing gear and exhaust otherwise unchanged. Use a high-visibility two-tone white-and-international-orange scheme."

run hull_taper "Change ONLY the fuselage profile to a TAPERED, streamlined teardrop hull: a pointed / tapered nose and an upswept boat-tail rear, sleeker overall, clearly distinct from the blunt flat-fronted Chinook nose. Keep the overall length, the rotor positions, cabin and landing gear otherwise unchanged. Use a deep navy-blue over light-gray belly scheme."

wait
echo "ALL SHAPE FORKS DONE"
