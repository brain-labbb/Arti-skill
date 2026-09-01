#!/usr/bin/env python3
"""Run the PartNet-ontology-aligned PV-A hierarchy correctness cohort."""

import run_pva_matched_hierarchy as base


base.DEFAULT_OUTPUT = (
    base.EXP_ROOT / "runtime/nano3d_hierarchy_correctness/pva"
)
base.PROTOCOL_ID = "nano3d_hierarchy_partnet_correctness_five_category_v1"
base.CATEGORY_TEMPLATES = {
    "storage_furniture": "drawer_cabinet_with_sliding_drawers",
    "table": "folding_camp_table",
    "refrigerator": "refrigerator_with_hinged_doors",
    "dishwasher": "dishwasher_with_dropdown_door_and_sliding_racks",
    "microwave": "Kitchen_Microwave",
}


if __name__ == "__main__":
    raise SystemExit(base.main())
