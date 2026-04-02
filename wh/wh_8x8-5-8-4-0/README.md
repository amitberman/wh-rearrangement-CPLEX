# wh_8x8-5-8-4-0

This directory contains a slightly more complex `8x8` benchmark set.

Characteristics:

- `8x8` map
- `5` agents
- `8` obstacle entries per scenario
- `4` active rearrangement tasks per scenario
- a structured map with fixed shelves (`#`) that create narrower routing choices

Files:

- `wh_8x8.map`: shared map
- `wh_8x8-5-8-4-0.scen` to `wh_8x8-5-8-4-10.scen`: 11 scenario instances

The format is documented in [`SCENARIO_FORMAT.md`](/home/nir.miller/wh-rearrangement-CPLEX-main/SCENARIO_FORMAT.md).
