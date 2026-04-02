# wh_20x20-5-5-1-0

This directory contains a relatively simple `20x20` benchmark set.

Characteristics:

- `20x20` map
- `5` agents
- `5` obstacle entries per scenario
- `1` active rearrangement task per scenario
- a mostly open map with a tiny central shelf block to make it only slightly more constrained than the original open `8x8` set

Files:

- `wh_20x20.map`: shared map
- `wh_20x20-5-5-1-0.scen` to `wh_20x20-5-5-1-10.scen`: 11 scenario instances

The format is documented in [`SCENARIO_FORMAT.md`](/home/nir.miller/wh-rearrangement-CPLEX-main/SCENARIO_FORMAT.md).
