# wh_40x40-9-10-5-0

This directory contains a revised `40x40` benchmark family designed to be both solvable and warm-start friendly.

Characteristics:

- `40x40` map
- `9` agents
- `10` obstacle entries per scenario
- `5` active rearrangement tasks per scenario
- one central `10x10` shelf block, mirroring the successful structure of the existing `30x30` hard set
- long task routes that compete for the same approach corridors around the center, increasing the chance of repeated NATCBS low-level replans and CPLEX model reuse

Design intent:

- preserve readability and structure instead of random coordinates
- keep scenarios solvable under the current solver
- trigger repeated flow solves often enough for differential updates and basis reuse to matter

Files:

- `wh_40x40.map`: shared map
- `wh_40x40-9-10-5-0.scen` to `wh_40x40-9-10-5-10.scen`: 11 scenario instances

The format is documented in [`SCENARIO_FORMAT.md`](/home/nir.miller/wh-rearrangement-CPLEX-main/SCENARIO_FORMAT.md).
