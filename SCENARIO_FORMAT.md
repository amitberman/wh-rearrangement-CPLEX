# Scenario and Map Encoding

This repository uses a simple plain-text encoding for maps and scenarios.

## Map files

Map files use this layout:

```text
<rows> <cols>
<row_0>
<row_1>
...
<row_{rows-1}>
```

Rules:

- The first line gives the map size.
- Each following row must contain exactly `cols` characters.
- Supported cell symbols are:
  - `.`: passable cell
  - `#`: static obstacle
  - `@`: blocked cell

Example:

```text
8 8
........
.##..##.
........
..####..
........
.##..##.
........
........
```

## Scenario files

Scenario files use this layout:

```text
<num_agents> <num_obstacles> <num_tasks>
<agent_0_y> <agent_0_x>
...
<agent_{num_agents-1}_y> <agent_{num_agents-1}_x>
<pickup_0_y> <pickup_0_x> <delivery_0_y> <delivery_0_x>
...
<pickup_{num_obstacles-1}_y> <pickup_{num_obstacles-1}_x> <delivery_{num_obstacles-1}_y> <delivery_{num_obstacles-1}_x>
```

Rules:

- The first line is:
  - `num_agents`: number of agent start locations
  - `num_obstacles`: number of obstacle entries
  - `num_tasks`: number of obstacle entries whose pickup and delivery differ
- The next `num_agents` lines are agent start positions.
- The next `num_obstacles` lines are obstacle definitions.
- Coordinates are written as `y x` in the file.
- Inside the code, locations are stored as `(x, y)`, where `x` is the row index and `y` is the column index.
- An obstacle line is:
  - inactive/already placed if `pickup == delivery`
  - an active task if `pickup != delivery`
- Therefore `num_tasks` must equal the count of active obstacle lines.

Example:

```text
5 5 1
0 6
3 7
1 5
7 0
4 0
4 0 4 0
1 5 1 5
0 3 0 3
1 0 3 1
3 6 3 6
```

Human interpretation of the example:

- `5 5 1` means 5 agents, 5 obstacle entries, and 1 real task.
- The first 5 two-number lines are agent starts.
- The last 5 four-number lines are obstacle pickup and delivery pairs.
- Only `1 0 3 1` is an active task, because its pickup and delivery differ.
