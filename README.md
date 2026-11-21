# Sokoban Solver

![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)
![make.fun rank](https://make.fun/badge/sokoban-solver-01-run-20261121/taufik-hidayat/sokoban-solver-01-run-20261121/rank.svg)
![make.fun reward](https://make.fun/badge/sokoban-solver-01-run-20261121/taufik-hidayat/sokoban-solver-01-run-20261121/reward.svg)

Referral: `2z9vc20p`

A push-optimal Sokoban solver for the make.fun `sokoban-solver-01-run-20261121`
bounty. One program (`main.ts`, Node 22, no dependencies) answers all 5
instances of a set in a single run.

## How it works

Every level in this competition is a one-cell-wide hallway with 1–4 rooms
hanging off it, 3–4 boxes per room, and a box pushed out of its room can never
come back — so a level's push-optimal count is the sum of its rooms' optima.
The solver therefore:

1. splits the level into rooms at the hallway row,
2. solves each room on its own with A\* over **pushes** (the player position
   is canonicalised to its reachable region), using a minimum-assignment
   box-to-goal heuristic, dead-cell pruning from the pull graph, and 2×2
   freeze pruning,
3. solves each room twice — once unconstrained and once requiring the final
   state to leave the door free and reachable, so the player can walk out —
   and puts the room whose exit costs the most extra pushes last,
4. stitches the rooms together with plain BFS walks along the hallway.

Work is budgeted by node expansions, never by the wall clock, so two runs
produce byte-identical output. No randomness, no network, no clock reads.

## Practice set

| instance | pushes | P\* | points |
|----------|--------|----|--------|
| i001 | 13 | 13 | 1,000 |
| i002 | 16 | 16 | 1,000 |
| i003 | 29 | 29 | 1,000 |
| i004 | 47 | 47 | 1,000 |
| i005 | 60 | 60 | 1,000 |

**Practice score: 1,000,000 / 1,000,000** (all five push-optimal) in ~0.6 s
for the whole set. The house baseline (plain-astar 1.0.0) scores 200,000 on
the same set — it solves only i001.

## Run

```sh
MAKEFUN_INPUT_DIR=<input> MAKEFUN_OUTPUT_DIR=<output> node main.ts
```

It reads `$MAKEFUN_INPUT_DIR/<id>/level.txt` and writes
`$MAKEFUN_OUTPUT_DIR/<id>/solution.txt` for every instance directory found.
