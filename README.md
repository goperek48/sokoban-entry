# Sokoban solver — make.fun sokoban-solver-01-run-20261121

Push-optimal Sokoban solver, Node 22, stdlib only, deterministic.

## How it works

A generated level is a 1-wide hallway (the player's start row) plus 1–4
rooms, each opening onto the hallway through exactly one door. A box pushed
into the hallway can never be pushed back, so in any optimal solution every
box stays in its room and the level's push-optimum is the sum of the rooms'
optima. The solver therefore:

1. splits the level into rooms by removing the hallway row,
2. solves each room independently with A* over push states (boxes +
   canonical player region), using an exact Manhattan-assignment heuristic
   and dead-square + corner pruning,
3. stitches the per-room move strings with shortest player walks along the
   hallway (rooms whose door cell is a goal are solved last, so a box
   ending on the door cannot trap the player inside).

A whole-level A* is kept as a fallback for levels that do not decompose.
Search budgets are node counts only — the wall clock is never read, so two
runs write identical bytes.

## Run

```
node main.ts            # reads /input/<id>/level.txt, writes /output/<id>/solution.txt
node main.ts --input <dir> --output <dir>
```

## Tests (local)

- Practice set: all 5 instances push-optimal (13/16/29/47/60 pushes),
  score 1,000,000, ~1.2 s for the set.
- Two fresh sets drawn with the kit's own generator from other seeds: all
  10 instances push-optimal, score 1,000,000 each, ~1 s per set.
- Two runs on the practice set produce byte-identical outputs (sha256).
