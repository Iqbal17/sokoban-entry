# Sokoban Solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Iqbal17/sokoban-entry/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Iqbal17/sokoban-entry/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver in a single `main.ts` (Node 22, stdlib only). It reads each `level.txt` under `/input/<id>/` and writes the player's moves to `/output/<id>/solution.txt`.

How it works: every generated level is a hallway with rooms hanging off it, and a box pushed through a door can never come back, so the solver splits the level into rooms, solves each room on its own with a push-based A* (min-cost box-to-goal assignment heuristic, dead-square pruning and 2x2 freeze detection), and stitches the room solutions together with walks along the hallway. All budgets are counted in search expansions, never in wall-clock time, so two runs on the same set write identical bytes.

Entered on [make.fun](https://make.fun/?ref=sjc4apwe&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
