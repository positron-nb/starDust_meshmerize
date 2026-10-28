# Example arenas

| File | Content |
|---|---|
| `sample_arena.json` (+ `.png`) | rulebook Figure 1, reconstructed by pixel measurement (±2 cm). True shortest START→box = 6580 mm |
| `generated/gen_seed*.json` (+ `.png`) | random rule-compliant mazes from `tools/generate_maze.py` |

Arena JSON format (mm, y up):
```json
{"name": "...", "size_mm": [2300, 2300], "line_width_mm": 30,
 "segments": [[x1, y1, x2, y2], ...],          // axis-aligned centrelines
 "end_box": [x, y, 400, 400],                  // lower-left corner
 "start": {"x": ..., "y": ..., "heading_deg": ...},
 "checkpoints": [[x, y], ...]}
```
Run any of them: `python -m simulator.main --arena simulator/examples/sample_arena.json`.
To model the real competition arena once it is known, measure it, write it in this format, and run
the batch tool on it with several noise seeds.
