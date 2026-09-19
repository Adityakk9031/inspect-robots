# inspect-robots-jev — agent guide

Policy `jev`: TypeSafe's Jev (text-only, picks one option from a menu) drives
the YAM arms from AprilTag world state. Spec and task list:
`plans/0076-jev-decision-policy.md` at the repo root. Read §0 (terms) and the
"Global constraints" of the implementation plan before editing.

## Modules

| File | Responsibility |
|---|---|
| `world.py` | `Vec3`, `ObjectView`, `GripperView`, `WorldState`, direction words, `describe_offset` |
| `_decisions.py` | `DecisionsClient` for OpenRouter alpha / TypeSafe direct; injectable `http_post` |
| `menu.py` | `Move`, `build_menu`, `parse_option` (option-id grammar) |
| `phase.py` | `TaskConfig`, `Phase`, `PhaseMachine` (cube-into-bowl) |
| `serializer.py` | `build_state`, `instructions_for` — curated state text in directional words |
| `motion.py` | `MotionMapper`: menu pick → bounded absolute Cartesian `ActionChunk` |
| `calibration.py` | camera→arm transforms from a touched table tag; JSON load/save |
| `perceiver.py` | AprilTag detections → `WorldState` (bundle fusion, depth refinement, occlusion memory) |
| `policy.py` | `JevPolicy`, `jev_policy` registry entry |
| `scorer.py` | `cube_in_bowl` pose-based scorer |

## Invariants

- Wording is load-bearing: directional words, 0.5 cm rounding, and per-option
  hints are pinned by tests. Rerun `examples/jev_textsim.py` (live API) before
  changing them.
- Code curates what Jev sees: only the current target's geometry, other
  objects by name. Code owns phases and completion; Jev only picks a move.
- One Choice question per request. Confidence is logged, never gated.
- Every I/O edge (HTTP, tag detector) is injectable; the whole policy runs in
  tests with no network, camera, or robot.
- Gates: ruff, ruff format, mypy strict (src + tests), pytest 100 % coverage.
