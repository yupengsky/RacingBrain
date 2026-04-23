# Planning Function

This function starts the planning-facing interface for RacingBrain. It does not
run the final racing planner yet; it extracts a sparse track graph from the
stable cone map so later planning code can consume a small, inspectable input.

The public localization and mapping entry point exposes:

```bash
enable_planning:=true
```

Outputs:

```text
/planning/track_graph
/racingbrain/planning/input_state
```
