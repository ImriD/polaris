# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-02-23)

**Core value:** Commands from the Mac reach the real Pixhawk through the Jetson — the full hardware path works end-to-end on the table
**Current focus:** Phase 1 — Jetson Provisioning

## Current Position

Phase: 1 of 4 (Jetson Provisioning)
Plan: 0 of TBD in current phase
Status: Ready to plan
Last activity: 2026-02-23 — Roadmap created

Progress: [░░░░░░░░░░] 0%

## Performance Metrics

**Velocity:**
- Total plans completed: 0
- Average duration: -
- Total execution time: 0 hours

**By Phase:**

| Phase | Plans | Total | Avg/Plan |
|-------|-------|-------|----------|
| - | - | - | - |

**Recent Trend:**
- Last 5 plans: -
- Trend: -

*Updated after each plan completion*

## Accumulated Context

### Decisions

Decisions are logged in PROJECT.md Key Decisions table.
Recent decisions affecting current work:

- Native compilation on Jetson (not cross-compile) — r2r bindgen requires target's libclang headers; Jetson has 16GB RAM
- ROS2 Humble from apt — Ubuntu 22.04 only; Jazzy would require source build
- MAVLink over UDP — Pixhawk as UDP client reaching out to Jetson at udpin:0.0.0.0:14550
- fastgpu untouched — all rig changes are strictly additive

### Pending Todos

None yet.

### Blockers/Concerns

- Phase 2: Pixhawk exact model and NET_P1 defaults unknown — verify live with mavproxy before writing Phase 2 plan
- Phase 3: polaris_wheel may have fastgpu address hardcoded — needs CLI arg or POLARIS_SERVER env var
- Phase 4: polaris_bridge GPS-absent behavior unverified — may panic if Pixhawk sends no GPS indoors

## Session Continuity

Last session: 2026-02-23
Stopped at: Roadmap created, ready to plan Phase 1
Resume file: None
