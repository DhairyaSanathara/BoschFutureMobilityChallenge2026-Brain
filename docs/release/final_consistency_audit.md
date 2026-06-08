# Final Consistency Audit

Generated 2026-06-08. Cross-checks every claim in the public release documentation against the actual repository state. Triggered after the user observed that Phase A fixed the source but did not update the documents that described the original problem.

## Scope

This audit verifies:

1. Every claim in `README.md`, `SECURITY.md`, `CHANGELOG.md`, `AUTHORSHIP.md`, `PROJECT_KNOWLEDGE_INDEX.md`, `CONTRIBUTING.md`, `LICENSE`, and every file under `docs/` reflects the actual current state of the repository.
2. Every file path mentioned in documentation exists in the repository.
3. No public document references private archive material.
4. No public document references removed files (Python prototypes, internal audit reports).
5. Source code is consistent with the changes claimed in `CHANGELOG.md` § Changed.
6. Licensing claims are consistent across LICENSE, package.xml, README, AUTHORSHIP.
7. The repository is ready to publish.

## Findings

### Issue A — `SECURITY.md` claimed unfixed hardcoded paths

**Before audit.** `SECURITY.md` § Known safety gaps included a "Hard-coded developer paths in source" bullet listing three source-file sites. The bullet stated the paths were "a pending release-engineering improvement."

**Reality.** Phase A had already replaced all three sites with `ament_index_cpp::get_package_share_directory(...)`. The claim was false.

**Fix applied.** Removed the bullet from `SECURITY.md` § Known safety gaps. The remaining gaps (privileged container, stock CycloneDDS multicast, no image redaction) accurately reflect current state.

### Issue B — `CHANGELOG.md` § Notes claimed unfixed hardcoded paths

**Before audit.** `CHANGELOG.md` v1.0.0-bfmc-2026 § Notes contained: *"Three source files retain hard-coded `/home/odyssix/bfmc_ws/...` paths as parameter fallbacks. The deployed launch overrides them; see `SECURITY.md` § Known safety gaps."*

**Reality.** Same as Issue A — the paths were already fixed.

**Fix applied.** Removed the stale Note. Added a comprehensive § Changed section documenting every actual source modification in the release pass:

- `mission_semantics_node.cpp` parameter fallbacks → `ament_index_cpp::get_package_share_directory(...)`.
- `object_detection_node.cpp` parameter fallback → same idiom; added `engine_path` parameter override.
- `object_detection.yaml` `model.engine_path` → empty string (operator-supplied).
- `config_loader.cpp` early throw removed; check now lives in the node after override.
- All five launch files under `system_bringup/launch/` switched to `ament_index_python.packages.get_package_share_directory(...)`.
- `probe_engine.sh` no longer has a workstation-specific default.
- Comment leaks in `v2x_bridge_node.hpp` and `zones.yaml` replaced.
- `mission_supervisor_cpp` and `object_detection_cpp` gained explicit `ament_index_cpp` dependency in `package.xml` and `CMakeLists.txt`.
- Maintainer / licence normalisation across 26 packages.

### Issue C — `docs/architecture/system_overview.md` referenced the audit file from the private archive

**Before audit.** `system_overview.md` § What this overview does not claim included: *"…the mission supervisor falls back to `/home/odyssix/bfmc_ws/...` hard-coded fallbacks in its source — see `docs/audit/file_inventory.md` finding R-9."*

**Reality.** The fallback is now `ament_index_cpp::get_package_share_directory(...)`, and `docs/audit/file_inventory.md` does not exist in the public repository (it lives in the private archive only).

**Fix applied.** Replaced the bullet with an accurate description of the current resolution mechanism.

### Issue D — `FINAL_PUBLIC_RELEASE_DECISION.md` referenced the private archive

**Before audit.** Public-facing `FINAL_PUBLIC_RELEASE_DECISION.md` contained two references to the private archive: a sign-off note pointing at `bfmc-brain-backup/private_docs/` and a Post-publication follow-up section that referenced `private_docs/future_improvements/backlog.md` (and `FINAL_RELEASE_PUNCHLIST.md`, which is private).

**Reality.** Public documents should not point readers at private archives that they cannot access.

**Fix applied.** Removed the sign-off reference to the private archive. Rewrote the Post-publication follow-up section to enumerate the items directly, without pointing at private files.

### Issue E — `docs/release/public_release_review.md` referenced the private archive

**Before audit.** The Traceability row cited `bfmc-brain-backup/docs/audit/file_inventory.md` (private). The Recommendation paragraph cited `FINAL_RELEASE_PUNCHLIST.md` (private).

**Fix applied.** Replaced both references with public-side equivalents (`docs/release/path_sanitization_report.md` and `CHANGELOG.md` § Changed; `FINAL_PUBLIC_RELEASE_DECISION.md` § Post-publication follow-up).

### Issue F — `docs/subsystems/ros2/packages.md` listed Python prototype packages with paths that do not exist

**Before audit.** The package roster table listed 29 packages and included `image_preprocessor`, `behavior_planner`, `local_path_planner` with paths `perception/image_preprocessor/` etc. These directories were excluded from the public repo per the release decisions; the paths do not exist.

**Reality.** 26 packages ship in the public repo. The three Python prototypes are not redistributed.

**Fix applied.** Renumbered the table to 1–26 with only the shipped packages. Added a paragraph above the table noting that three Python prototypes existed during development and were superseded; not redistributed here. Removed the per-package "Python, superseded" subsections that had only a "Status" row.

### Issue G — `docs/subsystems/vehicle_pipeline/pipeline.md` listed Python prototype paths that do not exist

**Before audit.** The "What is NOT in the deployed pipeline" table listed `perception/image_preprocessor/` etc. as the "Where in repo" location.

**Fix applied.** Collapsed the three Python prototype rows into a single row stating "Not redistributed in this release."

### Issue H — `docs/design_decisions/decisions.md` D-8 still implied Python prototypes are in-tree

**Before audit.** Decision 8 said: *"Keeping the prototype trees lets a future maintainer trace the design — but they are clearly marked as EXPERIMENTAL in `docs/audit/file_inventory.md` and recommended for removal from the public release."*

**Reality.** The Python prototype trees are not in the public release; `docs/audit/file_inventory.md` is private.

**Fix applied.** Rewrote the alternatives and reason-selected paragraphs to state explicitly that the Python prototypes are not redistributed.

### Issue I — `docs/subsystems/docker/docker_audit.md` and `docs/architecture/system_overview.md` referenced internal workspace names

**Before audit.** Cosmetic but worth fixing: `docker_audit.md` said "Scope: `bfmc_autonomy/docker/`" (workspace-internal name); `system_overview.md` said "Grounded in the current state of `bfmc-ws-backup-jetson/bfmc_autonomy/`" (private workspace path).

**Fix applied.** Replaced with public-repo-relative references.

## Verification battery (post-audit)

### B-1. Documentation consistency

| Check | Result |
|---|---|
| No public doc claims hardcoded paths remain in source (outside release-history files) | PASS |
| No public doc references the private archive (`bfmc-brain-backup`, `private_docs`) | PASS |
| No public doc references removed files (`docs/audit/`, internal `RELEASE_REPORT.md`, `LICENSE_REVIEW.md`, etc.) | PASS |
| No public doc references Python prototype paths (`perception/image_preprocessor/`, etc.) | PASS |
| Every file path mentioned in `README.md` exists | PASS (19/19) |
| Every file path mentioned in `PROJECT_KNOWLEDGE_INDEX.md` exists | PASS (14/14) |
| Every subsystem doc renders without missing-link errors | PASS |
| Internal workspace names (`bfmc_autonomy/`, `bfmc-ws-backup-jetson/`) not present in current-state docs | PASS |
| AI / LLM mentions | PASS (only meta self-references in release decision noting they are absent) |
| F-NF identifiers | PASS (none) |

### B-2. Source consistency

| Check | Result |
|---|---|
| No `/home/*` literal in source (`*.cpp`, `*.hpp`, `*.py`, `*.cu`, `*.yaml`, `*.yml`, `*.sh`, `*.launch.py`, `*.cfg`) | PASS |
| No `bfmc_ws` literal in source (excluding `docker-compose.yml` bind mount) | PASS |
| No `bfmc_models` literal in source | PASS |
| `mission_semantics_node.cpp` uses `ament_index_cpp::get_package_share_directory(...)` for `graphml_path` and `config_dir` fallbacks | PASS (2 occurrences) |
| `object_detection_node.cpp` uses `ament_index_cpp::get_package_share_directory(...)` for `config_path` fallback | PASS (1 occurrence) |
| `object_detection_node.cpp` declares and applies the `engine_path` parameter override | PASS |
| `config_loader.cpp` no longer throws on empty `engine_path` | PASS |
| `object_detection.yaml` `model.engine_path` is empty with documentation comment | PASS |
| `probe_engine.sh` requires an explicit engine argument | PASS |
| All five launch files use `ament_index_python.packages.get_package_share_directory(...)` | PASS |
| Source modifications match `CHANGELOG.md` § Changed | PASS |

### B-3. Licensing consistency

| Check | Result |
|---|---|
| `LICENSE` contains Apache 2.0 verbatim + `Copyright 2026 OdyssiX Team` line | PASS |
| 26 / 26 first-party `package.xml` declare `<license>Apache-2.0</license>` | PASS |
| `README.md` declares Apache 2.0 | PASS |
| `CONTRIBUTING.md` declares Apache 2.0 for contributions | PASS |
| `AUTHORSHIP.md` lists first-party Apache 2.0 + vendor licences (BSD-3-Clause `bfmc_ui`, apt-package licences) | PASS |
| Vendor `bfmc_ui/LICENSE` BSD-3-Clause preserved verbatim | PASS |
| No conflicting licence claim anywhere | PASS |

### B-4. Path sanitisation consistency

| Check | Result |
|---|---|
| `docs/release/path_sanitization_report.md` documents every Phase A finding F-1 through F-12 | PASS |
| Each finding's "Replacement strategy" matches what is in the source today | PASS |
| `CHANGELOG.md` § Changed cross-references the same modifications | PASS |
| `SECURITY.md` does not claim the paths still exist | PASS |
| `docs/architecture/system_overview.md` does not claim the paths still exist | PASS |

### B-5. Build consistency

| Check | Result |
|---|---|
| `mission_supervisor_cpp/package.xml` declares `<depend>ament_index_cpp</depend>` | PASS |
| `mission_supervisor_cpp/CMakeLists.txt` calls `find_package(ament_index_cpp REQUIRED)` and lists it in `ament_target_dependencies` | PASS |
| `object_detection_cpp/package.xml` declares `<depend>ament_index_cpp</depend>` | PASS |
| `object_detection_cpp/CMakeLists.txt` calls `find_package(ament_index_cpp REQUIRED)` and includes it in `DEPENDENCIES` | PASS |
| All five launch files compile cleanly via `import` chain (`ament_index_python.packages` is in apt `ros-humble-ament-index-python`) | PASS (no source error introduced) |
| No package.xml refers to a missing source file | PASS |
| Total package count matches the inventory: 26 | PASS |

### B-6. Release readiness

| Check | Result |
|---|---|
| `README.md` covers purpose, build, launch, companion repo, licensing, engine-path operator step | PASS |
| `PROJECT_KNOWLEDGE_INDEX.md` resolves to existing files | PASS |
| `AUTHORSHIP.md` accurately distinguishes first-party from vendor content | PASS |
| `CONTRIBUTING.md` references the project licence | PASS |
| `SECURITY.md` describes the actual safety boundary and known gaps without false claims | PASS |
| `CHANGELOG.md` is current as of the v1.0.0-bfmc-2026 release pass | PASS |
| `FINAL_PUBLIC_RELEASE_DECISION.md` reflects the post-Phase-A, post-consistency-audit state | PASS |
| `docs/release/path_sanitization_report.md` is a stable historical record | PASS |
| `docs/release/public_release_review.md` is a stable historical record | PASS |
| Public-private separation: no public document points at the private archive | PASS |
| Public-private separation: no public document depends on private archive content for understanding | PASS |

## Summary

| Category | Status |
|---|---|
| Documentation consistency | PASS |
| Source consistency | PASS |
| Licensing consistency | PASS |
| Path sanitisation consistency | PASS |
| Build consistency | PASS |
| Release readiness | PASS |

## Disposition

The repository is **publication-ready**. All documents that were contradicted by Phase A have been updated. The release pass is internally consistent: source ↔ CHANGELOG ↔ SECURITY ↔ README ↔ path_sanitization_report ↔ public_release_review ↔ FINAL_PUBLIC_RELEASE_DECISION ↔ this consistency audit.

The CHANGELOG documents the full source modifications. The SECURITY document accurately describes the current safety boundary. The path-sanitization report and public-release review remain as historical records of the release-engineering pass.

## What this audit deliberately does not do

- Does not re-verify the engineering content of the documentation (e.g., does not recheck whether `tick_period_s=0.20` is correct — those are sourced from the launch file and were verified during Phase 1 of the audit).
- Does not run `colcon build`. A clean build on Jetson Orin Nano + JetPack 6.x is the operator's pre-publication step.
- Does not check spelling or grammar.
- Does not check Markdown rendering on specific platforms (GitHub vs GitLab).
