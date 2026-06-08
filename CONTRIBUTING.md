# Contributing

Thanks for your interest in the OdyssiX BFMC 2026 Brain repository.

## Scope of contributions

This repository represents the OdyssiX BFMC 2026 race-day deployment. The goal of accepting contributions is to keep the published stack maintainable for future BFMC teams and other small autonomous-vehicle projects. Contributions that fit:

- Documentation improvements (clarifications, fixes, missing context).
- Bug fixes that improve reproducibility on other Jetson Orin Nano units.
- Build / dependency fixes for newer JetPack releases.
- Compatibility patches when ROS 2 Humble reaches EOL.
- Hard-coded-path elimination and similar release-engineering improvements.

Contributions that we will likely decline:

- Algorithmic redesigns (the deployed perception / planning / control behavior is preserved as-is for traceability).
- Feature additions beyond what the deployed system did.
- Stylistic-only PRs without a clear motivation.

## How to propose a change

1. Open an issue describing the problem or proposal.
2. Fork the repository and create a branch named after the issue.
3. Make the change. Keep the diff focused on one concern.
4. Run a local colcon build (`colcon build --symlink-install`) and ensure the package(s) you touched still build.
5. Open a PR referencing the issue.

## Style

- C++ — match the surrounding style in each file. The repo does not currently enforce clang-format; we will adopt it before formalising style rules.
- Python — PEP 8, 4-space indent. The `ament_python` packages already include `test_pep257.py` / `test_flake8.py` in the prototype trees (now archived) — feel free to wire equivalent linters into any new Python package.
- ROS 2 — follow the patterns visible in the deployed code: composable nodes where intra-process comms matter; lifecycle nodes for anything the lifecycle manager owns; topics over services.

## Branch hygiene

- One change per PR.
- Rebase on `main` (or the protected branch) before opening a PR.
- Squash merges are preferred for clean history.

## Reporting safety issues

Safety-relevant issues — anything that could let the vehicle take a non-commanded action, hold last actuator state past the 500 ms deadman, or bypass the safety supervisor — should be reported following `SECURITY.md` rather than as a public issue.

## License

Contributions are accepted under the project's Apache License, Version 2.0. See `LICENSE`.

## Code of conduct

Be respectful. Disagree with the idea, not the person. We are a small student-team project; we ask for the same patience you would extend to any volunteer maintainer.
