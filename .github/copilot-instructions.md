# Copilot Instructions — FRC Team 4982 (Olympus Robotics)

## Project Overview

This is a multi-year FRC robotics repository for Team 4982 — Olympus Robotics. It contains robot code from multiple seasons in different languages.

### Active Project

- **Theseus (2026)** — Java, GradleRIO 2026.2.1. Current season robot.

### Legacy Projects (reference only)

- **Riptide (2025)** — Python, RobotPy 2025.3.2.2 (Reef game). Dead code.
- **Perry (2024)** — Python, RobotPy 2024.2.1.1 (Crescendo).
- **BobTheCheezIt, Marvin, Kinkmaster69** — Legacy C++/Java projects (2020–2023).

## Architecture

All robots use the **WPILib Command-Based** architecture:

- `LoggedRobot` (AdvantageKit) as the robot base class for Theseus.
- Subsystems encapsulate hardware (motors, sensors, encoders).
- Commands compose subsystem actions for autonomous and teleop.
- `RobotContainer` wires subsystems, commands, and controller bindings.

## Hardware & Vendor Libraries

- **Drive motors**: CTRE TalonFX (Phoenix 6).
- **Rotation / auxiliary motors**: REV Spark Max (REVLib).
- **Gyro**: CTRE Pigeon2.
- **Swerve drive**: 4-module setup with TalonFX drive + Spark Max rotation + absolute encoders.
- **Autonomous paths**: PathPlanner (PathPlannerLib).
- **Vision**: PhotonVision (photonlib).

## Language & Build Conventions

- Build with Gradle: `./gradlew build`, deploy with `./gradlew deploy`.
- Vendor dependencies live in `vendordeps/` as JSON files.
- Follow standard WPILib Java project layout under `src/main/java/frc/robot/`.
- Use `commands2` (WPILibNewCommands) for command framework.

## Code Style

- Use descriptive names for subsystems, commands, and constants.
- Group motor configuration (PID gains, current limits, idle mode) near the top of subsystem constructors.
- Define positions and setpoints as named constants, not magic numbers.
- Prefer `Subsystem.setDefaultCommand()` for continuous teleop control.
- Log telemetry through AdvantageKit `Logger.recordOutput()` (see Logging section below).
- Use SmartDashboard only for interactive widgets (toggles, Field2d, Mechanism2d).

## Logging & Telemetry (Theseus)

Theseus uses **AdvantageKit** (`org.littletonrobotics.junction`) for all structured telemetry.

- **Always use `Logger.recordOutput("Subsystem/Key", value)`** for data that should be viewable in AdvantageScope.
- **Never publish structured data directly to NetworkTables** (e.g. `StructPublisher`, manual `NetworkTable.getTable()`). Direct NT publishers create keys under a different namespace than AdvantageKit, so AdvantageScope layouts break when switching between live and log-file sources.
- AdvantageKit's `NT4Publisher` handles live NT publishing automatically — `Logger.recordOutput()` keys appear under `/RealOutputs/` both live and in `.wpilog` files.
- **CTRE `SignalLogger`** writes are fine for low-level hoot replay but are separate from AdvantageKit.
- **SmartDashboard** is acceptable for interactive widgets (`Field2d`, `Mechanism2d`, boolean toggles, command buttons) that don't need log replay.
- Key naming convention: `"Subsystem/Key"` (e.g. `"Drivetrain/Pose"`, `"Turret/RotationPosition"`, `"Vision/EstimatedPose"`).

### Log Analysis Tools (`Theseus/tools/`)

Python scripts for working with `.wpilog` files offline. All require the `wpiutil` package.

- **`fetch_logs.sh`** / **`fetch_logs.bat`** — Download logs from the roboRIO (`10.49.82.2`) to `Theseus/logs/`. Supports `--purge` and `--purge-all`.
- **`wpilog_inspect.py`** — List, grep, or dump signals from a `.wpilog` file.
  ```
  python3 tools/wpilog_inspect.py <file.wpilog> --keys
  python3 tools/wpilog_inspect.py <file.wpilog> --grep Pose
  python3 tools/wpilog_inspect.py <file.wpilog> --dump "/RealOutputs/Drivetrain/Pose"
  ```
- **`pose_inspect.py`** — Decode and print `Pose2d` values in a time range. Watches `/RealOutputs/Drivetrain/Pose`, `/RealOutputs/Turret/DesiredPose`, and `/RealOutputs/Turret/ActualPose`.
  ```
  python3 tools/pose_inspect.py <file.wpilog> [start_sec] [end_sec]
  ```

## When Writing New Code

- Target the **Theseus** project for new 2026-season work unless told otherwise.
- Follow existing patterns in the same project — match file organization, naming, and style.
- Always use the command-based pattern: new mechanisms get a Subsystem class, actions get Command classes.
- Configure motors with appropriate current limits, idle/brake modes, and PID tuning.
- Use PathPlanner's `AutoBuilder` for autonomous routines.
- Keep CAN IDs, ports, and physical constants organized and documented.

## Code Quality Principles

<!-- https://github.com/mieweb/template-mieweb-opensource/blob/main/.github/copilot-instructions.md -->

### 🎯 DRY (Don't Repeat Yourself)
- **Never duplicate code**: If you find yourself copying code, extract it into a reusable function
- **Single source of truth**: Each piece of knowledge should have one authoritative representation
- **Refactor mercilessly**: When you see duplication, eliminate it immediately
- **Shared utilities**: Common patterns should be abstracted into utility functions

### 💋 KISS (Keep It Simple, Stupid)
- **Simple solutions**: Prefer the simplest solution that works
- **Avoid over-engineering**: Don't add complexity for hypothetical future needs
- **Clear naming**: Functions and variables should be self-documenting
- **Small functions**: Break down complex functions into smaller, focused ones
- **Readable code**: Code should be obvious to understand at first glance

### 🧹 Folder Philosophy
- **Clear purpose**: Every folder should have a main thing that anchors its contents.
- **No junk drawers**: Don’t leave loose files without context or explanation.
- **Explain relationships**: If it’s not elegantly obvious how files fit together, add a README or note.
- **Immediate clarity**: Opening a folder should make its organizing principle clear at a glance.

### 🔄 Refactoring Guidelines
- **Continuous improvement**: Refactor as you work, not as a separate task
- **Safe refactoring**: Always run tests before and after refactoring
- **Incremental changes**: Make small, safe changes rather than large rewrites
- **Preserve behavior**: Refactoring should not change external behavior
- **Code reviews**: All refactoring should be reviewed for correctness

### ⚰️ Dead Code Management
- **Immediate removal**: Delete unused code immediately when identified
- **Historical preservation**: Move significant dead code to `.attic/` directory with context
- **Documentation**: Include comments explaining why code was moved to attic
- **Regular cleanup**: Review and clean attic directory periodically
- **No accumulation**: Don't let dead code accumulate in active codebase

### 🌐 Testing with MCP Browser
- Use MCP browser in Playwright if available to test functionality
- **Never close the browser** after running MCP browser commands unless explicitly asked
- Let the user interact with the browser after navigation or testing
- Only use `browser_close` when the user specifically requests it

## Documentation Preferences

### Diagrams and Visual Documentation
- **Always use Mermaid diagrams** instead of ASCII art for workflow diagrams, architecture diagrams, and flowcharts
- **Use memorable names** instead of single letters in diagrams (e.g., `Engine`, `Auth`, `Server` instead of `A`, `B`, `C`)
- Use appropriate Mermaid diagram types:
  - `graph TB` or `graph LR` for workflow architectures 
  - `flowchart TD` for process flows
  - `sequenceDiagram` for API interactions
  - `gitgraph` for branch/release strategies
- Include styling with `classDef` for better visual hierarchy
- Add descriptive comments and emojis sparingly for clarity

### Documentation Standards
- Keep documentation DRY (Don't Repeat Yourself) - reference other docs instead of duplicating
- Use clear cross-references between related documentation files
- Update the main architecture document when workflow structure changes

## Working with GitHub Actions Workflows

### Development Philosophy
- **Script-first approach**: All workflows should call scripts that can be run locally
- **Local development parity**: Developers should be able to run the exact same commands locally as CI runs
- **Simple workflows**: GitHub Actions should be thin wrappers around scripts, not contain complex logic
- **Easy debugging**: When CI fails, developers can reproduce the issue locally by running the same script

## Quick Reference

### 🪶 All Changes should be considered for Pull Request Philosophy

* **Smallest viable change**: Always make the smallest change that fully solves the problem.
* **Fewest files first**: Start with the minimal number of files required.
* **No sweeping edits**: Broad refactors or multi-module changes must be split or proposed as new components.
* **Isolated improvements**: If a change grows complex, extract it into a new function, module, or component instead of modifying multiple areas.
* **Direct requests only**: Large refactors or architectural shifts should only occur when explicitly requested.
 
### Code Quality Checklist
- [ ] **DRY**: No code duplication - extracted reusable functions?
- [ ] **KISS**: Simplest solution that works?
- [ ] **Minimal Changes**: Smallest viable change made for PR?
- [ ] **Naming**: Self-documenting function/variable names?
- [ ] **Size**: Functions small and focused?
- [ ] **Dead Code**: Removed or archived appropriately?
- [ ] **Accessibility**: ARIA labels and semantic HTML implemented?
- [ ] **I18N**: User-facing text externalized for translation?
- [ ] **Lint**: Run linter if appropriate
- [ ] **Test**: Run tests
