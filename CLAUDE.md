# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Deploy

```bash
./gradlew build         -Dorg.gradle.java.home="/Users/a.melihsen/wpilib/2026/jdk"  # Compile
./gradlew deploy        -Dorg.gradle.java.home="/Users/a.melihsen/wpilib/2026/jdk"  # Deploy to RoboRIO
./gradlew simulateJava  -Dorg.gradle.java.home="/Users/a.melihsen/wpilib/2026/jdk"  # Run simulation
```

GradleRIO 2026.2.1, Java 17. Team number from `.wpilib/wpilib_preferences.json`.

## Architecture

FRC 2026 REBUILT robot code. Command-based, using WPILib's `SubsystemBase` + `Command` pattern via `CommandScheduler`. Entry point: `Robot.java` (extends `LoggedRobot` from AdvantageKit) -> `RobotContainer.java`.

### Core Libraries
- **YAGSL** — Swerve drive abstraction. JSON configs in `src/main/deploy/swerve/`. Do NOT hand-write swerve math.
- **YALL** — Limelight vision. Uses `Limelight`, `LimelightPoseEstimator`, `PoseEstimate`. Currently on MegaTag1 (no gyro dependency).
- **YAMS** — Mechanism abstraction (`SmartMotorController`, `FlyWheel`, `Pivot`). Used by Feeder, Flywheel, Hood, Turret. NOT used by Intake (raw Phoenix6).
- **PathPlannerLib** — Autonomous paths. `AutoBuilder` configured in `SwerveSubsystem`.
- **AdvantageKit** — Logging via `Logger`. Always init Logger before RobotContainer.

### Subsystem Layout
```
subsystems/
  swerve/SwerveSubsystem.java      — Drive + vision fusion (YAGSL + YALL)
  intake/IntakeSubsystem.java       — Raw Phoenix6 TalonFX, Motion Magic, CANivore bus
  feeder/FeederSubsystem.java       — YAMS SparkWrapper
  spindexer/SpindexerSubsystem.java — YAMS SparkWrapper
  shooter/
    ShooterSubsystem.java           — Orchestrator (owns Turret, Hood, Flywheel)
    TurretSubsystem.java            — YAMS SparkWrapper + DutyCycleEncoder
    HoodSubsystem.java              — YAMS TalonFXWrapper Pivot + DutyCycleEncoder
    FlywheelSubsystem.java          — YAMS TalonFXWrapper FlyWheel
```

### Hardware Specifics
- **CAN buses**: CANivore `"Penci Zorno Canivore"` for Krakens + CANCoders. RoboRIO bus for SparkMAXes.
- **Intake** uses raw Phoenix6 (not YAMS) — intentional, do not migrate.
- **NavX SPI** gyro is unreliable — vision uses MT1 (no gyro input needed).
- **Swerve modules are split-bus**: Kraken drive + CANCoder on CANivore, SparkMAX angle on RoboRIO.

### Vision Pipeline
`SwerveSubsystem.addVisionFromEstimator()` handles MT1 pose estimates with:
- Ambiguity < 0.2, distance < 4m
- Quadratic distance-scaled std devs
- Single-tag rotation rejected (std dev = 9999)
- Per-measurement std devs (not global)

### Game Logic
`Robot.java` tracks FRC 2026 REBUILT shift structure (Transition -> Shift 1-4 -> Endgame) using `DriverStation.getGameSpecificMessage()`. Hub activity alternates between alliances.

## Conventions
- Turkish variable/class names are common (e.g., `YerToplamaCommand`, `intakeicerigeri`, `KAPALI_POZISYON`)
- Comments are often in Turkish
- YAMS configs use fluent builder pattern: `.withClosedLoopController().withGearing().withIdleMode()...`
- YAMS followers: `.withFollowers(Pair.of(rawMotor, invertedBoolean))` using `edu.wpi.first.math.Pair`
