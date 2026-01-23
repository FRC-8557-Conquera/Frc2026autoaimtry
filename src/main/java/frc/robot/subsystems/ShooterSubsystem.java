
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private final TurretSubsystem turret;
    private final HoodSubsystem hood;
    private final FlywheelSubsystem flywheel;
    private final SwerveSubsystem swerve;

    // Hub pose on field (example, blue alliance)
   private static final Pose2d HUB_POSE =
    new Pose2d(12.742, 4.238, new Rotation2d());

    public ShooterSubsystem(
        TurretSubsystem turret,
        HoodSubsystem hood,
        FlywheelSubsystem flywheel,
        SwerveSubsystem swerve
    ) {
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.swerve = swerve;

        // 🔥 Default command: sürekli açı setpoint’i gönder
        turret.setDefaultCommand(
            turret.setAngle(this::calculateTurretAngle)
        );
    }

    /** 🎯 Field-relative turret angle (SMC PID handles control) */
    private edu.wpi.first.units.measure.Angle calculateTurretAngle() {
        Pose2d robotPose = swerve.getPose();

        Rotation2d fieldTargetAngle =
            HUB_POSE.getTranslation()
                .minus(robotPose.getTranslation())
                .getAngle();

        // Turret robot-relative çalışır
        Rotation2d turretAngle =
            fieldTargetAngle.minus(robotPose.getRotation());

        return Degrees.of(turretAngle.getDegrees());
    }
}
