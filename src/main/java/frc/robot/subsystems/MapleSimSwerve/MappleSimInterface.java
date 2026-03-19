package frc.robot.subsystems.MapleSimSwerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Represents a subsystem responsible for controlling the drive system of a robot. This includes methods for controlling
 * the movement, managing swerve drive modules, tracking the robot's position and orientation, and other related tasks.
 */
public interface MappleSimInterface extends Subsystem{
    void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop);

    void setModuleStates(SwerveModuleState[] desiredStates);

    ChassisSpeeds getMeasuredSpeeds();

    Rotation2d getGyroYaw();

    Pose2d getPose();

    void setPose(Pose2d pose);

    default Rotation2d getHeading() {
        return getPose().getRotation();
    }

    default void setHeading(Rotation2d heading) {
        setPose(new Pose2d(getPose().getTranslation(), heading));
    }

    default void zeroHeading() {
        setHeading(new Rotation2d());
    }

    void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds);
    void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    
        /** Configures the PathPlanner auto builder */
    default void configurePPAutoBuilder() {
        try {
        RobotConfig config = RobotConfig.fromGUISettings();
        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getMeasuredSpeeds,
            (speeds) -> this.drive(speeds, false, true),
            new PPHolonomicDriveController(
                new PIDConstants(5, 0, 0),
                new PIDConstants(4, 0, 0)),
            config, // copied integration from SwerveSubsystem 
            () -> DriverStation.getAlliance()
                    .orElse(DriverStation.Alliance.Blue)
                    .equals(DriverStation.Alliance.Red),
            this
        );
    } catch (Exception e) {
        throw new RuntimeException("Failed to load RobotConfig from PathPlanner GUI settings", e);
    }
 }

}