package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class SwerveSubsystem extends SubsystemBase {

  private static final String LIMELIGHT_BACK  = "limelight-back";
  private static final String LIMELIGHT_FRONT = "limelight-front";

  Field2d field;
  SwerveDrive swerveDrive;
  private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

  public SwerveSubsystem() {
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.Swerve.maxSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    field = new Field2d();
    SmartDashboard.putData("Field", field);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    swerveDrive.setAngularVelocityCompensation(true, true, 0.2);
    swerveDrive.setModuleStateOptimization(true);
    swerveDrive.setAutoCenteringModules(false);
    swerveDrive.setHeadingCorrection(false);
    LimelightHelpers.setPipelineIndex(LIMELIGHT_BACK,  0);
    LimelightHelpers.setPipelineIndex(LIMELIGHT_FRONT, 0);
    setupPathPlanner();
  }

  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      boolean enableFeedforward = true;

      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getChassisSpeeds,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              SwerveModuleState[] states = swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative);
              SwerveDriveKinematics.desaturateWheelSpeeds(states, swerveDrive.getMaximumChassisVelocity());
              swerveDrive.drive(
                  speedsRobotRelative,
                  states,
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(5, 0, 0),
              new PIDConstants(7, 0, 0)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);
    } catch (Exception e) {
      Thread.currentThread().interrupt();
      e.printStackTrace();
      throw new RuntimeException("Failed to initialize RobotConfig", e);
    }
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(() -> {
      double x     = MathUtil.applyDeadband(translationX.getAsDouble(), 0.1);
      double y     = MathUtil.applyDeadband(translationY.getAsDouble(), 0.1);
      double omega = MathUtil.applyDeadband(angularRotationX.getAsDouble(), 0.1);

      swerveDrive.drive(
          new Translation2d(
              x * swerveDrive.getMaximumChassisVelocity(),
              y * swerveDrive.getMaximumChassisVelocity()),
          omega * swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    });
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
    for (SwerveModule mod : swerveDrive.getModules()) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false, false);
    }
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public void zeroGyroWithAlliance() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        zeroGyro();
        resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
        zeroGyro();
        resetOdometry(new Pose2d(0, 0, new Rotation2d()));
    }
}
  

  public void lock() {
    swerveDrive.lockPose();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void setMaximumSpeed(double speed) {
    swerveDrive.setMaximumAllowableSpeeds(speed, 9.424);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveDrive.getModules()) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public void driveFieldOriented(ChassisSpeeds velSpeeds) {
    swerveDrive.driveFieldOriented(velSpeeds);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.drive(chassisSpeeds);
  }

  @Override
  public void periodic() {
    double yawDeg = getHeading().getDegrees();
    double yawRateDegS = swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond);

    LimelightHelpers.SetRobotOrientation(LIMELIGHT_BACK,  yawDeg, yawRateDegS, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LIMELIGHT_FRONT, yawDeg, yawRateDegS, 0, 0, 0, 0);

    updateVision();
    field.setRobotPose(swerveDrive.getPose());
  }

  private static final double MAX_AMBIGUITY    = 0.3;
  private static final double MAX_TAG_DIST_M   = 5;
  private static final double MAX_YAW_RATE_DPS = 1000;

  private boolean isValidEstimate(PoseEstimate est) {
    if (est == null || est.tagCount == 0) return false;
    if (est.avgTagDist > MAX_TAG_DIST_M) return false;
    if (avgAmbiguity(est) > MAX_AMBIGUITY) return false;
    double yawRateDegS = swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond);
    if (Math.abs(yawRateDegS) > MAX_YAW_RATE_DPS) return false;
    return true;
  }

  private void updateVision() {
    PoseEstimate back  = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_BACK);
    PoseEstimate front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_FRONT);

    boolean backValid  = isValidEstimate(back);
    boolean frontValid = isValidEstimate(front);

    SmartDashboard.putBoolean("Back Valid",    backValid);
    SmartDashboard.putBoolean("Front Valid",   frontValid);
    SmartDashboard.putNumber("Back/TagCount",  back  != null ? back.tagCount  : -1);
    SmartDashboard.putNumber("Front/TagCount", front != null ? front.tagCount : -1);

    Rotation3d rot = swerveDrive.getGyro().getRotation3d();
    double pitch = Math.abs(Units.radiansToDegrees(rot.getY()));
    double roll  = Math.abs(Units.radiansToDegrees(rot.getX()));

    if (pitch > 8 || roll > 8) return;
    if (Math.abs(swerveDrive.getRobotVelocity().vxMetersPerSecond) > 4) return;

    if (backValid && frontValid) {

     if (front.avgTagDist > back.avgTagDist) {
        swerveDrive.addVisionMeasurement(back.pose, back.timestampSeconds,
            visionStdDevs(back.tagCount, back.avgTagDist));
        SmartDashboard.putString("Vision/Source", "Both (back weighted)");
      } else {
        swerveDrive.addVisionMeasurement(front.pose, front.timestampSeconds,
            visionStdDevs(front.tagCount, front.avgTagDist));
        SmartDashboard.putString("Vision/Source", "Both (front weighted)");
     }

    } else if (backValid) {
      swerveDrive.addVisionMeasurement(back.pose, back.timestampSeconds,
          visionStdDevs(back.tagCount, back.avgTagDist));
      SmartDashboard.putString("Vision/Source", "Back only");

    } else if (frontValid) {
      swerveDrive.addVisionMeasurement(front.pose, front.timestampSeconds,
          visionStdDevs(front.tagCount, front.avgTagDist));
      SmartDashboard.putString("Vision/Source", "Front only");

    } else {
      SmartDashboard.putString("Vision/Source", "None");
    }
  }

  private double avgAmbiguity(PoseEstimate est) {
    if (est.rawFiducials == null || est.rawFiducials.length == 0) return 1.0;
    double sum = 0;
    for (var f : est.rawFiducials) sum += f.ambiguity;
    return sum / est.rawFiducials.length;
  }

  private Matrix<N3, N1> visionStdDevs(int tagCount, double avgDistMeters) {
    return VecBuilder.fill(0.07, 0.07, Units.degreesToRadians(10));
  }
}
