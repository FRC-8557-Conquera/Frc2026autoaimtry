package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;


public class SwerveSubsystem extends SubsystemBase {

  Field2d field;
  SwerveDrive swerveDrive;
  LimelightPoseEstimator limelightBackPoseEstimator;
  LimelightPoseEstimator limelightFrontPoseEstimator;
  Limelight limelightBack = new Limelight("limelight-back");
  Limelight limelightFront = new Limelight("limelight-front");
  private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

  public SwerveSubsystem() {
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.Swerve.maxSpeed);
    } catch (Exception e)
     {
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
    setupLimelight();
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
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(5, 0, 0),
              new PIDConstants(4, 0, 0)),
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
      double x = MathUtil.applyDeadband(translationX.getAsDouble(), 0.1);
      double y = MathUtil.applyDeadband(translationY.getAsDouble(), 0.1);
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

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
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
  
  public void setupLimelight(){

  limelightBack.getSettings()
      .withPipelineIndex(0)
      .withLimelightLEDMode(LEDMode.PipelineControl)
      .save();

  limelightFront.getSettings()
      .withPipelineIndex(0)
      .withLimelightLEDMode(LEDMode.PipelineControl)
      .save();

  limelightBackPoseEstimator = limelightBack.createPoseEstimator(EstimationMode.MEGATAG2);
  limelightFrontPoseEstimator = limelightFront.createPoseEstimator(EstimationMode.MEGATAG2);
  
  // Per-measurement stddevs are computed dynamically in updateVisionCombined()
}

@Override
public void periodic() {
    double yawRateDegS = swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond);

    Orientation3d orientation = new Orientation3d(
        swerveDrive.getGyro().getRotation3d(),
        new AngularVelocity3d(
            DegreesPerSecond.of(0),
            DegreesPerSecond.of(0),
            DegreesPerSecond.of(yawRateDegS)));

    limelightBack.getSettings().withRobotOrientation(orientation).save();
    limelightFront.getSettings().withRobotOrientation(orientation).save();

    updateVisionCombined();
    field.setRobotPose(swerveDrive.getPose());
  }

  private void updateVisionCombined() {
    Optional<PoseEstimate> backOpt  = limelightBackPoseEstimator.getPoseEstimate();
    Optional<PoseEstimate> frontOpt = limelightFrontPoseEstimator.getPoseEstimate();

    boolean backValid  = backOpt.isPresent()
        && backOpt.get().tagCount > 0
        && backOpt.get().getAvgTagAmbiguity() < 0.3;
    boolean frontValid = frontOpt.isPresent()
        && frontOpt.get().tagCount > 0
        && frontOpt.get().getAvgTagAmbiguity() < 0.3;

    if (backValid && frontValid) {
      PoseEstimate back  = backOpt.get();
      PoseEstimate front = frontOpt.get();

      // Weight = (1 - ambiguity): lower ambiguity → higher trust
      double wB = 1.0 - back.getAvgTagAmbiguity();
      double wF = 1.0 - front.getAvgTagAmbiguity();
      double total = wB + wF;
      wB /= total;
      wF /= total;

      Pose2d bPose = back.pose.toPose2d();
      Pose2d fPose = front.pose.toPose2d();

      double x = wB * bPose.getX() + wF * fPose.getX();
      double y = wB * bPose.getY() + wF * fPose.getY();

      // Average rotation via unit-vector interpolation to handle wrap-around correctly
      double bAngle = bPose.getRotation().getRadians();
      double fAngle = fPose.getRotation().getRadians();
      Rotation2d avgRot = new Rotation2d(
          wB * Math.cos(bAngle) + wF * Math.cos(fAngle),
          wB * Math.sin(bAngle) + wF * Math.sin(fAngle));

      double timestamp = wB * back.timestampSeconds + wF * front.timestampSeconds;

      double avgDist = wB * back.avgTagDist + wF * front.avgTagDist;
      int totalTags  = back.tagCount + front.tagCount;
      swerveDrive.addVisionMeasurement(new Pose2d(x, y, avgRot), timestamp,
          visionStdDevs(totalTags, avgDist));

      SmartDashboard.putString("Vision/Source", "Both (weighted)");
      SmartDashboard.putNumber("Vision/WeightBack",  wB);
      SmartDashboard.putNumber("Vision/WeightFront", wF);

    } else if (backValid) {
      PoseEstimate back = backOpt.get();
      swerveDrive.addVisionMeasurement(back.pose.toPose2d(), back.timestampSeconds,
          visionStdDevs(back.tagCount, back.avgTagDist));
      SmartDashboard.putString("Vision/Source", "Back only");

    } else if (frontValid) {
      PoseEstimate front = frontOpt.get();
      swerveDrive.addVisionMeasurement(front.pose.toPose2d(), front.timestampSeconds,
          visionStdDevs(front.tagCount, front.avgTagDist));
      SmartDashboard.putString("Vision/Source", "Front only");

    } else {
      SmartDashboard.putString("Vision/Source", "None");
    }
  }

  private Matrix<N3, N1> visionStdDevs(int tagCount, double avgDistMeters) {
    double distFactor = 1.0 + (avgDistMeters * avgDistMeters) / 30.0;
    double xy = (tagCount >= 2 ? 0.5 : 4.0) * distFactor;
    return VecBuilder.fill(xy, xy, 9999999);
  }
}