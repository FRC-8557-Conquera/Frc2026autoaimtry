package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFields;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.io.File;
import java.lang.reflect.Field;
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
import limelight.networktables.LimelightResults;


public class SwerveSubsystem extends SubsystemBase {

  Field2d field;
  SwerveDrive swerveDrive; 
  LimelightPoseEstimator limelightBackPoseEstimator;
  Limelight limelightBack = new Limelight("limelight-back");
  private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      boolean enableFeedforward = true;
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.Swerve.maxSpeed);
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

    field = new Field2d();
    SmartDashboard.putData("Field", field);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    swerveDrive.setAngularVelocityCompensation(true, true, 0.2);
    swerveDrive.setModuleStateOptimization(true);
    swerveDrive.setAutoCenteringModules(false);
    swerveDrive.setHeadingCorrection(true);
    setupLimelight();
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public Rotation2d calculateHubAngle() {
    Translation2d hub = new Translation2d(12.742, 4.238); 
    Translation2d pos = getPose().getTranslation();

    double angle = Math.asin(pos.minus(hub).dot(new Translation2d(0,1)) / pos.getDistance(hub));
    angle += Math.PI/2;
    return new Rotation2d(angle);
  }

  public void turnToAngle(Supplier<Rotation2d> angle) {
    while(Math.abs(getHeading().minus(angle.get()).getRadians()) > 0.1) {
      swerveDrive.drive(new ChassisSpeeds(0,0, 
      Math.signum(-getHeading().minus(angle.get()).getRadians())));
    }
  }
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.Swerve.maxSpeed);
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
    double diff = getHeading().minus(calculateHubAngle()).getRadians();
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
             .withLimelightLEDMode(LEDMode.PipelineControl).save();
      limelightBackPoseEstimator = limelightBack.createPoseEstimator(EstimationMode.MEGATAG2);
  }

private int     outofAreaReading = 0;
private boolean initialReading = false;
@Override
public void periodic() {
  
    double yawVelocity = swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond);

        limelightBack.getSettings()
             .withRobotOrientation(new Orientation3d(swerveDrive.getGyro().getRotation3d(),
                                                     new AngularVelocity3d(DegreesPerSecond.of(0),
                                                                           DegreesPerSecond.of(0),
                                                                           DegreesPerSecond.of(yawVelocity))));
            
    Optional<PoseEstimate> est1 = limelightBackPoseEstimator.getPoseEstimate();
    
    // Optional<LimelightResults> results = limelightBack.getLatestResults();
        if (est1.isPresent()) // & poseEstimates.isPresent())
    { 
      
      PoseEstimate     poseEstimate = est1.get();
      SmartDashboard.putNumber("Avg Tag Ambiguity", poseEstimate.getAvgTagAmbiguity());
      SmartDashboard.putNumber("Min Tag Ambiguity", poseEstimate.getMinTagAmbiguity());
      SmartDashboard.putNumber("Max Tag Ambiguity", poseEstimate.getMaxTagAmbiguity());
      SmartDashboard.putNumber("Avg Distance", poseEstimate.avgTagDist);
      SmartDashboard.putNumber("Avg Tag Area", poseEstimate.avgTagArea);
      SmartDashboard.putNumber("Odom Pose/x", swerveDrive.getPose().getX());
      SmartDashboard.putNumber("Odom Pose/y", swerveDrive.getPose().getY());
      SmartDashboard.putNumber("Odom Pose/degrees", swerveDrive.getPose().getRotation().getDegrees());
      SmartDashboard.putNumber("Limelight Pose/x", poseEstimate.pose.getX());
      SmartDashboard.putNumber("Limelight Pose/y", poseEstimate.pose.getY());
      SmartDashboard.putNumber("Limelight Pose/degrees", poseEstimate.pose.toPose2d().getRotation().getDegrees());
      
        // Pose2d estimatorPose = poseEstimate.pose.toPose2d();
        Pose2d usefulPose     = poseEstimate.pose.toPose2d();
        double distanceToPose = usefulPose.getTranslation().getDistance(swerveDrive.getPose().getTranslation());
        if (poseEstimate.tagCount > 0 && (distanceToPose < 0.5 || (outofAreaReading > 10) || (outofAreaReading > 10 && !initialReading)))
        {
          if (!initialReading)
          {
            initialReading = true;
          }
          outofAreaReading = 0;
          // System.out.println(usefulPose.toString());
          swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 0.022));
          // System.out.println(result.timestamp_LIMELIGHT_publish);
          // System.out.println(result.timestamp_RIOFPGA_capture);
          swerveDrive.addVisionMeasurement(usefulPose, Timer.getTimestamp());
        } else
        {
          outofAreaReading += 1;
        }
        
       field.setRobotPose(swerveDrive.getPose());

       } 
      }
      


  }

