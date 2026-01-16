package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
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
import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class SwerveSubsystem extends SubsystemBase {


  private Field2d field;
  public SwerveDrive swerveDrive;
  private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

  public SwerveSubsystem() {
    RobotConfig config;
    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
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
    // napıyonuz siz kod yazmayın kapatın robotu
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    swerveDrive.setModuleStateOptimization(true);
    swerveDrive.setAutoCenteringModules(false);
    swerveDrive.setHeadingCorrection(true);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
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
      double x = MathUtil.applyDeadband(translationX.getAsDouble(), 0.05);
      double y = MathUtil.applyDeadband(translationY.getAsDouble(), 0.05);
      double omega = MathUtil.applyDeadband(angularRotationX.getAsDouble(), 0.05);

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

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
  }

}