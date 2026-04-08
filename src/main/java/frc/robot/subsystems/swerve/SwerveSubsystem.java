package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



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
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;


public class SwerveSubsystem extends SubsystemBase {

  Field2d field;
  SwerveDrive swerveDrive; 
  LimelightPoseEstimator limelightBackPoseEstimator;
  LimelightPoseEstimator limelightFrontPoseEstimator;
  Limelight limelightBack = new Limelight("limelight-front");
  Limelight limelightFront = new Limelight("limelight-back");
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

    public void lock() {
    swerveDrive.lockPose();
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

  limelightBackPoseEstimator = limelightBack.createPoseEstimator(EstimationMode.MEGATAG1);
  limelightFrontPoseEstimator = limelightFront.createPoseEstimator(EstimationMode.MEGATAG1);
}

@Override
public void periodic() {
    addVisionFromEstimator(limelightBackPoseEstimator);
    addVisionFromEstimator(limelightFrontPoseEstimator);
    field.setRobotPose(swerveDrive.getPose());
  }

    private void addVisionFromEstimator(LimelightPoseEstimator estimator) {
    Optional<PoseEstimate> est = estimator.getPoseEstimate();
    if (est.isEmpty()) return;

    PoseEstimate poseEstimate = est.get();
    if (poseEstimate.tagCount == 0) return;

    double avgDist = poseEstimate.avgTagDist;

    // ====================================================================
    // 1. SENARYO: ÇOKLU TAG (MULTI-TAG) KUSURSUZLUĞU
    // Ekranda 2 veya daha fazla tag varsa, kamera 3D uzayı kusursuz anlar.
    // ====================================================================
    if (poseEstimate.tagCount > 1) {
        // Uzaklığın karesi (Quadratic) ile büyüyen çok hafif bir hata payı.
        // Başlangıç hatası (0.1) çok düşüktür çünkü çoklu tag aşırı güvenilirdir.
        double xyStdDev = 0.1 + (Math.pow(avgDist, 2) * 0.02);
        double rotStdDev = 0.1 + (Math.pow(avgDist, 2) * 0.05);

        // Veriyi doğrudan StdDev matrisi ile şasiye gömüyoruz!
        swerveDrive.addVisionMeasurement(
            poseEstimate.pose.toPose2d(),
            poseEstimate.timestampSeconds,
            VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
        );
        return; // İşlem bitti, alt satırlara inme
    }

    // ====================================================================
    // 2. SENARYO: TEK TAG (SINGLE-TAG) HAYATTA KALMA MODU
    // Ekranda 1 tag varsa, kamera açıyı uydurur ve mesafe ölçümü zayıflar.
    // ====================================================================
    if (poseEstimate.tagCount == 1) {
        // Belirsizlik (Ambiguity) 0.2'den büyükse kamera halüsinasyon görüyordur, direkt reddet!
        if (poseEstimate.getAvgTagAmbiguity() > 0.2) return;

        // MT1 modunda tek tag ile 4.5 metreden sonrası matematiksel olarak çöp veridir.
        if (avgDist > 4.5) return;

        // Mesafe arttıkça X/Y hatasını çok agresif (karesel) olarak artırıyoruz.
        // Başlangıç hatası (0.5) çoklu tag'e göre yüksektir.
        double xyStdDev = 0.5 + (Math.pow(avgDist, 2) * 0.15);

        // ÖLÜMCÜL KORUMA: Tek tag MT1 ASLA doğru robot açısı veremez!
        // Değeri 9999.0 yaparak Kalman Filtresine "Vizyon açısını SİL, donanım Gyro'sunu kullan" diyoruz.
        double rotStdDev = 9999.0;

        swerveDrive.addVisionMeasurement(
            poseEstimate.pose.toPose2d(),
            poseEstimate.timestampSeconds,
            VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
        );
    }
  }
}