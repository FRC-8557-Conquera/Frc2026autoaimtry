package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

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
  private AprilTagFieldLayout fieldLayout;
  SwerveDrive swerveDrive; 
  LimelightPoseEstimator limelightBackPoseEstimator;
  LimelightPoseEstimator limelightFrontPoseEstimator;
  Limelight limelightBack = new Limelight("limelight-back");
  Limelight limelightFront = new Limelight("limelight-front");
  boolean[] resultsPresent = {false, false};
  boolean[] validReading = {false, false};
  private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

  public SwerveSubsystem() {
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.Swerve.maxSpeed);
      fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    } catch (Exception e)
     {
      System.err.println("AprilTag Field Layout yüklenemedi! Simülasyon görüşü düzgün çalışmayabilir.");
      throw new RuntimeException(e);
    }
    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.LOW;
    field = new Field2d();
    SmartDashboard.putData("Field", field);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    swerveDrive.setAngularVelocityCompensation(true, true, 0.2); //
    swerveDrive.setModuleStateOptimization(true);
    swerveDrive.setAutoCenteringModules(false);
    swerveDrive.setHeadingCorrection(false);
    setupLimelight();
    setupPathPlanner();
    // Eğer kod simülasyonda çalışıyorsa robotu doğrudan sahanın içine (Örn: X=7.0, Y=4.0) yerleştir
    if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
        // İstediğin başlangıç koordinatını buradan ayarlayabilirsin
        swerveDrive.resetOdometry(new Pose2d(14.0, 4.0, Rotation2d.fromDegrees(180))); 
    }
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
  
  swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 0.022));
}

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    field.setRobotPose(swerveDrive.getPose());
    
    if (RobotBase.isReal()) {
        double yawVelocity = swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond);

        limelightBack.getSettings()
              .withRobotOrientation(new Orientation3d(swerveDrive.getGyro().getRotation3d(),
                                                      new AngularVelocity3d(DegreesPerSecond.of(0),
                                                                            DegreesPerSecond.of(0),
                                                                            DegreesPerSecond.of(yawVelocity))));
        limelightFront.getSettings()
              .withRobotOrientation(new Orientation3d(swerveDrive.getGyro().getRotation3d(),
                                                      new AngularVelocity3d(DegreesPerSecond.of(0),
                                                                            DegreesPerSecond.of(0),
                                                                            DegreesPerSecond.of(yawVelocity))));
        addVisionFromEstimator(limelightBackPoseEstimator, 0);
        addVisionFromEstimator(limelightFrontPoseEstimator, 1);
    } else {
        simulateVision();
    }
    
    SmartDashboard.putBoolean("Limelight Back Present", resultsPresent[0]);
    SmartDashboard.putBoolean("Limelight Back Valid", validReading[0]);
    SmartDashboard.putBoolean("Limelight Front Present", resultsPresent[1]);
    SmartDashboard.putBoolean("Limelight Front Valid", validReading[1]);
  }
  public Matrix<N3, N1> getVisionStdDevs(int tagCount, double avgDistance) {
    if (tagCount == 0) return VecBuilder.fill(99999, 99999, 99999);

    // Temel sapma değerleri (Tag dibindeyken çok güvenilir)
    double estStdDevsX = 0.05;
    double estStdDevsY = 0.05;
    double estHeadingStdDev = 0.01; // MegaTag2 için gyro zaten mükemmeldir, başlık sapmasını düşük tutarız

    // Uzaklığın karesini tag sayısına bölüyoruz
    double distanceMultiplier = Math.pow(avgDistance, 2) / tagCount;
    
    estStdDevsX += distanceMultiplier * 0.1;
    estStdDevsY += distanceMultiplier * 0.1;
    estHeadingStdDev += distanceMultiplier * 0.05;

    return VecBuilder.fill(estStdDevsX, estStdDevsY, estHeadingStdDev);
  }
  
  private void addVisionFromEstimator(LimelightPoseEstimator estimator, int index) {
    Optional<PoseEstimate> est = estimator.getPoseEstimate();

    if (est.isPresent()) {
      PoseEstimate poseEstimate = est.get();
      resultsPresent[index] = true;
      
      if (poseEstimate.tagCount > 0 && poseEstimate.getAvgTagAmbiguity() < 0.3) {
        validReading[index] = true;
        Pose2d visionPose = poseEstimate.pose.toPose2d();
        
        // Dinamik std-dev ile gerçekçilik ve stabilite (PoseEstimate içinde avgTagDist mevcuttur)
        double avgDist = poseEstimate.avgTagDist > 0 ? poseEstimate.avgTagDist : 3.0;
        Matrix<N3, N1> stdDevs = getVisionStdDevs(poseEstimate.tagCount, avgDist);
        
        swerveDrive.addVisionMeasurement(visionPose, poseEstimate.timestampSeconds, stdDevs);

        // AdvantageScope için görselleştirme datası
        SmartDashboard.putNumberArray("AdvantageScope/Vision_" + (index == 0 ? "Back" : "Front"), 
            new double[] {visionPose.getX(), visionPose.getY(), visionPose.getRotation().getRadians()});
            
      } else {
        validReading[index] = false;
      }
    } else {
      resultsPresent[index] = false;
      validReading[index] = false;
    }
  }
  public void simulateVision() {
    if (fieldLayout == null) return;

    // Robotun anlık pozisyonu
    Pose2d robotPose = swerveDrive.getPose();
    double timestamp = Timer.getFPGATimestamp();

    // 1. Kamera: ÖN LIMELIGHT (Robotun baktığı yön: 0 derece sapma)
    simulateSingleCamera(robotPose, timestamp, 1, 0.0, 40.0, "Front");

    // 2. Kamera: ARKA LIMELIGHT (Robotun tam tersi yön: 180 derece sapma)
    simulateSingleCamera(robotPose, timestamp, 0, 180.0, 40.0, "Back");
  }

  /**
   * Tek bir kameranın fiziksel görüşünü simüle eder.
   * @param index 1: Front, 0: Back (resultsPresent array'i için)
   * @param cameraYawOffset Kameranın robota göre montaj açısı (Ön: 0, Arka: 180)
   * @param fovHalfAngle Kameranın görüş açısının yarısı (Limelight 3 için yatayda ~40 derece)
   */
  private void simulateSingleCamera(Pose2d robotPose, double timestamp, int index, double cameraYawOffset, double fovHalfAngle, String name) {
    boolean seesTag = false;
    double minDistance = Double.MAX_VALUE;
    int visibleTags = 0;

    for (var tag : fieldLayout.getTags()) {
      Translation2d tagTranslation = tag.pose.toPose2d().getTranslation();
      double distance = robotPose.getTranslation().getDistance(tagTranslation);

      if (distance < 6.5) { 
        Rotation2d angleToTag = tagTranslation.minus(robotPose.getTranslation()).getAngle();
        Rotation2d cameraAngle = robotPose.getRotation().plus(Rotation2d.fromDegrees(cameraYawOffset));
        
        double relativeAngle = angleToTag.minus(cameraAngle).getDegrees();
        relativeAngle = edu.wpi.first.math.MathUtil.inputModulus(relativeAngle, -180, 180);

        if (Math.abs(relativeAngle) < fovHalfAngle) {
          seesTag = true;
          visibleTags++;
          if (distance < minDistance) minDistance = distance;
        }
      }
    }

    if (seesTag) {
      double noise = (minDistance * minDistance) * 0.005; 
      
      Pose2d noisyPose = new Pose2d(
          robotPose.getX() + (Math.random() - 0.5) * noise,
          robotPose.getY() + (Math.random() - 0.5) * noise,
          robotPose.getRotation() // Açı her zaman robotPose ile BİREBİR aynıdır (MegaTag2)
      );

      var stdDevs = getVisionStdDevs(visibleTags, minDistance);
      swerveDrive.addVisionMeasurement(noisyPose, timestamp, stdDevs);
      
      // KAMERA GÖRÜYOR: Veriyi yolla
      SmartDashboard.putNumberArray("AdvantageScope/Vision_" + name + "_Sim", 
          new double[] {noisyPose.getX(), noisyPose.getY(), noisyPose.getRotation().getRadians()});
          
      resultsPresent[index] = true;
      validReading[index] = true;
    } else {
      // ÇÖZÜM: KAMERA KÖR OLDU: Sahneden dondurulmuş hayaleti SİL!
      SmartDashboard.putNumberArray("AdvantageScope/Vision_" + name + "_Sim", new double[] {});
      
      resultsPresent[index] = false;
      validReading[index] = false;
    }
    
  }
  // Eğer kod simülasyonda çalışıyorsa robotu doğrudan sahanın içine (Örn: X=7.0, Y=4.0) yerleştir
  /**
   * Maçın başında veya otonom başlarken robotun konumunu ve GYRO açısını
   * MegaTag 1 kullanarak mutlak doğru şekilde sıfırlar.
   */
 /**
   * Maçın başında veya otonom başlarken robotun konumunu ve GYRO açısını
   * MegaTag 1 kullanarak mutlak doğru şekilde sıfırlar.
   */
  public boolean seedOdometryWithMegaTag1() {
      // ÇÖZÜM 2: limelight.networktables yerine doğrudan LimelightHelpers.PoseEstimate kullanıyoruz
      LimelightHelpers.PoseEstimate mt1Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");
      
      // Eğer arka kamera tag görmüyorsa öne bakalım
      if (mt1Pose == null || mt1Pose.tagCount == 0) {
          mt1Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
      }

      // Eğer herhangi bir kamera güvenilir bir MT1 verisi yakaladıysa
      if (mt1Pose != null && mt1Pose.tagCount > 0 && mt1Pose.avgTagDist < 4.0) {
          // ÇÖZÜM 1: Metot artık static olmadığı için swerveDrive'ı sorunsuz görebilir
          swerveDrive.resetOdometry(mt1Pose.pose);
          System.out.println("BAŞARILI: Odometri ve Gyro MegaTag1 ile sıfırlandı!");
          return true;
      }
      
      System.out.println("HATA: Kameralar Tag Görmüyor, Odometri Sıfırlanamadı!");
      return false;
  }
}