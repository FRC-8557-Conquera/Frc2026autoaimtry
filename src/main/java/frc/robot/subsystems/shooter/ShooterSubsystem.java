package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.util.GeometryUtil;
import com.thethriftybot.wrappers.RobotStateWrapper;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.fieldConstants;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Turret;

public class ShooterSubsystem extends SubsystemBase {

    private final SwerveSubsystem swerve;
    private ShuffleboardTab tab;
    public ShotIntent intent = ShotIntent.OFF;
    public TurretSubsystem turret = new TurretSubsystem();
    public FlywheelSubsystem flywheel = new FlywheelSubsystem();
    public HoodSubsystem hood = new HoodSubsystem();
    public GenericEntry entry;
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
    private Pose2d getHubPose() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        return fieldConstants.RED_HUB_POSE;
    }
    return fieldConstants.BLUE_HUB_POSE;
    }

    public ShooterSubsystem(SwerveSubsystem swerve) {
        this.swerve = swerve;
        buildLookupTables();
        tab = Shuffleboard.getTab("Shooter");
        entry = tab.addPersistent("Flywheel Speed (RPS)", 40)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();
        // Örnek Kalibrasyon Verileri (Mesafe Metre -> Hedef Değer)
        // TODO: Gerçek robotla veya simülasyondaki atış çizgisine bakarak bu değerleri ayarlayacağız
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance", getHubDistance().in(Meters));
        SmartDashboard.putNumber("Slider", entry.getDouble(10));
        
        // Atış yörüngesi simülasyonunu sürekli çalıştır
        updateTrajectoryVisualization();
    }

    public Command debugShoot() {
        double speedRPS = entry.getDouble(40);
        return flywheel.setVelocity(() -> RotationsPerSecond.of(speedRPS));
    }

    public void setIntent(ShotIntent intent) {
        this.intent = intent;
    }

    public ShotIntent getIntent() {
        return intent;
    }

    private void buildLookupTables() {
        //tooning kesin yapılcak************************************
        // HOOD: Açıyı 40 dereceye (maksa) yakın tutarak topun dik inmesini sağlıyoruz.
        hoodMap.put(1.5, 40.0); // En yakın mesafe, en dik açı
        hoodMap.put(2.5, 40.0); 
        hoodMap.put(3.5, 38.0);
        hoodMap.put(4.5, 35.0);
        hoodMap.put(5.5, 33.0);

        // FLYWHEEL: Hızları biraz daha kısarak menzilin potayı aşmasını engelliyoruz.
        flywheelMap.put(1.5, 20.0); 
        flywheelMap.put(2.5, 25.5);
        flywheelMap.put(3.5, 29.0);
        flywheelMap.put(4.5, 30.0);
        flywheelMap.put(5.5, 33.0);
    }


    public Angle getTurretSetpoint() {
        Pose2d robotPose = swerve.getPose();
        if (intent == ShotIntent.HUB || intent == ShotIntent.SOTM) {
            Translation2d turretOffset = new Translation2d(Turret.turretDist,0); 
            Translation2d turretFieldPosition = robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

            // KRİTİK AYRIM: SOTM açıksa Sanal Hedefe, HUB açıksa Gerçek Pota'ya bak
            Translation2d targetTranslation = (intent == ShotIntent.SOTM) ? getVirtualTarget() : getHubPose().getTranslation();

            Translation2d toHub = targetTranslation.minus(turretFieldPosition);
            Rotation2d fieldAngle = toHub.getAngle();
            Rotation2d turretRelative = fieldAngle.minus(robotPose.getRotation()); 
            double setpointDeg = turretRelative.getDegrees();
            double wrapped = MathUtil.inputModulus(setpointDeg, -180, 180);
            return Degrees.of(wrapped);
        }
        if (intent == ShotIntent.DUMP) {
            return Degrees.of(Rotation2d.k180deg.minus(robotPose.getRotation()).getDegrees());
    }
        return Degrees.of(0);
    }
    
    public double getMagnitude(Transform2d pose) {
        return Math.sqrt(pose.getX() * pose.getX() + pose.getY() * pose.getY());
    }
    public Distance getHubDistance() {
        Pose2d robotPose = swerve.getPose();
        // Mesafe (RPS ve Hood haritaları için) hesaplanırken de bu ayrıma dikkat ediyoruz:
        Translation2d targetTranslation = (intent == ShotIntent.SOTM) ? getVirtualTarget() : getHubPose().getTranslation();
        return Meters.of(robotPose.getTranslation().getDistance(targetTranslation));
    }

    public AngularVelocity getFlywheelSetpoint() {
        // ÇÖZÜM: Hem HUB hem de SOTM durumunda bu haritalar okunmalı!
        if (intent == ShotIntent.HUB || intent == ShotIntent.SOTM) {
            // getHubDistance() metodunu zaten sanal hedefe göre ayarlamıştık, direkt onu kullanıyoruz:
            double distance = getHubDistance().in(Meters);
            return RotationsPerSecond.of(flywheelMap.get(distance));
        }

        if (intent == ShotIntent.DUMP) {
            return RotationsPerSecond.of(30);
        }

        return RotationsPerSecond.of(0);
    }

    public Angle getHoodSetpoint() {
        // ÇÖZÜM: Hem HUB hem de SOTM durumunda bu haritalar okunmalı!
        if (intent == ShotIntent.HUB || intent == ShotIntent.SOTM) {
            double distance = getHubDistance().in(Meters);
            
            // Aynı zamanda daha önce konuştuğumuz 20-40 derece mekanik güvenlik sınırını (Clamp) da ekliyoruz
            double rawAngle = hoodMap.get(distance);
            double safeAngle = MathUtil.clamp(rawAngle, 20.0, 40.0);
            
            return Degrees.of(safeAngle);
        }

        if (intent == ShotIntent.DUMP) {
            return Degrees.of(15);
        }

        return Degrees.of(0);
    }

    // Mesafe parametresi alan eski hood metodun da varsa onu da düzeltelim:
    public Angle getHoodSetpoint(double distance) {
        if(intent == ShotIntent.HUB || intent == ShotIntent.SOTM) {
            double safeAngle = MathUtil.clamp(hoodMap.get(distance), 20.0, 40.0);
            return Degrees.of(safeAngle);
        } else return getHoodSetpoint();
    }

   public LinearVelocity getBaseExitVelocity(double distance) {
    double rps = flywheelMap.get(distance) + Shooter.flywheelOffsetRPS;
    return MetersPerSecond.of(rps);
}
    
    public enum ShotIntent {
        HUB,
        SOTM, // HAREKETLİ ATIŞ İÇİN YENİ DURUM
        DUMP,
        OFF
    }
    public void updateTrajectoryVisualization() {
        Pose2d robotPose = swerve.getPose(); 
        
        double turretYaw = getTurretSetpoint().in(edu.wpi.first.units.Units.Radians);        
        double mechanicalOffset = Math.toRadians(20.0); 
        double hoodPitch = getHoodSetpoint().in(edu.wpi.first.units.Units.Radians) + mechanicalOffset;        
        double flywheelRPS = getFlywheelSetpoint().in(edu.wpi.first.units.Units.RotationsPerSecond);
        
        double radius = 0.0508; 
        double v0 = (flywheelRPS * 2 * Math.PI * radius) * 0.8; 

        double shooterHeightMeters = 0.45; 
        Translation3d startPos = new Translation3d(robotPose.getX(), robotPose.getY(), shooterHeightMeters);

        double globalYaw = robotPose.getRotation().getRadians() + turretYaw;
        double v_xy = v0 * Math.cos(hoodPitch); 

        // DÜZELTME BURADA: Robotun hızı sadece 12. buton (SOTM) aktifse mermiye eklenir!
        double vx, vy;
        if (intent == ShotIntent.SOTM) {
            var robotSpeeds = swerve.getChassisSpeeds();
            Translation2d fieldVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                                              .rotateBy(robotPose.getRotation());
            vx = (v_xy * Math.cos(globalYaw)) + fieldVelocity.getX(); 
            vy = (v_xy * Math.sin(globalYaw)) + fieldVelocity.getY(); 
        } else {
            // 11. buton (HUB) veya diğer durumlarda robot hızı sarı çizgiye EKLENMEZ
            vx = v_xy * Math.cos(globalYaw); 
            vy = v_xy * Math.sin(globalYaw); 
        }
        
        double vz = v0 * Math.sin(hoodPitch);   

        java.util.ArrayList<Pose3d> trajectory = new java.util.ArrayList<>();
        double t = 0;
        double dt = 0.05; 
        double z = startPos.getZ();

        while (z > 0 && t < 3.0) {
            double x = startPos.getX() + (vx * t);
            double y = startPos.getY() + (vy * t);
            z = startPos.getZ() + (vz * t) - (0.5 * 9.81 * Math.pow(t, 2));

            if (z > 0) {
                trajectory.add(new Pose3d(x, y, z, new edu.wpi.first.math.geometry.Rotation3d()));
            }
            t += dt;
        }

        double[] trajectoryData = new double[trajectory.size() * 3];
        for (int i = 0; i < trajectory.size(); i++) {
            Pose3d p = trajectory.get(i);
            trajectoryData[i * 3]     = p.getX(); 
            trajectoryData[i * 3 + 1] = p.getY(); 
            trajectoryData[i * 3 + 2] = p.getZ(); 
        }

        SmartDashboard.putNumberArray("AdvantageScope/Shooter_Trajectory", trajectoryData);
    }
    // Robotun hızını hesaba katarak hedeflenecek sanal noktayı bulur
    public Translation2d getVirtualTarget() {
        Pose2d robotPose = swerve.getPose();
        Translation2d actualHub = getHubPose().getTranslation();
        double distanceToActual = robotPose.getTranslation().getDistance(actualHub);

        // 1. ROBOT HIZINI SAHA (FIELD) HIZINA ÇEVİRME
        var robotSpeeds = swerve.getChassisSpeeds();
        Translation2d fieldVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                                          .rotateBy(robotPose.getRotation());
        double fieldVx = fieldVelocity.getX();
        double fieldVy = fieldVelocity.getY();

        // 2. MERMİNİN GERÇEK YATAY HIZINI BULMA 
        // DÜZELTME: Sonsuz döngüye girmemek için "getHoodSetpoint()" yerine haritadan (Map) direkt okuyoruz!
        double currentRPS = flywheelMap.get(distanceToActual);
        double currentHoodRaw = hoodMap.get(distanceToActual);
        double hoodPitch = Math.toRadians(currentHoodRaw) + Math.toRadians(20.0); 
        
        double radius = 0.0508;
        double v0 = (currentRPS * 2 * Math.PI * radius) * 0.8;
        double v_xy = v0 * Math.cos(hoodPitch);

        if (v_xy < 0.1) v_xy = 0.1; // Sıfıra bölünme koruması

        // 3. GERÇEK UÇUŞ SÜRESİ + GECİKME TELAFİSİ
        double timeOfFlight = (distanceToActual / v_xy) + 0.04;

        // 4. SANAL HEDEF HESAPLAMASI
        double virtualX = actualHub.getX() - (fieldVx * timeOfFlight);
        double virtualY = actualHub.getY() - (fieldVy * timeOfFlight);

        return new Translation2d(virtualX, virtualY);
    }
}
