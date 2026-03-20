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
        // HOOD: Açıyı 40 dereceye (maksa) yakın tutarak topun dik inmesini sağlıyoruz.
        hoodMap.put(1.5, 40.0); // En yakın mesafe, en dik açı
        hoodMap.put(2.5, 40.0); 
        hoodMap.put(3.5, 38.0);
        hoodMap.put(4.5, 35.0);
        hoodMap.put(5.5, 33.0);

        // FLYWHEEL: Hızları biraz daha kısarak menzilin potayı aşmasını engelliyoruz.
        flywheelMap.put(1.5, 20.0); 
        flywheelMap.put(2.5, 25.0);
        flywheelMap.put(3.5, 29.0);
        flywheelMap.put(4.5, 30.0);
        flywheelMap.put(5.5, 33.0);
    }


    public Angle getTurretSetpoint() {
        Pose2d robotPose = swerve.getPose();
        if (intent == ShotIntent.HUB) {
        Translation2d turretOffset =
            new Translation2d(Turret.turretDist,0); 

        Translation2d turretFieldPosition =
            robotPose.getTranslation().plus(
                turretOffset.rotateBy(robotPose.getRotation())
            );

        Translation2d toHub =
            getHubPose().getTranslation()
                .minus(turretFieldPosition);

        Rotation2d fieldAngle = toHub.getAngle();

        Rotation2d turretRelative =
            fieldAngle
                .minus(robotPose.getRotation()); 
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
        return Meters.of(getMagnitude(robotPose.minus(getHubPose())));
    }

    public AngularVelocity getFlywheelSetpoint() {
        if (intent == ShotIntent.HUB) {
            double distance =
                swerve.getPose()
                    .getTranslation()
                    .getDistance(getHubPose().getTranslation());

            return RotationsPerSecond.of(flywheelMap.get(distance));
        }

        if (intent == ShotIntent.DUMP) {
            return RotationsPerSecond.of(30);
        }

        return RotationsPerSecond.of(0);
    }

    public Angle getHoodSetpoint() {
        if (intent == ShotIntent.HUB) {
            double distance =
                swerve.getPose()
                    .getTranslation()
                    .getDistance(getHubPose().getTranslation());

            double rawAngle = hoodMap.get(distance);
            double safeAngle = MathUtil.clamp(rawAngle, 20.0, 40.0);
            return Degrees.of(safeAngle);
        }

        if (intent == ShotIntent.DUMP) {
            return Degrees.of(15);
        }

        return Degrees.of(0);
    }

    // distance - latency comp distance
    public Angle getHoodSetpoint(double distance) {
        if(intent == ShotIntent.HUB) {
            return Degrees.of(hoodMap.get(distance));
        } else return getHoodSetpoint();
    }

   public LinearVelocity getBaseExitVelocity(double distance) {
    double rps = flywheelMap.get(distance) + Shooter.flywheelOffsetRPS;
    return MetersPerSecond.of(rps);
}
    
public enum ShotIntent {
    HUB,
    DUMP,
    OFF
}
public void updateTrajectoryVisualization() {
        Pose2d robotPose = swerve.getPose(); 
        
        // Taret, Hood ve Flywheel'in O ANKİ gerçek değerlerini alıyoruz
        double turretYaw = getTurretSetpoint().in(Radians);        
        // Hood'un mekanik offset'i. Kendi robotuna göre bu 35.0 değerini değiştir!
        double mechanicalOffset = Math.toRadians(20.0); 
        double hoodPitch = getHoodSetpoint().in(Radians) + mechanicalOffset;        
        double flywheelRPS = getFlywheelSetpoint().in(RotationsPerSecond);
        // Çıkış hızı (m/s). 4 inç çap = 0.1016 m. Yarıçap = 0.0508 m.
        double radius = 0.0508; 
        double v0 = (flywheelRPS * 2 * Math.PI * radius) * 0.8; // %80 verim varsayımı

        // Atıcının sahadaki 3D başlangıç noktası
        double shooterHeightMeters = 0.45; // Robotunun atıcı yüksekliği
        Translation3d startPos = new Translation3d(robotPose.getX(), robotPose.getY(), shooterHeightMeters);

        // Fırlatma açısının sahadaki gerçek (Global) yönü
        double globalYaw = robotPose.getRotation().getRadians() + turretYaw;

        // Hız vektörlerini X, Y ve Z eksenlerine böl
        double v_xy = v0 * Math.cos(hoodPitch); 
        double vx = v_xy * Math.cos(globalYaw); 
        double vy = v_xy * Math.sin(globalYaw); 
        double vz = v0 * Math.sin(hoodPitch);   

        // Yörüngeyi oluştur (Maksimum 3 saniye uçuş simülasyonu)
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

        // Translation Array formatı (Sadece X, Y, Z)
        double[] trajectoryData = new double[trajectory.size() * 3];
        for (int i = 0; i < trajectory.size(); i++) {
            Pose3d p = trajectory.get(i);
            trajectoryData[i * 3]     = p.getX(); 
            trajectoryData[i * 3 + 1] = p.getY(); 
            trajectoryData[i * 3 + 2] = p.getZ(); 
        }

        SmartDashboard.putNumberArray("AdvantageScope/Shooter_Trajectory", trajectoryData);
    }
}
