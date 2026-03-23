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
    // Havada uçan topların anlık fiziğini tutacak mini bir sınıf
    private static class SimulatedFuel {
        double x, y, z;
        double vx, vy, vz;
        double timeAlive = 0;

        public SimulatedFuel(double x, double y, double z, double vx, double vy, double vz) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.vx = vx;
            this.vy = vy;
            this.vz = vz;
        }
    }

    // Aktif olarak havada uçan tüm topların listesi
    private java.util.ArrayList<SimulatedFuel> flyingFuels = new java.util.ArrayList<>();
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
        // HOOD: Topun ufuk çizgisiyle yaptığı gerçek çıkış açısı (70 - 50 arası)
        // 70 derece = Yakın mesafeden çok güvenli ve isabetli aşırtma (Lob Shot)
        // 50 derece = Uzak mesafeden stabil, orta kavisli atış
        hoodMap.put(1.5, 70.0); 
        hoodMap.put(2.5, 65.0); 
        hoodMap.put(3.5, 60.0); 
        hoodMap.put(4.5, 55.0); 
        hoodMap.put(5.5, 50.0);

        // FLYWHEEL: 80 dereceye göre daha yatay attığımız için ileri doğru gidiş artacak.
        // Bu yüzden hızları 80 haritasına göre biraz azalttık, mükemmel dengeyi bulduk.
        flywheelMap.put(1.5, 23.0); 
        flywheelMap.put(2.5, 25.5);
        flywheelMap.put(3.5, 27.5);
        flywheelMap.put(4.5, 30.0);
        flywheelMap.put(5.5, 32.0);
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
       // getTurretSetpoint() içindeki DUMP kısmı:
        if (intent == ShotIntent.DUMP) {
            Translation2d turretOffset = new Translation2d(Turret.turretDist, 0); 
            Translation2d turretFieldPosition = robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

            // Fizik kurallarıyla hesaplanan o kusursuz sanal hedefi çağırıyoruz
            Translation2d virtualDropZone = getDumpVirtualTarget();

            // Tareti sanal noktaya kilitliyoruz
            Translation2d toTarget = virtualDropZone.minus(turretFieldPosition);
            Rotation2d turretRelative = toTarget.getAngle().minus(robotPose.getRotation()); 
            
            return Degrees.of(MathUtil.inputModulus(turretRelative.getDegrees(), -180, 180));
        }
        return Degrees.of(0);
    
    }
    // Taretin, seçilen güvenli Dump noktasına (Üst veya Alt) olan anlık uzaklığını ölçer
    public edu.wpi.first.units.measure.Distance getDumpDistance() {
        Pose2d robotPose = swerve.getPose();
        Translation2d turretOffset = new Translation2d(Turret.turretDist, 0).rotateBy(robotPose.getRotation());
        Translation2d turretPos = robotPose.getTranslation().plus(turretOffset);
        
        // Üst veya Alt köşeyi hedeflendiğini zaten bu metot biliyor
        Translation2d dumpTarget = getDumpTargetPosition(); 

        double distMeters = turretPos.getDistance(dumpTarget);
        return edu.wpi.first.units.Units.Meters.of(distMeters);
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
        if (intent == ShotIntent.HUB || intent == ShotIntent.SOTM) {
            double distance = getHubDistance().in(edu.wpi.first.units.Units.Meters);
            return RotationsPerSecond.of(flywheelMap.get(distance));
        }

        if (intent == ShotIntent.DUMP) {
            // DÜZELTME: Uzaklığı ölçüp tıpkı HUB gibi mermi hızını haritadan ayarlıyor.
            double distance = getDumpDistance().in(edu.wpi.first.units.Units.Meters);
            return RotationsPerSecond.of(flywheelMap.get(distance));
        }
        
        return RotationsPerSecond.of(0);
    }
 
    public Angle getHoodSetpoint() {
        if (intent == ShotIntent.HUB || intent == ShotIntent.SOTM) {
            double distance = getHubDistance().in(edu.wpi.first.units.Units.Meters);
            double rawAngle = hoodMap.get(distance);
            return Degrees.of(MathUtil.clamp(rawAngle, 50.0, 70.0));
        }

        if (intent == ShotIntent.DUMP) {
            // DÜZELTME: Artık sabit 50 değil! Hedefe uzaklığı ölçüp haritadan açıyı çekiyor.
            double distance = getDumpDistance().in(edu.wpi.first.units.Units.Meters);
            double rawAngle = hoodMap.get(distance);
            return Degrees.of(MathUtil.clamp(rawAngle, 50.0, 70.0));
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
// Offset tamamen silindi, haritadaki saf açıyı alıyoruz
        double hoodPitch = getHoodSetpoint().in(edu.wpi.first.units.Units.Radians);        double turretYaw = getTurretSetpoint().in(edu.wpi.first.units.Units.Radians);        
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
        double hoodPitch = Math.toRadians(currentHoodRaw); // + 20.0 silindi!
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
    // Feeder her çalıştığında (ateş edildiğinde) sanal bir top oluşturur
    public void spawnSimulatedFuel() {
        Pose2d robotPose = swerve.getPose();
        
        // Yörünge çizgisindeki birebir aynı çıkış hesaplamaları
        double turretYaw = getTurretSetpoint().in(edu.wpi.first.units.Units.Radians);        
        double flywheelRPS = getFlywheelSetpoint().in(edu.wpi.first.units.Units.RotationsPerSecond);
        double hoodPitch = getHoodSetpoint().in(edu.wpi.first.units.Units.Radians); // + 20.0 silindi!
        double radius = 0.0508; 
        
        double v0 = (flywheelRPS * 2 * Math.PI * radius) * 0.8; 

        // Topun çıkış noktası
        double startX = robotPose.getX();
        double startY = robotPose.getY();
        double startZ = 0.45; // Shooter yüksekliği

        double globalYaw = robotPose.getRotation().getRadians() + turretYaw;
        double v_xy = v0 * Math.cos(hoodPitch); 

        // Robotun gidiş hızını topa ekliyoruz (SOTM fiziği)
        var robotSpeeds = swerve.getChassisSpeeds();
        Translation2d fieldVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                                          .rotateBy(robotPose.getRotation());
        
        double vx = (v_xy * Math.cos(globalYaw)) + fieldVelocity.getX(); 
        double vy = (v_xy * Math.sin(globalYaw)) + fieldVelocity.getY(); 
        double vz = v0 * Math.sin(hoodPitch);   

        // Yeni topu havaya fırlat!
        flyingFuels.add(new SimulatedFuel(startX, startY, startZ, vx, vy, vz));
    }

    // WPILib'in simülasyon döngüsü (Saniyede 50 kere çalışır)
    @Override
    public void simulationPeriodic() {
        double dt = 0.02; // 20ms simülasyon adımı
        java.util.ArrayList<Double> poseArray = new java.util.ArrayList<>();

        var iterator = flyingFuels.iterator();
        while (iterator.hasNext()) {
            SimulatedFuel fuel = iterator.next();
            
            // Fiziği işlet: Konumu hıza göre değiştir
            fuel.x += fuel.vx * dt;
            fuel.y += fuel.vy * dt;
            fuel.z += fuel.vz * dt;
            
            // Yerçekimi: Saniyede 9.81 m/s^2 aşağı çeker
            fuel.vz -= 9.81 * dt; 
            fuel.timeAlive += dt;

            // Eğer top yere düşerse (z < 0.1) veya havada 3 saniyeden fazla kalırsa onu sil
            if (fuel.z < 0.1 || fuel.timeAlive > 3.0) {
                iterator.remove();
            } else {
                // AdvantageScope için Pose3d formatı: [X, Y, Z, Q_W, Q_X, Q_Y, Q_Z]
                poseArray.add(fuel.x);
                poseArray.add(fuel.y);
                poseArray.add(fuel.z);
                // Topun dönüşü (rotasyonu) önemli olmadığı için sabit bir Quaternion (W=1) veriyoruz
                poseArray.add(1.0); 
                poseArray.add(0.0); 
                poseArray.add(0.0); 
                poseArray.add(0.0); 
            }
        }

        // Aktif topları NetworkTables'a yolla
        double[] arr = new double[poseArray.size()];
        for (int i = 0; i < poseArray.size(); i++) {
            arr[i] = poseArray.get(i);
        }
        SmartDashboard.putNumberArray("AdvantageScope/FlyingFuels", arr);
    }
    // Sahanın Y Eksenine (Bump'a) Göre ÇOK DAHA GÜVENLİ Fiziksel Hedefi Bulur
    public Translation2d getDumpTargetPosition() {
        Pose2d robotPose = swerve.getPose();
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        // DÜZELTME 1: Duvarlardan ÇOK DAHA UZAK (Maksimum Güvenli) Hedefler
        // Y ekseninde saha ~8.2 metredir. Duvarlara sıfır atmak yerine,
        // Y=6.0 (Üst koridor ortası) ve Y=2.2 (Alt koridor ortası) seçiyoruz.
        // Bu bize duvardan 2.2 metrelik devasa bir sapma payı (hata payı) verir!
        double targetY = (robotPose.getY() > 4.1) ? 6.0 : 2.2; 
        
        // DÜZELTME 2: X Ekseninde de duvar dibine değil, saha içine atıyoruz.
        // Kırmızıların atacağı X konumu ~13.5 (Duvar 16.5'te)
        // Mavilerin atacağı X konumu ~3.0 (Duvar 0'da)
        double targetX = isRed ? 13.5 : 3.0;

        return new Translation2d(targetX, targetY);
    }
    // 2. GECİKME TELAFİLİ KUSURSUZ SOTM DUMP MATEMATİĞİ (Sapmayı %0'a indirir!)
    // KUSURSUZ FİZİK: Momentum hesaplar ve SAHA DIŞINA ÇIKMAYI YASAKLAR (Clamping)
    public Translation2d getDumpVirtualTarget() {
        Translation2d dropZone = getDumpTargetPosition();
        Pose2d robotPose = swerve.getPose();
        
        Translation2d turretOffset = new Translation2d(Turret.turretDist, 0).rotateBy(robotPose.getRotation());
        Translation2d turretPos = robotPose.getTranslation().plus(turretOffset);
        
        // Dump esnasında ortalama tekerlek hızı ve açısı (Momentum telafisi için)
        double dumpRPS = 30.0; 
        double dumpHoodDegrees = 55.0; 
        double wheelRadiusMeters = 0.0508; 
        
        double v0 = (dumpRPS * 2 * Math.PI * wheelRadiusMeters) * 0.8; 
        double v_xy = v0 * Math.cos(Math.toRadians(dumpHoodDegrees));
        if (v_xy < 0.1) v_xy = 0.1; 
        
        double distance = dropZone.getDistance(turretPos);
        double timeOfFlight = distance / v_xy;
        
        var robotSpeeds = swerve.getChassisSpeeds();
        Translation2d fieldVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                                          .rotateBy(robotPose.getRotation());
                                          
        double virtualX = dropZone.getX() - (fieldVelocity.getX() * timeOfFlight);
        double virtualY = dropZone.getY() - (fieldVelocity.getY() * timeOfFlight);

        // ŞAMPİYONLUK DÜZELTMESİ (CLAMPING - KELEPÇE):
        // Sahanın X ekseni 0-16.5m, Y ekseni 0-8.2m'dir.
        // Sanal hedefin (Taretin nişan alacağı yerin) bu sınırları aşmasını MATEMATİKSEL OLARAK YASAKLIYORUZ!
        // Duvarlardan 1.5 metre içeride "Görünmez Bir Kutu" (Safe Box) yaratıyoruz.
        virtualX = MathUtil.clamp(virtualX, 1.5, 15.0); 
        virtualY = MathUtil.clamp(virtualY, 1.5, 6.7);  

        return new Translation2d(virtualX, virtualY);
    }
    // Motorlar hedefe ulaştı mı kontrolü (Hata payı toleransları)
    public boolean isReadyToShoot() {
        if (intent == ShotIntent.OFF) return false;

        // Flywheel hız kontrolü (Hedef hıza ±1.5 RPS yaklaştı mı?)
        double currentRPS = flywheel.getVelocity().in(edu.wpi.first.units.Units.RotationsPerSecond); 
        double targetRPS = getFlywheelSetpoint().in(edu.wpi.first.units.Units.RotationsPerSecond);
        boolean isFlywheelReady = Math.abs(currentRPS - targetRPS) < 1.5;

        // Taret açı kontrolü (Hedef açıya ±2.0 Derece yaklaştı mı?)
        double currentTurretDeg = turret.getAngle().in(edu.wpi.first.units.Units.Degrees);
        double targetTurretDeg = getTurretSetpoint().in(edu.wpi.first.units.Units.Degrees);
        boolean isTurretReady = Math.abs(currentTurretDeg - targetTurretDeg) < 2.0;

        // Eğer DUMP veya SOTM/HUB yapıyorsak ve her şey yerine oturduysa TRUE döndür
        return isFlywheelReady && isTurretReady;
    }
}
