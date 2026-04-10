package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.util.GeometryUtil;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.fieldConstants;
import frc.robot.commands.DebugShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Spindexer;
import frc.robot.Constants.Turret;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private final SwerveSubsystem swerve;
    private ShuffleboardTab tab;
    public ShotIntent intent = ShotIntent.OFF;
    public TurretSubsystem turret = new TurretSubsystem();
    public FlywheelSubsystem flywheel = new FlywheelSubsystem();
    public HoodSubsystem hood = new HoodSubsystem();
    public GenericEntry flywheelEntry;
    public GenericEntry hoodEntry;
    private int trajTickCounter = 0; // Görselleştirme sayacı
    
    private static class SimulatedFuel {
        double x, y, z;
        double vx, vy, vz;
        double timeAlive = 0;

        public SimulatedFuel(double x, double y, double z, double vx, double vy, double vz) {
            this.x = x; this.y = y; this.z = z;
            this.vx = vx; this.vy = vy; this.vz = vz;
        }
    }

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
        flywheelEntry = Shuffleboard.getTab("Shooter")
            .add("FlywheelSpeed", 40.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();
        hoodEntry = Shuffleboard.getTab("Shooter")
            .add("HoodAngle", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("hubButtonPressed", intent != ShotIntent.OFF);
        updateTrajectoryVisualization();
    }

    

    public DebugShootCommand debugShoot(SpindexerSubsystem spindexer, FeederSubsystem feeder) {
        return new DebugShootCommand(spindexer, feeder, this, () -> flywheelEntry.getDouble(40.0));
    }

    public void setIntent(ShotIntent intent) { this.intent = intent; }
    public ShotIntent getIntent() { return intent; }

    private void buildLookupTables() {
        hoodMap.put(1.5, 70.0); hoodMap.put(2.5, 65.0); hoodMap.put(3.5, 60.0); 
        hoodMap.put(4.5, 55.0); hoodMap.put(5.5, 50.0);

        flywheelMap.put(1.5, 23.0); flywheelMap.put(2.5, 25.5); flywheelMap.put(3.5, 27.5);
        flywheelMap.put(4.5, 30.0); flywheelMap.put(5.5, 32.0);
    }

    
    public Angle getTurretSetpoint() {
        Pose2d robotPose = swerve.getPose();
        Translation2d turretOffset = new Translation2d(-0.125, -0.105); 
        Translation2d turretFieldPosition = robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

        if (intent == ShotIntent.HUB || intent == ShotIntent.SOTM) {
            Translation2d targetTranslation = (intent == ShotIntent.SOTM) ? getVirtualTarget() : getHubPose().getTranslation();
            
            // DÜZELTME 2: ESKİ KODDAKİ 180 DERECE FLIP YAMASI EKLENDİ! (Uzağa bakmayı çözer)
            Rotation2d fieldAngle = targetTranslation.minus(turretFieldPosition).getAngle().plus(Rotation2d.k180deg);
            double dist = targetTranslation.minus(turretFieldPosition).getNorm();

            SmartDashboard.putNumber("Shooter/Distance", dist);
            
            double setpointDeg = fieldAngle.minus(robotPose.getRotation()).getDegrees();
            return Degrees.of((setpointDeg));
        }

        if (intent == ShotIntent.DUMP) {
            Translation2d virtualDropZone = getDumpVirtualTarget();
            // Dump için de 180 flip eklendi
            Rotation2d fieldAngle = virtualDropZone.minus(turretFieldPosition).getAngle().plus(Rotation2d.k180deg);
            Rotation2d turretRelative = fieldAngle.minus(robotPose.getRotation()); 
            return Degrees.of((turretRelative.getDegrees())); 
        }
        
        // OFF iken tarete "olduğun yerde kal" demek için iç açıya 0.25 eklememiz lazım,
        // çünkü setAngle() her zaman 0.25 çıkarıyor. Eklemezsen feedback döngüsü oluşur ve taret döner.
        return Rotations.of(0.25);
    }

    public Distance getDumpDistance() {
        Pose2d robotPose = swerve.getPose();
        Translation2d turretPos = robotPose.getTranslation().plus(new Translation2d(Turret.turretDist, 0).rotateBy(robotPose.getRotation()));
        return Meters.of(turretPos.getDistance(getDumpVirtualTarget()));
    }

    public double getMagnitude(Transform2d pose) { return Math.sqrt(pose.getX() * pose.getX() + pose.getY() * pose.getY()); }

    public Distance getHubDistance() {
        Pose2d robotPose = swerve.getPose();
        Translation2d targetTranslation = (intent == ShotIntent.SOTM) ? getVirtualTarget() : getHubPose().getTranslation();
        return Meters.of(robotPose.getTranslation().getDistance(targetTranslation));
    }

    public AngularVelocity getFlywheelSetpoint() {
        if (intent == ShotIntent.HUB || intent == ShotIntent.SOTM) return RotationsPerSecond.of(flywheelMap.get(getHubDistance().in(Meters)));
        if (intent == ShotIntent.DUMP) return RotationsPerSecond.of(flywheelMap.get(getDumpDistance().in(Meters)));
        return RotationsPerSecond.of(0);
    }
 
    public Angle getHoodSetpoint() {
        if (intent == ShotIntent.HUB || intent == ShotIntent.SOTM) return Degrees.of(MathUtil.clamp(hoodMap.get(getHubDistance().in(Meters)), 50.0, 70.0));
        if (intent == ShotIntent.DUMP) return Degrees.of(MathUtil.clamp(hoodMap.get(getDumpDistance().in(Meters)), 50.0, 70.0));
        return Degrees.of(0);
    }

    public Angle getHoodSetpoint(double distance) {
        if(intent == ShotIntent.HUB || intent == ShotIntent.SOTM) return Degrees.of(MathUtil.clamp(hoodMap.get(distance), 20.0, 40.0));
        return getHoodSetpoint();
    }

    public LinearVelocity getBaseExitVelocity(double distance) {
        return MetersPerSecond.of(flywheelMap.get(distance) + Shooter.flywheelOffsetRPS);
    }
    
    public enum ShotIntent { HUB, SOTM, DUMP, OFF }

    public void updateTrajectoryVisualization() {
        // EĞER OFF MODUNDAYSAK (ATEŞ ETMİYORSAK) RAM'İ YORMA, SADECE ÇİZGİYİ SİL VE ÇIK!
        if (intent == ShotIntent.OFF) {
            Logger.recordOutput("AdvantageScope/Shooter_Trajectory", new Pose3d[0]);
            return; 
        }

        // TİCK SAYACI: İşlemi her 20ms'de bir değil, her 100ms'de bir (5 tick) yapıyoruz!
        trajTickCounter++;
        if (trajTickCounter < 5) return; 
        trajTickCounter = 0;

        Pose2d robotPose = swerve.getPose(); 
        double hoodPitch = getHoodSetpoint().in(Radians);        
        double turretYaw = getTurretSetpoint().in(Radians);        
        double flywheelRPS = getFlywheelSetpoint().in(RotationsPerSecond);
        
        double radius = 0.0508; 
        double v0 = (flywheelRPS * 2 * Math.PI * radius) * 0.8; 
        Translation3d startPos = new Translation3d(robotPose.getX(), robotPose.getY(), 0.45);

        double globalYaw = robotPose.getRotation().getRadians() + turretYaw;
        double v_xy = v0 * Math.cos(hoodPitch); 

        double vx, vy;
        if (intent == ShotIntent.SOTM) {
            var robotSpeeds = swerve.getChassisSpeeds();
            Translation2d fieldVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());
            vx = (v_xy * Math.cos(globalYaw)) + fieldVelocity.getX(); 
            vy = (v_xy * Math.sin(globalYaw)) + fieldVelocity.getY(); 
        } else {
            vx = v_xy * Math.cos(globalYaw); 
            vy = v_xy * Math.sin(globalYaw); 
        }
        double vz = v0 * Math.sin(hoodPitch);   

        java.util.ArrayList<Pose3d> trajectory = new java.util.ArrayList<>();
        double t = 0; double dt = 0.05; double z = startPos.getZ();

        while (z > 0 && t < 3.0) {
            double x = startPos.getX() + (vx * t);
            double y = startPos.getY() + (vy * t);
            z = startPos.getZ() + (vz * t) - (0.5 * 9.81 * Math.pow(t, 2));

            if (z > 0) trajectory.add(new Pose3d(x, y, z, new Rotation3d()));
            t += dt;
        }   
        Logger.recordOutput("AdvantageScope/Shooter_Trajectory", trajectory.toArray(new Pose3d[0]));
    }

    public Translation2d getVirtualTarget() {
        Pose2d robotPose = swerve.getPose();
        Translation2d actualHub = getHubPose().getTranslation();
        double distanceToActual = robotPose.getTranslation().getDistance(actualHub);

        var robotSpeeds = swerve.getChassisSpeeds();
        Translation2d fieldVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());
        
        double currentRPS = flywheelMap.get(distanceToActual) != null ? flywheelMap.get(distanceToActual) : 32.0;
        double currentHoodRaw = hoodMap.get(distanceToActual) != null ? hoodMap.get(distanceToActual) : 50.0;
        double v0 = (currentRPS * 2 * Math.PI * 0.0508) * 0.8;
        double v_xy = v0 * Math.cos(Math.toRadians(currentHoodRaw));
        if (v_xy < 0.1) v_xy = 0.1; 

        double timeOfFlight = (distanceToActual / v_xy) + 0.04;

        return new Translation2d(actualHub.getX() - (fieldVelocity.getX() * timeOfFlight), actualHub.getY() - (fieldVelocity.getY() * timeOfFlight));
    }

    public void spawnSimulatedFuel() {
        Pose2d robotPose = swerve.getPose();
        double v0 = (getFlywheelSetpoint().in(RotationsPerSecond) * 2 * Math.PI * 0.0508) * 0.8; 
        double globalYaw = robotPose.getRotation().getRadians() + getTurretSetpoint().in(Radians);
        double hoodPitch = getHoodSetpoint().in(Radians);
        double v_xy = v0 * Math.cos(hoodPitch); 

        var robotSpeeds = swerve.getChassisSpeeds();
        Translation2d fieldVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());
        
        flyingFuels.add(new SimulatedFuel(robotPose.getX(), robotPose.getY(), 0.45, 
            (v_xy * Math.cos(globalYaw)) + fieldVelocity.getX(), (v_xy * Math.sin(globalYaw)) + fieldVelocity.getY(), v0 * Math.sin(hoodPitch)));
    }

    @Override
    public void simulationPeriodic() {
        double dt = 0.02; 
        java.util.ArrayList<Pose3d> poseArray = new java.util.ArrayList<>();

        var iterator = flyingFuels.iterator();
        while (iterator.hasNext()) {
            SimulatedFuel fuel = iterator.next();
            fuel.x += fuel.vx * dt; fuel.y += fuel.vy * dt; fuel.z += fuel.vz * dt;
            fuel.vz -= 9.81 * dt; 
            fuel.timeAlive += dt;

            if (fuel.z < 0.1 || fuel.timeAlive > 3.0) {
                iterator.remove();
            } else {
                poseArray.add(new Pose3d(fuel.x, fuel.y, fuel.z, new Rotation3d()));
            }
        }
        Logger.recordOutput("AdvantageScope/FlyingFuels", poseArray.toArray(new Pose3d[0]));
    }

    public Translation2d getDumpTargetPosition() {
        Pose2d robotPose = swerve.getPose();
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        return new Translation2d(isRed ? 13.5 : 3.0, (robotPose.getY() > 4.1) ? 6.5 : 1.7);
    }

    public Translation2d getDumpVirtualTarget() {
        Translation2d physicalTarget = getDumpTargetPosition();
        Translation2d turretPos = swerve.getPose().getTranslation().plus(new Translation2d(Turret.turretDist, 0).rotateBy(swerve.getPose().getRotation()));
        var robotSpeeds = swerve.getChassisSpeeds();
        Translation2d fieldVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond).rotateBy(swerve.getPose().getRotation());

        Translation2d virtualTarget = physicalTarget;
        for(int i = 0; i < 3; i++) {
            double distance = turretPos.getDistance(virtualTarget);
            double rps = flywheelMap.get(distance) != null ? flywheelMap.get(distance) : 32.0;
            double hoodDeg = hoodMap.get(distance) != null ? hoodMap.get(distance) : 50.0;
            
            double v_xy = ((rps * 2 * Math.PI * 0.0508) * 0.8) * Math.cos(Math.toRadians(hoodDeg));
            if (v_xy < 0.1) v_xy = 0.1; 
            
            double timeOfFlight = distance / v_xy;
            virtualTarget = new Translation2d(physicalTarget.getX() - (fieldVelocity.getX() * timeOfFlight), physicalTarget.getY() - (fieldVelocity.getY() * timeOfFlight));
        }
        return virtualTarget;
    }

    public boolean isReadyToShoot() {
        if (intent == ShotIntent.OFF) return false;
        
        double flywheelRPS = flywheel.getVelocity().in(RotationsPerSecond);
        double targetRPS = getFlywheelSetpoint().in(RotationsPerSecond);
        boolean flywheelReady = Math.abs(flywheelRPS - targetRPS) < 1.5;

        double turretDeg = turret.getAngle().in(Degrees);
        double targetDeg = getTurretSetpoint().in(Degrees);
        double turretError = MathUtil.inputModulus(turretDeg - targetDeg, -180.0, 180.0);
        boolean turretReady = Math.abs(turretError) < 2.0;

        double hoodDeg = hood.getAngle().in(Degrees);
        double targetHoodDeg = getHoodSetpoint().in(Degrees);
        boolean hoodReady = Math.abs(hoodDeg - targetHoodDeg) < 1.5; 

        Logger.recordOutput("Ready_Debug/FlywheelReady", flywheelReady);
        Logger.recordOutput("Ready_Debug/TurretReady", turretReady);
        Logger.recordOutput("Ready_Debug/HoodReady", hoodReady); 
        Logger.recordOutput("Ready_Debug/Intent", intent.name());
        
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) return true; 

        return flywheelReady && turretReady && hoodReady;
    }
}