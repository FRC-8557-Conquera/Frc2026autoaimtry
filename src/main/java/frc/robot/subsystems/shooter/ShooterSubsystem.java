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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class ShooterSubsystem extends SubsystemBase {

    private final SwerveSubsystem swerve;
    public ShotIntent intent = ShotIntent.HUB;
    public TurretSubsystem turret = new TurretSubsystem();
    public FlywheelSubsystem flywheel = new FlywheelSubsystem();
    public GenericEntry flywheelEntry;
    public GenericEntry hoodEntry;

   
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
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("hubButtonPressed", intent != ShotIntent.OFF);
    }

    public Command debugShoot() {
        return flywheel.setVelocity(() -> getFlywheelSetpoint());
    }

    public DebugShootCommand debugShoot(SpindexerSubsystem spindexer, FeederSubsystem feeder) {
        return new DebugShootCommand(spindexer, feeder, this, () -> flywheelEntry.getDouble(40.0));
    }

    public void setIntent(ShotIntent intent) { this.intent = intent; }
    public ShotIntent getIntent() { return intent; }

    private void buildLookupTables() {
        flywheelMap.put(1.3, 27.5); 
        flywheelMap.put(2.0, 29.0); 
        flywheelMap.put(2.4, 30.5);
        flywheelMap.put(3.1, 35.0);
        flywheelMap.put(4.0, 40.0); 
    }

    
    public Angle getTurretSetpoint() {
        Pose2d robotPose = swerve.getPose();
        Translation2d turretOffset = new Translation2d(-0.125, -0.105); 
        Translation2d turretFieldPosition = robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

        if (intent == ShotIntent.HUB) {
            Translation2d targetTranslation = getHubPose().getTranslation();
            
            // DÜZELTME 2: ESKİ KODDAKİ 180 DERECE FLIP YAMASI EKLENDİ! (Uzağa bakmayı çözer)
            Rotation2d fieldAngle = targetTranslation.minus(turretFieldPosition).getAngle().plus(Rotation2d.k180deg);
            double dist = targetTranslation.minus(turretFieldPosition).getNorm();

            SmartDashboard.putNumber("Shooter/Distance", dist);
            
            double setpointDeg = fieldAngle.minus(robotPose.getRotation()).getDegrees();
            return Degrees.of((setpointDeg));
        }


        
        // OFF iken tarete "olduğun yerde kal" demek için iç açıya 0.25 eklememiz lazım,
        // çünkü setAngle() her zaman 0.25 çıkarıyor. Eklemezsen feedback döngüsü oluşur ve taret döner.
        return Rotations.of(0.25);

    }
    public AngularVelocity getDumpVelocity() {
        boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        double robotX = swerve.getPose().getX();
        double dumpX = isRed ? 6.5 : 2.5;
        double xDist = Math.abs(robotX - dumpX);
        return RotationsPerSecond.of(flywheelMap.get(xDist) != null ? flywheelMap.get(xDist) : 50.0);
        
    }
    public Distance getHubDistance() {
        Pose2d robotPose = swerve.getPose();
        Translation2d turretOffset = new Translation2d(-0.125, -0.105); 
        Translation2d turretFieldPosition = robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));
        Translation2d targetTranslation = getHubPose().getTranslation();
        return Meters.of(turretFieldPosition.getDistance(targetTranslation));
    }

    public AngularVelocity getFlywheelSetpoint() {
        if (intent == ShotIntent.HUB) {
            double rps = (flywheelMap.get(getHubDistance().in(Meters)) != null) ? flywheelMap.get(getHubDistance().in(Meters)) : 45.0;
            return RotationsPerSecond.of(rps);   
        }
        if (intent == ShotIntent.DUMP) {
            return getDumpVelocity();   
        }
        return RotationsPerSecond.of(0);
    }

    public LinearVelocity getBaseExitVelocity(double distance) {
        return MetersPerSecond.of(flywheelMap.get(distance) + Shooter.flywheelOffsetRPS);
    }
    
    public enum ShotIntent { HUB, DUMP, OFF }
}