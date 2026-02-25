package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.GeometryUtil;

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
    private ShotIntent intent = ShotIntent.OFF;
    public TurretSubsystem turret = new TurretSubsystem();
    public FlywheelSubsystem flywheel = new FlywheelSubsystem();
    public GenericEntry entry;

    private Pose2d getHubPose() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        return fieldConstants.RED_HUB_POSE;
    }
    return fieldConstants.BLUE_HUB_POSE;
    }

    private final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

    public ShooterSubsystem(SwerveSubsystem swerve) {
        this.swerve = swerve;
        buildLookupTables();
        tab = Shuffleboard.getTab("Shooter");
        entry = tab.addPersistent("Flywheel Speed (RPS)", 40)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance", getHubDistance().in(Meters));
        SmartDashboard.putNumber("Slider", entry.getDouble(10));
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
        flywheelMap.put(1.5, 55.0);         // distance to RPS
        flywheelMap.put(2.0, 65.0);
        flywheelMap.put(2.5, 75.0);
        flywheelMap.put(3.0, 85.0);
        flywheelMap.put(3.5, 95.0);

    }


    public Angle getTurretSetpoint() {
        if (intent == ShotIntent.HUB) {
            Pose2d robotPose = swerve.getPose(); /* 
            Pose2d rotationPose = new Pose2d(
                robotPose.getRotation().getCos(),
                robotPose.getRotation().getSin(),
                Rotation2d.kZero
            );*/
          //  Pose2d turretPose = new Pose2d(robotPose.minus(rotationPose.times(-Turret.turretDist)).toMatrix());
            Rotation2d fieldAngle =
                getHubPose().getTranslation()
                    .minus(robotPose.getTranslation())
                    .getAngle().plus(Rotation2d.k180deg);
            SmartDashboard.putNumber("Field Angle", fieldAngle.getDegrees());
            return Degrees.of(
                fieldAngle.minus(robotPose.getRotation()).getDegrees()
            );
        }
        if (intent == ShotIntent.DUMP) {
            return Degrees.of(0); 
        }
        return Degrees.of(90); 
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

            return RotationsPerSecond.of(flywheelMap.get(distance)+Shooter.flywheelOffsetRPS);
        }

        if (intent == ShotIntent.DUMP) {
            return RotationsPerSecond.of(30);
        }

        return RotationsPerSecond.of(0);
    }

   public LinearVelocity getBaseExitVelocity(double distance) {
    double rps = flywheelMap.get(distance) + Shooter.flywheelOffsetRPS;
    return MetersPerSecond.of(rps);
    }

}
