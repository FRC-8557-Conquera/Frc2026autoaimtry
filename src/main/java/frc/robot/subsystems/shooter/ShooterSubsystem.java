package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.fieldConstants;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem {

    private final SwerveSubsystem swerve;
    private ShotIntent intent = ShotIntent.OFF;

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
            Pose2d pose = swerve.getPose();
            Rotation2d fieldAngle =
                getHubPose().getTranslation()
                    .minus(pose.getTranslation())
                    .getAngle();

            return Degrees.of(
                fieldAngle.minus(pose.getRotation()).getDegrees()
            );
        }
        if (intent == ShotIntent.DUMP) {
            return Degrees.of(0); 
        }
        return Degrees.of(0); 
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
    return MetersPerSecond.of(rps * Shooter.METERS_PER_ROTATION);
    }

      public AngularVelocity getFlyWheelRPS(double distance) {
        return RotationsPerSecond.of(flywheelMap.get(distance));
    }
    
}
