package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.SwerveSubsystem;

public class ShooterSubsystem {

    private final SwerveSubsystem swerve;
    private ShotIntent intent = ShotIntent.OFF;

    private static final Pose2d HUB_POSE =
        new Pose2d(12.742, 4.238, new Rotation2d());

    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

    public static final double METERS_PER_ROTATION = 0.08; //TODO: measure this


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
        hoodMap.put(1.5, 20.0);             // distance to Hood angle
        hoodMap.put(2.0, 28.0);
        hoodMap.put(2.5, 35.0);
        hoodMap.put(3.0, 42.0);
        hoodMap.put(3.5, 48.0);

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
                HUB_POSE.getTranslation()
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

    public Angle getHoodSetpoint() {
        if (intent == ShotIntent.HUB) {
            double distance =
                swerve.getPose()
                    .getTranslation()
                    .getDistance(HUB_POSE.getTranslation());

            return Degrees.of(hoodMap.get(distance));
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

    public AngularVelocity getFlywheelSetpoint() {
        if (intent == ShotIntent.HUB) {
            double distance =
                swerve.getPose()
                    .getTranslation()
                    .getDistance(HUB_POSE.getTranslation());

            return RotationsPerSecond.of(flywheelMap.get(distance));
        }

        if (intent == ShotIntent.DUMP) {
            return RotationsPerSecond.of(30);
        }

        return RotationsPerSecond.of(0);
    }

   public LinearVelocity getBaseExitVelocity(double distance) {
    double rps = flywheelMap.get(distance);
    return MetersPerSecond.of(rps * METERS_PER_ROTATION);
}


    public AngularVelocity getFlyWheelRPS(double distance) {
        return RotationsPerSecond.of(flywheelMap.get(distance));
    }
    
}
