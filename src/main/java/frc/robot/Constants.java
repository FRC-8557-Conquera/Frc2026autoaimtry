package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class Swerve {

    public static final double stickDeadband = 0.09;

    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = 0.585;
    public static final double wheelBase = 0.585;
    public static final double wheelDiameter = Units.inchesToMeters(3.91);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 30;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.001;
    public static final double angleKI = 0.000;
    public static final double angleKD = 0.00;
    public static final double angleKFF = 0.000;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.0001;
    public static final double driveKI = 0.00005;
    public static final double driveKD = 0.0005;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 6.5; // meters per second
    public static final double maxAngularVelocity =
        maxSpeed / Math.hypot(trackWidth / 2, wheelBase / 2); // 11.65

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    public static final double kTranslationVarianceThreshold = 0.1; // Örneğin, 0.1 metre
    public static final double kAngleVarianceThreshold = 5.0;
  }

  /* ===================== INTAKE ===================== */
  public static final class Intake {

    public static final int intakeRoller = 30;
    public static final int intakeFollower1 = 31;
    public static final int intakeFollower2 = 32;

    public static final IdleMode idleMode = IdleMode.kBrake;

    public static final boolean rollerInverted = false;
    public static final boolean followerInverted = true;

    public static final double intakeSpeed = 0.9;
    public static final double outtakeSpeed = -0.8;
    public static final double holdSpeed = 0.1;
  }

  /* ===================== SHOOTER ===================== */
  public static final class Shooter {

    public static final int shooterFlywheel = 40;
    public static final int shooterAngle = 41;

    public static final IdleMode flywheelIdleMode = IdleMode.kCoast;
    public static final IdleMode angleIdleMode = IdleMode.kBrake;

    public static final boolean flywheelInverted = true;
    public static final boolean angleInverted = false;

    public static final double shootSpeed = 1.0;
    public static final double idleSpeed = 0.15;

    // Angle PID (ileride kapalı döngü için)
    public static final double angleKP = 0.015;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.001;
  }

  /* ===================== FEEDER ===================== */
  public static final class Feeder {

    public static final int feederMotor = 50;
    public static final IdleMode idleMode = IdleMode.kBrake;
    public static final boolean inverted = false;

    public static final double feedSpeed = 0.7;
    public static final double reverseSpeed = -0.5;
  }

  /* ===================== TURRET ===================== */
  public static final class Turret {

    public static final int turretMotor = 51;
    public static final IdleMode idleMode = IdleMode.kBrake;
    public static final boolean inverted = false;

    public static final double manualSpeed = 0.4;
  }

  /* ===================== CLIMB ===================== */
  public static final class Climb {

    public static final int climbLeft = 60;
    public static final int climbRight = 61;

    public static final IdleMode idleMode = IdleMode.kBrake;

    public static final boolean leftInverted = false;
    public static final boolean rightInverted = true;

    public static final double climbUpSpeed = 1.0;
    public static final double climbDownSpeed = -0.8;
    public static final double holdSpeed = 0.05;
  }
}
