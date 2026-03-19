package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Degrees;

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
    public static final double driveConversionPositionFactor =(wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 5.0; // meters per second
    public static final double maxAngularVelocity = maxSpeed / Math.hypot(trackWidth / 2, wheelBase / 2); // 11.65

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

    public static final Angle MIN_ANGLE = Angle.ofBaseUnits(-20, Degrees);                        //TODO: Change this value
    public static final Angle MAX_ANGLE = Angle.ofBaseUnits(100, Degrees);      //TODO: Change this value
    
    public static final double intakeEncoderOffsetDeg = 0.6339 * 360 ; 

    public static final int intakeRoller = 30;
    public static final int intakeLeft = 31;
    public static final int intakeRight = 32;

    public static final int intakeEncoderPort = 7;

    public static final double rollerInSpeed = 0.5;
    public static final double rollerOutSpeed = -0.5;

    public static final double kP = 0.03;  // tune later
  }

  /* ===================== SPINDEXER ===================== */
  public static final class Spindexer {
    public static final int spindexerMotor = 33; 
    public static final double feedSpeed = 1;
    public static final double reverseSpeed = -1;
  }

  /* ===================== FEEDER ===================== */
  public static final class Feeder {
    public static final int feederMotor = 34;
    public static final IdleMode idleMode = IdleMode.kBrake;
    public static final boolean inverted = false;

    public static final double feedSpeed = -1;
    public static final double reverseSpeed = 1;

  }

  /* ===================== TURRET ===================== */
  public static final class Turret {
    public static final int turretMotor = 41;
    public static final int encoderPort = 5; 
    public static final double encoderOffset = 0.756984; 
    public static final double turretDist = -0.141;
  }

    public static final class Hood {

    public static final int hoodMotor = 52;
    public static final int encoderPort = 1; // EXAMPLE CHANGE THIS VALUE
    public static final IdleMode idleMode = IdleMode.kBrake;
    public static final boolean inverted = false;

    public static final double minAngleDegrees = 5.0;
    public static final double maxAngleDegrees = 100.0;
    public static final double hoodOffsetDeg = 73.2; // EXAMPLE CHANGE THIS VALUE
  }

  /* ===================== FLYWHEEL ===================== */

  public static final class Flywheel{
    public static final int flywheelMotor = 40;
    public static final double MAX_RPS = 90;
    public static final double MIN_RPS = 10;
    public static final Distance flywheelDiameter = Inches.of(4);
  }
  /* ===================== SHOOTER ===================== */
  public static final class Shooter {
    public static final double flywheelOffsetRPS = 0.0;                   // TODO: tune this
    public static final Angle FIXED_HOOD = Degrees.of(38);      //TODO: CHANGE THIS VALUE
    public static final double METERS_PER_ROTATION = 0.08;                //TODO: measure this
  }

  /* ===================== CLIMB ===================== */
  public static final class Climb {
    public static final int climbLeft = 50;
    public static final int climbRight = 51;
    public static final double climbUpSpeed = 1.0;
    public static final double climbDownSpeed = -0.8;
  }

    /* ===================== FIELD ===================== */
  public static final class fieldConstants{
    public static final Pose2d BLUE_HUB_POSE =new Pose2d(4.6, 4.04, new Rotation2d());
    public static final Pose2d RED_HUB_POSE =  new Pose2d(11.9,4.04,new Rotation2d());
  }
}
