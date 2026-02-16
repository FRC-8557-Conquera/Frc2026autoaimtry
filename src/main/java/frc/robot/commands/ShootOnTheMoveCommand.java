package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Flywheel;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.Constants.Shooter;

import java.util.List;
import java.util.function.Supplier;


/**
 * Largely written by Eeshwar based off their blog at https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 */
public class ShootOnTheMoveCommand extends Command
{

  // Subsystems
  private final TurretSubsystem   turretSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final ShooterSubsystem  shooterSubsystem;

  /**
   * Current robot pose. (Blue-alliance)
   */
  private final Supplier<Pose2d>        robotPose;
  /**
   * Current field-oriented chassis speeds.
   */
  private final Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds;
  /**
   * Pose to shoot at.
   */
  private final Pose2d                  goalPose;

  // Tuned Constants
  double ballTime = 0.12; //TODO: time in seconds for the ball to reach the target after leaving the shooter. You can measure this by logging the time when the ball leaves the shooter (e.g., using a sensor on the shooter) and the time when the ball hits the target (e.g., using a sensor on the target), and taking the difference.
  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  private final double                     latency      = 0.15;     //TODO: tune this value to match the actual latency of your system. This includes the time it takes for the robot to start moving, the time it takes for the shooter to spin up, and any other delays in the system. You can measure this by logging the time when you command the shooter to shoot and the time when the shooter actually shoots, and taking the difference.
  /**
   * Maps Distance to RPM
   */
  
  //private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

  /**
   * Shoot on the move command to always have the turret ready to fire.
   *
   * @param turret                     Turret subsystem
   * @param hood                       Hood subsystem
   * @param flyWheel                   Flywheel subsystem
   * @param currentPose                Current robot pose.
   * @param fieldOrientedChassisSpeeds Current field-oriented chassis speeds.
   * @param goal                       Goal to shoot at.
   */
  public ShootOnTheMoveCommand(TurretSubsystem turret, ShooterSubsystem shooter, FlywheelSubsystem flyWheel,
                               Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                               Pose2d goal)
  {
    shooterSubsystem = shooter;
    turretSubsystem = turret;
    flywheelSubsystem = flyWheel;
    robotPose = currentPose;
    this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
    this.goalPose = goal;
  }

  @Override
  public void initialize(){}
  

  @Override
  public void execute()
  {
    // Please look here for the original authors work!
    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly


    var robotSpeed = fieldOrientedChassisSpeeds.get();
    // 1. LATENCY COMP
    Translation2d futurePos = robotPose.get().getTranslation().plus(
      new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency+ballTime));

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = goalPose.getTranslation();
    Translation2d targetVec    = goalLocation.minus(futurePos);
    double        dist         = targetVec.getNorm();
    if (dist < 0.01) {
        return;   // too close to solve safely
    }

    Angle baseHood = Shooter.FIXED_HOOD;
    LinearVelocity baseExitVelocity = shooterSubsystem.getBaseExitVelocity(dist);

    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Note: This returns HORIZONTAL velocity component
    double idealHorizontalSpeed = baseExitVelocity.in(MetersPerSecond) * Math.cos(baseHood.in(Radians));

    // 4. VECTOR SUBTRACTION
    Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

    // 5. CONVERT TO CONTROLS
    double turretAngle        = shotVec.getAngle().getDegrees();
    double newHorizontalSpeed = shotVec.getNorm();

    // 6. CALCULATE REQUIRED FLYWHEEL SPEED
    double exitVel = newHorizontalSpeed / Math.cos(Shooter.FIXED_HOOD.in(Radians));
    double newRPS = exitVel / Shooter.METERS_PER_ROTATION;

    newRPS = MathUtil.clamp(newRPS, Flywheel.MIN_RPS, Flywheel.MAX_RPS);

    // 7. SET OUTPUTS
   turretSubsystem.setTargetAngle(Degrees.of(turretAngle));
   flywheelSubsystem.setVelocity(RotationsPerSecond.of(newRPS)); 
    
  }

  @Override
  public boolean isFinished()
  {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {

  }
}
