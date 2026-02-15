package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  public ClimbSubsystem() {
    leftMotor = new SparkMax(Constants.Climb.climbLeft, MotorType.kBrushless);
    rightMotor = new SparkMax(Constants.Climb.climbRight, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

    leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void climbUp() {
    leftMotor.set(1.0);
    rightMotor.set(1.0);
  }

  public void climbDown() {
    leftMotor.set(-0.8);
    rightMotor.set(-0.8);
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
