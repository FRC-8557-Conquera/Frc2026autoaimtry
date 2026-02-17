package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.Feeder;

public class FeederSubsystem extends SubsystemBase {

  private final SparkMax feederMotor = new SparkMax(Feeder.feederMotor, MotorType.kBrushless);

  public FeederSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(40);
    feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // run forward
  public Command feed() {
    return run(() -> feederMotor.set(Feeder.feedSpeed));
  }

  // reverse
  public Command reverse() {
    return run(() -> feederMotor.set(Feeder.reverseSpeed));
  }

  // stop
  public Command stop() {
    return runOnce(() -> feederMotor.set(0));
  }

}
