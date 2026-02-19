package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.Feeder;

public class FeederSubsystem extends SubsystemBase {

  private final SparkMax feederMotor = new SparkMax(Feeder.feederMotor, MotorType.kBrushless);
  private final SparkClosedLoopController sparkClosedLoopController = feederMotor.getClosedLoopController();

  public FeederSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(40);
    config.closedLoop.pid(0.01,0,0);
    feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
        SmartDashboard.putNumber("Feeder Encoder", getVelocity());
  }

  public Command dutyCycleFeed() {
    return run(() -> feederMotor.set(Feeder.feedSpeed));
  }
  public Command dutyCycleReverse() {
    return run(() -> feederMotor.set(Feeder.reverseSpeed));
  }
  // run forward
  public Command feed() {
    return run(() -> sparkClosedLoopController.setSetpoint(Feeder.feedSpeed * Feeder.feederMaxSpeed.in(RPM), ControlType.kVelocity));
  }

  // reverse
  public Command reverse() {
    return run(() -> sparkClosedLoopController.setSetpoint(Feeder.reverseSpeed * Feeder.feederMaxSpeed.in(RPM), ControlType.kVelocity));
  }

  // stop
  public Command stop() {
    return runOnce(() -> feederMotor.set(0));
  }

  public double getVelocity() {
    return feederMotor.getEncoder().getVelocity();
  }

}
