package frc.robot.subsystems.spindexer;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Spindexer;

public class SpindexerSubsystem extends SubsystemBase {

  private final SparkMax spindexerMotor = new SparkMax(Spindexer.spindexerMotor, MotorType.kBrushless);

  public SpindexerSubsystem() {}

  public Command spinForward() {
    return run(() -> spindexerMotor.set(Spindexer.feedSpeed));
  }

  public Command spinReverse() {
    return run(() -> spindexerMotor.set(Spindexer.reverseSpeed));
  }

  public Command stop() {
    return run(() -> spindexerMotor.set(0));
  }
}
