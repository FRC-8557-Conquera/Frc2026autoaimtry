package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX leftMotor = new TalonFX(Intake.intakeLeft, "*");
  private final TalonFX rightMotor = new TalonFX(Intake.intakeRight, "*");
  private final TalonFX roller = new TalonFX(Intake.intakeRoller, "*");

  private IntakePosition position = IntakePosition.UP;

  private final DutyCycleEncoder leftEncoder =
      new DutyCycleEncoder(Intake.intakeLeftEncoderPort);
  private final DutyCycleEncoder rightEncoder =
      new DutyCycleEncoder(Intake.intakeRightEncoderPort);

  public IntakeSubsystem() {
    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.CurrentLimits.StatorCurrentLimit = 40;
    armConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    leftMotor.getConfigurator().apply(armConfig);
    rightMotor.getConfigurator().apply(armConfig);
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }
  public Angle getAngle() {
    double leftDeg  = leftEncoder.get()  * 360.0 - Intake.intakeLeftEncoderOffsetDeg;
    double rightDeg = rightEncoder.get() * 360.0 - Intake.intakeRightEncoderOffsetDeg;
    leftDeg  = MathUtil.inputModulus(leftDeg,  -180, 180);
    rightDeg = MathUtil.inputModulus(rightDeg, -180, 180);
    return Degrees.of((leftDeg + rightDeg) / 2.0);
  }

  public Command open() {
    return runOnce(() -> position = IntakePosition.UP);
  }

  public Command close() {
    return runOnce(() -> position = IntakePosition.DOWN);
  }

  public Command rollerIn() {
    return run(() -> roller.set(Intake.rollerInSpeed));
  }

  public Command rollerOut() {
    return run(() -> roller.set(Intake.rollerOutSpeed));
  }

  public Command rollerStop() {
    return run(() -> roller.set(0));
  }

  @Override
  public void periodic() {
    double current = getAngle().in(Degrees);
    // TODO: Yerin, yukarisinin ve 45in encoderdan degerlerini al
    double target = (position == IntakePosition.UP ? 90 : (position == IntakePosition.HALF ? 45 : 15));

    double error = MathUtil.inputModulus(target - current, -180, 180);

    double output = MathUtil.clamp(error * Intake.kP, -0.5, 0.5);

    // stop at limits
    if (current <= Intake.MIN_ANGLE.in(Degrees) && output < 0) output = 0;
    if (current >= Intake.MAX_ANGLE.in(Degrees) && output > 0) output = 0;

    leftMotor.set(output);
  }
  public enum IntakePosition {
    UP,
    DOWN,
    HALF
} 
}


