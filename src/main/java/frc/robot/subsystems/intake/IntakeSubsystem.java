package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {

  // ================= Motors =================
  private final SparkMax leftMotor = new SparkMax(Intake.intakeLeft, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(Intake.intakeRight, MotorType.kBrushless);
  private final TalonFX roller = new TalonFX(Intake.intakeRoller);

  // ================= Encoders =================
  private final DutyCycleEncoder leftEncoder =
      new DutyCycleEncoder(Intake.intakeLeftEncoderPort);
  private final DutyCycleEncoder rightEncoder =
      new DutyCycleEncoder(Intake.intakeRightEncoderPort);


  // ================= PID =================
  private double kP = 0.03;  // tune later

  private Angle targetAngle = Degrees.of(90);

  public IntakeSubsystem() {

    // make right follow left
    SparkMaxConfig follow = new SparkMaxConfig();
    follow.follow(leftMotor, true);
    rightMotor.configure(follow, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Angle getAngle() {
    double leftDeg  = leftEncoder.get()  * 360.0 - Intake.intakeLeftEncoderOffsetDeg;
    double rightDeg = rightEncoder.get() * 360.0 - Intake.intakeRightEncoderOffsetDeg;
    leftDeg  = MathUtil.inputModulus(leftDeg,  -180, 180);
    rightDeg = MathUtil.inputModulus(rightDeg, -180, 180);
    return Degrees.of((leftDeg + rightDeg) / 2.0);
  }

  public void setTargetAngle(Angle angle) {
     targetAngle = Degrees.of(MathUtil.clamp(angle.in(Degrees), Intake.MIN_ANGLE.in(Degrees), Intake.MAX_ANGLE.in(Degrees)));
  }

  public Command open() {
    return runOnce(() -> setTargetAngle(Degrees.of(120)));
  }

  public Command close() {
    return runOnce(() -> setTargetAngle(Degrees.of(10)));
  }

  public Command rollerIn() {
    return run(() -> roller.set(1.0));
  }

  public Command rollerOut() {
    return run(() -> roller.set(-1.0));
  }

  public Command rollerStop() {
    return runOnce(() -> roller.setControl(new NeutralOut()));
  }

 
  @Override
  public void periodic() {
    double current = getAngle().in(Degrees);
    double target = targetAngle.in(Degrees);

    double error = target - current;

    double output = MathUtil.clamp(error * kP, -0.5, 0.5);

    // stop at limits
    if (current <= Intake.MIN_ANGLE.in(Degrees) && output < 0) output = 0;
    if (current >= Intake.MAX_ANGLE.in(Degrees) && output > 0) output = 0;

    leftMotor.set(output);
  }
}
