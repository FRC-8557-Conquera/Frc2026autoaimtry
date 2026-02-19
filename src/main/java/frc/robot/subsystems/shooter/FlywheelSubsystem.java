package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.flatbuffers.Constants;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import frc.robot.Constants.Flywheel;

public class FlywheelSubsystem extends SubsystemBase
{
  private final TalonFX flywheelMotor    = new TalonFX(Flywheel.flywheelMotor);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.15,0, 0, RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(2500))        // TODO: Change the PID values
      .withGearing(new MechanismGearing(1))                      
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
      .withSupplyCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))   
      .withFeedforward(new SimpleMotorFeedforward(0.31148, 0.12197, 0.0057185))
      .withControlMode(ControlMode.CLOSED_LOOP);


  private final SmartMotorController motor = new TalonFXWrapper(flywheelMotor, DCMotor.getKrakenX60(1), motorConfig);

  private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(motor)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
      .withSoftLimit(RotationsPerSecond.of(-100), RotationsPerSecond.of(100))
      .withSpeedometerSimulation(RotationsPerSecond.of(120));

  private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

  public FlywheelSubsystem() {}

  public void deneme(){
    flywheel.set(-1);
  }
  public void denemeStop() {
    flywheel.set(0);
  }
  public AngularVelocity getVelocity()
  {
    return flywheel.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed)
  {
    return flywheel.setSpeed(speed);
  }

  public Command setDutyCycle(double dutyCycle)
  {
    return flywheel.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed)
  {
    return flywheel.setSpeed(speed);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle)
  {
    return flywheel.set(dutyCycle);
  }

  public Command sysId()
  {
    return flywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
  }

  @Override
  public void periodic()
  {
    flywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    flywheel.simIterate();
  }
    public Command setRPM(LinearVelocity speed)
  {
    return flywheel.setSpeed(RotationsPerSecond.of(speed.in(MetersPerSecond) / Flywheel.flywheelDiameter.times(Math.PI).in(Meters)));
  }

}

