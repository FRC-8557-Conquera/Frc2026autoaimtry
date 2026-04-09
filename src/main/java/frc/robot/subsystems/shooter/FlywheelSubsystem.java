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

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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

public class FlywheelSubsystem extends SubsystemBase {
  private final TalonFX flywheelMotor = new TalonFX(Flywheel.flywheelMotor);
  private double setPoint = 1; // in RPS
  
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.2, 0.0, 0.005, RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(2500))
      .withGearing(new MechanismGearing(1))                      
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
      .withSupplyCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0))
      .withOpenLoopRampRate(Seconds.of(0))   
      .withFeedforward(new SimpleMotorFeedforward(0.42049, 0.11603, 0.0090764))
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

  public AngularVelocity getVelocity() {
    return flywheel.getSpeed();
  }

  public double getSetpoint() {
    return setPoint;
  }

  public Command setVelocity(AngularVelocity speed) {
    setPoint = speed.in(RotationsPerSecond);
    return flywheel.setSpeed(speed);
  }

  public Command setDutyCycle(double dutyCycle) {
    return flywheel.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return flywheel.setSpeed(speed);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return flywheel.set(dutyCycle);
  }

  public Command sysId() {
    return flywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
  }

  // =========================================================================
  // YENİ EKLENEN KISIM: JOYSTICK EKSENİNE (AXIS 3) GÖRE HIZ KONTROL KOMUTU
  // =========================================================================
  
  /**
   * Joystick tetiğinden (Right Trigger) gelen 0.0 ile 1.0 arasındaki değeri alır,
   * MAX_RPS ile çarparak motora hedef hız olarak verir.
   */
  public Command runFromTrigger(DoubleSupplier triggerAxis) {
      return run(() -> {
          // Tetik ekseni 0.0 ile 1.0 arası değer verir.
          double rawValue = triggerAxis.getAsDouble();
          
          // Ufak dokunuşları (deadband) görmezden gel, kazara motor dönmesin
          if (rawValue < 0.05) {
              setPoint = 0.0;
              flywheel.set(0.0); // DutyCycle 0 yap, motor dursun
          } else {
              // Tetiğe ne kadar basıldıysa, MAX_RPS'in (Örn: 90) o kadarı hız iste.
              // Örneğin tetiğe yarım basıldıysa (0.5), hız 45 RPS olur.
              setPoint = rawValue * Flywheel.MAX_RPS;
              flywheel.setSpeed(RotationsPerSecond.of(setPoint));
          }
      });
  }
  // =========================================================================

  @Override
  public void periodic() {
    double velocityRPS = getVelocity().in(RotationsPerSecond);
    double error = setPoint - velocityRPS;
    
    // ELASTIC DASHBOARD'A YAZILACAK VERİLER
    SmartDashboard.putNumber("Flywheel/Velocity_RPS", velocityRPS);
    SmartDashboard.putNumber("Flywheel/Setpoint_RPS", setPoint);
    SmartDashboard.putNumber("Flywheel/Error_RPS", error);
    
    flywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    flywheel.simIterate();
  }

  public Command setRPM(LinearVelocity speed) {
    return flywheel.setSpeed(RotationsPerSecond.of(speed.in(MetersPerSecond) / Flywheel.flywheelDiameter.times(Math.PI).in(Meters)));
  }

  public boolean isAtSpeed() {
    double velocity = getVelocity().in(RotationsPerSecond);
    return Math.abs(setPoint - velocity) < 2.0; // tolerans (RPS)
  }
}