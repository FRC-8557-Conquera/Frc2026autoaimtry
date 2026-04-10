package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants.Feeder;

public class FeederSubsystem extends SubsystemBase {

  private final SparkMax feederMotor = new SparkMax(Constants.Feeder.feederMotor, MotorType.kBrushless);
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(0.01, 0.0, 0.0002, RPM.of(5000),
        RotationsPerSecondPerSecond.of(1000)) // TODO: Change the PID values
        .withGearing(new MechanismGearing(4))
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("FeederMotor", TelemetryVerbosity.LOW)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withClosedLoopRampRate(Seconds.of(0))
        .withOpenLoopRampRate(Seconds.of(0))
        .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController feederSMC = new SparkWrapper(feederMotor,
                  DCMotor.getNEO(1),
                  motorConfig);
  // Komut sınıflarından motoru doğrudan yüzdelik güç ile çalıştırmak için
  public void setMotor(double speed) {
        // Eğer motorun adı feederMotor ise (SparkMax veya TalonSRX varsayımıyla):
        
    // NOT: Eğer Feeder motorun Kraken (TalonFX) ise yukarıdaki yerine şunu kullanmalısın:
    // For SparkMax, call set with a double percent output (-1.0 .. 1.0)
    feederMotor.set(speed);
    }
  public FeederSubsystem() {}
  public Command dutyCycleFeed() {
    return run(() -> feederSMC.setDutyCycle(-Feeder.feedSpeed));
  }
  @Override
  public void periodic() {
    feederSMC.updateTelemetry();
  }
  // run forward
  public Command feed() {
    return run(() -> {feederSMC.setVelocity(RPM.of(3000));});
  }

  // reverse
  public Command reverse() {
    return run(() -> {feederSMC.setVelocity(RPM.of(-3000));});
  }
  // Feeder'ı Command oluşturmadan doğrudan belirtilen hızda çalıştırır
    public void setDutyCycleDirect(double speed) {
        // Eğer YAMS (SmartMotorController) kullanıyorsan:
        // feederSMC.setDutyCycle(speed);
        
        // Eğer normal WPILib/SparkMax kullanıyorsan:
        feederMotor.set(speed); // (Kendi motor ismine göre düzelt)
    }

    // Feeder'ı anında durdurur
    public void stopDirect() {
        // feederSMC.setDutyCycle(0); VEYA
        feederMotor.set(0.0);
    }
  // stop
  public Command stop() {
    return runOnce(() -> feederSMC.setVelocity(RPM.of(0)));
  }
  
  public double getVelocity() {
    return feederMotor.getEncoder().getVelocity();
  }
  // ShootOnTheMove gibi komutların execute döngüsünde anlık hız verebilmesi için
  public void setSpeed(double speed) {
    feederSMC.setDutyCycle(speed);
  }
  

}
