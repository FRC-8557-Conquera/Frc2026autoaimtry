package frc.robot.subsystems.spindexer;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Spindexer;

public class SpindexerSubsystem extends SubsystemBase {

  private final SparkMax spindexerMotor = new SparkMax(Spindexer.spindexerMotor, MotorType.kBrushless);

  public SpindexerSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(60);
    spindexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  

  public Command spinForward() {
    return run(() -> spindexerMotor.set(Spindexer.spindexerspeed));
  }

  public Command spinReverse() {
    return run(() -> spindexerMotor.set(Spindexer.reverseSpeed));
  }

  public Command stop() {
    return runOnce(() -> spindexerMotor.set(0));
  }
  public void setMotor(){
    spindexerMotor.set(Spindexer.spindexerspeed);
  }
  public void stopMotor(){
    spindexerMotor.set(0);
  }
  // Komut sınıflarından motoru doğrudan yüzdelik güç ile çalıştırmak için
  public void setMotor(double speed) {
      // Eğer motorun adı feederMotor ise (SparkMax veya TalonSRX varsayımıyla):
    spindexerMotor.set(speed); 
      // NOT: Eğer Feeder motorun Kraken (TalonFX) ise yukarıdaki yerine şunu kullanmalısın:
      // feederMotor.setControl(new com.ctre.phoenix6.controls.DutyCycleOut(speed));
  }
}

