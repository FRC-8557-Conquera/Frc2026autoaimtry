package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax; // Neo Vortex SparkFlex ile sürülse de kütüphanede aynı sınıfla çağrılabilir, takımınıza göre SparkFlex ile değiştirebilirsiniz.
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Hood;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper; // TalonFXWrapper yerine SparkWrapper!

public class HoodSubsystem extends SubsystemBase {
    // VORTEX MOTOR
    private final SparkMax hoodMotor = new SparkMax(Hood.hoodMotor, MotorType.kBrushless);
    private final DutyCycleEncoder hoodAbsoluteEncoder = new DutyCycleEncoder(Hood.encoderPort);
    
    private final SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(1.5, 0, 0)
            .withGearing(new MechanismGearing(Hood.gearRatio)) 
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new ArmFeedforward(0, 0, 0))
            .withControlMode(ControlMode.CLOSED_LOOP);
            
    // DCMotor.getNeoVortex olarak tanımlandı!
    private final SmartMotorController hoodSMC = new SparkWrapper(hoodMotor, DCMotor.getNeoVortex(1), hoodMotorConfig);

    private final ArmConfig hoodConfig = new ArmConfig(hoodSMC)
            .withStartingPosition(Degrees.of(Hood.minAngleDegrees)) 
            .withLength(Inches.of(6)).withMass(Pounds.of(1))     
            .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
            .withSoftLimits(Degrees.of(Hood.minAngleDegrees), Degrees.of(Hood.maxAngleDegrees))
            .withHardLimit(Degrees.of(Hood.minAngleDegrees - 2), Degrees.of(Hood.maxAngleDegrees + 2));
            
    private final Arm hood = new Arm(hoodConfig);

    public HoodSubsystem() {
        double currentAbsoluteAngleDeg = (hoodAbsoluteEncoder.get() * 360.0) - Hood.hoodOffsetDeg;
        double motorRotations = (currentAbsoluteAngleDeg / 360.0) * Hood.gearRatio;
        hoodMotor.getEncoder().setPosition(motorRotations);
    }

    public Angle getAngle() { return hood.getAngle(); }

    // MANUEL TEST METODU (Yavaş Dönüş)
    public Command rotateDutyCycle(double dutyCycle) {
        return run(() -> hoodSMC.setDutyCycle(dutyCycle));
    }

    public Command stop() {
        return runOnce(() -> hoodSMC.setDutyCycle(0));
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
        SmartDashboard.putNumber("Hood/ThroughBore_Raw", hoodAbsoluteEncoder.get());
        SmartDashboard.putNumber("Hood/Relative_Angle_Deg", hood.getAngle().in(Degrees));
        SmartDashboard.putBoolean("Hood/Encoder_Connected", hoodAbsoluteEncoder.isConnected());
    }
    // SİLİNEN HAYATİ METOTLAR GERİ GELDİ
    public Command setAngle(Angle angle) {
        return hood.setAngle(angle);
    }

    public Command setAngle(Supplier<Angle> angleSupplier) {
        return hood.setAngle(angleSupplier);
    }
}