package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Hood;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import static edu.wpi.first.units.Units.*;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX hoodMotor = new TalonFX(Hood.hoodMotor);
    private final SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)

            //.withClosedLoopController(0, 0, 0, RotationsPerSecond.of(100), RotationsPerSecondPerSecond.of(2500))  // TODO: Change the PID values
            .withClosedLoopController(1.5, 0, 0)
            .withGearing(new MechanismGearing(12.15)) // 243 / 20 = 12.15
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new ArmFeedforward(0, 0, 0))
            .withSimFeedforward(new ArmFeedforward(0, 0, 0))
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(1.5, 0, 0);
            

    private final SmartMotorController hoodSMC = new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX60(1), hoodMotorConfig);

    private final ArmConfig hoodConfig = new ArmConfig(hoodSMC)
            // DÜZELTME: Başlangıç noktası Offset (73.2) değil, mekaniğin dinlenme noktası (20) olmalıdır!
            .withStartingPosition(Degrees.of(Hood.minAngleDegrees)) 
            .withLength(Inches.of(6)).withMass(Pounds.of(1))     
            .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
            .withSoftLimits(Degrees.of(Hood.minAngleDegrees), Degrees.of(Hood.maxAngleDegrees))
            .withHardLimit(Degrees.of(Hood.minAngleDegrees - 2), Degrees.of(Hood.maxAngleDegrees + 2));
            
    private final Arm hood = new Arm(hoodConfig);

    public HoodSubsystem() {

    }

    public Command setAngle(Angle angle) {
        return hood.setAngle(angle);
    }

    public Command setAngle(Supplier<Angle> angleSupplier) {
        return hood.setAngle(angleSupplier);
    }

    public Angle getAngle() {
        return hood.getAngle();
    }

    public Command sysId() {
        return hood.sysId(
                Volts.of(4.0), // maximumVoltage
                Volts.per(Second).of(0.5), // step
                Seconds.of(8.0) // duration
        );
    }

    public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
        return hood.set(dutyCycleSupplier);
    }

    public Command setDutyCycle(double dutyCycle) {
        return hood.set(dutyCycle);
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}