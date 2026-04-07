package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX deployMotorLeft = new TalonFX(Constants.Intake.intakesol);
    private final TalonFX deployMotorRight = new TalonFX(Constants.Intake.intakesag);
    private final TalonFX rollerMotorRaw = new TalonFX(Constants.Intake.rollerMotorID);

    @SuppressWarnings("unchecked")
    private final SmartMotorControllerConfig deployConfig = new SmartMotorControllerConfig(this)
            .withGearing(new MechanismGearing(1))
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("IntakeDeployMotor", TelemetryVerbosity.HIGH)
            .withSupplyCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withOpenLoopRampRate(Seconds.of(0))
            .withControlMode(ControlMode.OPEN_LOOP)
            .withFollowers(Pair.of(deployMotorRight, true));

    private final SmartMotorController deploySMC = new TalonFXWrapper(deployMotorLeft, DCMotor.getKrakenX60(1), deployConfig);

    private final SmartMotorControllerConfig rollerConfig = new SmartMotorControllerConfig(this)
            .withGearing(new MechanismGearing(1))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
            .withSupplyCurrentLimit(Amps.of(30))
            .withMotorInverted(false)
            .withOpenLoopRampRate(Seconds.of(0))
            .withControlMode(ControlMode.OPEN_LOOP);

    private final SmartMotorController rollerSMC = new TalonFXWrapper(rollerMotorRaw, DCMotor.getKrakenX60(1), rollerConfig);

    public IntakeSubsystem() {
    }

    public void setRollerPower(double power) {
        rollerSMC.setDutyCycle(power);
    }

    public void set(double duty) {
        deploySMC.setDutyCycle(duty);
    }

    @Override
    public void periodic() {
        deploySMC.updateTelemetry();
        rollerSMC.updateTelemetry();
    }
}
