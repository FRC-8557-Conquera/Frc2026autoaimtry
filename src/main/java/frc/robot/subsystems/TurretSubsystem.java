package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class TurretSubsystem extends SubsystemBase
{
  private final SparkMax turretMotor      = new SparkMax(Constants.Turret.turretMotor,MotorType.kBrushless);
  private final SmartMotorControllerConfig motorConfig      = new SmartMotorControllerConfig(this)

      .withClosedLoopController(0, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90)) //TODO: Change the PID values
      .withSoftLimit(Degrees.of(-30), Degrees.of(100))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))     //TODO: Set the correct gear ratio
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController       turretSMC            = new SparkWrapper(turretMotor,
                                                                                  DCMotor.getNEO(1),
                                                                                  motorConfig);
  private final PivotConfig turretConfig = new PivotConfig(turretSMC)
                          .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
                          .withWrapping(Degrees.of(-180), Degrees.of(180))
                          .withHardLimit(Degrees.of(-180), Degrees.of(180))
                          .withTelemetry("TurretMech", TelemetryVerbosity.HIGH); // Telemetry

        private final Pivot turret = new Pivot(turretConfig);

        public TurretSubsystem() {
        }

        public Command setAngle(Angle angle) {
                return turret.setAngle(angle);
        }

        public void setAngleDirect(Angle angle) {
              turretSMC.setPosition(angle);
        }

        public Command setAngle(Supplier<Angle> angleSupplier) {
                return turret.setAngle(angleSupplier);
        }

        public Angle getAngle() {
                return turret.getAngle();
        }

        public Command sysId() {
                return turret.sysId(
                                Volts.of(4.0), // maximumVoltage
                                Volts.per(Second).of(0.5), // step
                                Seconds.of(8.0) // duration
                );
        }

        @Override
        public void periodic() {
                turret.updateTelemetry();
        }

        @Override
        public void simulationPeriodic() {
                turret.simIterate();
        }
}