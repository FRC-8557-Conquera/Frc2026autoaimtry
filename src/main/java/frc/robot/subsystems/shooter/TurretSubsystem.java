package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.security.Key;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretSubsystem extends SubsystemBase {
        private boolean turretZeroed = false;
        private double startTime = 0;
        private final DutyCycleEncoder turretThroughBoreEncoder = new DutyCycleEncoder(Constants.Turret.encoderPort);

        private final SparkMax turretMotor = new SparkMax(Constants.Turret.turretMotor, MotorType.kBrushless);
        private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
                        .withClosedLoopController(1.5, 0, 0)// TODO: Change the PID values
                        .withGearing(new MechanismGearing(48))
                        .withIdleMode(MotorMode.BRAKE)
                        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                        .withStatorCurrentLimit(Amps.of(60))
                        .withMotorInverted(true)
                        .withClosedLoopRampRate(Seconds.of(0.25))
                        .withOpenLoopRampRate(Seconds.of(0.25))
                        .withSoftLimit(Rotations.of(-0.5), Rotations.of(0.5))
                        .withFeedforward(new SimpleMotorFeedforward(0.0, 0.35, 0.05))
                        .withControlMode(ControlMode.CLOSED_LOOP);
                        
        private final SmartMotorController turretSMC = new SparkWrapper(turretMotor,
                        DCMotor.getNEO(1),
                        motorConfig);
        private final PivotConfig turretConfig = new PivotConfig(turretSMC)
                        .withMOI(Meters.of(0.24), Pounds.of(2))
                        .withStartingPosition(Rotations.of(0))
                        .withTelemetry("TurretMech", TelemetryVerbosity.HIGH) // Telemetry
                        .withHardLimit(Rotations.of(-0.6), Rotations.of(0.6));

        private final Pivot turret = new Pivot(turretConfig);

        public TurretSubsystem() {
        
        }

        public Command setAngle(Angle angle) {
                Angle offsetAngle = angle.minus(Rotations.of(0.25));
                Angle wrapped = Rotations.of(MathUtil.inputModulus(offsetAngle.in(Rotations), -0.5, 0.5));
                return turret.setAngle(wrapped);
        }

        public Command setAngle(Supplier<Angle> angleSupplier) {
                Supplier<Angle> wrapped = () -> Rotations.of(MathUtil.inputModulus(angleSupplier.get().minus(Rotations.of(0.25)).in(Rotations), -0.5, 0.5));
                return turret.setAngle(wrapped);
        }
        public void setAngleDirect(Angle angle){
                turretSMC.setPosition(angle);
        }
        public Angle getAngle() {
                return turret.getAngle();
        }

        private Angle targetAngle = Degrees.of(0);

        public void setTargetAngle(Angle angle) {
                targetAngle = angle;
        }

        public Angle getTargetAngle() {
                return targetAngle;
        }

        private double getAbsoluteAngle() {
                double raw = ((turretThroughBoreEncoder.get() - Constants.Turret.encoderOffset) % 1 + 0.5) % 1 - 0.5;
                return raw;
        }


        public Command rotateDutyCycle(double dutyCycle) {
                return run(() -> turretSMC.setDutyCycle(dutyCycle));
        }

        public Command stop() {
                return runOnce(() -> turretSMC.setDutyCycle(0));
        }

        @Override
        public void periodic() {
                turret.updateTelemetry();
                SmartDashboard.putNumber("TurretEncoder", turretThroughBoreEncoder.get());
                SmartDashboard.putNumber("TurretRaw", getAbsoluteAngle());
                SmartDashboard.putNumber("TurretRelative", turret.getAngle().in(Rotations));
                SmartDashboard.putBoolean("TurretConnected", turretThroughBoreEncoder.isConnected());
                SmartDashboard.putNumber("TurretAngle", turret.getMechanismSetpoint().isPresent() ? turret.getMechanismSetpoint().get().in(Degrees) : 0);


                if (!turretZeroed) {
                        if (startTime == 0) {
                                startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                        } else if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime > 2.0) {
                                turretMotor.getEncoder().setPosition(getAbsoluteAngle() / 4);
                                turretZeroed = true;
                        }
                }
        }

        @Override
        public void simulationPeriodic() {
                turret.simIterate();
        }

}