package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class TurretSubsystem extends SubsystemBase {
    private boolean turretZeroed = false;
    private double startTime = 0;
    private final DutyCycleEncoder turretThroughBoreEncoder = new DutyCycleEncoder(Constants.Turret.encoderPort);
    private final SparkMax turretMotor = new SparkMax(Constants.Turret.turretMotor, MotorType.kBrushless);
    
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(1.7, 0, 0.01)
            .withGearing(new MechanismGearing(Constants.Turret.gearRatio)) 
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("TurretMotor", TelemetryVerbosity.LOW)
            .withStatorCurrentLimit(Amps.of(40)) 
            .withMotorInverted(true)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withSoftLimit(Rotations.of(-0.325), Rotations.of(0.350)) 
            // HATA DÜZELTİLDİ: Yatay dönen taret için SimpleMotorFeedforward kullanıldı!
            .withFeedforward(new SimpleMotorFeedforward(0.0, 0.0, 0.0)) 
            .withControlMode(ControlMode.CLOSED_LOOP);
                        
    private final SmartMotorController turretSMC = new SparkWrapper(turretMotor, DCMotor.getNEO(1), motorConfig);
    
    private final PivotConfig turretConfig = new PivotConfig(turretSMC)
            .withMOI(Meters.of(0.24), Pounds.of(2))
            .withStartingPosition(Rotations.of(0))
            .withTelemetry("TurretMech", TelemetryVerbosity.LOW)
            .withHardLimit(Rotations.of(-0.5), Rotations.of(0.350));

    private final Pivot turret = new Pivot(turretConfig);

    public TurretSubsystem() {}

    public void setAngleDirect(Angle angle) {
        Angle offsetAngle = angle.minus(edu.wpi.first.units.Units.Rotations.of(0.25));
        Angle wrapped = edu.wpi.first.units.Units.Rotations.of(MathUtil.inputModulus(offsetAngle.in(edu.wpi.first.units.Units.Rotations), -0.5, 0.5));
        turretSMC.setPosition(wrapped);
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
    
    public Angle getAngle() { return turret.getAngle(); }

    private double getAbsoluteAngle() {
        double raw = ((turretThroughBoreEncoder.get() - Constants.Turret.encoderOffset) % 1.0 + 0.5) % 1.0 - 0.5;
        return raw;
    }

    public Command sysId() {
        return turret.sysId(Volts.of(7), Volts.of(0.5).per(Second), Seconds.of(8));
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
        SmartDashboard.putNumber("Turret/ThroughBore_Raw", turretThroughBoreEncoder.get());
        SmartDashboard.putNumber("Turret/Absolute_Rotations", getAbsoluteAngle());
        SmartDashboard.putNumber("Turret/Relative_Angle_Deg", turret.getAngle().in(Degrees));
        SmartDashboard.putBoolean("Turret/Encoder_Connected", turretThroughBoreEncoder.isConnected());

    }
}