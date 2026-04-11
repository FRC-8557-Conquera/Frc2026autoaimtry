package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX deployMotorLeft = new TalonFX(Constants.Intake.intakesol, "Penci Zorno Canivore");
    private final TalonFX deployMotorRight = new TalonFX(Constants.Intake.intakesag, "Penci Zorno Canivore");
    private final TalonFX rollerMotor = new TalonFX(Constants.Intake.rollerMotorID);
    
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    //private final DutyCycleOut rollerDutyCycle = new DutyCycleOut(0);

    // Başlangıçta mekanik en güvenli yerinde dursun
    private double targetPositionRotations = Constants.Intake.KAPALI_POZISYON;

    public void toggleIntake() {
        if (targetPositionRotations == Constants.Intake.KAPALI_POZISYON) {
            targetPositionRotations = Constants.Intake.ACIK_POZISYON;
        } else {
            targetPositionRotations = Constants.Intake.KAPALI_POZISYON;
        }
    }

    // YENİ: Roller'ın çalışması güvenli mi? (En aşağıda mı?)
    public boolean isFullyDeployed() {
        // Mevcut pozisyon, hedef olan -2.628'e çok yakınsa (0.1 tolerans ile) true döndür
        return Math.abs(deployMotorLeft.getPosition().getValueAsDouble() - Constants.Intake.ACIK_POZISYON) < 0.1;
    }

    public IntakeSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.Slot0.kP = Constants.Intake.kP;
        config.Slot0.kI = Constants.Intake.kI;
        config.Slot0.kD = Constants.Intake.kD;
        
        config.MotionMagic.MotionMagicCruiseVelocity = 20.0; 
        config.MotionMagic.MotionMagicAcceleration = 40.0;   

        // --- YAZILIMSAL LİMİT (SOFTWARE LIMITS) ---
        // Matematiksel olarak -0.796, -2.628'den daha büyük bir sayıdır.
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Intake.KAPALI_POZISYON;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Intake.ACIK_POZISYON;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        deployMotorLeft.getConfigurator().apply(config);
        deployMotorRight.getConfigurator().apply(config);

        deployMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        deployMotorRight.setNeutralMode(NeutralModeValue.Brake);

        // Takipçi ayarı (Zıtsa Opposed yaparsın cano)
        deployMotorRight.setControl(new Follower(deployMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));

        rollerMotor.setNeutralMode(NeutralModeValue.Coast); 
    }

    @Override
    public void periodic() {
        deployMotorLeft.setControl(positionRequest.withPosition(targetPositionRotations));

        SmartDashboard.putNumber("Intake/Mevcut_Pozisyon", deployMotorLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Hedef", targetPositionRotations);
    }

    public void setDeployTargetRotations(double rotations) {
        this.targetPositionRotations = rotations;
    }

    
    public void setRollerPower(double power) {
        // Güvenlik: Eğer tam inmemişse ve biz güç vermeye çalışıyorsak gücü kes!
        // (Sadece içe alırken bu güvenlik çalışsın, dışa atarken her türlü dönsün diyebilirsin)
        if (power > 0 && !isFullyDeployed()) {
            rollerMotor.set(0);
        } else {
            rollerMotor.set(power);
        }
    }
}