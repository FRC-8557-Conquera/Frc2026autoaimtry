package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX deployMotorLeft = new TalonFX(Constants.Intake.intakesol);
    private final TalonFX deployMotorRight = new TalonFX(Constants.Intake.intakesag);
    private final TalonFX rollerMotor = new TalonFX(Constants.Intake.rollerMotorID);
    
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(false);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(false);

    // Başlangıçta hedefimiz 0 tur (Yani tam kapalı pozisyon)
    private double targetPositionRotations = 0.0;

    public IntakeSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        deployConfig.CurrentLimits.SupplyCurrentLimit = 40;
        deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        deployMotorLeft.getConfigurator().apply(deployConfig);
        deployMotorRight.getConfigurator().apply(deployConfig);

        // Mükemmel frenleme için Brake modu şart
        deployMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        deployMotorRight.setNeutralMode(NeutralModeValue.Brake);

        deployMotorRight.setControl(new Follower(deployMotorLeft.getDeviceID(),MotorAlignmentValue.Opposed));

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 30;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        rollerMotor.getConfigurator().apply(rollerConfig);
        rollerMotor.setNeutralMode(NeutralModeValue.Coast); 
    }

    @Override
    public void periodic() {
        /* double currentRotations = deployMotorLeft.getPosition().getValueAsDouble();

        // Hedefi güvenlik sınırları içine al (0.0 ile MAX arasında tut, mekaniği kırma)
        double clampedRotations = MathUtil.clamp(targetPositionRotations, 0.0, Constants.Intake.MAX_EXTENSION_ROTATIONS);
        
        // Motion Magic ile Kraken'i hedefe yolla. 
        // Sen hedefi 0 yaptığında motor 0 noktasına zımk diye gidip Brake ile tutunacak.
        deployMotorLeft.setControl(positionRequest.withPosition(clampedRotations)); */

        //SmartDashboard.putNumber("Intake/Mevcut_Tur", currentRotations);
        //SmartDashboard.putNumber("Intake/Hedef_Tur", targetPositionRotations);
    }

    // --- KOMUTLAR İÇİN KULLANILACAK METOTLAR ---
    
    /*public void setDeployTargetRotations(double rotations) {
        this.targetPositionRotations = rotations;
    }*/

    public void setRollerPower(double power) {
        rollerMotor.setControl(dutyCycleRequest.withOutput(power));
    }

    public void set(double duty) {
        deployMotorLeft.setControl(dutyCycleRequest.withOutput(duty));
    } 
    
}

        