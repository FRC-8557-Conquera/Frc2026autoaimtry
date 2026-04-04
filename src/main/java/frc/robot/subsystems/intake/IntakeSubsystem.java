package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    // Doğrusal hareketi sağlayan Kraken motoru
    private final TalonFX deployMotor = new TalonFX(Constants.Intake.deployMotorID);
    
    // Ucundaki rolları çeviren Kraken motoru
    private final TalonFX rollerMotor = new TalonFX(Constants.Intake.rollerMotorID);
    
    // Intake tamamen İÇERİDE olduğunda basılacak olan Limit Switch
    private final DigitalInput stowLimitSwitch = new DigitalInput(Constants.Intake.limitSwitchPort);

    // TalonFX Kontrol İstekleri (Control Requests)
    // Pozisyon kontrolü için (Profiled PID etkisini Motion Magic ile sağlayabiliriz)
    private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(false); 
    // Yüzdelik güç vermek için (Manuel Homing ve Roller için)
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(false);

    // Durum Değişkenleri
    private boolean isHomed = false;
    private double targetPositionMeters = 0.0;
    private boolean isDeploying = false;

    public IntakeSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        // --- Deploy Motor Konfigürasyonu ---
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        
        // PID Ayarları
        deployConfig.Slot0.kP = Constants.Intake.kP;
        deployConfig.Slot0.kI = Constants.Intake.kI;
        deployConfig.Slot0.kD = Constants.Intake.kD;
        
        // Akım Sınırlandırma (Mekaniği korumak için)
        deployConfig.CurrentLimits.SupplyCurrentLimit = 40;
        deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        deployMotor.getConfigurator().apply(deployConfig);
        deployMotor.setNeutralMode(NeutralModeValue.Brake);

        // --- Roller Motor Konfigürasyonu ---
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 30;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        rollerMotor.getConfigurator().apply(rollerConfig);
        rollerMotor.setNeutralMode(NeutralModeValue.Coast); // Roller serbest durabilir
    }

    @Override
    public void periodic() {
        // Kraken'in dönüş (rotation) sayısını alıp metreye çeviriyoruz
        double currentRotations = deployMotor.getPosition().getValueAsDouble();
        double currentPositionMeters = currentRotations * Constants.Intake.METERS_PER_ROTATION;
        
        // Limit Switch Mantığı
        boolean isStowed = !stowLimitSwitch.get(); 

        if (isStowed) {
            if (!isHomed) {
                // Limit switch'e değdik, burası SIFIR noktamız.
                deployMotor.setPosition(0.0);
                isHomed = true;
                System.out.println("Intake SIFIRLANDI!");
            }
            
            // Eğer sıfırdaysak ve hedefimiz de sıfırsa motoru durdur
            if(targetPositionMeters <= 0.01) {
                deployMotor.setControl(dutyCycleRequest.withOutput(0.0));
                isDeploying = false;
            }
        }

        // Pozisyon Kontrolü
        if (isDeploying && isHomed) {
            double clampedTargetMeters = MathUtil.clamp(targetPositionMeters, 0.0, Constants.Intake.MAX_EXTENSION_METERS);
            
            // Hedef metreyi, motorun atması gereken dönüş sayısına (rotations) geri çeviriyoruz
            double targetRotations = clampedTargetMeters / Constants.Intake.METERS_PER_ROTATION;
            
            // Motoru hedef pozisyona gönder
            deployMotor.setControl(positionRequest.withPosition(targetRotations));
        }

        SmartDashboard.putNumber("Intake/Mevcut_Metre", currentPositionMeters);
        SmartDashboard.putNumber("Intake/Hedef_Metre", targetPositionMeters);
        SmartDashboard.putBoolean("Intake/LimitSwitch", isStowed);
        SmartDashboard.putBoolean("Intake/Homed", isHomed);
    }

    // --- AÇMA/KAPAMA KOMUTLARI (OTONOMA UYGUN) ---

    public Command setPosition(double positionMeters) {
        return runOnce(() -> { // run() yerine runOnce() KULLANILMALI!
            targetPositionMeters = positionMeters;
            isDeploying = true;
        }).withName("SetIntakePosition");
    }

    public Command stow() {
        return setPosition(0.0);
    }

    public Command deployFull() {
        return setPosition(Constants.Intake.MAX_EXTENSION_METERS);
    }

    // --- ROLLER KOMUTLARI ---
    // Bunlar sürekli çalışacağı için run() kalabilir
    public Command intakeFuel() {
        return run(() -> rollerMotor.setControl(dutyCycleRequest.withOutput(Constants.Intake.rollerInSpeed)))
               .withName("IntakeFuel");
    }

    public Command stopRollers() {
        return runOnce(() -> rollerMotor.setControl(dutyCycleRequest.withOutput(0.0)))
               .withName("StopRollers");
    }

    // --- SIFIRLAMA KOMUTU ---
    public Command homeIntake() {
        return run(() -> {
            isDeploying = false;
            isHomed = false;
            // Yavaşça içeri (negatif güç) çek
            deployMotor.setControl(dutyCycleRequest.withOutput(-0.15)); 
        })
        .until(() -> !stowLimitSwitch.get())
        .andThen(runOnce(() -> deployMotor.setControl(dutyCycleRequest.withOutput(0.0))))
        .withName("HomeIntake");
    }
    // --- KOMUT SINIFLARI İÇİN DOĞRUDAN KONTROL METOTLARI ---
    
    // Intake'in hedef metresini ayarlar
    public void setDeployTargetMeters(double meters) {
        this.targetPositionMeters = meters;
        this.isDeploying = true;
    }

    // Roller motoruna doğrudan yüzdelik güç (-1.0 ile 1.0 arası) verir
    public void setRollerPower(double power) {
        rollerMotor.setControl(dutyCycleRequest.withOutput(power));
    }
}