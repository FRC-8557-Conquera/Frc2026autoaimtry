package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    // --- KRAKEN MOTORLAR ---
    // Lider Motor (Tüm zeka ve komutlar bu motora gidecek)
    private final TalonFX deployMotorLeft = new TalonFX(Constants.Intake.intakesol, "*");
    // Takipçi Motor (Sadece Lideri kopyalayacak)
    private final TalonFX deployMotorRight = new TalonFX(Constants.Intake.intakesag, "*");

    // Ucundaki rolları çeviren Kraken motoru
    private final TalonFX rollerMotor = new TalonFX(Constants.Intake.rollerMotorID);
    
    // Intake tamamen İÇERİDE olduğunda (0 noktasında) basılacak olan Limit Switch
    private final DigitalInput stowLimitSwitch = new DigitalInput(Constants.Intake.limitSwitchPort);

    // --- KONTROL İSTEKLERİ ---
    // Motion Magic (Yumuşak hareket) sadece LİDER motora yollanır
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(false);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(false);

    // Durum Değişkenleri
    private boolean isHomed = false;
    private double targetPositionMeters = 0.0;
    private boolean isDeploying = false;

    public IntakeSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        // --- LİDER MOTOR (SOL) KONFİGÜRASYONU ---
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        
        // PID Ayarları
        deployConfig.Slot0.kP = Constants.Intake.kP;
        deployConfig.Slot0.kI = Constants.Intake.kI;
        deployConfig.Slot0.kD = Constants.Intake.kD;
        
        // Motion Magic (Yumuşak Hızlanma ve Yavaşlama)
        deployConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0; // Maksimum hız (rps)
        deployConfig.MotionMagic.MotionMagicAcceleration = 40.0;   // İvme (rps/s)
        
        // Akım Sınırlandırma (Mekaniği ve dişlileri korumak için 40 Amper ideal)
        deployConfig.CurrentLimits.SupplyCurrentLimit = 40;
        deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Ayarları Lider'e ve Takipçi'ye uygula
        deployMotorLeft.getConfigurator().apply(deployConfig);
        deployMotorRight.getConfigurator().apply(deployConfig);

        deployMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        deployMotorRight.setNeutralMode(NeutralModeValue.Brake);

        // ÖNEMLİ: SAĞ MOTOR, SOL MOTORU TAKİP ETSİN!
        // Eğer mekanikte motorlar birbirine zıt bakıyorsa (biri saat yönüne dönerken diğeri tersine dönmeliyse)
        //OPPPOOOOSE YA DA AAAAA ALLİGENDDDDD
        deployMotorRight.setControl(new Follower(deployMotorLeft.getDeviceID(),MotorAlignmentValue.Opposed));
        // --- ROLLER MOTOR KONFİGÜRASYONU ---
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 30;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        rollerMotor.getConfigurator().apply(rollerConfig);
        rollerMotor.setNeutralMode(NeutralModeValue.Coast); // Roller serbest durabilir
    }

    @Override
    public void periodic() {
        // Pozisyonu sadece Lider motordan okuyoruz
        double currentRotations = deployMotorLeft.getPosition().getValueAsDouble();
        double currentPositionMeters = currentRotations * Constants.Intake.METERS_PER_ROTATION;
        
        // Limit Switch Mantığı (Basılıyken false veriyorsa ! kullanırız, true veriyorsa ! işaretini kaldırın)
        boolean isStowed = !stowLimitSwitch.get(); 

        if (isStowed) {
            if (!isHomed) {
                // Limit switch'e değdik, intake tamamen kapalı! 
                // Motorların pozisyonunu 0 dereceye (0 tura) sabitliyoruz.
                deployMotorLeft.setPosition(0.0);
                isHomed = true;
                System.out.println("Intake tamamen kapandi ve SIFIRLANDI!");
            }
            
            // Eğer sıfırdaysak ve hedefimiz de sıfırsa motoru durdur ki zorlanmasın
            if(targetPositionMeters <= 0.01) {
                deployMotorLeft.setControl(dutyCycleRequest.withOutput(0.0));
                isDeploying = false;
            }
        }

        // Pozisyon Kontrolü (Sadece sıfırlama yapıldıysa çalışır)
        if (isDeploying && isHomed) {
            // Hedefi sınırlar arasında tut (En fazla 0.40 metre açılabilir)
            double clampedTargetMeters = MathUtil.clamp(targetPositionMeters, 0.0, Constants.Intake.MAX_EXTENSION_METERS);
            
            // Hedef metreyi motor turuna çeviriyoruz
            double targetRotations = clampedTargetMeters / Constants.Intake.METERS_PER_ROTATION;
            
            // Lider motoru hedefe gönder (Takipçi motor otomatik olarak onunla aynı hareketi yapacak)
            deployMotorLeft.setControl(positionRequest.withPosition(targetRotations));
        }

        SmartDashboard.putNumber("Intake/Mevcut_Metre", currentPositionMeters);
        SmartDashboard.putNumber("Intake/Hedef_Metre", targetPositionMeters);
        SmartDashboard.putBoolean("Intake/LimitSwitch", isStowed);
        SmartDashboard.putBoolean("Intake/Homed", isHomed);
    }

    // --- AÇMA/KAPAMA KOMUTLARI ---

    public Command setPosition(double positionMeters) {
        return runOnce(() -> {
            targetPositionMeters = positionMeters;
            isDeploying = true;
        }).withName("SetIntakePosition");
    }

    // Intake tamamen kapalı konuma (0 noktasına) döner
    public Command stow() {
        return setPosition(0.0);
    }

    // Intake tamamen açık konuma (örneğin 0.40 metreye) gider
    public Command deployFull() {
        return setPosition(Constants.Intake.MAX_EXTENSION_METERS);
    }

    // --- ROLLER KOMUTLARI ---
    
    public Command intakeFuel() {
        return run(() -> rollerMotor.setControl(dutyCycleRequest.withOutput(Constants.Intake.rollerInSpeed)))
               .withName("IntakeFuel");
    }

    public Command stopRollers() {
        return runOnce(() -> rollerMotor.setControl(dutyCycleRequest.withOutput(0.0)))
               .withName("StopRollers");
    }

    // --- SIFIRLAMA (HOMING) KOMUTU ---
    // Bu komut çalışınca intake limit switch'e değene kadar geriye (içeri) gider.
    public Command homeIntake() {
        return run(() -> {
            isDeploying = false;
            isHomed = false;
            // Yavaşça içeri (negatif güç) çek. Takipçi motor otomatik eşlik eder.
            deployMotorLeft.setControl(dutyCycleRequest.withOutput(-0.15)); 
        })
        .until(() -> !stowLimitSwitch.get()) // Switch'e basılana kadar devam
        .andThen(runOnce(() -> deployMotorLeft.setControl(dutyCycleRequest.withOutput(0.0)))) // Basılınca gücü kes
        .withName("HomeIntake");
    }

    // --- DİĞER SUBSYSTEM'LER İÇİN MANUEL METOTLAR ---
    public void setDeployTargetMeters(double meters) {
        this.targetPositionMeters = meters;
        this.isDeploying = true;
    }

    public void setRollerPower(double power) {
        rollerMotor.setControl(dutyCycleRequest.withOutput(power));
    }
}