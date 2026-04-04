package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotIntent;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class ToplaVeAtesleCommand extends Command {
    private final IntakeSubsystem intake;
    private final SpindexerSubsystem spindexer;
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;

    public ToplaVeAtesleCommand(IntakeSubsystem intake, SpindexerSubsystem spindexer, 
                                ShooterSubsystem shooter, FeederSubsystem feeder) {
        this.intake = intake;
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements(intake, spindexer, shooter, feeder);
    }

    @Override
    public void initialize() {
        // 1. Intake'i aç
        intake.setDeployTargetMeters(Constants.Intake.MAX_EXTENSION_METERS);
        
        // 2. Shooter'ı hedefe (HUB veya SOTM) kilitlenmesi için uyandır
        shooter.setIntent(ShotIntent.HUB); 
    }

    @Override
    public void execute() {
        // Toplama motorları sürekli çalışsın
        intake.setRollerPower(Constants.Intake.rollerInSpeed);
        spindexer.setMotor(Constants.Spindexer.feedSpeed);

        // EN KRİTİK NOKTA: Shooter hedefe oturdu mu?
        if (shooter.isReadyToShoot()) {
            // Eğer taret ve hız mükemmelse, Feeder'ı çalıştır ve topu mermiye sür!
            feeder.setMotor(Constants.Feeder.feedSpeed);
        } else {
            // Henüz hedefe kilitlenmediyse veya şapka kalkmadıysa Feeder beklesin
            feeder.setMotor(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // KOMUT İPTAL EDİLDİĞİNDE (Buton bırakıldığında) SİSTEMİ KAPAT
        intake.setDeployTargetMeters(0.0); // Intake içeri girer
        intake.setRollerPower(0.0);
        spindexer.setMotor(0.0);
        feeder.setMotor(0.0);
        shooter.setIntent(ShotIntent.OFF); // Taret ve motorlar dinlenmeye geçer
    }
}