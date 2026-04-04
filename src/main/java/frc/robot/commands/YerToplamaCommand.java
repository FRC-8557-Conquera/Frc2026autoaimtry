package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class YerToplamaCommand extends Command {
    private final IntakeSubsystem intake;
    private final SpindexerSubsystem spindexer;

    public YerToplamaCommand(IntakeSubsystem intake, SpindexerSubsystem spindexer) {
        this.intake = intake;
        this.spindexer = spindexer;
        // Komut çalışırken bu iki alt sistemi meşgul eder
        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize() {
        // Komut başladığı an Intake dışarı çıksın
        intake.setDeployTargetMeters(Constants.Intake.MAX_EXTENSION_METERS);
    }

    @Override
    public void execute() {
        // Intake tekerlekleri topu içeri çeksin
        intake.setRollerPower(Constants.Intake.rollerInSpeed);
        
        // Spindexer topu depolamak için dönsün (Kendi spindexer metodunuza göre güncelleyin)
        spindexer.setMotor(Constants.Spindexer.feedSpeed); 
    }

    @Override
    public void end(boolean interrupted) {
        // KOMUT BİTTİĞİNDE (Veya buton bırakıldığında) MÜKEMMEL KORUMA:
        // Intake'i içeri çek (Stow)
        intake.setDeployTargetMeters(0.0);
        // Motorları durdur
        intake.setRollerPower(0.0);
        spindexer.setMotor(0.0);
    }
}