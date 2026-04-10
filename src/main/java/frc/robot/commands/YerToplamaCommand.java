package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Feeder;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class YerToplamaCommand extends Command {
    private final IntakeSubsystem intake;
    private final SpindexerSubsystem spindexer;
    private final FeederSubsystem feeder;

    public YerToplamaCommand(IntakeSubsystem intake, SpindexerSubsystem spindexer, FeederSubsystem feeder) {
        
        this.intake = intake;
        this.spindexer = spindexer;
        this.feeder = feeder;
        // Komut çalışırken bu iki alt sistemi meşgul eder
        addRequirements(intake, spindexer,feeder);
    }

    @Override
    public void initialize() {
        if(!intake.isFullyDeployed()) intake.toggleIntake();
    }

    @Override
    public void execute() {
        // Intake tekerlekleri topu içeri çeksin
        intake.setRollerPower(Constants.Intake.rollerInSpeed);
        feeder.setMotor(Constants.Feeder.feedSpeed);
        // Spindexer topu depolamak için dönsün (Kendi spindexer metodunuza göre güncelleyin)
        spindexer.setMotor(Constants.Spindexer.spindexerspeed); 
    }

    @Override
    public void end(boolean interrupted) {
        // KOMUT BİTTİĞİNDE (Veya buton bırakıldığında) MÜKEMMEL KORUMA:
        // Motorları durdur
        intake.setRollerPower(0.0);
        spindexer.setMotor(0.0);
        feeder.setMotor(0.0);
    }
}