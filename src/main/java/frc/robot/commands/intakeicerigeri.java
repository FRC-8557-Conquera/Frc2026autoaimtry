package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class intakeicerigeri extends Command {
    private final IntakeSubsystem intake;

    public intakeicerigeri(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Tuşa basıldığı an: Intake'i MAKSİMUM AÇIK pozisyona (Rotations olarak) fırlat!
    }

    @Override
    public void execute() {
        // Tuşa basılı tutulduğu sürece topu içeri çekecek rolları çalıştır
        intake.set(0.3);
    }

    @Override
    public void end(boolean interrupted) {
        // Tuş bırakıldığında veya komut kesildiğinde: SİSTEMİ KORUMAYA AL

        // 2. Rolları durdur
        intake.set(0.0);
    }
}