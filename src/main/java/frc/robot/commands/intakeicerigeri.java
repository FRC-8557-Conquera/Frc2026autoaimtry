package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class intakeicerigeri extends Command {
    private final IntakeSubsystem intake;

    public intakeicerigeri(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Tuşa basınca tam açılma değerine (-2.628) fırla!
        intake.setDeployTargetRotations(Constants.Intake.ACIK_POZISYON);
    }

    @Override
    public void execute() {
      //  intake.setRollerPower(Constants.Intake.rollerInSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Tuşu bırakınca geri çekil ve -0.796'da zımba gibi dur!
        intake.setDeployTargetRotations(Constants.Intake.KAPALI_POZISYON); 
        intake.setRollerPower(0.0);
    }
}