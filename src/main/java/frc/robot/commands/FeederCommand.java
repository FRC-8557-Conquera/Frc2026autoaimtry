package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class FeederCommand extends Command {

    private final FeederSubsystem feeder;
    private final boolean dir;

    public FeederCommand(FeederSubsystem feeder, boolean dir) {
        this.feeder = feeder;
        this.dir = dir;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        // Any initialization code if needed
    }

    @Override
    public void execute() {
        if(dir) {
            feeder.feed();
        } else {
            feeder.reverse();
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop(); 
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}