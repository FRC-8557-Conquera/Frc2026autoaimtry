package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.*;

public class ShootCommand extends Command {
    private final SpindexerSubsystem spindexer;
    private final Timer timer = new Timer();
    private final double duration;

    public ShootCommand(SpindexerSubsystem spindexer, double duration) {
        this.spindexer = spindexer;
        this.duration = duration;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        spindexer.spinForward();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stop();
        timer.stop();
    }
}