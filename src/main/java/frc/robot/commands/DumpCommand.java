package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotIntent;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.Constants;

public class DumpCommand extends Command {
    private final SpindexerSubsystem spindexer;
    private final FeederSubsystem feeder;
    private final FlywheelSubsystem flywheel;
    private final ShooterSubsystem shooter;

    private final Timer timer = new Timer();
    private final double duration;

    public DumpCommand(SpindexerSubsystem spindexer, FeederSubsystem feeder, ShooterSubsystem shooter, double duration) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.flywheel = shooter.flywheel;
        this.shooter = shooter;
        this.duration = duration;
        // Only require spindexer/feeder — flywheel is owned by its YAMS command
        addRequirements(spindexer, feeder);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        shooter.setIntent(ShotIntent.DUMP);
        // Schedule flywheel YAMS command with live supplier
        flywheel.setVelocity(() -> shooter.getFlywheelSetpoint()).schedule();
    }

    @Override
    public void execute() {
        spindexer.setDutyCycleDirect(Constants.Spindexer.reverseSpeed);
        feeder.setDutyCycleDirect(Constants.Feeder.feedSpeed);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setIntent(ShotIntent.OFF);
        spindexer.stopDirect();
        feeder.stopDirect();
        flywheel.setVelocity(RotationsPerSecond.of(0)).schedule();
        timer.stop();
    }
}
