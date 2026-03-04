package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.*;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotIntent;

public class IntakeAlCommand extends Command {

    private final IntakeSubsystem intake;
    private final Timer timer = new Timer();
    private final double duration;

    public IntakeAlCommand(IntakeSubsystem intake, double duration) {
        this.intake = intake;
        this.duration = duration;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        intake.rollerIn().schedule();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        intake.rollerStop();
        timer.stop();
    }
}