package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.*;
import frc.robot.Constants;
import frc.robot.Constants.Feeder;
import frc.robot.Constants.Spindexer;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotIntent;

public class CombinedFeed extends Command {

    private final SpindexerSubsystem spindexer;
    private final FeederSubsystem feeder;
    private final boolean dir; // 1 = ileri

    public CombinedFeed(SpindexerSubsystem spindexer, FeederSubsystem feeder, boolean dir) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.dir = dir;
        addRequirements(spindexer, feeder);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        feeder.setDutyCycleDirect(Feeder.feedSpeed * (dir ? 1 : -1));
        spindexer.setDutyCycleDirect(Spindexer.spindexerspeed * (dir ? 1 : -1));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stopDirect();
        spindexer.stopDirect();
    }
}