package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.Constants;

public class DebugShootCommand extends Command {
    private final SpindexerSubsystem spindexer;
    private final FeederSubsystem feeder;
    private final FlywheelSubsystem flywheel;
    private final DoubleSupplier targetRPS;

    public DebugShootCommand(SpindexerSubsystem spindexer, FeederSubsystem feeder, ShooterSubsystem shooter, DoubleSupplier targetRPS) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.flywheel = shooter.flywheel;
        this.targetRPS = targetRPS;
        // Only require spindexer/feeder — flywheel is owned by its YAMS command
        addRequirements(spindexer, feeder);
    }

    @Override
    public void initialize() {
        // Schedule flywheel YAMS command with live supplier
        flywheel.setVelocity(() -> RotationsPerSecond.of(targetRPS.getAsDouble())).schedule();
    }

    @Override
    public void execute() {
        spindexer.setMotor(Constants.Spindexer.spindexerspeed);
        feeder.setMotor(Constants.Feeder.feedSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stopMotor();
        feeder.stopDirect();
        flywheel.setVelocity(RotationsPerSecond.of(0)).schedule();
    }
}
