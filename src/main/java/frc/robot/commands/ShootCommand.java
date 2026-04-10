package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.Constants;

public class ShootCommand extends Command {
    private final SpindexerSubsystem spindexer;
    private final FeederSubsystem feeder;
    private final FlywheelSubsystem flywheel;

    public ShootCommand(SpindexerSubsystem spindexer, FeederSubsystem feeder, ShooterSubsystem shooter, double targetRPS) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.flywheel = shooter.flywheel;
        // Only require spindexer — flywheel and feeder are owned by their YAMS commands
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        // Schedule YAMS commands once — Supplier keeps them live
        flywheel.setVelocity(() -> RotationsPerSecond.of(SmartDashboard.getNumber("FlywheelSpeed", 40.0))).schedule();
        feeder.feed().schedule();
    }

    @Override
    public void execute() {
        spindexer.setMotor(Constants.Spindexer.spindexerspeed);
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stopMotor();
        flywheel.setVelocity(RotationsPerSecond.of(0)).schedule();
        feeder.stop().schedule();
    }
}
