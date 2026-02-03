package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;

public class AutoShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final HoodSubsystem hood;
    private final FlywheelSubsystem flywheel;

    public AutoShootCommand(
        ShooterSubsystem shooter,
        TurretSubsystem turret,
        HoodSubsystem hood,
        FlywheelSubsystem flywheel
    ) {
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;

        addRequirements(turret, hood, flywheel);
    }

    @Override
    public void execute() {
        turret.setAngle(shooter.getTurretSetpoint());
        hood.setAngle(shooter.getHoodSetpoint());
        flywheel.setVelocity(shooter.getFlywheelSetpoint());
    }

    @Override
    public void end(boolean interrupted) {

    }
}
