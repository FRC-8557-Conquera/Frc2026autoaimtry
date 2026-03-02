package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.*;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotIntent;

public class ShootCommand extends Command {
    private final SpindexerSubsystem spindexer;
    private final FeederSubsystem feeder;
    private final FlywheelSubsystem flywheel;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final Timer timer = new Timer();
    private final double duration;

    public ShootCommand(SpindexerSubsystem spindexer,FeederSubsystem feeder,ShooterSubsystem shooter, double duration) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.flywheel = shooter.flywheel;
        this.shooter = shooter;
        this.turret = shooter.turret;
        this.duration = duration;
        addRequirements(spindexer,feeder,flywheel);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        shooter.intent = ShotIntent.HUB;
        turret.setAngle(shooter.getTurretSetpoint()).schedule();
        spindexer.spinReverse().schedule();
        feeder.feed().schedule();
        flywheel.setVelocity(shooter.getFlywheelSetpoint()).schedule();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stop();
        feeder.stop();
        flywheel.setVelocity(RotationsPerSecond.of(0)).schedule();
        timer.stop();
    }
}