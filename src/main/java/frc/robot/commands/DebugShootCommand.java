package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    
    private final Timer timer = new Timer();
    private final DoubleSupplier targetRPS; // PathPlanner'dan veya dışarıdan girilecek özel hız

    // Constructor'a "targetRPS" eklendi!
    public DebugShootCommand(SpindexerSubsystem spindexer, FeederSubsystem feeder, ShooterSubsystem shooter, DoubleSupplier targetRPS) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.flywheel = shooter.flywheel;
        this.targetRPS = targetRPS;     
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {        
        flywheel.setVelocity(() -> RotationsPerSecond.of(SmartDashboard.getNumber("FlywheelSpeed", 0)));
        spindexer.setMotor(Constants.Spindexer.spindexerspeed); 
        feeder.setMotor(Constants.Feeder.feedSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Motorları durdur
        flywheel.stopDirect();
        spindexer.stopMotor();
        feeder.stopDirect(); 
        timer.stop();
    }
}