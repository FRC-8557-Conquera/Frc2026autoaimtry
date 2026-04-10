package frc.robot.commands;

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
    // DÜZELTME 2: Turret değişkenini ve requirement'ını sildik, çünkü Taret kendi DefaultCommand'i ile çalışacak.
    
    private final Timer timer = new Timer();
    private final double duration;

    public DumpCommand(SpindexerSubsystem spindexer, FeederSubsystem feeder, ShooterSubsystem shooter, double duration) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.flywheel = shooter.flywheel;
        this.shooter = shooter;
        this.duration = duration;
        
        // Sadece çalıştıracağımız motorları require ediyoruz
        addRequirements(spindexer, feeder, flywheel);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        
        // DÜZELTME 3: Niyeti execute'da saniyede 50 kere atamak yerine, komut başlarken 1 kere atıyoruz.
        shooter.setIntent(ShotIntent.DUMP);
    }

    @Override
    public void execute() {
        
        double targetRPS = shooter.getFlywheelSetpoint().in(edu.wpi.first.units.Units.RotationsPerSecond);
        flywheel.setRPSDirect(targetRPS); 
        
        spindexer.setDutyCycleDirect(Constants.Spindexer.reverseSpeed); 
        feeder.setDutyCycleDirect(Constants.Feeder.feedSpeed);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        // DÜZELTME 4: Motorları durdur ve niyeti OFF yap ki taret rahatlasın
        shooter.setIntent(ShotIntent.OFF);
        
        spindexer.stopDirect();
        feeder.stopDirect();
        flywheel.stopDirect();
        
        timer.stop();
    }
}