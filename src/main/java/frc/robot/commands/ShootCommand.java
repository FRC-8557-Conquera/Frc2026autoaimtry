package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
    
    private final Timer timer = new Timer();
    private final double duration;
    private final double targetRPS; // PathPlanner'dan veya dışarıdan girilecek özel hız

    // Constructor'a "targetRPS" eklendi!
    public ShootCommand(SpindexerSubsystem spindexer, FeederSubsystem feeder, ShooterSubsystem shooter, double duration, double targetRPS) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.flywheel = shooter.flywheel;
        
        this.duration = duration;
        this.targetRPS = targetRPS;
        
        // DİKKAT: Turret ve Hood gereksinimden ÇIKARILDI! 
        // Turret kendi başına arka planda bağımsız çalışacak.
        addRequirements(spindexer, feeder, flywheel);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        // Niyeti (ShotIntent) buradan sildik. Artık otonom başında açacağız.
    }

    @Override
    public void execute() {
        // ÖNEMLİ: .schedule() KESİNLİKLE KULLANILMAZ!
        // Alt sistemlere gidip bu "Direct" (Command döndürmeyen void) metotları eklemelisin.
        
        flywheel.setRPSDirect(targetRPS); 
        spindexer.setDutyCycleDirect(Constants.Spindexer.spindexerspeed); 
        feeder.setDutyCycleDirect(Constants.Feeder.feedSpeed);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        // Motorları durdur
        flywheel.stopDirect();
        spindexer.stopDirect();
        feeder.stopDirect();
        timer.stop();
    }
}