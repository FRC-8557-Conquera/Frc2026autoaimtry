package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotIntent;

public class ShootOnTheMoveCommand extends Command {
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final Timer shotTimer = new Timer();

    public ShootOnTheMoveCommand(ShooterSubsystem shooter, FeederSubsystem feeder) {
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements(feeder); 
    }

    @Override
    public void initialize() {
        shooter.setIntent(ShotIntent.SOTM); 
        shotTimer.restart(); 
    }

    @Override
    public void execute() {
        // Taretin hata payını beklemeyi TAMAMEN sildik! 
        // 12. buton aktif olduğu sürece sistem ateşlemeye hazır kabul edilir.
        feeder.setSpeed(0.6); 
        
        // Eğer son atışın üzerinden 0.2 saniye geçtiyse
        if (shotTimer.hasElapsed(0.2)) {
            shooter.spawnSimulatedFuel(); // 3 BOYUTLU TOPU ÜRET VE FIRLAT!
            shotTimer.restart();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setIntent(ShotIntent.OFF);
        feeder.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}