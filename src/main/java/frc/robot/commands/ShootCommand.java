package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotIntent;
import frc.robot.Constants; // Constants kütüphanesini de import et

public class ShootCommand extends Command {
    private final SpindexerSubsystem spindexer;
    private final FeederSubsystem feeder;
    private final FlywheelSubsystem flywheel;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    
    private final Timer timer = new Timer();
    private final double duration;

    public ShootCommand(SpindexerSubsystem spindexer, FeederSubsystem feeder, ShooterSubsystem shooter, double duration) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.shooter = shooter;
        
        // Alt sistemleri Shooter'ın içinden çekiyoruz!
        this.flywheel = shooter.flywheel;
        this.turret = shooter.turret;
        
        this.duration = duration;
        
        // Turret ve Hood'u da gereksinimlere eklemek iyi pratiktir
        addRequirements(spindexer, feeder, flywheel, turret, shooter.hood);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        
        // Atışa başlarken niyeti HUB (Pota) olarak belirliyoruz ki matematik çalışsın
        shooter.setIntent(ShotIntent.HUB);
    }

    @Override
    public void execute() {
        // 1. Tareti hedefe çevir (Direkt komutla)
        turret.setAngleDirect(shooter.getTurretSetpoint()); 
        
        // 2. Flywheel'e hız ver (RPM/RPS) - Bunu ShooterSubsystem matematiğinden çekiyoruz
        flywheel.setVelocity(shooter.getFlywheelSetpoint()).schedule(); 

        // 3. Şapkayı (Hood) ayarla
        shooter.hood.setAngle(shooter.getHoodSetpoint()).schedule();

        // 4. Mermileri sür!
        // Eğer spindexer ve feeder'da 'setMotor' gibi voids metotların varsa onları kullan.
        // Eğer yoksa run/set komutlarını schedule et.
        // Örnek:
        // spindexer.setDutyCycle(Constants.Spindexer.spindexerspeed); 
        // feeder.setDutyCycle(Constants.Feeder.feedSpeed);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        // Her şeyi durdur ve niyeti kapat
        shooter.setIntent(ShotIntent.OFF);
        
        // Durdurma metotlarını çağır
        // spindexer.stop();
        // feeder.stop();
        flywheel.setDutyCycle(0.0).schedule();
        
        timer.stop();
    }
}