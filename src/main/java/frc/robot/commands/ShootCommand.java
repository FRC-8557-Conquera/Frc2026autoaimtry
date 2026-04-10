package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class ShootCommand extends Command {
    private final SpindexerSubsystem spindexer;
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;
    
    // KODDAN DEĞİŞTİREBİLECEĞİN SABİT HIZ (40 RPS = 2400 RPM)
    private final double TARGET_RPS = 40.0; 

    public ShootCommand(SpindexerSubsystem spindexer, FeederSubsystem feeder, ShooterSubsystem shooter) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.shooter = shooter;
        
        // Otonomda alt sistemlerin çakışmasını engellemek için Require zorunludur
        addRequirements(spindexer, feeder, shooter.flywheel); 
    }

    @Override
    public void initialize() {
        // Başlangıç temizliği
    }

    @Override
    public void execute() {
        // 1. FLYWHEEL'E GÜCÜ VER (Hedef: 40 RPS)
        shooter.flywheel.setRPSDirect(TARGET_RPS);
        
        // 2. SÜRÜCÜ 2'NİN YAPTIĞI "BEKLE-FIRLAT" MANTIĞI
        double currentRPS = shooter.flywheel.getVelocity().in(RotationsPerSecond);
        
        // Eğer motor hedef hıza yaklaştıysa (2.0 RPS hata payı ile)
        if (Math.abs(currentRPS - TARGET_RPS) < 2.0) { 
            // Topu fırlat!
            feeder.setDutyCycleDirect(Constants.Feeder.feedSpeed);
            spindexer.setDutyCycleDirect(Constants.Spindexer.spindexerspeed);
        } else {
            // Motor hala devir alıyor, topu erken verip sıkıştırma! Durdur.
            feeder.stopDirect();
            spindexer.stopDirect();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Komut (PathPlanner'daki wait süresi dolunca) bittiğinde her şeyi sustur
        shooter.flywheel.stopDirect();
        feeder.stopDirect();
        spindexer.stopDirect();
    }
    
    @Override
    public boolean isFinished() {
        // İstediğin gibi, komut kod içinde asla kendi kendine bitmez, süreyi PathPlanner yönetir.
        return false; 
    }
}