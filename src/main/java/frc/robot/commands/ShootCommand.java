package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class ShootCommand extends Command {
    private final SpindexerSubsystem spindexer;
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;

    public ShootCommand(SpindexerSubsystem spindexer, FeederSubsystem feeder, ShooterSubsystem shooter) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.shooter = shooter;
        
        // KRİTİK DÜZELTME: Kullanılan Tonal alt sistemler Require edilmeli.
        // Böylece Scheduler bu komutu iptal ettiğinde motorlar yetkisiz kalmaz.
        addRequirements(spindexer, feeder, shooter.flywheel); 
    }

    @Override
    public void initialize() {
        // .schedule() çağrıları tamamen kaldırıldı.
    }

    @Override
    public void execute() {
        // Maç içi veya test sırasında Elastic'ten anlık RPS verisini çek.
        // Execute döngüsü saniyede 50 kez (20ms) çalıştığı için değer değiştiği an motora yansır.
        double currentRPS = SmartDashboard.getNumber("FlywheelSpeed", 40.0);
        
        // Alt sistemlerde tanımladığın Direct metotları ile motorlara güç ver
        shooter.flywheel.setRPSDirect(currentRPS);
        feeder.setDutyCycleDirect(Constants.Feeder.feedSpeed);
        spindexer.setDutyCycleDirect(Constants.Spindexer.spindexerspeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Otonomdaki 5 saniyelik deadline bittiğinde veya teleop'ta tuş bırakıldığında motorlar anında durur.
        shooter.flywheel.stopDirect();
        feeder.stopDirect();
        spindexer.stopDirect();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Komut dışarıdan bir trigger veya timeout ile kesilene kadar çalışmaya devam eder.
    }
}