package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

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

    private final double targetRPS; // PathPlanner'dan veya dışarıdan girilecek özel hız

    // Constructor'a "targetRPS" eklendi!
    public ShootCommand(SpindexerSubsystem spindexer, FeederSubsystem feeder, ShooterSubsystem shooter, double targetRPS) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.flywheel = shooter.flywheel;
        
        this.targetRPS = targetRPS;
        
        // DİKKAT: Turret ve Hood gereksinimden ÇIKARILDI! 
        // Turret kendi başına arka planda bağımsız çalışacak.
        addRequirements(spindexer, feeder, flywheel);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // ÖNEMLİ: .schedule() KESİNLİKLE KULLANILMAZ!
        // Alt sistemlere gidip bu "Direct" (Command döndürmeyen void) metotları eklemelisin.
        
        flywheel.setVelocity(RotationsPerSecond.of(targetRPS)); 
        spindexer.setMotor(Constants.Spindexer.spindexerspeed); 
        feeder.setMotor(Constants.Feeder.feedSpeed);
    }


    @Override
    public void end(boolean interrupted) {
        // Motorları durdur
        flywheel.stopDirect();
        spindexer.stopMotor();
        feeder.stop();
    }
}