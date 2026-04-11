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

public class ShootCommand extends Command {
    private final SpindexerSubsystem spindexer;
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;


    // Constructor'a "targetRPS" eklendi!
    public ShootCommand(SpindexerSubsystem spindexer, FeederSubsystem feeder, ShooterSubsystem shooter) {
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.shooter = shooter;
        // DİKKAT: Turret ve Hood gereksinimden ÇIKARILDI! 
        // Turret kendi başına arka planda bağımsız çalışacak.
        addRequirements(spindexer, feeder, shooter.flywheel);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // ÖNEMLİ: .schedule() KESİNLİKLE KULLANILMAZ!
        // Alt sistemlere gidip bu "Direct" (Command döndürmeyen void) metotları eklemelisin.
        
<<<<<<< Updated upstream
        // 2. SÜRÜCÜ 2'NİN YAPTIĞI "BEKLE-FIRLAT" MANTIĞI
        double currentRPS = shooter.flywheel.getVelocity().in(RotationsPerSecond);
        
        if (Math.abs(currentRPS - TARGET_RPS) < 2.0) { 
            // Topu fırlat!
            feeder.setDutyCycleDirect(Constants.Feeder.feedSpeed);
            spindexer.setDutyCycleDirect(Constants.Spindexer.spindexerspeed);
        } else {
            feeder.stopDirect();
            spindexer.stopDirect();
        }
=======
        shooter.flywheel.setVelocity(() -> shooter.getFlywheelSetpoint())
            .schedule(); 
        spindexer.setMotor(Constants.Spindexer.spindexerspeed); 
        feeder.reverse().schedule();
>>>>>>> Stashed changes
    }


    @Override
    public void end(boolean interrupted) {
        // Motoları durdur
        shooter.flywheel.setVelocity(RotationsPerSecond.of(0)).schedule();
        spindexer.stop().schedule();;
        feeder.stop().schedule();;
    }
}