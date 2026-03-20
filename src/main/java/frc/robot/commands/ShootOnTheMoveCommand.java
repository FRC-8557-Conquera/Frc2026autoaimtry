package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotIntent;

public class ShootOnTheMoveCommand extends Command {
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;

    public ShootOnTheMoveCommand(ShooterSubsystem shooter, FeederSubsystem feeder) {
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements(feeder); 
    }

    @Override
    public void initialize() {
        // Komut başladığında (12. butona basıldığında) niyeti SOTM yap
        shooter.setIntent(ShotIntent.SOTM); 
    }

    @Override
    public void execute() {
        // Hata paylarını hesapla (Tolerans)
        double turretError = Math.abs(shooter.turret.getAngle().in(edu.wpi.first.units.Units.Degrees) 
                           - shooter.getTurretSetpoint().in(edu.wpi.first.units.Units.Degrees));
        double flywheelError = Math.abs(shooter.flywheel.getVelocity().in(edu.wpi.first.units.Units.RotationsPerSecond) 
                             - shooter.getFlywheelSetpoint().in(edu.wpi.first.units.Units.RotationsPerSecond));

        // Eğer Taret 2 derece hata payı içindeyse VE Motor hızını yakaladıysa OTO-ATEŞLE!
        if (turretError < 2.0 && flywheelError < 2.0) {
            feeder.setSpeed(1.0); 
        } else {
            feeder.setSpeed(0.0); // Değilse gücü kes ve bekle
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Komut kapatıldığında (Butona tekrar basıldığında) sistemi durdur
        shooter.setIntent(ShotIntent.OFF);
        feeder.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        // Toggle (Aç/Kapa) mantığıyla çalışacağı için kendi kendine bitmez
        return false; 
    }
}