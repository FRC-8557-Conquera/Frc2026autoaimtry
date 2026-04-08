package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FlywheelSubsystem;

public class shooterdeneme extends Command {
    private final FlywheelSubsystem flywheel;
    private final DoubleSupplier triggerSupplier;

    public shooterdeneme(FlywheelSubsystem flywheel, DoubleSupplier triggerSupplier) {
        this.flywheel = flywheel;
        this.triggerSupplier = triggerSupplier;
        addRequirements(flywheel);
    }

    @Override
    public void execute() {
        // Trigger 0 ile 1 arası değer verir. 
        // Bunu maksimum 90 RPS (devir/saniye) olacak şekilde ölçeklendiriyoruz.
        double targetRPS = triggerSupplier.getAsDouble() * 90.0;
        
        // Eşik değer: Tetiğe çok hafif basıldığında motor dönmesin
        if (targetRPS < 5.0) {
            flywheel.setDutyCycle(0.0);
        } else {
            flywheel.setVelocity(RotationsPerSecond.of(targetRPS));
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setDutyCycle(0.0); // Tetik bırakılınca veya komut ölünce dur
    }
}