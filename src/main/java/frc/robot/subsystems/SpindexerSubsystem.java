package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Spindexer;



public class SpindexerSubsystem extends SubsystemBase {
    private final TalonFX spindexerMotor = new TalonFX(Spindexer.spindexerMotor);

    public SpindexerSubsystem() {}

    public Command spinForward() {
        return run(() ->
            spindexerMotor.set(0.6)
        );
    }

    public Command spinReverse() {
        return run(() ->
            spindexerMotor.set(-0.6)
        );
    }

    public Command stop() {
        return run(() ->
            spindexerMotor.set(0)
        );
    }

}
