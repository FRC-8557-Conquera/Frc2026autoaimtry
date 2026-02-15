package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Autonomous extends SequentialCommandGroup {
  public Autonomous(SwerveSubsystem s_Swerve) {
    
    addCommands(
      
      new InstantCommand(() -> s_Swerve.zeroGyro()),
      new InstantCommand(() -> s_Swerve.driveCommand(
            () -> 0.0,
            () -> 0.0,
            () -> 0.0))  
     );     
  }
}