package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotIntent;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.MapleSimSwerve;
import swervelib.SwerveDrive;
import swervelib.simulation.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;

public class MappleSimDrive extends Command{
    MapleSimSwerve mapleSimSwerve;
    Joystick driver;
    public MappleSimDrive(MapleSimSwerve mapleSimSwerve,Joystick driver){
        this.mapleSimSwerve=mapleSimSwerve;
        this.driver=driver;
        addRequirements(mapleSimSwerve);
    }
    
     @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        mapleSimSwerve.drive(new ChassisSpeeds(-driver.getY(),-driver.getX(), driver.getRawAxis(4)),false,false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
