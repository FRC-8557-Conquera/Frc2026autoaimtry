// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.FuelSim;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

// 1. DEĞİŞİKLİK: TimedRobot sınıfını LoggedRobot olarak değiştiriyoruz
public class Robot extends LoggedRobot 
{
    private Command autonomousCommand;
    public RobotContainer robotContainer;
    private final Timer matchTimer = new Timer();
    
    @Override
    public void robotInit() {
        // AdvantageKit Loglama Kurulumu
        Logger.recordMetadata("ProjectName", "FRC2026_AutoAim"); 
        
        if (isReal()) {
            // Gerçek robotta hem USB Flash Belleğe (Log dosyası) yazar hem de NetworkTables'a (Anlık izleme) gönderir
            Logger.addDataReceiver(new WPILOGWriter()); 
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            // Simülasyonda çalışırken sadece NetworkTables üzerinden AdvantageScope'a canlı veri basar
            Logger.addDataReceiver(new NT4Publisher());
        }
        
        // Loglama sistemini başlat!
        Logger.start();

        // RobotContainer her zaman Logger.start() işleminden SONRA tanımlanmalıdır!
        robotContainer = new RobotContainer();
    }

    public Robot()
    {

    }
    
    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        publishMatchTime();
    }

    private void publishMatchTime() {
        boolean fmsConnected = DriverStation.isFMSAttached();
        double matchTime = DriverStation.getMatchTime();

        // FMS not connected -> use local timer as fallback
        if (!fmsConnected && matchTime < 0) {
            if (DriverStation.isAutonomousEnabled()) {
                matchTime = Math.max(0, 15.0 - matchTimer.get());
            } else if (DriverStation.isTeleopEnabled()) {
                matchTime = Math.max(0, 140.0 - matchTimer.get());
            } else {
                matchTime = 0;
            }
        }

        // Determine current shift based on FRC 2026 REBUILT match structure
        // Teleop is 2:20 (140s) with shifts:
        //   Transition: 140-130  (both Hubs active)
        //   Shift 1:    130-105  (auto loser's Hub active)
        //   Shift 2:    105-80   (auto winner's Hub active)
        //   Shift 3:    80-55    (auto loser's Hub active)
        //   Shift 4:    55-30    (auto winner's Hub active)
        //   Endgame:    30-0     (both Hubs active)
        String shift;
        boolean ourHubActive = true;

        if (DriverStation.isDisabled()) {
            shift = "Disabled";
        } else if (DriverStation.isAutonomousEnabled()) {
            shift = "Auto";
        } else if (DriverStation.isTeleopEnabled()) {
            // Game data: 'R' or 'B' = which alliance's Hub goes inactive FIRST (Shift 1)
            String gameData = DriverStation.getGameSpecificMessage();
            var alliance = DriverStation.getAlliance();
            boolean weAreRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

            // weGoInactiveFirst = true means OUR hub is inactive in Shift 1 & 3
            boolean weGoInactiveFirst = false;
            if (gameData != null && !gameData.isEmpty()) {
                weGoInactiveFirst = (gameData.charAt(0) == 'R' && weAreRed)
                                 || (gameData.charAt(0) == 'B' && !weAreRed);
            }

            if (matchTime > 130) {
                shift = "Transition";
                ourHubActive = true;
            } else if (matchTime > 105) {
                shift = "Shift 1";
                ourHubActive = !weGoInactiveFirst;
            } else if (matchTime > 80) {
                shift = "Shift 2";
                ourHubActive = weGoInactiveFirst;
            } else if (matchTime > 55) {
                shift = "Shift 3";
                ourHubActive = !weGoInactiveFirst;
            } else if (matchTime > 30) {
                shift = "Shift 4";
                ourHubActive = weGoInactiveFirst;
            } else {
                shift = "Endgame";
                ourHubActive = true;
            }
        } else {
            shift = "Test";
        }

        SmartDashboard.putNumber("Match Time", matchTime);
        SmartDashboard.putString("Match Shift", shift);
        SmartDashboard.putBoolean("Our Hub Active", ourHubActive);
        SmartDashboard.putBoolean("FMS Connected", fmsConnected);
        SmartDashboard.putBoolean("Endgame", shift.equals("Endgame"));
    }
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}
    
    
    @Override
    public void disabledExit() {}
    
    
    @Override
    public void autonomousInit()
    {
        matchTimer.restart();
        autonomousCommand = robotContainer.getAutonomousCommand();
        robotContainer.s_Swerve.seedOdometryWithMegaTag1();
        
        if (autonomousCommand != null)
        {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void autonomousExit() {}
    
    
    @Override
    public void teleopInit()
    {
        matchTimer.restart();
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void testExit() {}
    

    @Override
    public void simulationPeriodic() {
        robotContainer.fuelSim.updateSim();
    }
    
    @Override
    public void simulationInit() {
        // Set up FuelSim ball simulation
        FuelSim fuelSim = robotContainer.fuelSim;
        fuelSim.registerRobot(
            0.718, 0.718, 0.15, // bumper width/length (28.27"), bumper height (~6")
            robotContainer.s_Swerve::getPose,
            robotContainer.s_Swerve::getChassisSpeeds);
        fuelSim.spawnStartingFuel();
        fuelSim.start();

        SmartDashboard.putData(Commands.runOnce(() -> {
                    fuelSim.clearFuel();
                    fuelSim.spawnStartingFuel();
                })
                .withName("Reset Fuel")
                .ignoringDisable(true));
    }
}
