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



public class Robot extends TimedRobot
{
    private Command autonomousCommand;
    
    public final RobotContainer robotContainer;
    
    
    public Robot()
    {
        robotContainer = new RobotContainer();
    }
    
    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
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
        autonomousCommand = robotContainer.getAutonomousCommand();
        
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
    
    // Create and configure a drivetrain simulation configuration
final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
        // Specify gyro type (for realistic gyro drifting and error simulation)
        .withGyro(COTS.ofNav2X())
        // Specify swerve module (for realistic swerve dynamics)
        .withSwerveModule(new SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                DCMotor.getNEO(1), // Steer motor is a Falcon 500
                6.75, // Drive motor gear ratio.
                12.8, // Steer motor gear ratio.
                Volts.of(0.1), // Drive friction voltage.
                Volts.of(0.1), // Steer friction voltage
                Inches.of(2), // Wheel radius
                KilogramSquareMeters.of(0.03), // Steer MOI
                1.13)) // Wheel COF
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(19.3), Inches.of(19.3))
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(28.2677165), Inches.of(28.2677165));

    @Override
    public void simulationPeriodic() {
        // robotContainer.fuelSim.updateSim();
        SimulatedArena.getInstance().simulationPeriodic();

        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(3, 3)));


    }
    
    @Override
    public void simulationInit() {
        /*FuelSim fuelSim = robotContainer.fuelSim;
        fuelSim.spawnStartingFuel();
        fuelSim.start();
        SmartDashboard.putData(Commands.runOnce(() -> {
                    fuelSim.clearFuel();
                    fuelSim.spawnStartingFuel();
                })
                .withName("Reset Fuel")
                .ignoringDisable(true));*/

    // Obtains the default instance of the simulation world, which is a Crescendo Arena.
    SimulatedArena.getInstance();
    // Overrides the default simulation
    SimulatedArena.overrideInstance(new Arena2026Rebuilt());
    }
}