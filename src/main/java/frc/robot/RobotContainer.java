// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.dyn4j.dynamics.joint.AngleJoint;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Spindexer;
import frc.robot.commands.DumpCommand;
import frc.robot.commands.IntakeAlCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotIntent;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FuelSim;
import frc.robot.util.MapleSimSwerve;
import swervelib.SwerveInputStream;


public class RobotContainer {

  private final Joystick driver = new Joystick(0);
  private final Joystick driver2 = new Joystick(1);

  private final JoystickButton zeroGyro = new JoystickButton(driver, 3);
  private final JoystickButton xLock = new JoystickButton(driver, 6);

  private final JoystickButton turretZero = new JoystickButton(driver2,2);
  private final JoystickButton turretButtonLeft = new JoystickButton(driver2,8);
    private final JoystickButton turretButtonRight = new JoystickButton(driver2,7);

  private final JoystickButton triggerButton = new JoystickButton(driver2, 1);

  private final JoystickButton feederAl = new JoystickButton(driver2, 3);
  private final JoystickButton feederTers = new JoystickButton(driver2, 4);

  private final JoystickButton spindexerF = new JoystickButton(driver2, 5);
  private final JoystickButton spindexerR = new JoystickButton(driver2, 6);
  //private final JoystickButton flywheelSysID = new JoystickButton(driver2, 7);
  private final JoystickButton offButton = new JoystickButton(driver2, 10);
  //private final JoystickButton intakeKapa = new JoystickButton(driver2, 8);
  private final JoystickButton intakeAl = new JoystickButton(driver2, 9);
  JoystickButton hubButton = new JoystickButton(driver2, 11);
  JoystickButton dumpButton = new JoystickButton(driver2, 12);
  public FuelSim fuelSim = new FuelSim();
  private MapleSimSwerve maplesim = new MapleSimSwerve();


  public final SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();

  private final ShooterSubsystem shooter = new ShooterSubsystem(s_Swerve);
  
  private final TurretSubsystem turret = shooter.turret;
  private final FlywheelSubsystem flywheel = shooter.flywheel;
  private final SpindexerSubsystem spindexer = new SpindexerSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      s_Swerve.getSwerveDrive(),
      () -> -driver.getY(),
      () -> -driver.getX())
      .withControllerRotationAxis(() -> -Math.pow(driver.getRawAxis(4), 3))
      .deadband(Constants.Swerve.stickDeadband)
      .scaleTranslation(0.8) // YAVASLATMA!!!!
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> driver.getRawAxis(2), () -> driver.getRawAxis(3))
      .headingWhile(false);
  
    
    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(s_Swerve.getSwerveDrive(),
                                                                        () -> -driver.getY(),
                                                                        () -> -driver.getX())
                                                                    .withControllerRotationAxis(() -> driver.getRawAxis(
                                                                        4))
                                                                    .deadband(0.1)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driver.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driver.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  Command FOdriveAngularVelocity = s_Swerve.driveFieldOriented(driveAngularVelocity);
  Command FOdriveAngularVelocityKeyboard = s_Swerve.driveFieldOriented(driveAngularVelocityKeyboard);
  Command FOdriveDirectAngle = s_Swerve.driveFieldOriented(driveDirectAngle);
  Command FOdriveAngularVelocitySim = maplesim.mapleFieldOrientedDrive(() -> driveAngularVelocity.get());
  SendableChooser<Command> m_chooser;

  public RobotContainer() {

    // 1. Register NamedCommands BEFORE AutoBuilder.configure()
    NamedCommands.registerCommand("taretbah", turret.setAngle(shooter.getTurretSetpoint()));
    NamedCommands.registerCommand("tüküğr", flywheel.setVelocity(shooter.getFlywheelSetpoint()));
    NamedCommands.registerCommand("dump", new DumpCommand(spindexer, feeder, shooter,3));
    NamedCommands.registerCommand("intake", new IntakeAlCommand(intake, 4));
    NamedCommands.registerCommand("tumsel", new ShootCommand(spindexer,feeder,shooter,6));

    // 2. Now configure PathPlanner (AutoBuilder.configure runs here)
    s_Swerve.setupPathPlanner();

    // 3. Build chooser from PathPlanner auto files
    m_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_chooser);

    DriverStation.silenceJoystickConnectionWarning(true);
    if(RobotBase.isSimulation()) {
      // s_Swerve.setDefaultCommand(FOdriveAngularVelocitySim);
      maplesim.setDefaultCommand(FOdriveAngularVelocitySim);
    } else s_Swerve.setDefaultCommand(FOdriveAngularVelocity);
    configureButtonBindings();

    turret.setDefaultCommand(turret.setAngle(() -> shooter.getTurretSetpoint()));
    s_Swerve.zeroGyroWithAlliance();
  }

  private void configureButtonBindings() {
    hubButton.onTrue(new InstantCommand(() -> shooter.setIntent(ShotIntent.HUB)));
    dumpButton.onTrue(new InstantCommand(() -> shooter.setIntent(ShotIntent.DUMP)));
    offButton.onTrue(new InstantCommand(() -> shooter.setIntent(ShotIntent.OFF)));
   
    triggerButton.whileTrue(
    Commands.run(() -> { flywheel.setVelocity(() -> shooter.getFlywheelSetpoint()).schedule(); feeder.feed().schedule(); spindexer.spinReverse().schedule();})).onFalse(
    Commands.runOnce(() -> {flywheel.setVelocity(RotationsPerSecond.of(0)).schedule(); feeder.stop().schedule(); spindexer.stop().schedule();}));

    turretButtonLeft.whileTrue(turret.rotateDutyCycle(0.05)).onFalse(turret.stop());
    turretButtonRight.whileTrue(turret.rotateDutyCycle(-0.05)).onFalse(turret.stop());
    turretZero.onTrue(turret.setAngle(Degrees.of(0))).onFalse(turret.stop());

   // flywheelSysID.whileTrue(flywheel.sysId());
    intakeAl.whileTrue(intake.rollerIn()).whileFalse(intake.rollerStop());
    
   // intakeKapa.onTrue(intake.close());

    feederAl.whileTrue(feeder.feed()).onFalse(feeder.stop());
    feederTers.whileTrue(feeder.reverse()).onFalse(feeder.stop());

    spindexerF.whileTrue(spindexer.spinForward()).onFalse(spindexer.stop());
    spindexerR.whileTrue(spindexer.spinReverse()).onFalse(spindexer.stop());

    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    xLock.whileTrue(Commands.runOnce(() -> s_Swerve.lock(), s_Swerve).repeatedly());
    
  
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    s_Swerve.setMotorBrake(brake);
  }

  public void resetOdometry(Pose2d pose) {
    s_Swerve.resetOdometry(pose);
  }

  public void zeroGyro() {
    s_Swerve.zeroGyro();
  }
}
