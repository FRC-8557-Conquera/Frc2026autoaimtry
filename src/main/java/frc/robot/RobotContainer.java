// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShotIntent;
import frc.robot.subsystems.shooter.TurretSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final Joystick driver = new Joystick(0);

  private final JoystickButton zeroGyro = new JoystickButton(driver, 3);
  private final JoystickButton xLock = new JoystickButton(driver, 6);

  private final JoystickButton turnButton = new JoystickButton(driver, 4);

  JoystickButton hubButton  = new JoystickButton(driver, 5);
  JoystickButton dumpButton = new JoystickButton(driver, 6);

  public final SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();
  private final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem(s_Swerve);


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

  Command FOdriveAngularVelocity = s_Swerve.driveFieldOriented(driveAngularVelocity);
  Command FOdriveDirectAngle = s_Swerve.driveFieldOriented(driveDirectAngle);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    s_Swerve.setDefaultCommand(FOdriveAngularVelocity);
    configureButtonBindings();
    m_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(m_chooser);
  }

  private void configureButtonBindings() {

    hubButton.onTrue(new InstantCommand(() -> shooter.setIntent(ShotIntent.HUB)));
    hubButton.onFalse(new InstantCommand(() -> shooter.setIntent(ShotIntent.OFF)));

    dumpButton.onTrue(new InstantCommand(() -> shooter.setIntent(ShotIntent.DUMP)));
    dumpButton.onFalse(new InstantCommand(() -> shooter.setIntent(ShotIntent.OFF)));

    hubButton.or(dumpButton).whileTrue(new AutoShootCommand(shooter,turret,hood,flywheel));

    turnButton.whileTrue(Commands.run(() -> s_Swerve.turnToAngle(() -> s_Swerve.calculateHubAngle())));
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
