// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import frc.robot.Constants.Spindexer;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShotIntent;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;


public class RobotContainer {

  private final Joystick driver = new Joystick(0);
  private final Joystick driver2 = new Joystick(1);

  private final JoystickButton zeroGyro = new JoystickButton(driver, 3);
  private final JoystickButton xLock = new JoystickButton(driver, 6);

  private final JoystickButton wowButton = new JoystickButton(driver2, 10);
  private final JoystickButton turretButtonLeft = new JoystickButton(driver2, 8);
  private final JoystickButton turretButtonRight = new JoystickButton(driver2, 2);
  private final JoystickButton triggerButton = new JoystickButton(driver2, 1);

  private final JoystickButton feederAl= new JoystickButton(driver2, 3);
  private final JoystickButton feederTers= new JoystickButton(driver2, 4);

  private final JoystickButton spindexerF = new JoystickButton(driver2, 5);
  private final JoystickButton spindexerR = new JoystickButton(driver2, 6);
  private final JoystickButton sysIDButton = new JoystickButton(driver2, 7);
  JoystickButton hubButton  = new JoystickButton(driver2, 11);
  JoystickButton dumpButton = new JoystickButton(driver2, 12);
  

  public final SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();
  private final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem(s_Swerve);
  private final SpindexerSubsystem spindexer = new SpindexerSubsystem();


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
    turret.setDefaultCommand(
    turret.setAngle(() -> shooter.getTurretSetpoint()));
  }

  private void configureButtonBindings() {

    hubButton.whileTrue(new InstantCommand(() -> shooter.setIntent(ShotIntent.HUB)));
    hubButton.whileFalse(new InstantCommand(() -> shooter.setIntent(ShotIntent.OFF)));

    dumpButton.whileTrue(new InstantCommand(() -> shooter.setIntent(ShotIntent.DUMP)));
    dumpButton.whileFalse(new InstantCommand(() -> shooter.setIntent(ShotIntent.OFF)));

    wowButton.whileTrue(new InstantCommand(() -> {
      feeder.reverse();
      flywheel.setVelocity(RotationsPerSecond.of(90));
      spindexer.spinForward();
    }));

    triggerButton.whileTrue(flywheel.setVelocity(RotationsPerSecond.of(60))).onFalse(flywheel.setVelocity(RotationsPerSecond.of(0)));

    turretButtonLeft.whileTrue(turret.rotateLeft()).onFalse(turret.stop());
    turretButtonRight.whileTrue(turret.rotateRight()).onFalse(turret.stop());

    sysIDButton.whileTrue(flywheel.sysId());

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
