// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.dyn4j.dynamics.joint.AngleJoint;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  private final JoystickButton turretSysID = new JoystickButton(driver2, 10);
    private final JoystickButton turretZero = new JoystickButton(driver2,2);

  private final JoystickButton turretButtonLeft = new JoystickButton(driver2,8);
  private final JoystickButton turretButtonRight = new JoystickButton(driver2, 9);
  private final JoystickButton triggerButton = new JoystickButton(driver2, 1);

  private final JoystickButton feederAl = new JoystickButton(driver2, 3);
  private final JoystickButton feederTers = new JoystickButton(driver2, 4);

  private final JoystickButton spindexerF = new JoystickButton(driver2, 5);
  private final JoystickButton spindexerR = new JoystickButton(driver2, 6);
  private final JoystickButton sysIDButton = new JoystickButton(driver2, 7);
  JoystickButton hubButton = new JoystickButton(driver2, 11);
  JoystickButton dumpButton = new JoystickButton(driver2, 12);

  public final SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();

  private final ShooterSubsystem shooter = new ShooterSubsystem(s_Swerve);
  
  private final TurretSubsystem turret = shooter.turret;
  private final FlywheelSubsystem flywheel = shooter.flywheel;
  private final SpindexerSubsystem spindexer = new SpindexerSubsystem();

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      s_Swerve.getSwerveDrive(),
      () -> driver.getY(),
      () -> driver.getX())
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

    turret.setDefaultCommand(turret.setAngle(() -> shooter.getTurretSetpoint()));
  }

  private void configureButtonBindings() {

    hubButton.onTrue(new InstantCommand(() -> shooter.setIntent(ShotIntent.HUB)));
    dumpButton.onTrue(new InstantCommand(() -> shooter.setIntent(ShotIntent.OFF)));
   
    triggerButton.whileTrue(
    Commands.run(() -> { flywheel.setVelocity(RotationsPerSecond.of(40)).schedule();feeder.feed().schedule();})).onFalse(
    Commands.runOnce(() -> {flywheel.setVelocity(RotationsPerSecond.of(0)).schedule(); feeder.stop().schedule();}));

    turretButtonLeft.whileTrue(turret.rotateDutyCycle(0.1)).onFalse(turret.stop());
    turretButtonRight.whileTrue(turret.rotateDutyCycle(-0.1)).onFalse(turret.stop());
    turretZero.onTrue(turret.setAngle(Degrees.of(90))).onFalse(turret.stop());

    sysIDButton.whileTrue(flywheel.sysId());
    turretSysID.whileTrue(turret.sysId());

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
