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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Hood;
import frc.robot.Constants.Spindexer;
import frc.robot.commands.DumpCommand;
import frc.robot.commands.IntakeAlCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootOnTheMoveCommand; // EKLENDİ
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotIntent;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.util.FuelSim;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final Joystick driver = new Joystick(0);
  private final Joystick driver2 = new Joystick(1);

  private final JoystickButton zeroGyro = new JoystickButton(driver, 3);
  private final JoystickButton xLock = new JoystickButton(driver, 6);

  private final JoystickButton turretZero = new JoystickButton(driver2, 2);
  private final JoystickButton turretButtonLeft = new JoystickButton(driver2, 8);
  private final JoystickButton turretButtonRight = new JoystickButton(driver2, 7);

  private final JoystickButton triggerButton = new JoystickButton(driver2, 1);

  private final JoystickButton feederAl = new JoystickButton(driver2, 3);
  private final JoystickButton feederTers = new JoystickButton(driver2, 4);

  private final JoystickButton spindexerF = new JoystickButton(driver2, 5);
  private final JoystickButton spindexerR = new JoystickButton(driver2, 6);
  private final JoystickButton offButton = new JoystickButton(driver2, 10);
  private final JoystickButton intakeAl = new JoystickButton(driver2, 9);
  
  JoystickButton hubButton = new JoystickButton(driver2, 11);
  JoystickButton sotmButton = new JoystickButton(driver2, 12); // EKLENDİ (Hareketli Atış Butonu)
  JoystickButton dumpButton = new JoystickButton(driver2, 13); // Çakışmaması için 13'e alındı

  public FuelSim fuelSim = new FuelSim();  

  public final SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();

  private final ShooterSubsystem shooter = new ShooterSubsystem(s_Swerve);
  private final TurretSubsystem turret = shooter.turret;
  private final HoodSubsystem hood = shooter.hood;
  private final FlywheelSubsystem flywheel = shooter.flywheel;
  private final SpindexerSubsystem spindexer = new SpindexerSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();

  // YAGSL Sürüş Komutu: Eğer SOTM aktifse joystick girdilerini %40'a düşürür (0.4 ile çarpar)
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      s_Swerve.getSwerveDrive(),
      () -> -driver.getY() * (shooter.getIntent() == ShotIntent.SOTM ? 0.4 : 1.0),
      () -> -driver.getX() * (shooter.getIntent() == ShotIntent.SOTM ? 0.4 : 1.0))
      .withControllerRotationAxis(() -> -Math.pow(driver.getRawAxis(4), 3) * (shooter.getIntent() == ShotIntent.SOTM ? 0.4 : 1.0))
      .deadband(Constants.Swerve.stickDeadband)
      .scaleTranslation(0.8) // Genel YAVASLATMA
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> driver.getRawAxis(2), () -> driver.getRawAxis(3))
      .headingWhile(false);
  
  // Klavye / Simülasyon Sürüş Komutu: Aynı hız düşürme mantığı burada da var
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
      s_Swerve.getSwerveDrive(),
      () -> -driver.getY() * (shooter.getIntent() == ShotIntent.SOTM ? 0.4 : 1.0),
      () -> -driver.getX() * (shooter.getIntent() == ShotIntent.SOTM ? 0.4 : 1.0))
      .withControllerRotationAxis(() -> driver.getRawAxis(4) * (shooter.getIntent() == ShotIntent.SOTM ? 0.4 : 1.0))
      .deadband(0.1)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(driver.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                 () -> Math.cos(driver.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(0));

  Command FOdriveAngularVelocity = s_Swerve.driveFieldOriented(driveAngularVelocity);
  Command FOdriveAngularVelocityKeyboard = s_Swerve.driveFieldOriented(driveAngularVelocityKeyboard);
  Command FOdriveDirectAngle = s_Swerve.driveFieldOriented(driveDirectAngle);

  SendableChooser<Command> m_chooser;

  public RobotContainer() {

// 1. Register NamedCommands BEFORE AutoBuilder.configure()
    NamedCommands.registerCommand("AutoSOTM", new ShootOnTheMoveCommand(shooter, feeder));
    NamedCommands.registerCommand("taretbah", turret.setAngle(shooter.getTurretSetpoint()));
    NamedCommands.registerCommand("tukugr", flywheel.setVelocity(shooter.getFlywheelSetpoint()));
    NamedCommands.registerCommand("dump", new DumpCommand(spindexer, feeder, shooter, 3));
    NamedCommands.registerCommand("intake", new IntakeAlCommand(intake, 4));
    NamedCommands.registerCommand("tumsel", new ShootCommand(spindexer, feeder, shooter, 6));
    
    // YENİ EKLENEN SATIR: PathPlanner için SOTM Komutu
    // 2. Now configure PathPlanner (AutoBuilder.configure runs here)
    s_Swerve.setupPathPlanner();

    // 3. Build chooser from PathPlanner auto files
    m_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_chooser);

    DriverStation.silenceJoystickConnectionWarning(true);
    if(RobotBase.isSimulation()) {
      s_Swerve.setDefaultCommand(FOdriveAngularVelocityKeyboard);
    } else s_Swerve.setDefaultCommand(FOdriveAngularVelocity);
    
    configureButtonBindings();

    turret.setDefaultCommand(turret.setAngle(() -> shooter.getTurretSetpoint()));
    hood.setDefaultCommand(hood.setAngle(() -> shooter.getHoodSetpoint()));

    s_Swerve.zeroGyroWithAlliance();
  }

  private void configureButtonBindings() {
    hubButton.onTrue(new InstantCommand(() -> shooter.setIntent(ShotIntent.HUB)));
    dumpButton.onTrue(new InstantCommand(() -> shooter.setIntent(ShotIntent.DUMP)));
    offButton.onTrue(new InstantCommand(() -> shooter.setIntent(ShotIntent.OFF)));
    
    // YENİ: Shoot On The Move Butonu (Aç/Kapa mantığı ile çalışır)
    sotmButton.toggleOnTrue(new ShootOnTheMoveCommand(shooter, feeder));
   
    triggerButton.whileTrue(
    Commands.run(() -> { flywheel.setVelocity(() -> shooter.getFlywheelSetpoint()); feeder.feed(); spindexer.spinReverse();})).onFalse(
    Commands.runOnce(() -> {flywheel.setVelocity(RotationsPerSecond.of(0)); feeder.stop(); spindexer.stop();}));

    turretButtonLeft.whileTrue(turret.rotateDutyCycle(0.05)).onFalse(turret.stop());
    turretButtonRight.whileTrue(turret.rotateDutyCycle(-0.05)).onFalse(turret.stop());
    turretZero.onTrue(turret.setAngle(Degrees.of(0))).onFalse(turret.stop());

    intakeAl.whileTrue(intake.rollerIn()).whileFalse(intake.rollerStop());
    
    feederAl.whileTrue(feeder.feed()).onFalse(feeder.stop());
    feederTers.whileTrue(feeder.reverse()).onFalse(feeder.stop());

    spindexerF.whileTrue(spindexer.spinForward()).onFalse(spindexer.stop());
    spindexerR.whileTrue(spindexer.spinReverse()).onFalse(spindexer.stop());

    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    xLock.whileTrue(Commands.runOnce(() -> s_Swerve.lock(), s_Swerve).repeatedly());

    // configureButtonBindings() metodunun içine ekliyoruz:

    // 1. Sadece "Rakip Sahada (Hub'ın Diğer Tarafında)" Olma Durumunu Kontrol Eden Tetikleyici
    // ÖLÇÜLEN YENİ ÇİZGİLERE GÖRE RAKİP SAHA KONTROLÜ
    Trigger opponentSideTrigger = new Trigger(() -> {
        Pose2d pose = s_Swerve.getPose();
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        if (isRed) {
            // Kırmızıysak X, 11.0'dan küçük olduğunda rakip sahaya (sola) geçmiş oluyoruz.
            return pose.getX() < 11.0; 
        } else {
            // Maviysek X, 5.2'den büyük olduğunda rakip sahaya (sağa) geçmiş oluyoruz.
            return pose.getX() > 5.2; 
        }
    });

    // Robot rakip sahadayken ve sürücü atış/sotm tuşlarına basmıyorsa
    // Tareti otomatik olarak DUMP (Sarı Noktalar) pozisyonuna çevir.
  // DÜZELTME: Kendi sahamıza (11.9 çizgisinin gerisine) döndüğümüz an
    // robot uykudan uyanır ve namluyu anında tekrar potaya (SOTM) kilitler!
    opponentSideTrigger.and(() -> !sotmButton.getAsBoolean() && !hubButton.getAsBoolean())
        .whileTrue(Commands.runOnce(() -> shooter.setIntent(ShotIntent.DUMP)))
        .onFalse(Commands.runOnce(() -> shooter.setIntent(ShotIntent.SOTM)));
    // Robot ilk açıldığında varsayılan olarak SOTM modunda başlasın
    shooter.setIntent(ShotIntent.SOTM);
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