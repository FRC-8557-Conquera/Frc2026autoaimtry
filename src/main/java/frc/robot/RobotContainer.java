// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.YerToplamaCommand;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotIntent;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FuelSim;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private double getSpeedMultiplier() {
      // İleride Shooter modlarına göre burayı açarsın (Örn: SOTM için 0.4)
      return 1.0;
  }

  private final Joystick driver = new Joystick(0);
  private final Joystick driver2 = new Joystick(1);

  // Sürüş Butonları
  private final JoystickButton zeroGyro = new JoystickButton(driver, 3);
  private final JoystickButton xLock = new JoystickButton(driver, 6);

  // Mekanizma Butonları
  private final JoystickButton yerToplamaButonu = new JoystickButton(driver, 1);
  private final JoystickButton turretZero = new JoystickButton(driver, 2);
  private final JoystickButton intakeToggleButonu = new JoystickButton(driver, 4);
  private final JoystickButton rollerButonu = new JoystickButton(driver, 5); 
/*private final JoystickButton feederAl = new JoystickButton(driver2, 3);
  private final JoystickButton feederTers = new JoystickButton(driver2, 4);
  private final JoystickButton spindexerF = new JoystickButton(driver2, 5);
  private final JoystickButton spindexerR = new JoystickButton(driver2, 6);*/
  private final JoystickButton turretLeftTest = new JoystickButton(driver, 7);
  private final JoystickButton turretRightTest = new JoystickButton(driver, 8);
  private final JoystickButton hubButton = new JoystickButton(driver, 9);
  private final JoystickButton turretSysID = new JoystickButton(driver, 10);
  
  // Yazılım Butonları
  private final JoystickButton flywheelSysID = new JoystickButton(driver2, 7);

  public FuelSim fuelSim = new FuelSim();  

  public final SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();
  private final SpindexerSubsystem spindexer = new SpindexerSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  
  // EKSİK OLAN SHOOTER VE PARÇALARINI TANIMLIYORUZ!
  private final ShooterSubsystem shooter = new ShooterSubsystem(s_Swerve);
  private final TurretSubsystem turret = shooter.turret;
  private final FlywheelSubsystem flywheel = shooter.flywheel;
  private final HoodSubsystem hood = shooter.hood;
  
  // YAGSL Sürüş Komutu
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      s_Swerve.getSwerveDrive(),
      () -> -driver.getY() * getSpeedMultiplier(),
      () -> -driver.getX() * getSpeedMultiplier())
      .withControllerRotationAxis(() -> -Math.pow(driver.getRawAxis(4), 3) * getSpeedMultiplier())
      .deadband(Constants.Swerve.stickDeadband)
      .scaleTranslation(0.8) 
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> driver.getRawAxis(2), () -> driver.getRawAxis(3))
      .headingWhile(false);
  
  // Simülasyon Sürüş Komutu
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
      s_Swerve.getSwerveDrive(),
      () -> -driver.getY() * getSpeedMultiplier(),
      () -> -driver.getX() * getSpeedMultiplier())
      .withControllerRotationAxis(() -> driver.getRawAxis(4) * getSpeedMultiplier())
      .deadband(0.1)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  Command FOdriveAngularVelocity = s_Swerve.driveFieldOriented(driveAngularVelocity);
  Command FOdriveAngularVelocityKeyboard = s_Swerve.driveFieldOriented(driveAngularVelocityKeyboard);

  SendableChooser<Command> m_chooser;

  public RobotContainer() {
    // 1. HATA ÇÖZÜMÜ: Buradaki s_Swerve.setupPathPlanner(); satırını sildik! 
    // Çünkü SwerveSubsystem kendi içinde zaten bunu hallediyor.

    m_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_chooser);

    DriverStation.silenceJoystickConnectionWarning(true);
    if(RobotBase.isSimulation()) {
      s_Swerve.setDefaultCommand(FOdriveAngularVelocityKeyboard);
    } else {
      s_Swerve.setDefaultCommand(FOdriveAngularVelocity);
    }
    
    configureButtonBindings();
    s_Swerve.zeroGyroWithAlliance();
    
    // YENİDEN AKTİF EDİLDİ: Taretin hedefi canlı takip etmesini sağlayan Default Command
    turret.setDefaultCommand(turret.setAngle(() -> shooter.getTurretSetpoint()));
  }

  private void configureButtonBindings() {
    yerToplamaButonu.whileTrue(new YerToplamaCommand(intake, spindexer, feeder));
    intakeToggleButonu.onTrue(new InstantCommand(() -> intake.toggleIntake(), intake));

    rollerButonu.whileTrue(Commands.run(() -> intake.setRollerPower(Constants.Intake.rollerInSpeed), intake))
                .onFalse(Commands.runOnce(() -> intake.setRollerPower(0.0), intake));

    turretSysID.whileTrue(turret.sysId());
    flywheelSysID.whileTrue(flywheel.sysId());
    
    // YENİDEN YAZILDI: Hub Butonu ile HUB moduna geçme ve bırakınca OFF moduna dönme işlemi
    hubButton.onTrue(Commands.run(() -> shooter.setIntent(ShotIntent.HUB)));
  /*feederAl.whileTrue(feeder.feed()).onFalse(feeder.stop());
    feederTers.whileTrue(feeder.reverse()).onFalse(feeder.stop());

    spindexerF.whileTrue(spindexer.spinForward()).onFalse(spindexer.stop());
    spindexerR.whileTrue(spindexer.spinReverse()).onFalse(spindexer.stop());*/

    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    xLock.whileTrue(Commands.runOnce(() -> s_Swerve.lock(), s_Swerve).repeatedly());

    // TURRET (Sağ-Sol Dönüş) -> %30 güçle
    turretLeftTest.whileTrue(turret.rotateDutyCycle(0.3)).onFalse(turret.stop());
    turretRightTest.whileTrue(turret.rotateDutyCycle(-0.3)).onFalse(turret.stop());
    turretZero.whileTrue(turret.setAngle(Rotations.of(0.25)));

    // HOOD (Yukarı-Aşağı Kalkış) -> %30 güçle
    //hoodUpTest.whileTrue(hood.rotateDutyCycle(0.3)).onFalse(hood.stop());
   // hoodDownTest.whileTrue(hood.rotateDutyCycle(-0.3)).onFalse(hood.stop());

    // ==========================================================
    // KUSURSUZ FLYWHEEL TETİK KOMUTU (SADECE BİR KERE YAZILDI)
    // 2. Kolun (Operator / driver2) Sağ Tetiğini (Axis 3) dinler!
    // ==========================================================
    flywheel.setDefaultCommand(flywheel.runFromTrigger(() -> driver.getRawAxis(3)));
    // Rakip Saha Kontrol Tetikleyicisi
    Trigger opponentSideTrigger = new Trigger(() -> {
        Pose2d pose = s_Swerve.getPose();
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        if (isRed) {
            return pose.getX() < 11.0; 
        } else {
            return pose.getX() > 5.2; 
        }
    });
  }
  
  public Command getAutonomousCommand() { return m_chooser.getSelected(); }
  public void setMotorBrake(boolean brake) { s_Swerve.setMotorBrake(brake); }
  public void resetOdometry(Pose2d pose) { s_Swerve.resetOdometry(pose); }
  public void zeroGyro() { s_Swerve.zeroGyro(); }
}