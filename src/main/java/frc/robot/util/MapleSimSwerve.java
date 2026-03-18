package frc.robot.util;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class MapleSimSwerve implements MappleSimInterface{
    // simulatedDrive MappleSim in swerve ü gibi bununla simnule etmemiz lazım ama yagsl SwerveDrive ile simule ediyor obje farklılığından simüle edemdim.
    // drive methodu website da direk altaki gibi yazılmış onu command olarak çağırabiliriz ama FOangularDrive gibi olmaz muhtemelen
    // polymorphism ile yazmayı dendedim(MappleSimSwerve extends SwerveDrive implements MappleSim interface) ki obje uyumsuzluğunun
    // SwerveDrive s = new MappleSimSwerve(); daha sonra s.FOangularDrive ama SwerveDrive ı super() de initialize etmemi istedi o denemede ordan patladı
    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2d;
    
        // Create and configure a drivetrain simulation configuration
    private final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
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

    public MapleSimSwerve() {
        DriveTrainSimulationConfig config = driveTrainSimulationConfig;

        // Creating the SelfControlledSwerveDriveSimulation instance
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, new Pose2d(5, 5, new Rotation2d())));

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        // A field2d widget for debugging
        field2d = new Field2d();
        SmartDashboard.putData("simulation field", field2d);
    }

    /*public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        this.simulatedDrive.runChassisSpeeds(new ChassisSpeeds(translation.getX(), translation.getY(), rotation),new Translation2d(),fieldRelative,true);
    }*/

    
    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop) {
        this.simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), fieldRelative,isOpenLoop);
    }
    
    public Command mapleFieldOrientedDrive(Supplier<ChassisSpeeds> speeds){
        return run(() ->drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getGyroYaw()),false,false));//drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroYaw()),true,false);
    } // one to one copy of yagsl's interpretation of fieldOrientedDrive

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        simulatedDrive.runSwerveStates(desiredStates);
    }


    @Override
    public ChassisSpeeds getMeasuredSpeeds() {
        return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
    }

    @Override
    public Rotation2d getGyroYaw() {
        return simulatedDrive.getRawGyroAngle();
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
        simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,Matrix<N3, N1> visionMeasurementStdDevs) {
        simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        simulatedDrive.periodic(); // update the odometry of the SimplifedSwerveSimulation instance

        // send simulation data to dashboard for testing
        field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        field2d.getObject("odometry").setPose(getPose());
    }
}