package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

public class DriveSubsystem extends SubsystemBase{
    private final WPI_VictorSPX driveFrontLeft = new WPI_VictorSPX(DriveConstants.DRIVE_FRONT_LEFT);
    private final WPI_VictorSPX driveBackLeft = new WPI_VictorSPX(DriveConstants.DRIVE_BACK_LEFT);
    private final WPI_VictorSPX driveFrontRight = new WPI_VictorSPX(DriveConstants.DRIVE_FRONT_RIGHT);
    private final WPI_VictorSPX driveBackRight = new WPI_VictorSPX(DriveConstants.DRIVE_BACK_RIGHT);

    private final DifferentialDrive driveRobot = new DifferentialDrive(driveFrontLeft, driveFrontRight);
    private final PIDController turnController = new PIDController(DriveConstants.TURN_KP, DriveConstants.TURN_KI,
        DriveConstants.TURN_KD);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final Encoder encoderLeftDrive = new Encoder(DriveConstants.LEFT_ENCODER_A, DriveConstants.LEFT_ENCODER_B);
    private final Encoder encoderRightDrive = new Encoder(DriveConstants.RIGHT_ENCODER_A, DriveConstants.RIGHT_ENCODER_B);
    private final Field2d field = new Field2d();
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d(),
        encoderLeftDrive.getDistance(), encoderRightDrive.getDistance());

    Pose2d pose;

    SlewRateLimiter filter = new SlewRateLimiter(3.8);

    public DriveSubsystem() {
        driveBackLeft.follow(driveFrontLeft);
        driveBackRight.follow(driveFrontRight);
        driveFrontRight.setInverted(true);

        driveFrontLeft.setNeutralMode(NeutralMode.Brake);
        driveFrontRight.setNeutralMode(NeutralMode.Brake);
        driveBackRight.setNeutralMode(NeutralMode.Brake);
        driveBackLeft.setNeutralMode(NeutralMode.Brake);
        turnController.setTolerance(DriveConstants.TURN_CONTROLLER_POSITION_TOLERANCE,
            DriveConstants.TURN_CONTROLLER_VELOCITY_TOLERANCE);
        SmartDashboard.putData("Field", field);


        //https://pathplanner.dev/pplib-build-an-auto.html#create-a-sendablechooser-with-all-autos-in-project
        AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public double getEncoderDrivePosition() {
        return (encoderLeftDrive.getDistance());
    }

    public double getGyroYaw() {
        return (gyro.getYaw());
    }

    public double getGyroPitch() {
        return (gyro.getPitch());
    }

    public void resetGyro() {
        gyro.reset();
    }

    public double getGyroRoll() {
        return (gyro.getRoll());
    }

    public double getTurnControllerSpeedRight() {
        return (turnController.calculate(getGyroYaw(), 90));
    }

    public double getTurnControllerSpeedLeft() {
        return (turnController.calculate(getGyroYaw(), -90));
    }

    public boolean atSetpoint() {
        return (turnController.atSetpoint());
    }

    public void setMotor(double forwardSpeed, double turnSpeed) {
        driveRobot.arcadeDrive(filter.calculate(forwardSpeed), turnSpeed);
    }

    public Pose2d getPose() {
        pose = odometry.update(gyro.getRotation2d(), encoderLeftDrive.getDistance(), encoderRightDrive.getDistance());
        return pose;
    }

    public void resetPose(Pose2d newPose) {
        DifferentialDriveWheelPositions wheelPositions = new DifferentialDriveWheelPositions(
            encoderLeftDrive.getDistance(),
            encoderRightDrive.getDistance()
        );

        odometry.resetPosition(gyro.getRotation2d(), wheelPositions, newPose);

        pose = newPose;
    }

    public ChassisSpeeds getCurrentSpeeds() {
        // Get the current speeds from the left and right encoders
        double leftSpeed = encoderLeftDrive.getRate();  // Speed in meters per second
        double rightSpeed = encoderRightDrive.getRate(); // Speed in meters per second

        // Get the current rotation rate from the gyro
        double rotationRate = gyro.getRate(); // Rotation rate in radians per second

        // Create and return the ChassisSpeeds object
        return new ChassisSpeeds(
            (leftSpeed + rightSpeed) / 2.0, // Average forward speed
            0.0, // No lateral movement for differential drive
            rotationRate // Rotational speed
        );
    }

    public void drive(ChassisSpeeds speeds) {
        // Convert the desired chassis speeds to forward and turn speeds
        double forwardSpeed = speeds.vxMetersPerSecond;
        double turnSpeed = speeds.omegaRadiansPerSecond;
    
        // Use arcadeDrive to set the motor outputs
        setMotor(forwardSpeed, turnSpeed);
    }


    @Override
    public void periodic() {
        // Update odometry and field visualization
        pose = getPose();
        field.setRobotPose(pose);
    }

    

}
