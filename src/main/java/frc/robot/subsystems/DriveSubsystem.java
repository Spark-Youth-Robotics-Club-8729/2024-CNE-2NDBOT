package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

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
    }

    public void setMotor(double forwardSpeed, double turnSpeed) {
        driveRobot.arcadeDrive(filter.calculate(forwardSpeed), turnSpeed);
    }

    @Override
    public void periodic() {
        // Update odometry and field visualization
        pose = odometry.update(gyro.getRotation2d(), encoderLeftDrive.getDistance(), encoderRightDrive.getDistance());
        field.setRobotPose(pose);
    }

}
