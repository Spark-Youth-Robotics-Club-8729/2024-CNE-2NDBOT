package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR);
    private final PIDController armPidController = new PIDController(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    
    public ArmSubsystem() {
        armPidController.setTolerance(ArmConstants.ARM_DEGREE_TOLERANCE);
    }

    public void setRotate(double speed) {
        armMotor.set(speed);
    }

    public double rotatePID(double PIDTarget) {
        double pidOutput = armPidController.calculate(armEncoder.getPosition(), PIDTarget);
        return MathUtil.clamp(pidOutput, -0.5, 0.5);
    }

    public double getRotation() {
        return armMotor.get();
    }

    public double getRotationEncoder() {
        return armEncoder.getPosition();
    }

    public void resetRotationEncoder() {
        armEncoder.setPosition(0.0);
    }

    public void periodic() {
        SmartDashboard.putNumber("Rotation Encoder", getRotationEncoder());
        SmartDashboard.putNumber("Current Rotation Speed", getRotation());
        SmartDashboard.putData("Encoder PID Data", armPidController);
        SmartDashboard.putNumber("Sim Rotation Encoder", armEncoder.getPosition());
        logOutputs();
    }

}
