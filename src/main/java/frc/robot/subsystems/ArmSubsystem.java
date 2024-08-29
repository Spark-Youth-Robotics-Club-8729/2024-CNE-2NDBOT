package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.revrobotics.RelativeEncoder;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR);
    private final PIDController armPidController = new PIDController(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
    private final PIDController armDownPidController = new PIDController(ArmConstants.ARM_DOWN_KP, ArmConstants.ARM_DOWN_KI, ArmConstants.ARM_DOWN_KD);


    public ArmSubsystem() {
        armPidController.setTolerance(ArmConstants.ARM_DEGREE_TOLERANCE);
        armDownPidController.setTolerance(ArmConstants.ARM_DEGREE_TOLERANCE);
        resetRotationEncoder();
    }

    public void setRotate(double speed) {
        //armMotor.set(TalonFXControlMode.PercentOutput, speed);
        armMotor.set(speed);
    }

    public double getRotationEncoder() {
        double currentValue = armMotor.getPosition().getValue();
        return currentValue;
    }

    public double rotatePID(double PIDTarget) {
        double pidOutput = armPidController.calculate(getRotationEncoder(), PIDTarget);
        return MathUtil.clamp(pidOutput, -0.5, 0.5);
    }

    public double rotateDownPID(double PIDTarget) {
        double pidOutput = armDownPidController.calculate(getRotationEncoder(), PIDTarget);
        return MathUtil.clamp(pidOutput, -0.5, 0.5);
    }


    public void resetRotationEncoder() {
        armMotor.setPosition(0.0);
    }

    public void periodic() {
        SmartDashboard.putNumber("Rotation Encoder", getRotationEncoder());
        SmartDashboard.putData("Encoder PID Data", armPidController);
        SmartDashboard.putData("Encoder PID Down Data", armDownPidController);
        // logOutputs();
    }

}