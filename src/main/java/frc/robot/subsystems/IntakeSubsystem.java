package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    CANSparkMax IntakeSpinMotor;

    public IntakeSubsystem() {
        IntakeSpinMotor = new CANSparkMax(IntakeConstants.INTAKESPINMOTORCANID, MotorType.kBrushed);
        IntakeSpinMotor.setInverted(true);
    }

    public void setSpin(double speed) {
        IntakeSpinMotor.set(speed);
    }

    public void periodic() {
        SmartDashboard.putNumber("Current Intake Speed", getSpin());

    }

    public double getSpin(){
        return IntakeSpinMotor.get();
    }

}
