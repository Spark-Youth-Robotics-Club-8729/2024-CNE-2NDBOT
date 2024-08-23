package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.lang.System.LoggerFinder;

import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem {
    CANSparkMax IntakeSpinMotor;

    private final String INTAKE_SPIN_LOG_PATH = "/Spin";

    public IntakeSubsystem() {
        IntakeSpinMotor = new CANSparkMax(IntakeConstants.IntakeSpinMotorCanID, MotorType.kBrushed);
        IntakeSpinMotor.setInverted(true);
    }

    public void setSpin(double speed) {
        IntakeSpinMotor.set(speed);
    }

    public void periodic() {
        SmartDashboard.putNumber("Current Intake Speed", getSpin());

    }

    // Log IntakeSubsystem Outputs for help when testing
    public void logOutputs() {
        Logger.recordOutput(getName() + INTAKE_SPIN_LOG_PATH, getSpin());
    }

    public double getSpin(){
        return IntakeSpinMotor.get();
    }

}
