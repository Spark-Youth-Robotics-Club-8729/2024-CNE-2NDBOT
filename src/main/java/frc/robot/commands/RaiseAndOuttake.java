package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;;

public class RaiseAndOuttake extends SequentialCommandGroup {
    public RaiseAndOuttake(IntakeSubsystem m_robotIntake, ArmSubsystem m_robotArm) {
        addCommands(
            new ArmSetRotation(m_robotArm, ArmConstants.ARM_UP_SETPOINT),
            new IntakeSetSpin(m_robotIntake, IntakeConstants.INTAKE_SPIN_MOTOR_SPEED).withTimeout(1)
        );
    }
}