package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeAndRaise extends SequentialCommandGroup {
    public IntakeAndRaise(IntakeSubsystem m_robotIntake, ArmSubsystem m_robotArm) {
        addCommands(
            new IntakeSetSpin(m_robotIntake, 0.05).withTimeout(0.5),
            new ArmSetRotation(m_robotArm, 95)
        );
    }
}