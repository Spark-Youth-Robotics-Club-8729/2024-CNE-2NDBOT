package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Turn90Degrees;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;

public class OneAmpAutoRed extends SequentialCommandGroup{
    public OneAmpAutoRed(DriveSubsystem m_robotdrive, ArmSubsystem m_robotarm, IntakeSubsystem m_robotintake) {
        addCommands(
            new ArcadeDriveCommand(m_robotdrive, 
                () -> 0.7,  
                () -> 0.0   
            ).withTimeout(5),
            new RaiseAndOuttake(m_robotintake, m_robotarm).withTimeout(1),
            new ArmResetRotation(m_robotarm, ArmConstants.ARM_DOWN_SETPOINT, ArmConstants.ARM_STALL_SPEED)
        );
    }
}
