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

public class OneAmpAutoBlue extends SequentialCommandGroup{
    public OneAmpAutoBlue(DriveSubsystem m_robotdrive, ArmSubsystem m_robotarm, IntakeSubsystem m_robotintake) {
        addCommands(
            new ParallelCommandGroup (
                new ArmStall(m_robotarm, ArmConstants.ARM_STALL_SPEED_TWO, ArmConstants.ARM_RAISE_SPEED),
                new SequentialCommandGroup (
                    new ArcadeDriveCommand(m_robotdrive, 
                        () -> 0.5,  // DoubleSupplier for speed
                        () -> 0.0   // DoubleSupplier for rotation
                    ).withTimeout(2),
                    new Turn90Degrees(m_robotdrive, false, DriveConstants.AUTO_TURN_TIME), 
                    new ArcadeDriveCommand(m_robotdrive, 
                        () -> -0.5,  
                        () -> 0.0   
                    ).withTimeout(1)
                )
            ),
            new RaiseAndOuttake(m_robotintake, m_robotarm),
            new ArmResetRotation(m_robotarm, ArmConstants.ARM_DOWN_SETPOINT, ArmConstants.ARM_STALL_SPEED)
        );
    }
}
