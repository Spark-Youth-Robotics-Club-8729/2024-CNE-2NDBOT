package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

public class Turn90Degrees extends Command{
    private final DriveSubsystem driveSubsystem;
    private final boolean turnLeft;
    private final double duration;
    private long startTime;

    public Turn90Degrees(DriveSubsystem subsystem, boolean left, double time) {
        driveSubsystem = subsystem;
        if (left) {
            turnLeft = true;
        } else {
            turnLeft = false;
        }
        duration = time;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        if (turnLeft) {
            driveSubsystem.setTurnMotor(DriveConstants.AUTO_TURN_SPEED); 
        } else {
            driveSubsystem.setTurnMotor(-DriveConstants.AUTO_TURN_SPEED);  
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setTurnMotor(0); 
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) >= duration * 1000;
    }

}
