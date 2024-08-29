package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmStall extends Command{
    private final ArmSubsystem armSubsystem; 
    private final double raiseSpeed;
    private final double stallSpeed;
    private long startTime;


    public ArmStall(ArmSubsystem subsystem, double stallSpeed, double raiseSpeed) {
        this.armSubsystem = subsystem;
        this.stallSpeed = stallSpeed;
        this.raiseSpeed = raiseSpeed;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        armSubsystem.setRotate(raiseSpeed);

    }

    @Override
    public void execute() {
        if ((System.currentTimeMillis() - startTime) >= ArmConstants.ARM_RAISE_TIME * 1000) {
            armSubsystem.setRotate(stallSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setRotate(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
