package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmResetRotation extends Command {
    private final ArmSubsystem m_armsubsystem;
    private final double m_pidtargetpoint;
    private final double m_stallspeed;
    //testing
    public ArmResetRotation(ArmSubsystem armsubsystem, double pidtarget, double stallspeed) {
        m_pidtargetpoint = pidtarget;
        m_armsubsystem = armsubsystem;
        m_stallspeed = stallspeed;
        addRequirements(armsubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double pidoutput = m_armsubsystem.rotateDownPID(m_pidtargetpoint);
        m_armsubsystem.setRotate(pidoutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (isFinished() && !interrupted) {
            // Apply stall speed when target is reached
            m_armsubsystem.setRotate(m_stallspeed);
        } else {
            // Stop the motor if the command is interrupted
            m_armsubsystem.setRotate(0.0);
        }
        // m_armsubsystem.setRotate(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_armsubsystem.getRotationEncoder() - m_pidtargetpoint) < ArmConstants.ARM_DEGREE_TOLERANCE;
    }

}