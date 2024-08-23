package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetRotation extends Command {
    private final ArmSubsystem m_armsubsystem;
    private final double m_pidtargetpoint;

    public ArmSetRotation(ArmSubsystem armsubsystem, double pidtarget) {
        m_pidtargetpoint = pidtarget;
        m_armsubsystem = armsubsystem;
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
        double pidOutput = m_armsubsystem.rotatePID(m_pidtargetpoint);
        m_armsubsystem.setRotate(pidOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_armsubsystem.setRotate(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_armsubsystem.getRotationEncoder() - m_pidtargetpoint) < ArmConstants.ARM_DEGREE_TOLERANCE;
    }


}
