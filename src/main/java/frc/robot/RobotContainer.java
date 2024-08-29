// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.IntakeSetSpin;
import frc.robot.commands.OneAmpAutoRed;
import frc.robot.commands.RaiseAndOuttake;
import frc.robot.commands.Turn90Degrees;
import frc.robot.commands.ArmSetRotation;
import frc.robot.commands.ArmResetRotation;
import frc.robot.commands.ArmStall;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick; 
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  //The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  // The driver's controller
  CommandXboxController driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  
  // The operator's controller
  CommandXboxController operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //Configure the trigger bindings
    configureBindings();

    // Configure default commands
    driveSubsystem.setDefaultCommand(
            new ArcadeDriveCommand(driveSubsystem,
                            () -> driverController.getRawAxis(DriveConstants.DRIVE_AXIS),
                            (() -> driverController.getRawAxis(DriveConstants.TURN_AXIS)
                                            * DriveConstants.TURN_PROPORTION)));

  }

  public void configureBindings() {
    operatorController.y().whileTrue(new IntakeSetSpin(intakeSubsystem, IntakeConstants.INTAKE_SPIN_MOTOR_SPEED));
    operatorController.x().whileTrue(new IntakeSetSpin(intakeSubsystem, -IntakeConstants.INTAKE_SPIN_MOTOR_SPEED));
    operatorController.b().onTrue(new ArmSetRotation(armSubsystem, ArmConstants.ARM_UP_SETPOINT));
    operatorController.a().onTrue(new ArmResetRotation(armSubsystem, ArmConstants.ARM_DOWN_SETPOINT, ArmConstants.ARM_STALL_SPEED));
    //operatorController.leftBumper().onTrue(new Turn90Degrees(driveSubsystem, true, DriveConstants.AUTO_TURN_TIME));
    //operatorController.rightBumper().onTrue(new Turn90Degrees(driveSubsystem, false, DriveConstants.AUTO_TURN_TIME));
    operatorController.leftBumper().onTrue(new RaiseAndOuttake(intakeSubsystem, armSubsystem));
    //operatorController.rightBumper().onTrue(new ArmStall(armSubsystem, ArmConstants.ARM_STALL_SPEED_TWO, ArmConstants.ARM_RAISE_SPEED));
    operatorController.rightBumper().onTrue(new OneAmpAutoRed(driveSubsystem, armSubsystem, intakeSubsystem));
  }

}