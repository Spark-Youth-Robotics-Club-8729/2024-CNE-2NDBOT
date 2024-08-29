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
import frc.robot.commands.ArmSetRotation;
import frc.robot.commands.ArmResetRotation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick; //update xbox, ps4 in testing!
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
    operatorController.y().whileTrue(new IntakeSetSpin(intakeSubsystem, 0.2));
    operatorController.b().onTrue(new ArmSetRotation(armSubsystem, 95));
    operatorController.x().onTrue(new ArmResetRotation(armSubsystem, 6.5, ArmConstants.INTAKE_SPIN_MOTOR_SPEED));
  }

}