// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */


//ALL CONSTANT TBD/REVISED
public final class Constants {

  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double DriveDeadband = 0.05;
  }

  public static class DriveConstants {
    //ports
    public static final int DRIVE_FRONT_LEFT = 2;
    public static final int DRIVE_BACK_LEFT = 3;
    public static final int DRIVE_FRONT_RIGHT = 4;
    public static final int DRIVE_BACK_RIGHT = 5;
    //values
    //public static final int LEFT_ENCODER_A = 0;
    //public static final int LEFT_ENCODER_B = 1;
    //public static final int RIGHT_ENCODER_A = 2;
    //public static final int RIGHT_ENCODER_B = 3;
    public static final int DRIVE_AXIS = 1;
    public static final int TURN_AXIS = 4;
    public static final double TURN_PROPORTION = 0.7;
    public static final double DRIVE_SLOW = 0.5;
    public static final double TURN_SLOW = 0.8;
    public static final double TURN_KP = 0.017;
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 0.0;
    public static final int TURN_CONTROLLER_POSITION_TOLERANCE = 1;
    public static final int TURN_CONTROLLER_VELOCITY_TOLERANCE = 10;
  }

  public static class ArmConstants {
    public static final int ARM_MOTOR = 6;
    public static final double ARM_KP = 0.01;
    public static final double ARM_KI = 0.0;
    public static final double ARM_KD = 0.0;
    public static final double ARM_DEGREE_TOLERANCE = 1.0;

    public static final double INTAKE_SPIN_MOTOR_SPEED = 0.3;


  }

  public static final class IntakeConstants {
    public static final double INTAKE_STALL_SPEED = 0.0205;
    public static final int INTAKE_SPIN_MOTOR_CAN_ID = 1; 
  }

}

