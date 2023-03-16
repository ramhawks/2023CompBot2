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
public final class Constants {
  
  public static class DriveTrainConstants {
    public static final int PIGEON2_CAN_ID               = 9;
    public static final int LEFT_DRIVE_PRIMARY_CAN_ID    =10;
    public static final int LEFT_DRIVE_SECONDARY_CAN_ID  =11; 
    public static final int RIGHT_DRIVE_PRIMARY_CAN_ID   =12;
    public static final int RIGHT_DRIVE_SECONDARY_CAN_ID =13;

    public static final double DRIVE_WHEEL_DIAMETER       =6; 
    public static final double DRIVE_WHEEL_CIRC           =DRIVE_WHEEL_DIAMETER * Math.PI;
    public static final double GEAR_BOX_RATIO             =8.46;
    public static final double MOTOR_ENCODER_CPR          =42;
    public static final double WHEEL_CPR                  =GEAR_BOX_RATIO * MOTOR_ENCODER_CPR;
    public static final double DRIVE_CPI                  =WHEEL_CPR / DRIVE_WHEEL_CIRC; 
    public static final double SLOW_DRIVE_SCALAR          =.3; 
    public static final double LEFT_DRIVE_KP              = .01;
    public static final double RIGHT_DRIVE_KP             = .01;

    public static final double OPEN_LOOP_RAMP_RATE        =2;
    public static final double TURN_SPEED_SCALER          =4;

  }

public static class ArmSubSystemConstants {
  public static final int ROTATE_CAN_ID                    =14;
  public static final int TILT_CAN_ID                      =15;
  public static final int EXTEND_CAN_ID                    =16;
  public static final double SPIN_RAMP_RATE                =1;
  public static final double TILT_RAMP_RATE                =1;
  public static final double EXTENDO_RAMP_RATE             =1;
  public static final int SPIN_CURRENT_LIMIT               =15;
  public static final int EXTENDO_CURRENT_LIMIT            =27; 
  public static final int TILT_CURRENT_LIMIT               =30;
  public static final int SPIN_UPPER_LIMIT                 =500;
  public static final int SPIN_LOWER_LIMIT                 =-500;
  public static final float EXTENDO_UPPER_LIMIT            =0;
  public static final float EXTENDO_LOWER_LIMIT            =-10;
  public static final double EXTENDO_KP                    =.01;
  public static final float TILT_UPPER_LIMIT               =0;
  public static final float TILT_LOWER_LIMIT               =-30;
  public static final double TILT_GEAR_RATIO               = 100;
  public static final double TILT_ENCODER_CPR              = 1;
  public static final double TILT_CPR                      = TILT_ENCODER_CPR * TILT_GEAR_RATIO;
  public static final double TILT_COUNTS_PER_DEGREE        = TILT_CPR / 360;
  public static final double TILT_KP                       =.01;

  public static final double SPIN_KP                       =.01;


  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort         = 0;
    public static final int kArmControllerPort            = 1;
  }
}
