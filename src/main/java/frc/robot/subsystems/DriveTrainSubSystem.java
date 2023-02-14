// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DriveTrainConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubSystem extends SubsystemBase {

private static final Pigeon2     pigeonIMU      = new Pigeon2(DriveTrainConstants.PIGEON2_CAN_ID);
private static final CANSparkMax leftPrimary    = new CANSparkMax(DriveTrainConstants.LEFT_DRIVE_PRIMARY_CAN_ID, MotorType.kBrushless);
private static final CANSparkMax leftSecondary  = new CANSparkMax(DriveTrainConstants.LEFT_DRIVE_SECONDARY_CAN_ID, MotorType.kBrushless);
private static final CANSparkMax rightPrimary   = new CANSparkMax(DriveTrainConstants.RIGHT_DRIVE_PRIMARY_CAN_ID, MotorType.kBrushless);
private static final CANSparkMax rightSecondary = new CANSparkMax(DriveTrainConstants.RIGHT_DRIVE_SECONDARY_CAN_ID, MotorType.kBrushless);
private static  RelativeEncoder leftEncoder;
private static RelativeEncoder  rightEncoder;
private static DifferentialDrive diffDrive = new DifferentialDrive(rightPrimary, leftPrimary);

  public DriveTrainSubSystem() {
    leftPrimary.restoreFactoryDefaults();   leftSecondary.restoreFactoryDefaults();
    rightPrimary.restoreFactoryDefaults();  rightSecondary.restoreFactoryDefaults();

    leftPrimary.setIdleMode(IdleMode.kCoast);  leftSecondary.setIdleMode(IdleMode.kCoast);
    rightPrimary.setIdleMode(IdleMode.kCoast); rightSecondary.setIdleMode(IdleMode.kCoast);

    rightPrimary.setInverted(true); rightSecondary.setInverted(true);

    leftSecondary.follow(leftPrimary); rightSecondary.follow(rightPrimary);

    leftPrimary.setOpenLoopRampRate(DriveTrainConstants.OPEN_LOOP_RAMP_RATE); 
    rightPrimary.setOpenLoopRampRate(DriveTrainConstants.OPEN_LOOP_RAMP_RATE);

    leftEncoder = leftPrimary.getEncoder(); 
    rightEncoder = rightPrimary.getEncoder();

    leftPrimary.burnFlash(); leftSecondary.burnFlash();
    rightPrimary.burnFlash(); rightSecondary.burnFlash();
  }

  public static void GTA_Drive(double leftPower, double rightPower, double turn) {
    setNeutral();

    double leftPow = ((rightPower - leftPower) - turn);
    if(leftPow < 0)leftPow = -(leftPow * leftPow);
    else leftPow = (leftPow * leftPow);
    
    double rightPow = ((rightPower - leftPower) + turn);
    if(rightPow < 0)rightPow = -(rightPow * rightPow);
    else rightPow = rightPow * rightPow;

    setMotors(leftPow, rightPow);
  }

  public static void GTA_Drive_Slow(double leftPower, double rightPower, double turn) {
    setNeutral();

    double leftPowerMapped = (rightPower - leftPower) - turn;
    leftPowerMapped = mapDouble(leftPowerMapped, -2, 2, -DriveTrainConstants.SLOW_DRIVE_SCALAR, DriveTrainConstants.SLOW_DRIVE_SCALAR);
    double rightPowerMapped = (leftPower - rightPower) - turn;
    rightPowerMapped = mapDouble(rightPowerMapped, -2, 2, -DriveTrainConstants.SLOW_DRIVE_SCALAR, DriveTrainConstants.SLOW_DRIVE_SCALAR);
    setMotors(leftPowerMapped, -rightPowerMapped);

  }

  public static void curvatureDrive(double power, double turn, Boolean spin) {
    setNeutral();
    diffDrive.curvatureDrive(power, turn, spin);
  }

  public static void setMotors(double leftPower, double rightPower){
    leftPrimary.set(leftPower); rightPrimary.set(rightPower);
  }

  public static void setNeutral(){
    leftPrimary.setIdleMode(IdleMode.kCoast);  leftSecondary.setIdleMode(IdleMode.kCoast);
    rightPrimary.setIdleMode(IdleMode.kCoast); rightSecondary.setIdleMode(IdleMode.kCoast);
    leftPrimary.setOpenLoopRampRate(DriveTrainConstants.OPEN_LOOP_RAMP_RATE); 
    rightPrimary.setOpenLoopRampRate(DriveTrainConstants.OPEN_LOOP_RAMP_RATE); 
  }

  public static void setBrake(){
    leftPrimary.setIdleMode(IdleMode.kBrake); leftSecondary.setIdleMode(IdleMode.kBrake);
    rightPrimary.setIdleMode(IdleMode.kBrake); rightSecondary.setIdleMode(IdleMode.kBrake);
    leftPrimary.setOpenLoopRampRate(0);
    rightPrimary.setOpenLoopRampRate(0);
    setMotors(0, 0);
  }

    public static double getYaw()  { return pigeonIMU.getYaw(); }
    public static double getPitch(){ return pigeonIMU.getPitch();}
    public static double getRoll() { return pigeonIMU.getRoll();}

    public static double getLeftpos(){ return leftEncoder.getPosition();}
    public static double getRightpos(){ return rightEncoder.getPosition();}

    public static double getLeftposFT(){ 
    return ((leftEncoder.getPosition() / DriveTrainConstants.GEAR_BOX_RATIO) 
                * DriveTrainConstants.DRIVE_WHEEL_CIRC) / 12;}

    public static double getRightposFT(){
      return ((rightEncoder.getPosition() / DriveTrainConstants.GEAR_BOX_RATIO)
                * DriveTrainConstants.DRIVE_WHEEL_CIRC) / 12;}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  // Mapping the inputs.
  private static double mapDouble(double valueIn, double baseMin, double baseMax, double limitMin, double limitMax) {
    return ((limitMax - limitMin) * (valueIn - baseMin) / (baseMax - baseMin)) + limitMin;
  }
}
