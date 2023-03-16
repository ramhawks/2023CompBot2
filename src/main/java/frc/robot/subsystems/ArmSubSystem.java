// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Externalizable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmSubSystemConstants;

public class ArmSubSystem extends SubsystemBase {

private static final CANSparkMax spin    = new CANSparkMax(ArmSubSystemConstants.ROTATE_CAN_ID, MotorType.kBrushed);
private static final CANSparkMax tilt     = new CANSparkMax(ArmSubSystemConstants.TILT_CAN_ID, MotorType.kBrushless);
private static final CANSparkMax extendo = new CANSparkMax(ArmSubSystemConstants.EXTEND_CAN_ID, MotorType.kBrushless);

private static RelativeEncoder extendoEncoder;
private static RelativeEncoder tiltEncoder;
private static Encoder         spinEncoder; 

private static double tiltSetpoint;
private static double tiltError;

private static double spinSetpoint;
private static double spinError;

private static double extendoSetpoint;
private static double extendoError;


private static Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);                                                                    
private static Solenoid tester = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
private static Solenoid tester2 = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

private static boolean grabberIsOpen;
  /** Creates a new ArmSubSystem. */
  public ArmSubSystem() {
      spin.restoreFactoryDefaults();
      tilt.restoreFactoryDefaults();
      extendo.restoreFactoryDefaults();

      spin.setIdleMode(IdleMode.kBrake);
      tilt.setIdleMode(IdleMode.kBrake);
      extendo.setIdleMode(IdleMode.kBrake);

      spin.setOpenLoopRampRate(ArmSubSystemConstants.SPIN_RAMP_RATE);
      tilt.setOpenLoopRampRate(ArmSubSystemConstants.TILT_RAMP_RATE);
      extendo.setOpenLoopRampRate(ArmSubSystemConstants.EXTENDO_RAMP_RATE);

      spin.setSmartCurrentLimit(ArmSubSystemConstants.SPIN_CURRENT_LIMIT);
      tilt.setSmartCurrentLimit(ArmSubSystemConstants.TILT_CURRENT_LIMIT);
      extendo.setSmartCurrentLimit(ArmSubSystemConstants.EXTENDO_CURRENT_LIMIT);

      tilt.setSoftLimit(SoftLimitDirection.kForward, ArmSubSystemConstants.TILT_UPPER_LIMIT);
      tilt.setSoftLimit(SoftLimitDirection.kReverse, ArmSubSystemConstants.TILT_LOWER_LIMIT);
      tilt.enableSoftLimit(SoftLimitDirection.kForward, true);
      tilt.enableSoftLimit(SoftLimitDirection.kReverse, true);

      extendo.setSoftLimit(SoftLimitDirection.kForward, ArmSubSystemConstants.EXTENDO_UPPER_LIMIT);
      extendo.setSoftLimit(SoftLimitDirection.kReverse, ArmSubSystemConstants.EXTENDO_LOWER_LIMIT);
      extendo.enableSoftLimit(SoftLimitDirection.kForward, true);
      extendo.enableSoftLimit(SoftLimitDirection.kReverse, true);

      spin.burnFlash();
      tilt.burnFlash();
      extendo.burnFlash();

      tiltEncoder = tilt.getEncoder();      
      spinEncoder= new Encoder(0, 1);
      extendoEncoder  = extendo.getEncoder();

      compressor.enableDigital();

      tester2.set(false);
      tester.set(true);

      homeEncoders();
  }

  /***********spin stuff***************************/
  public static void setSpin(double spinPower){
    if( (spinPower < 0 && spinEncoder.getDistance() >= ArmSubSystemConstants.SPIN_UPPER_LIMIT)) {
      spin.setOpenLoopRampRate(0);
      spin.set(0);
      return;
    }
    if( (spinPower > 0 && spinEncoder.getDistance() <= ArmSubSystemConstants.SPIN_LOWER_LIMIT)) {
      spin.setOpenLoopRampRate(0);
      spin.set(0);
      return;
    }
    spin.setOpenLoopRampRate(ArmSubSystemConstants.SPIN_RAMP_RATE);
    spin.set(spinPower);
  }

  public static double getSpinDegrees(){
    return spinEncoder.getDistance() / ArmSubSystemConstants.TILT_COUNTS_PER_DEGREE;
  }
  public static void setSpinSetpoint(double setpoint){
    spinError = setpoint - getSpinDegrees();
    spin.set(spinError * ArmSubSystemConstants.SPIN_KP);
  }
/*************End spin stuff***********************/

/*************Tilt stuff***************************/
  public static void setTilt(double power){
      if(power != 0){
        tilt.set(power);
        tiltSetpoint = getTiltDegrees();
      }else{
        setTiltSetpoint(tiltSetpoint);
      }  
    }

  public static void setTiltSetpoint(double setpoint){
      tiltError = setpoint - getTiltDegrees();
      tilt.set(tiltError * ArmSubSystemConstants.TILT_KP);
    }
  
  public static double getTiltDegrees(){
      return tiltEncoder.getPosition() / ArmSubSystemConstants.TILT_COUNTS_PER_DEGREE;
    }
/************End tilt stuff**************************/

/************Start extendo stuff*********************/
  public static void setExtendo(double power){
    extendo.set(power);
  }

  public static void setExtendoSetpoint(double setpoint){
    extendoError = setpoint - getExtendoDistance();
    extendo.set(extendoError * ArmSubSystemConstants.EXTENDO_KP);
  }

  public static double getExtendoDistance(){
    return extendoEncoder.getPosition();
  }
/************End extendo stuff***********************/

/************Start grabber stuff*********************/
public static void openGrabber() {
    tester2.set(false);
    tester.set(true);
    grabberIsOpen = true;
  }

public static void closeGrabber() {
  tester.set(false);
  tester2.set(true);
  grabberIsOpen = false;
}

public static boolean getGrabberOpen(){return grabberIsOpen;}
/*************End grabber stuff**********************/

public static void homeEncoders(){
  spinEncoder.reset();
  extendoEncoder.setPosition(0);
  tiltEncoder.setPosition(0);

  tiltSetpoint    = 0;
  spinSetpoint    = 0;
  extendoSetpoint = 0;
  
}


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("grabber", getGrabberOpen());
    SmartDashboard.putNumber("tilt", tiltEncoder.getPosition());
    SmartDashboard.putNumber("spin", spinEncoder.getDistance());
    SmartDashboard.putNumber("Extend", extendoEncoder.getPosition());

    SmartDashboard.putNumber("tilt temp", tilt.getMotorTemperature());
    SmartDashboard.putNumber("extend temp", extendo.getMotorTemperature());

    SmartDashboard.putNumber("tilt amps", tilt.getOutputCurrent());
    
  }
}
