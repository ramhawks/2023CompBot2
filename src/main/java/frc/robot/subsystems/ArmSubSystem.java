// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

      //tilt.setSoftLimit(SoftLimitDirection.kForward, 0);
      //tilt.setSoftLimit(SoftLimitDirection.kReverse, -25);

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

  public static void setSpin(double spinPower){
    spin.set(spinPower);

    /*if( (spinPower > 0 && getSpinDegrees() >= ArmSubSystemConstants.TILT_UPPER_LIMIT) ||
    (spinPower < 0 && getSpinDegrees() <= ArmSubSystemConstants.TILT_LOWER_LIMIT) ) return;
    if(spinPower != 0){
      spin.set(spinPower);
      spinSetpoint = getSpinDegrees();
    }else{
      spinError = spinSetpoint - getSpinDegrees();
      spin.set(spinError * ArmSubSystemConstants.SPIN_KP);
    }*/
  }

  public static double getSpinDegrees(){
    return spinEncoder.getDistance() / ArmSubSystemConstants.TILT_COUNTS_PER_DEGREE;
  }

  public static void setTilt(double power){
     //tilt.set(power);
    
    //if( (power > 0 && getTiltDegrees() >= ArmSubSystemConstants.TILT_UPPER_LIMIT)  ||
      //(power < 0 && getTiltDegrees() <= ArmSubSystemConstants.TILT_LOWER_LIMIT) ) {
        //setTiltSetpoint(tiltSetpoint);
        //return;
      //}

      if(power != 0){
        tilt.set(power);
        tiltSetpoint = getTiltDegrees();
      }else{
        setTiltSetpoint(tiltSetpoint);
      }  
    }

    public static void setTiltSetpoint(double setpoint){
      tiltError = tiltSetpoint - getTiltDegrees();
      tilt.set(tiltError * ArmSubSystemConstants.TILT_KP);
    }
  
    public static double getTiltDegrees(){
      return tiltEncoder.getPosition() / ArmSubSystemConstants.TILT_COUNTS_PER_DEGREE;
    }

  public static void setExtendo(double power){
    extendo.set(power);

    /*if( (power > 0 && getExtendoDistance() >= ArmSubSystemConstants.EXTENDO_UPPER_LIMIT)  ||
    (power < 0 && getExtendoDistance() <= ArmSubSystemConstants.EXTENDO_LOWER_LIMIT) ) return;

    if(power != 0){
      tilt.set(power);
      tiltSetpoint = getExtendoDistance();
    }else{
      tiltError = tiltSetpoint - getExtendoDistance();
      tilt.set(tiltError * ArmSubSystemConstants.EXTENDO_KP);
    }*/
  }

  public static double getExtendoDistance(){
    return extendoEncoder.getPosition();
  }

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
public static void homeEncoders(){
  spinEncoder.reset();
  extendoEncoder.setPosition(0);
  tiltEncoder.setPosition(0);

  tiltSetpoint = 0;
}


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("grabber", getGrabberOpen());
    SmartDashboard.putNumber("tilt", getTiltDegrees());
    SmartDashboard.putNumber("spin", getSpinDegrees());
    SmartDashboard.putNumber("Extend", getExtendoDistance());

    SmartDashboard.putNumber("tilt temp", tilt.getMotorTemperature());
    SmartDashboard.putNumber("extend temp", extendo.getMotorTemperature());

    SmartDashboard.putNumber("tilt amps", tilt.getOutputCurrent());
    
  }
}
