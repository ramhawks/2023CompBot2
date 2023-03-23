// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class lightingSubsystem extends SubsystemBase {
  /** Creates a new lightingSubsystem. */
  private static Victor redPWM;
  private static Victor greenPWM;
  private static Victor bluePWM;


  public lightingSubsystem() {
    redPWM = new Victor(0);
    greenPWM = new Victor(1);
    bluePWM = new Victor(2);
  }

  public static void setRed(double setPoint){
    if(setPoint < 0) setPoint = 0;
    else if (setPoint > 1) setPoint = 1;
    redPWM.set(setPoint);
  }

  public static void setGreen(double setPoint){
    if(setPoint < 0) setPoint = 0;
    else if (setPoint > 1) setPoint = 1;
    greenPWM.set(setPoint);
  }

  public static void setBlue(double setPoint){
    if(setPoint < 0) setPoint = 0;
    else if (setPoint > 1) setPoint = 1;
    bluePWM.set(setPoint);
  }

  @Override
  public void periodic() {

  }
}
