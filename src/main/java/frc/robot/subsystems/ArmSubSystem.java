// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmSubSystemConstants;

public class ArmSubSystem extends SubsystemBase {

private static final CANSparkMax spin    = new CANSparkMax(ArmSubSystemConstants.ROTATE_CAN_ID, MotorType.kBrushed);
private static final CANSparkMax arm     = new CANSparkMax(ArmSubSystemConstants.TILT_CAN_ID, MotorType.kBrushless);
private static final CANSparkMax extendo = new CANSparkMax(ArmSubSystemConstants.EXTEND_CAN_ID, MotorType.kBrushless);

  /** Creates a new ArmSubSystem. */
  public ArmSubSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
