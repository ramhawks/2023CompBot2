// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubSystem;

public class Drive2Command extends CommandBase {
  /** Creates a new Drive2Command. */
  private static DoubleSupplier leftDistance;
  private static DoubleSupplier rightDistance;
  public Drive2Command(DriveTrainSubSystem driveTrain, 
                          DoubleSupplier leftDistance, 
                          DoubleSupplier rightDistance) {
    Drive2Command.leftDistance = leftDistance;
    Drive2Command.rightDistance = rightDistance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
      DriveTrainSubSystem.setLeftPosFT(leftDistance.getAsDouble());
      DriveTrainSubSystem.setRightPosFT(rightDistance.getAsDouble());
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
