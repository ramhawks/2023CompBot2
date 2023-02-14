// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubSystem;

public class DefaultDriveCommand extends CommandBase {
  /** Creates a new DefaultDriveCommand. */
  //public static DriveTrainSubSystem driveTrain;
  static DoubleSupplier leftPower; 
  static DoubleSupplier rightPower; 
  static DoubleSupplier turn;
  public DefaultDriveCommand(DriveTrainSubSystem driveTrain, 
                              DoubleSupplier leftPower, 
                              DoubleSupplier rightPower, 
                              DoubleSupplier turn) {
    // Use addRequirements() here to declare subsystem dependencies.

    //DefaultDriveCommand.driveTrain = driveTrain; 
    DefaultDriveCommand.leftPower = leftPower;
    DefaultDriveCommand.rightPower = rightPower;
    DefaultDriveCommand.turn = turn;
    
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveTrainSubSystem.GTA_Drive(leftPower.getAsDouble(), rightPower.getAsDouble(), -turn.getAsDouble());   
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
