// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubSystem;

public class diffDriveCommand extends CommandBase {
  /** Creates a new diffDriveCommand. */
  static DoubleSupplier leftPower; 
  static DoubleSupplier rightPower; 
  static DoubleSupplier turn;
  public diffDriveCommand(DriveTrainSubSystem driveTrain, 
                              DoubleSupplier leftPower, 
                              DoubleSupplier rightPower, 
                              DoubleSupplier turn) {
    // Use addRequirements() here to declare subsystem dependencies.

    //DefaultDriveCommand.driveTrain = driveTrain; 
    diffDriveCommand.leftPower = leftPower;
    diffDriveCommand.rightPower = rightPower;
    diffDriveCommand.turn = turn;
    
    addRequirements(driveTrain);
                              }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = rightPower.getAsDouble() - leftPower.getAsDouble();
    DriveTrainSubSystem.setDiffDrive(power, turn.getAsDouble());
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
