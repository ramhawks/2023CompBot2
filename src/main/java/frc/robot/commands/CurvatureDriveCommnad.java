// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubSystem;

public class CurvatureDriveCommnad extends CommandBase {
  /** Creates a new CurvatureDriveCommnad. */
  DoubleSupplier left;
  DoubleSupplier right;
  DoubleSupplier turn;
  BooleanSupplier spin;
  public CurvatureDriveCommnad(DriveTrainSubSystem driveTrain, 
                              DoubleSupplier left, 
                              DoubleSupplier right, 
                              DoubleSupplier turn, 
                              BooleanSupplier spin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.left = left;
    this.right = right;
    this.turn = turn;
    this.spin = spin;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = left.getAsDouble() - right.getAsDouble();
    DriveTrainSubSystem.curvatureDrive(-power, turn.getAsDouble(), spin.getAsBoolean());
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
