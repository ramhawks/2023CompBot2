// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TelemetrySubsystem extends CommandBase {
  /** Creates a new TelemetrySubsystem. */
  public TelemetrySubsystem() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Pitch", DriveTrainSubSystem.getPitch());
    SmartDashboard.putNumber("Yaw", DriveTrainSubSystem.getYaw());
    SmartDashboard.putNumber("Roll", DriveTrainSubSystem.getRoll());
    SmartDashboard.putNumber("Leftpos", DriveTrainSubSystem.getLeftposFT());
    SmartDashboard.putNumber("Rightpos", DriveTrainSubSystem.getRightposFT());

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
