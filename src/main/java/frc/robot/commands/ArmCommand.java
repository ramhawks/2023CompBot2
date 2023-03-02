// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubSystem;



public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  private DoubleSupplier spin;
  private DoubleSupplier tilt;
  private DoubleSupplier extend;
  public ArmCommand(ArmSubSystem arm, DoubleSupplier spin, DoubleSupplier tilt, DoubleSupplier extend) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.spin = spin;
    this.tilt = tilt;
    this.extend = extend;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmSubSystem.setSpin(spin.getAsDouble() * .2);
    ArmSubSystem.setExtendo(extend.getAsDouble() * .2);
    ArmSubSystem.setTilt(tilt.getAsDouble() * .15);
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
