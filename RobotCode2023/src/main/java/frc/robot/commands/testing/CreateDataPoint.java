// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CreateDataPoint extends CommandBase {
  /** Creates a new CreateDataPoint. */

  /*
   * READ THIS:
   * the purpose of this command is to help with testing by documenting each
   * datapoint
   * recorded in the NetworkTables
   * 
   * move arm or intake to desired position, then execute this command to document
   * angles
   */
  public CreateDataPoint() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
