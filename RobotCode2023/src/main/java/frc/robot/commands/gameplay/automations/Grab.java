// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gameplay.automations;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Grab extends CommandBase {
  /*
   * Combines Arm and Swerve and Vision for robot to
   *  - detect which game piece
   *  - detect its orientation
   *  - drives into position to grab
   *  - grab piece
   *  - stow arm into robot
   */
  /** Creates a new Grab. */
  public Grab(Arm arm) {
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
