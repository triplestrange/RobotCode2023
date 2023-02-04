// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gameplay.automations;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Score extends CommandBase {
  private final Arm arm;
  /** Creates a new Score. */
  /*
   * Should do the following:
   *  - precondition: already has AprilTag in sight
   *  - precondition: knows which game piece it holds
   *  - precondition: key already pressed for which to score in
   *  - drives into position to score
   *  - puts arm at correct angle to reach
   *  - releases game piece
   */
  public Score(Arm arm) {
    addRequirements(arm);
    this.arm = arm;
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
