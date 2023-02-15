// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gameplay.automations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveTo extends CommandBase {

  private Pose2d pose2d; 


  private SwerveDrive m_SwerveDrive;
  /** Creates a new Approach. */
  /*
   * Robot will drive to specified point (parameter)
   */
  public DriveTo(Pose2d pose2D, SwerveDrive m_SwerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SwerveDrive);
    this.m_SwerveDrive = m_SwerveDrive;
    this.pose2d = pose2D;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_SwerveDrive.driveTo(pose2d);
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
