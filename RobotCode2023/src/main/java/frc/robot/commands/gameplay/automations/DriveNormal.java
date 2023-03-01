// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gameplay.automations;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickButtons;
import frc.robot.subsystems.SwerveDrive;

public class DriveNormal extends CommandBase {
  SwerveDrive m_swerveDrive;

  /*
   * Combines Arm and Swerve and Vision for robot to
   *  - detect which game piece
   *  - detect its orientation
   *  - drives into position to grab
   *  - grab piece
   *  - stow arm into robot
   */
  /** Creates a new Grab. */
  public DriveNormal (SwerveDrive m_SwerveDrive) {
    addRequirements(m_SwerveDrive);
    this.m_swerveDrive = m_SwerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 1;
    m_swerveDrive.drive(
      JoystickButtons.m_driverController.getLeftY() * speed,
      JoystickButtons.m_driverController.getLeftX() * speed,
      -JoystickButtons.m_driverController.getRightX() * 4,
      true
    );
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
