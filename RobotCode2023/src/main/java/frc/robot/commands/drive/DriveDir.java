// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickButtons;
import frc.robot.subsystems.SwerveDrive;

public class DriveDir extends CommandBase {
  private SwerveDrive m_swerve;
  private double desired_heading;
  private PIDController rotation_controller;

  /** Creates a new DriveDir. */
  public DriveDir(SwerveDrive m_swerve, double desired_heading) {
    addRequirements(m_swerve);
    this.m_swerve = m_swerve;
    this.desired_heading = desired_heading;

    rotation_controller = new PIDController(0.15, 0, 0);
    rotation_controller.enableContinuousInput(0, 360);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double cur_heading = m_swerve.getHeading();
    double rot = rotation_controller.calculate(cur_heading, desired_heading);
    rot = MathUtil.clamp(rot, -1, 1);

    m_swerve.drive(JoystickButtons.m_driverController.getLeftY(), 
                   JoystickButtons.m_driverController.getLeftX(), 
        rot,
                   true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (Math.abs(JoystickButtons.m_operatorController.getRightX()) > 0.05) {
    //   return true;
    // }
    return false;
  }
}
