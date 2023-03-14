// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.JoystickButtons;
import frc.robot.subsystems.SwerveDrive;

public class DriveTurbo extends CommandBase {
  SwerveDrive m_swerveDrive;
  // Timer timer = new Timer();

  /*
   * Combines Arm and Swerve and Vision for robot to
   *  - detect which game piece
   *  - detect its orientation
   *  - drives into position to grab
   *  - grab piece
   *  - stow arm into robot
   */
  /** Creates a new Grab. */
  public DriveTurbo (SwerveDrive m_SwerveDrive) {
    addRequirements(m_SwerveDrive);
    this.m_swerveDrive = m_SwerveDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double speed = (Constants.SwerveConstants.kMaxSpeedMetersPerSecond - 1) * timer.get()/0.1 + 1;
    double speed = Constants.SwerveConstants.kMaxSpeedMetersPerSecond;
    speed = MathUtil.clamp(speed, -Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        Constants.SwerveConstants.kMaxSpeedMetersPerSecond);

    double speedY = JoystickButtons.m_driverController.getLeftY() * speed;
    double speedX = JoystickButtons.m_driverController.getLeftX() * speed;
    double speedR = JoystickButtons.m_driverController.getRightX() * -4;

    if (Math.abs(JoystickButtons.m_driverController.getLeftY()) <= .1) {
      speedY = 0;
    }
    if (Math.abs(JoystickButtons.m_driverController.getLeftX()) <= .1) {
      speedX = 0;
    }
    if (Math.abs(JoystickButtons.m_driverController.getRightX()) <= .1) {
      speedR = 0;
    }

    SmartDashboard.putNumber("left Y", JoystickButtons.m_driverController.getLeftY());
    SmartDashboard.putNumber("left X", JoystickButtons.m_driverController.getLeftX());
    SmartDashboard.putNumber("right X", JoystickButtons.m_driverController.getRightX());

    m_swerveDrive.drive(
        speedY,
        speedX,
        speedR,
        true);
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
