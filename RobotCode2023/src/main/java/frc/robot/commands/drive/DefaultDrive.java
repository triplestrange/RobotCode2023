// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.JoystickButtons;
import frc.robot.subsystems.SwerveDrive;

public class DefaultDrive extends CommandBase {
  private SwerveDrive m_swerve;
  private double speed;
  private double rotationSpeed;
  private Timer timer = new Timer();
  private double time;
  private double deadzone;
  private PIDController rotation_controller;

  /**
   * Creates a new Drive.
   * 
   * normal = 2.5
   * slow = 0.75
   */
  public DefaultDrive(SwerveDrive m_swerve, double speed, double rotationSpeed) {
    addRequirements(m_swerve);
    this.m_swerve = m_swerve;
    this.speed = speed;
    this.rotationSpeed = rotationSpeed;
    speed = MathUtil.clamp(speed, -Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        Constants.SwerveConstants.kMaxSpeedMetersPerSecond);
    deadzone = 0.05;
    rotation_controller = new PIDController(0.075, 0, 0.0);
    rotation_controller.enableContinuousInput(0, 360);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedR;

    if (Math.abs(JoystickButtons.m_driverController.getRightX()) > 0.075) {
      m_swerve.setPresetEnabled(false);
    }

    double rot = rotation_controller.calculate(m_swerve.getAngle().getDegrees(), m_swerve.getRotationPreset());
    // rot = MathUtil.clamp(rot, -1, 1);
    // System.out.println("rotation PID: " + rot);

    // System.out.println("gyro: " + m_swerve.getAngle().getDegrees());
    // System.out.println("desiredHeading: " + m_swerve.getRotationPreset());
    double speedY = JoystickButtons.m_driverController.getLeftY() * speed;
    double speedX = JoystickButtons.m_driverController.getLeftX() * speed;

    if (m_swerve.getPresetEnabled()) {

      speedR = rot;
    } else {
      if (Math.abs(JoystickButtons.m_driverController.getRightX()) <= deadzone) {
        speedR = 0;
      }
      // rotation was reversed
      else {
        speedR = JoystickButtons.m_driverController.getRightX() * -4 * rotationSpeed;
      }
    }

    if (Math.abs(JoystickButtons.m_driverController.getLeftY()) <= deadzone) {
      speedY = 0;
    }
    if (Math.abs(JoystickButtons.m_driverController.getLeftX()) <= deadzone) {
      speedX = 0;
    }

    m_swerve.drive(
        -speedY,
        -speedX,
        speedR,
        true);

    SmartDashboard.putNumber("left Y", JoystickButtons.m_driverController.getLeftY());
    SmartDashboard.putNumber("left X", JoystickButtons.m_driverController.getLeftX());
    SmartDashboard.putNumber("right X", JoystickButtons.m_driverController.getRightX());

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
