// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickButtons;
import frc.robot.subsystems.SwerveDrive;

public class ConeAlign extends CommandBase {
  private SwerveDrive m_swerve;
  private PIDController hor;
  /** Creates a new ConeAlign. */
  public ConeAlign(SwerveDrive m_swerve) {
    hor = new PIDController(0.05, 0, 0);
    addRequirements(m_swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double setpoint = 0;

    if (Math.abs(tx - setpoint) <= 2) {
      tx = 0;
    }

    if (tv > 0) {
      m_swerve.drive(JoystickButtons.m_driverController.getLeftY(),
                     hor.calculate(tx, setpoint),
                     JoystickButtons.m_driverController.getRightX(),
                     true);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(JoystickButtons.m_driverController.getLeftX()) > 0.05) {
      return true;
    }
    return false;
  }
}
