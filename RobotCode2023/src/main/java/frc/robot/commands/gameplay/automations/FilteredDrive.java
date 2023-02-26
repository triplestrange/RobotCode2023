// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gameplay.automations;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class FilteredDrive extends CommandBase {

  private final SwerveDrive m_swerveDrive;
  private final DoubleSupplier xInput, yInput, rInput;
  private double x, y, r;
  private final double timeConst = 0.2;
  private final double alpha = 0.02 / (timeConst + 0.02);

  /** Creates a new Approach. */
  /*
   * Robot will drive to specified point (parameter)
   */
  public FilteredDrive(SwerveDrive swerveDrive, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
    m_swerveDrive = swerveDrive;
    this.xInput = xInput;
    this.yInput = yInput;
    this.rInput = rInput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x = y = r = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = alpha * xInput.getAsDouble() + (1-alpha) * x;
    y = alpha * yInput.getAsDouble() + (1-alpha) * y;
    r = alpha * rInput.getAsDouble() + (1-alpha) * r;
    m_swerveDrive.drive(x, y, r, false);
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
