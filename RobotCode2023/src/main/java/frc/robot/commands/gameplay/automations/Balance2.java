// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gameplay.automations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;


public class Balance2 extends CommandBase {
  /** Creates a new Balance. */
  /*
   * Balances on endgame piece
   */
  private final SwerveDrive swerveDrive;
  private boolean finished;

  public Balance2(SwerveDrive swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
    this.swerveDrive = swerveDrive;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tilt = swerveDrive.navXRoll();

    SmartDashboard.putNumber("Robot Tilt", tilt);
    
    if (Math.abs(tilt) < 5) finished = true;

    double xSpeed = -Math.signum(tilt) * 0.3;
    swerveDrive.drive(xSpeed, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(0,0,0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
