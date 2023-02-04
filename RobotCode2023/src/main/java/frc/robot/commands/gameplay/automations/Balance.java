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


public class Balance extends CommandBase {
  /** Creates a new Balance. */
  /*
   * Balances on endgame piece
   */
  private final SwerveDrive swerveDrive;
  PIDController robotGyro = new PIDController(0.05, 0, 0);
  private Timer lastUnbalancedTime =  new Timer();

  public Balance(SwerveDrive swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
    this.swerveDrive = swerveDrive;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tilt = swerveDrive.navXPitch();


    SmartDashboard.putNumber("Time since !balanced", lastUnbalancedTime.get());
    SmartDashboard.putNumber("Robot Tilt", tilt);
    

    
    if (Math.abs(tilt) > 2.5) {lastUnbalancedTime.reset();}
    double xSpeed = robotGyro.calculate(tilt, 0);
    //FIXME probably a method for this
    if (xSpeed > SwerveConstants.climbMaxSpeedMetersPerSecond) {
      xSpeed = SwerveConstants.climbMaxSpeedMetersPerSecond;}
    else if (xSpeed < -SwerveConstants.climbMaxSpeedMetersPerSecond) {
    xSpeed = -SwerveConstants.climbMaxSpeedMetersPerSecond;}
    swerveDrive.drive(xSpeed, 0, 0, isFinished());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lastUnbalancedTime.hasElapsed(1);
  }
}
