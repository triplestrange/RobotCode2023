// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gameplay.automations;

import javax.swing.text.rtf.RTFEditorKit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private final SwerveDrive m_SwerveDrive;
  PIDController robotGyro = new PIDController(0.04, 0, 0.015);
  Pose2d balancePos = new Pose2d(); 
  private Timer lastUnbalancedTime =  new Timer();

  public Balance(SwerveDrive swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
    this.m_SwerveDrive = swerveDrive;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tilt = m_SwerveDrive.navXRoll();


    SmartDashboard.putNumber("Time since !balanced", lastUnbalancedTime.get());
    SmartDashboard.putNumber("Robot Tilt", tilt);
    

    
    if (Math.abs(tilt) > 2.5) {lastUnbalancedTime.reset();}
    double xSpeed = robotGyro.calculate(tilt, 0);
    //FIXME probably a method for this
    //FIXME fix the speed param for field orient
    xSpeed = MathUtil.clamp(xSpeed, -SwerveConstants.climbMaxSpeedMetersPerSecond, SwerveConstants.climbMaxSpeedMetersPerSecond);
    balancePos = new Pose2d(balancePos.getX() + xSpeed/50, balancePos.getY(), new Rotation2d(balancePos.getRotation().getRadians()));
    SmartDashboard.putNumber("balanceX", xSpeed);
    m_SwerveDrive.drive(xSpeed, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveDrive.setXWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lastUnbalancedTime.hasElapsed(1);
  }
}
