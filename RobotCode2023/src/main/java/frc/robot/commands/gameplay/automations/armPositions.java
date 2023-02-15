// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gameplay.automations;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.JointAngles;

public class armPositions extends CommandBase {
  private final Arm arm;
  private double sAngle;
  private double eAngle;
  private double wAngle;
  public armPositions(JointAngles jointAngles, Arm arm) {
    addRequirements(arm);
    this.arm = arm;
    sAngle = jointAngles.shoulderAngle;
    eAngle = jointAngles.elbowAngle;
    wAngle = jointAngles.wristAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setShoulder(sAngle);
    arm.setElbow(eAngle);
    arm.setWrist(wAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(sAngle - arm.getShoulder()) < Constants.armConstants.ERROR_IN_RADIANS && 
    Math.abs(eAngle - arm.getElbow()) < Constants.armConstants.ERROR_IN_RADIANS && 
    Math.abs(wAngle - arm.getWrist()) < Constants.armConstants.ERROR_IN_RADIANS)  {
      return true;
    }
    return false;
  }


}
